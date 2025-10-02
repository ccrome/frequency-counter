#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Gpt2FreqMeter.h"
#include "pins.h"
#include "SiT5501.h"
#include "display.h"
#include <ArduinoNmeaParser.h>
#include <MTP_Teensy.h>
void onRmcUpdate(nmea::RmcData const rmc);

SiT5501 oscillator(0x60);
ArduinoNmeaParser parser(onRmcUpdate, nullptr);
// PPS/Frequency measurement data structure
struct PpsData {
  uint32_t ticks;              // Also represents freq_hz (ticks = Hz for 1 second PPS)
  double avg_freq_hz;          // Running average frequency
  double ppm_instantaneous;    // Instantaneous PPM error
  double ppm_average;          // Average PPM error
  bool has_data;
};

// GPS data structure - using fixed-size char arrays to prevent fragmentation
struct GpsData {
  char timestamp[24];      // "2025-09-29T12:34:56Z" + null terminator
  char source[16];         // "GLONASS" + null terminator  
  double latitude;
  double longitude;
  double speed;
  double course;
  double magnetic_variation;
  bool is_valid;
};

// Statistics class using Welford's algorithm for numerical stability
class FrequencyStats {
private:
  double running_mean;
  double running_m2;  // Sum of squares of differences from mean
  uint32_t sample_count;

public:
  FrequencyStats() : running_mean(0.0), running_m2(0.0), sample_count(0) {}
  
  void reset() {
    running_mean = 0.0;
    running_m2 = 0.0;
    sample_count = 0;
  }
  
  void add_sample(double value) {
    sample_count++;
    double delta = value - running_mean;
    running_mean += delta / sample_count;
    double delta2 = value - running_mean;
    running_m2 += delta * delta2;
  }
  
  double get_mean() const { return running_mean; }
  uint32_t get_count() const { return sample_count; }
  
  double get_variance() const {
    return (sample_count > 1) ? (running_m2 / (sample_count - 1)) : 0.0;
  }
  
  double get_std_dev() const {
    return sqrt(get_variance());
  }
  
  bool has_samples() const {
    return sample_count > 0;
  }
  
  // Get PPM error relative to reference frequency
  double get_ppm_error(double reference_hz) const {
    if (!has_samples()) return 0.0;
    return ((running_mean - reference_hz) / reference_hz) * 1e6;
  }
  
  // Get PPB error relative to reference frequency  
  double get_ppb_error(double reference_hz) const {
    return get_ppm_error(reference_hz) * 1000.0;
  }
};

static FrequencyStats g_freq_stats;

// Calibration state
enum CalibrationState {
  CAL_IDLE,
  CAL_PHASE1_AVERAGING,  // First phase of averaging
  CAL_PHASE2_AVERAGING   // Second phase of averaging after offset applied
};

struct CalibrationData {
  CalibrationState state;
  uint32_t start_time;
  double phase1_result;
  double original_offset;
  uint32_t duration_seconds;  // Configurable calibration duration
  uint32_t expected_pulses;   // Expected number of pulses for current phase
  uint32_t missed_pulses;     // Count of missed pulses during calibration
  bool auto_started;          // True if calibration was auto-started
};

static CalibrationData g_calibration = {CAL_IDLE, 0, 0.0, 0.0, 300, 0, 0, false};

// Data structures
static PpsData g_pps_data = {0};
static GpsData g_gps_data = {0};


// SD card logging
static bool g_sd_available = false;
static bool g_pause_updates = false;
static bool g_verbose_timing = false;  // Start with verbose timing off
static char g_current_log_file[32] = "";  // Fixed-size buffer for log filename
static File g_log_file;
static uint32_t g_last_pps_millis = 0;


// Persistent frequency offset (stored in EEPROM)
static double g_frequency_offset_ppm = 0.0;  // Default 0.0 ppm (originally 15.26 ppb = 0.01526 ppm)

// EEPROM data structure (version 1 - for migration)
struct EepromDataV1 {
  uint32_t magic;              // Magic number for validity check
  uint16_t version;            // Data structure version
  double frequency_offset_ppm; // Frequency offset in PPM
  uint16_t checksum;           // Simple checksum for data integrity
};

// EEPROM data structure (version 2 - current)
struct EepromData {
  uint32_t magic;              // Magic number for validity check
  uint16_t version;            // Data structure version
  double frequency_offset_ppm; // Frequency offset in PPM
  uint8_t duty_cycle_percent;  // PPS output duty cycle percentage
  uint8_t reserved;            // Reserved for future use (alignment)
  uint16_t checksum;           // Simple checksum for data integrity
};

static const uint32_t EEPROM_MAGIC = 0x12345678;
static const uint16_t EEPROM_VERSION = 2;  // Incremented for new structure
static const int EEPROM_DATA_ADDR = 0;


const char* rmc_source_map[] = {
    "Unknown", "GPS", "Galileo", "GLONASS", "GNSS", "BDS"
};

// Convert float to JSON string - writes to provided buffer to avoid String allocation
void float_to_json_string(char* buffer, size_t buffer_size, double value, int precision) {
  if (isnan(value)) {
    strncpy(buffer, "null", buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
  } else {
    snprintf(buffer, buffer_size, "%.*f", precision, value);
  }
}

void log_json_field(File& file, const char* name, const char* value, bool is_first = false) {
  if (!is_first) file.print(",");
  file.printf("\"%s\":\"%s\"", name, value);
}

void log_json_field(File& file, const char* name, uint32_t value, bool is_first = false) {
  if (!is_first) file.print(",");
  file.printf("\"%s\":%lu", name, value);
}

void log_json_field_if_valid(File& file, const char* name, double value, int precision) {
  if (!isnan(value)) {
    char buffer[32];
    float_to_json_string(buffer, sizeof(buffer), value, precision);
    file.print(",");
    file.printf("\"%s\":%s", name, buffer);
  }
}

void log_json_field_if_valid(File& file, const char* name, uint32_t value) {
  if (value != 0) {  // Assume 0 means invalid for uint32_t
    log_json_field(file, name, value);
  }
}

void onRmcUpdate(nmea::RmcData const rmc)
{
    if (rmc.is_valid) {
	nmea::RmcSource rmc_source = rmc.source;
	if (rmc_source > nmea::RmcSource::BDS)
	    rmc_source = nmea::RmcSource::Unknown;
	int rmc_source_i = static_cast<int>(rmc_source);
	// Bounds check for safety
	if (rmc_source_i < 0 || rmc_source_i >= (int)(sizeof(rmc_source_map)/sizeof(rmc_source_map[0]))) {
	    rmc_source_i = 0; // Default to "Unknown"
	}
	const char *rmc_source_s = rmc_source_map[rmc_source_i];

	// Store GPS data in struct - using snprintf to avoid String fragmentation
	snprintf(g_gps_data.timestamp, sizeof(g_gps_data.timestamp), 
	         "%04d-%02d-%02dT%02d:%02d:%02dZ",
	         rmc.date.year, rmc.date.month, rmc.date.day,
	         rmc.time_utc.hour, rmc.time_utc.minute, rmc.time_utc.second);
	strncpy(g_gps_data.source, rmc_source_s, sizeof(g_gps_data.source) - 1);
	g_gps_data.source[sizeof(g_gps_data.source) - 1] = '\0';  // Ensure null termination
	g_gps_data.latitude = rmc.latitude;
	g_gps_data.longitude = rmc.longitude;
	g_gps_data.speed = rmc.speed;
	g_gps_data.course = rmc.course;
	g_gps_data.magnetic_variation = rmc.magnetic_variation;
	g_gps_data.is_valid = true;

	// Create log file on first GPS message using GPS timestamp
	if (!g_log_file && g_sd_available) {
	    // Create filename from GPS timestamp (replace : with - for filesystem compatibility)
	    char filename[32];
	    strncpy(filename, g_gps_data.timestamp, sizeof(filename) - 7);  // Leave room for ".jsonl"
	    filename[sizeof(filename) - 7] = '\0';
	    
	    // Replace : with - for filesystem compatibility
	    for (int i = 0; filename[i] != '\0'; i++) {
	        if (filename[i] == ':') filename[i] = '-';
	    }
	    strcat(filename, ".jsonl");
	    
	    strncpy(g_current_log_file, filename, sizeof(g_current_log_file) - 1);
	    g_current_log_file[sizeof(g_current_log_file) - 1] = '\0';
	    g_log_file = SD.open(filename, FILE_WRITE);
	    
	    if (g_log_file) {
	        Serial.printf("Created GPS-timestamped log file: %s\r\n", filename);
	    } else {
	        Serial.printf("Failed to create log file: %s\r\n", filename);
	    }
	}

	// Create combined log entry with both frequency and GPS data
	if (g_pps_data.has_data && g_log_file) {
	    // Calculate derived values
	    double freq_hz = (double)g_pps_data.ticks;  // ticks = Hz for 1 second PPS
	    
	    // Start JSON object
	    g_log_file.print("{");
	    
	    // GPS timestamp and source (always present)
            log_json_field(g_log_file, "gps_timestamp", g_gps_data.timestamp, true);
            log_json_field(g_log_file, "gps_source", g_gps_data.source);
	    
	    // GPS data (only if valid)
	    log_json_field_if_valid(g_log_file, "gps_lat", g_gps_data.latitude, 6);
	    log_json_field_if_valid(g_log_file, "gps_lon", g_gps_data.longitude, 6);
	    log_json_field_if_valid(g_log_file, "gps_speed", g_gps_data.speed, 4);
	    log_json_field_if_valid(g_log_file, "gps_course", g_gps_data.course, 2);
	    log_json_field_if_valid(g_log_file, "gps_magnetic_variation", g_gps_data.magnetic_variation, 4);
	    
	    // Frequency data (only if valid)
	    log_json_field_if_valid(g_log_file, "ticks", g_pps_data.ticks);
	    log_json_field_if_valid(g_log_file, "freq_hz", freq_hz, 6);
	    log_json_field_if_valid(g_log_file, "avg_freq_hz", g_pps_data.avg_freq_hz, 12);
	    log_json_field_if_valid(g_log_file, "ppm_instantaneous", g_pps_data.ppm_instantaneous, 6);
	    log_json_field_if_valid(g_log_file, "ppm_average", g_pps_data.ppm_average, 6);
	    log_json_field_if_valid(g_log_file, "oscillator_offset_ppm", g_frequency_offset_ppm, 6);
	    
	    // End JSON object
	    g_log_file.println("}");
	    g_log_file.flush();
	    
	    // Reset frequency data flag after logging
	    g_pps_data.has_data = false;
	}
    }
}

void check_gpt2_counter() {
  Serial.println("Checking if GPT2 counter is running...\r");
  uint32_t count1 = GPT2_CNT;
  delay(10);
  uint32_t count2 = GPT2_CNT;
  
  if (count2 > count1) {
    Serial.printf("GPT2 counter is running: %lu -> %lu (diff: %lu)\r\n", 
                  (unsigned long)count1, (unsigned long)count2, (unsigned long)(count2 - count1));
  } else if (count2 < count1) {
    Serial.printf("GPT2 counter wrapped: %lu -> %lu\r\n", 
                  (unsigned long)count1, (unsigned long)count2);
  } else {
    Serial.printf("WARNING: GPT2 counter not counting! Stuck at: %lu\r\n", (unsigned long)count1);
  }
}


void increase_peripheral_clock() {
  Serial.println("=== INCREASING PERIPHERAL CLOCK FOR 10MHz EXTERNAL CLOCK ===\r");
  
  Serial.printf("Current CCM_CSCMR1 = 0x%08lX\r\n", (unsigned long)CCM_CSCMR1);
  
  uint32_t cscmr1 = CCM_CSCMR1;
  cscmr1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;
  cscmr1 &= ~(0x3F << 0);
  cscmr1 |= CCM_CSCMR1_PERCLK_PODF(1);
  CCM_CSCMR1 = cscmr1;
  
  Serial.printf("New CCM_CSCMR1 = 0x%08lX\r\n", (unsigned long)CCM_CSCMR1);
  Serial.println("PERCLK increased to ~75MHz (10MHz < 75MHz/4 = 18.75MHz ✅)\r");
  
  delay(10);
}

void print_mode_commands() {
  Serial.println("System Control:\r");
  Serial.println("  s - Show current status\r");
  Serial.println("  r - Reset frequency measurement statistics\r");
  Serial.println("  c - Cycle GPS PPS capture edge (rising/falling/both)\r");
  Serial.printf("  l       - Start calibration procedure (2x %lu second averaging)\r\n", g_calibration.duration_seconds);
  Serial.println("  l<time> - Start calibration with custom duration in seconds (e.g., l60 or l600)\r");
}

void print_output_commands() {
  Serial.println("Output Generation (Always Active):\r");
  Serial.println("  f       - Re-arm GPT-driven 1 PPS output\r");
  Serial.println("  g0/g1   - Force PPS output low/high via GPIO\r");
  Serial.println("  d       - Show current PPS output duty cycle\r");
  Serial.println("  d<pct>  - Set PPS output duty cycle (20-80%, e.g., d20 or d50)\r");
}

void print_oscillator_commands() {
  Serial.println("SiT5501 Oscillator Control:\r");
  Serial.println("  p<ppb>  - Set frequency offset in PPB and save to EEPROM (e.g., p+5200 or p-3100)\r");
  Serial.println("  p       - Show current frequency offset\r");
  Serial.println("  e       - Enable oscillator output\r");
  Serial.println("  z       - Disable oscillator output\r");
}

void print_other_commands() {
  Serial.println("Other:\r");
  Serial.println("  h       - Show this help\r");
  Serial.println("  v       - Toggle verbose timing output (currently OFF)\r");
  Serial.println("  x       - Clear EEPROM and reset all settings to defaults\r");
  Serial.println("  b       - Reboot to bootloader mode\r");
}

void print_help() {
  Serial.println("\n=== Frequency Counter Commands ===\r");
  print_mode_commands();
  Serial.println("\r");
  print_output_commands();
  Serial.println("\r");
  print_oscillator_commands();
  Serial.println("\r");
  print_other_commands();
  Serial.printf("\rDisplay: %s\r\n", display_available() ? "OLED detected" : "(none)");
  Serial.println("\r");
}

void wait_for_serial() {
  int startup_delay = 200;
  while (startup_delay > 0) {
    Serial.printf("Waiting for serial...%d\r\n", startup_delay/1000);
    delay(1000);
    startup_delay -= 1000;
  }
}

void print_startup_info() {
  Serial.println("=== Precision Frequency Counter ===\r");
  Serial.printf("Pin Configuration:\r\n");
  Serial.printf("  Pin %d: External clock input (optional)\r\n", GPT2_EXTCLK_PIN);
  Serial.printf("  Pin %d: GPS PPS input (always monitoring)\r\n", GPT2_CAPTURE_PIN);
  Serial.printf("  Pin %d: SDA (I2C for SiT5501 oscillator)\r\n", 18);
  Serial.printf("  Pin %d: SCL (I2C for SiT5501 oscillator)\r\n", 19);
  Serial.printf("  Pin %d: Clock generator output enable\r\n", CLKGEN_OE);
  Serial.printf("  Pin %d: 10 MHz clock output enable\r\n", CLK_10MHZ_OE);
  Serial.printf("  Pin %d: PPS output enable\r\n", PPS_OE);
  Serial.printf("  Pin %d: 1 PPS output (always active)\r\n", GPT2_COMPARE_PIN);
  Serial.println("\r");
}

void initialize_gpt2() {
  increase_peripheral_clock();
  gpt2_begin_dual_mode(1, GPT_EDGE_RISING, true);  // 1 PPS output, rising edge capture
  check_gpt2_counter();
}

void initialize_oscillator() {
  oscillator.begin();
  if (!oscillator.isPresent()) {
    Serial.println("WARNING: SiT5501 oscillator not found on I2C bus\r");
    Serial.println("Frequency counter will work without oscillator control\r");
  } else {
    Serial.println("SiT5501 oscillator found and initialized\r");
    oscillator.setOutputEnable(true);
  }
}
void initialize_mtp() {
    if (g_sd_available) {
	MTP.begin();
	MTP.addFilesystem(SD, "SD Card", MTP_FSTYPE_SD);
    }
}
    
void setup() {
  Serial1.begin(9600);
  setup_pins();
  Serial.begin(115200);
  wait_for_serial();

  print_startup_info();
  initialize_sd_card();
  initialize_mtp();
  initialize_gpt2();
  initialize_oscillator();
  if (display_init()) {
    Serial.println("OLED display initialized successfully\r");
  } else {
    Serial.println("OLED display unavailable, continuing without it\r");
  }
  load_settings();  // Load persistent settings (frequency offset and duty cycle)

  print_help();
  Serial.println("System initialized. 1 PPS output active, GPS PPS monitoring enabled.\r\n");
  
  // Apply the loaded frequency offset
  oscillator.setFrequencyOffsetPPM(g_frequency_offset_ppm);
  Serial.printf("Applied frequency offset: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);

}


void reset_measurement_stats() {
  g_freq_stats.reset();
}

uint16_t calculate_checksum(const EepromData& data) {
  uint16_t checksum = 0;
  const uint8_t* bytes = (const uint8_t*)&data;
  
  // Calculate checksum for all fields except checksum itself
  size_t checksum_offset = offsetof(EepromData, checksum);
  for (size_t i = 0; i < checksum_offset; i++) {
    checksum += bytes[i];
  }
  
  return checksum;
}

uint16_t calculate_checksum_v1(const EepromDataV1& data) {
  uint16_t checksum = 0;
  const uint8_t* bytes = (const uint8_t*)&data;
  
  // Calculate checksum for all fields except checksum itself
  size_t checksum_offset = offsetof(EepromDataV1, checksum);
  for (size_t i = 0; i < checksum_offset; i++) {
    checksum += bytes[i];
  }
  
  return checksum;
}

bool migrate_from_v1() {
  EepromDataV1 old_data;
  EEPROM.get(EEPROM_DATA_ADDR, old_data);
  
  // Validate V1 data
  if (old_data.magic != EEPROM_MAGIC || old_data.version != 1) {
    return false;  // Not valid V1 data
  }
  
  // Validate V1 checksum
  uint16_t calculated_checksum = calculate_checksum_v1(old_data);
  if (old_data.checksum != calculated_checksum) {
    Serial.println("V1 EEPROM checksum invalid, cannot migrate\r");
    return false;
  }
  
  // Validate frequency offset range
  double pull_range = oscillator.isPresent() ? oscillator.getPullRange() : 50.0;
  if (isnan(old_data.frequency_offset_ppm) || 
      old_data.frequency_offset_ppm < -pull_range || 
      old_data.frequency_offset_ppm > pull_range) {
    Serial.printf("V1 frequency offset out of range (±%.0f ppb), cannot migrate\r\n", pull_range * 1000.0);
    return false;
  }
  
  // Migration successful - preserve PPM offset, set default duty cycle
  g_frequency_offset_ppm = old_data.frequency_offset_ppm;
  gpt2_set_duty_cycle(20);  // Default duty cycle for migration
  
  Serial.printf("Migrated from EEPROM V1: frequency offset %.1f ppb, duty cycle set to default 20%%\r\n", 
                g_frequency_offset_ppm * 1000.0);
  
  // Save as V2 format
  save_settings();
  
  return true;
}

void load_settings() {
  EepromData data;
  EEPROM.get(EEPROM_DATA_ADDR, data);
  
  // Validate magic number and version
  if (data.magic != EEPROM_MAGIC || data.version != EEPROM_VERSION) {
    // Try to migrate from V1 before using defaults
    if (migrate_from_v1()) {
      return;  // Migration successful, settings are now loaded and saved as V2
    }
    
    Serial.println("EEPROM data invalid or outdated, using defaults\r");
    g_frequency_offset_ppm = 0.0;  // Default to 0 ppm
    gpt2_set_duty_cycle(20);       // Default to 20% duty cycle
    save_settings();
    return;
  }
  
  // Validate checksum
  uint16_t calculated_checksum = calculate_checksum(data);
  if (data.checksum != calculated_checksum) {
    Serial.println("EEPROM checksum invalid, using defaults\r");
    g_frequency_offset_ppm = 0.0;  // Default to 0 ppm
    gpt2_set_duty_cycle(20);       // Default to 20% duty cycle
    save_settings();
    return;
  }
  
  // Validate frequency offset range
  double pull_range = oscillator.isPresent() ? oscillator.getPullRange() : 50.0; // Default to reasonable range if oscillator not present
  if (isnan(data.frequency_offset_ppm) || data.frequency_offset_ppm < -pull_range || data.frequency_offset_ppm > pull_range) {
    Serial.printf("EEPROM frequency offset out of range (±%.0f ppb), using defaults\r\n", pull_range * 1000.0);
    g_frequency_offset_ppm = 0.0;  // Default to 0 ppm
    gpt2_set_duty_cycle(20);       // Default to 20% duty cycle
    save_settings();
    return;
  }
  
  // Validate duty cycle range
  if (data.duty_cycle_percent < 20 || data.duty_cycle_percent > 80) {
    Serial.printf("EEPROM duty cycle out of range (%u%%), using default 20%%\r\n", data.duty_cycle_percent);
    gpt2_set_duty_cycle(20);       // Default to 20% duty cycle
  } else {
    gpt2_set_duty_cycle(data.duty_cycle_percent);
    Serial.printf("Loaded duty cycle: %u%%\r\n", data.duty_cycle_percent);
  }
  
  g_frequency_offset_ppm = data.frequency_offset_ppm;
  Serial.printf("Loaded frequency offset: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
}

void save_settings() {
  EepromData data;
  data.magic = EEPROM_MAGIC;
  data.version = EEPROM_VERSION;
  data.frequency_offset_ppm = g_frequency_offset_ppm;
  data.duty_cycle_percent = gpt2_get_duty_cycle();
  data.reserved = 0;  // Clear reserved field
  data.checksum = calculate_checksum(data);
  
  EEPROM.put(EEPROM_DATA_ADDR, data);
  Serial.printf("Saved frequency offset: %.1f ppb, duty cycle: %u%%\r\n", 
                g_frequency_offset_ppm * 1000.0, data.duty_cycle_percent);
}

bool initialize_sd_card() {
  // Teensy 4.1 built-in SD card uses BUILTIN_SDCARD
  if (SD.begin(BUILTIN_SDCARD)) {
    g_sd_available = true;
    Serial.println("SD card initialized successfully\r");
    Serial.println("Log file will be created when first GPS message is received\r");
    return true;
  } else {
    Serial.println("SD card initialization failed\r");
    g_sd_available = false;
    return false;
  }
}


void cmd_reset_measurements() {
  reset_measurement_stats();
  Serial.println("Frequency measurement statistics reset\r");
  Serial.printf("GPS PPS input: pin %d (always monitoring)\r\n", GPT2_CAPTURE_PIN);
  Serial.printf("1 PPS output: pin %d (always active)\r\n", GPT2_COMPARE_PIN);
}

void cmd_set_capture_edge() {
  // Cycle through edge detection modes
  static GptCaptureEdge current_edge = GPT_EDGE_RISING;

  switch (current_edge) {
    case GPT_EDGE_RISING:
      current_edge = GPT_EDGE_FALLING;
      Serial.println("Capture edge set to FALLING\r");
      break;
    case GPT_EDGE_FALLING:
      current_edge = GPT_EDGE_BOTH;
      Serial.println("Capture edge set to BOTH\r");
      break;
    case GPT_EDGE_BOTH:
      current_edge = GPT_EDGE_RISING;
      Serial.println("Capture edge set to RISING\r");
      break;
    default:
      current_edge = GPT_EDGE_RISING;
      Serial.println("Capture edge set to RISING\r");
      break;
  }

  gpt2_set_capture_edge(current_edge);
}

void cmd_set_output_frequency(const char* command) {
  (void)command;
  if (pps_release_to_gpt()) {
    Serial.println("Resumed GPT2 control of PPS output.\r");
  }
  gpt2_begin_dual_mode(1, GPT_EDGE_RISING, true);
  Serial.println("Output frequency set to 1 Hz\r");
  Serial.printf("Signal available on pin %d\r\n", GPT2_COMPARE_PIN);
}


void show_gpt2_status() {
  Serial.printf("GPS PPS samples collected: %lu\r\n", g_freq_stats.get_count());
  if (g_freq_stats.has_samples()) {
    Serial.printf("Average measured frequency: %.6f MHz\r\n", g_freq_stats.get_mean() / 1e6);
  } else {
    Serial.println("No GPS PPS signals received yet\r");
  }
  
  Serial.println("Pin Configuration:\r");
  Serial.printf("  Pin %d: External clock input (optional)\r\n", GPT2_EXTCLK_PIN);
  Serial.printf("  Pin %d: GPS PPS input (always monitoring)\r\n", GPT2_CAPTURE_PIN);
  Serial.printf("  Pin %d: SDA (I2C for SiT5501 oscillator)\r\n", 18);
  Serial.printf("  Pin %d: SCL (I2C for SiT5501 oscillator)\r\n", 19);
  Serial.printf("  Pin %d: Clock generator output enable\r\n", CLKGEN_OE);
  Serial.printf("  Pin %d: 10 MHz clock output enable\r\n", CLK_10MHZ_OE);
  Serial.printf("  Pin %d: PPS output enable\r\n", PPS_OE);
  Serial.printf("  Pin %d: 1 PPS output (always active)\r\n", GPT2_COMPARE_PIN);
  
  Serial.printf("PPS Output Duty Cycle: %u%%\r\n", gpt2_get_duty_cycle());
  Serial.printf("GPT2 Counter: %lu\r\n", GPT2_CNT);
  Serial.printf("GPT2 Control: 0x%08lX\r\n", GPT2_CR);
  Serial.printf("Compare Register: %lu\r\n", GPT2_OCR1);
  if (pps_gpio_override_active()) {
    Serial.printf("PPS override: GPIO driving %s\r\n", pps_gpio_state_high() ? "HIGH" : "LOW");
  }
}

void show_oscillator_status() {
  if (oscillator.isPresent()) {
      // 
  } else {
    Serial.println("SiT5501: Not present\r");
  }
}


void show_calibration_status() {
  if (g_calibration.state == CAL_IDLE) {
    Serial.println("Calibration: Not active\r");
    return;
  }
  
  uint32_t elapsed_ms = millis() - g_calibration.start_time;
  uint32_t elapsed_seconds = elapsed_ms / 1000;
  uint32_t remaining_seconds = g_calibration.duration_seconds - elapsed_seconds;
  
  double current_avg_ppm = 0.0;
  if (g_freq_stats.has_samples()) {
    current_avg_ppm = g_freq_stats.get_ppm_error(10000000.0);
  }
  
  if (g_calibration.state == CAL_PHASE1_AVERAGING) {
    Serial.printf("Calibration Phase 1: %lu/%lu seconds (%lu remaining)\r\n", 
                  elapsed_seconds, g_calibration.duration_seconds, remaining_seconds);
    Serial.printf("Current average: %.1f ppb (%lu samples, %lu missed)\r\n", 
                  current_avg_ppm * 1000.0, g_freq_stats.get_count(), g_calibration.missed_pulses);
  } else if (g_calibration.state == CAL_PHASE2_AVERAGING) {
    Serial.printf("Calibration Phase 2: %lu/%lu seconds (%lu remaining)\r\n", 
                  elapsed_seconds, g_calibration.duration_seconds, remaining_seconds);
    Serial.printf("Applied correction: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
    Serial.printf("Current average: %.1f ppb (%lu samples, %lu missed)\r\n", 
                  current_avg_ppm * 1000.0, g_freq_stats.get_count(), g_calibration.missed_pulses);
  }
}

void cmd_show_status() {
  Serial.println("\r\n=== System Status ===\r");
  show_gpt2_status();
  show_oscillator_status();
  show_calibration_status();
}

void cmd_set_oscillator_ppm(const char* command) {
  if (!oscillator.isPresent()) {
    Serial.println("Error: SiT5501 oscillator not found\r");
    return;
  }

  const char* param = command + 1;  // Skip the command character
  
  if (strlen(param) == 0) {
    // Show current offset
    double pull_range = oscillator.getPullRange();
    Serial.printf("Current frequency offset: %.1f ppb (range: ±%.0f ppb)\r\n", g_frequency_offset_ppm * 1000.0, pull_range * 1000.0);
    return;
  }
  
  double ppb = atof(param);
  double ppm = ppb / 1000.0;  // Convert ppb input to ppm for internal use
  double pull_range = oscillator.getPullRange();
  
  // Check against oscillator's actual range
  if (ppm < -pull_range || ppm > pull_range) {
    Serial.printf("Error: Value must be between %.0f and %.0f ppb (oscillator pull range)\r\n", -pull_range * 1000.0, pull_range * 1000.0);
    return;
  }
  
  if (oscillator.setFrequencyOffsetPPM(ppm)) {
    // Update global variable and save to EEPROM
    g_frequency_offset_ppm = ppm;
    save_settings();
    Serial.printf("Oscillator frequency offset set to %.1f ppb and saved to EEPROM\r\n", ppb);
  } else {
    Serial.println("Error: Failed to set oscillator frequency offset\r");
  }
}

void cmd_read_oscillator() {
  if (oscillator.isPresent()) {
      Serial.printf("SiT5501 Registers: ");
      uint16_t value;
      for (int i = 0; i < oscillator.N_REGISTERS; i++) {
          oscillator.readRegister(i, value);
          Serial.printf("0x%08x, ", value);
      }
      Serial.printf("\r\n");
  } else {
      Serial.println("Error: SiT5501 oscillator not found\r");
      return;
  }
}

void cmd_oscillator_output_enable(bool enable) {
  if (!oscillator.isPresent()) {
    Serial.println("Error: SiT5501 oscillator not found\r");
    return;
  }

  if (oscillator.setOutputEnable(enable)) {
    Serial.printf("Oscillator output %s\r\n", enable ? "enabled" : "disabled");
  } else {
    Serial.printf("Error %s oscillator output\r\n", enable ? "enabling" : "disabling");
  }
}

void cmd_set_duty_cycle(const char* command) {
  const char* param = command + 1;  // Skip the command character
  
  if (strlen(param) == 0) {
    // Show current duty cycle
    Serial.printf("Current PPS output duty cycle: %u%%\r\n", gpt2_get_duty_cycle());
    return;
  }
  
  int duty_cycle = atoi(param);
  
  if (duty_cycle < 20 || duty_cycle > 80) {
    Serial.println("Error: Duty cycle must be between 20 and 80 percent\r");
    return;
  }
  
  gpt2_set_duty_cycle((uint8_t)duty_cycle);
  save_settings();  // Save both frequency offset and duty cycle to EEPROM
  Serial.printf("PPS output duty cycle set to %u%% and saved to EEPROM\r\n", gpt2_get_duty_cycle());
}

void cmd_clear_eeprom() {
  Serial.println("\r\n=== CLEAR EEPROM ===\r");
  Serial.println("This will erase all saved settings and reset to defaults:\r");
  Serial.println("- Frequency offset: 0.0 ppb\r");
  Serial.println("- Duty cycle: 20%\r");
  Serial.println("\r");
  Serial.println("WARNING: This action cannot be undone!\r");
  Serial.println("Type 'y' to clear EEPROM, any other key to cancel: ");
  
  // Wait for user confirmation
  while (!Serial.available()) {
    delay(10);
  }
  
  char response = Serial.read();
  Serial.println(response); // Echo the response
  
  // Clear any remaining characters
  while (Serial.available()) {
    Serial.read();
  }
  
  if (response != 'y' && response != 'Y') {
    Serial.println("EEPROM clear cancelled.\r");
    return;
  }
  
  Serial.println("Clearing EEPROM...\r");
  
  // Reset to defaults
  g_frequency_offset_ppm = 0.0;
  gpt2_set_duty_cycle(20);
  
  // Apply the defaults to hardware
  if (oscillator.isPresent()) {
    oscillator.setFrequencyOffsetPPM(0.0);
  }
  
  // Save defaults to EEPROM (this overwrites the old data)
  save_settings();
  
  Serial.println("EEPROM cleared and reset to defaults.\r");
  Serial.println("System settings:\r");
  Serial.printf("- Frequency offset: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
  Serial.printf("- Duty cycle: %u%%\r\n", gpt2_get_duty_cycle());
}







void cmd_reboot_to_bootloader() {
  Serial.println("Rebooting to bootloader mode...\r");
  Serial.println("Device will disconnect and enter bootloader for firmware updates.\r");
  Serial.flush();  // Ensure all output is sent before reboot

  delay(500);  // Give time for serial output to complete

  // Disable all peripherals and interrupts before reboot
  __disable_irq();
  gpt2_stop();

  // Use Teensy-specific function to enter bootloader mode
  // This is the proper way to enter bootloader on Teensy 4.1
  _reboot_Teensyduino_();

  // Should never reach here
  while (1) {
    // Wait for reboot
  }
}

void cmd_start_calibration_with_time(const char* command) {
  const char* param = command + 1;  // Skip the command character
  
  // Set calibration duration
  if (strlen(param) > 0) {
    uint32_t duration = atoi(param);
    if (duration < 10 || duration > 3600) {
      Serial.println("Error: Calibration time must be between 10 and 3600 seconds\r");
      return;
    }
    g_calibration.duration_seconds = duration;
    Serial.printf("Calibration duration set to %lu seconds per phase\r\n", duration);
  } else {
    // Reset to default if no parameter
    g_calibration.duration_seconds = 300;
    Serial.printf("Using default calibration duration: %lu seconds per phase\r\n", g_calibration.duration_seconds);
  }
  
  // Call the existing calibration function
  cmd_start_calibration();
}

void start_calibration_internal(bool auto_started) {
  // Store original offset
  g_calibration.original_offset = g_frequency_offset_ppm;
  g_calibration.auto_started = auto_started;
  
  // Set PPM offset to 0
  g_frequency_offset_ppm = 0.0;
  oscillator.setFrequencyOffsetPPM(0.0);
  
  // Wait 1 millisecond as requested
  delay(1);
  
  // Reset averaging
  reset_measurement_stats();
  
  // Start calibration
  g_calibration.state = CAL_PHASE1_AVERAGING;
  g_calibration.start_time = millis();
  g_calibration.phase1_result = 0.0;
  g_calibration.expected_pulses = g_freq_stats.get_count();  // Track expected pulses from current count
  g_calibration.missed_pulses = 0;
  
  if (auto_started) {
    Serial.println("\r\n=== AUTO-CALIBRATION STARTED ===\r");
    Serial.println("No calibration offset detected. Starting automatic calibration.\r");
  }
  
  Serial.printf("Calibration phase 1 started. Collecting data for %lu seconds...\r\n", g_calibration.duration_seconds);
  Serial.println("Status updates every 1 second. Use 's' command to check progress anytime.\r");
}

void cmd_start_calibration() {
  if (g_calibration.state != CAL_IDLE) {
    Serial.println("Calibration already in progress. Wait for completion or restart system.\r");
    return;
  }
  
  if (!oscillator.isPresent()) {
    Serial.println("Error: SiT5501 oscillator not found - calibration requires oscillator control\r");
    return;
  }
  
  // Show calibration information and ask for confirmation
  Serial.println("\r\n=== CALIBRATION PROCEDURE ===\r");
  Serial.printf("This will run a 2-phase calibration (%lu seconds each phase):\r\n", g_calibration.duration_seconds);
  Serial.println("Phase 1: Reset offset to 0 and measure frequency error\r");
  Serial.println("Phase 2: Apply correction and verify results\r");
  Serial.printf("Current offset: %.1f ppb will be temporarily changed\r\n", g_frequency_offset_ppm * 1000.0);
  Serial.println("\r");
  uint32_t total_minutes = (g_calibration.duration_seconds * 2) / 60;
  Serial.printf("WARNING: This will take approximately %lu minutes to complete.\r\n", total_minutes);
  Serial.println("Make sure GPS PPS signal is stable before proceeding.\r");
  Serial.println("\r");
  Serial.println("Type 'y' to start calibration, any other key to cancel: ");
  
  // Wait for user confirmation
  while (!Serial.available()) {
    delay(10);
  }
  
  char response = Serial.read();
  Serial.println(response); // Echo the response
  
  // Clear any remaining characters
  while (Serial.available()) {
    Serial.read();
  }
  
  if (response != 'y' && response != 'Y') {
    Serial.println("Calibration cancelled.\r");
    return;
  }
  
  Serial.println("Starting calibration procedure...\r");
  Serial.printf("Phase 1: Setting PPM offset to 0 and averaging for %lu seconds\r\n", g_calibration.duration_seconds);
  
  start_calibration_internal(false);
}

void handle_serial_commands() {
  static char command_buffer[16] = "";  // Fixed-size buffer to prevent heap allocation
  static uint8_t buffer_pos = 0;
  static bool reading_parameter_command = false;

  while (Serial.available()) {
    char c = Serial.read();

    if (!reading_parameter_command) {
      // Convert to lowercase for consistency
      if (c >= 'A' && c <= 'Z') {
        c = c - 'A' + 'a';
      }

      // Skip whitespace and control characters for single commands
      if (c < 32 || c > 126) continue;

      // Echo the character
      Serial.print(c);

        // Check if this is a parameter command that needs more input
      if (c == 'f' || c == 'p' || c == 'd' || c == 'x' || c == 'g' || c == 'l') {
          command_buffer[0] = c;
          command_buffer[1] = '\0';
          buffer_pos = 1;
          reading_parameter_command = true;
          continue;
      }

      // Process single-character commands immediately
      process_single_command(c);
      } else {
      // Reading parameter for commands that require arguments
      if (c == '\n' || c == '\r') {
        // Echo newline
        Serial.println();
        // Process complete parameter command
        process_parameter_command(command_buffer);
        command_buffer[0] = '\0';
        buffer_pos = 0;
        reading_parameter_command = false;
      } else if (c == 8 || c == 127) {  // Backspace or DEL
        if (buffer_pos > 1) {  // Don't delete the command character
          buffer_pos--;
          command_buffer[buffer_pos] = '\0';
          Serial.print("\b \b");  // Backspace, space, backspace to erase character
        }
      } else if (c >= 32 && c <= 126 && buffer_pos < sizeof(command_buffer) - 1) {  // Printable characters only
        command_buffer[buffer_pos++] = c;
        command_buffer[buffer_pos] = '\0';
        Serial.print(c);  // Echo the character
      }
    }
  }
}

void process_single_command(char cmd) {
  switch (cmd) {
    case 's': cmd_show_status(); break;
    case 'r': cmd_reset_measurements(); break;
    case 'c': cmd_set_capture_edge(); break;
    case 'o': cmd_read_oscillator(); break;
    case 'e': cmd_oscillator_output_enable(true); break;
    case 'z': cmd_oscillator_output_enable(false); break;
    case 'h': print_help(); break;
    case 'b': cmd_reboot_to_bootloader(); break;
    case 'v': 
      g_verbose_timing = !g_verbose_timing;
      Serial.printf("Verbose timing: %s\r\n", g_verbose_timing ? "ON" : "OFF");
      break;
    case 'x': cmd_clear_eeprom(); break;
    default:
      Serial.println("Unknown command. Type 'h' for help.\r");
      break;
  }
  Serial.println("\r");
}

void process_parameter_command(const char* command) {
  // No need to trim or toLowerCase with char array - handle directly

  if (command[0] == '\0') return;

  char cmd = command[0];

  switch (cmd) {
    case 'f':
      if (strlen(command) > 1) {
        Serial.println("Usage: f\r");
        break;
      }
      cmd_set_output_frequency(command);
      break;
    case 'p': cmd_set_oscillator_ppm(command); break;
    case 'l': cmd_start_calibration_with_time(command); break;
    case 'd': cmd_set_duty_cycle(command); break;
    case 'g':
      if (strlen(command) == 2) {
        char arg = command[1];
        if (arg == '0') {
          pps_force_gpio(false);
          Serial.printf("PPS output forced LOW via GPIO on pin %d\r\n", GPT2_COMPARE_PIN);
          Serial.println("Use any f<freq> command to resume GPT control.\r");
        } else if (arg == '1') {
          pps_force_gpio(true);
          Serial.printf("PPS output forced HIGH via GPIO on pin %d\r\n", GPT2_COMPARE_PIN);
          Serial.println("Use any f<freq> command to resume GPT control.\r");
        } else {
          Serial.println("Usage: g0 (low) or g1 (high)\r");
        }
      } else {
        Serial.println("Usage: g0 (low) or g1 (high)\r");
      }
      break;
    default:
      Serial.println("Unknown parameter command.\r");
      break;
  }
  Serial.println("\r");
}

void process_frequency_measurement() {
  if (!gpt2_capture_available()) return;

  // Only process once per PPS pulse - read capture and immediately process
  uint32_t ticks = gpt2_read_capture();  // This resets capture_available to false
  double freq_hz = (double)ticks;  // Ticks = frequency in Hz (since PPS = 1 second)
  const double ref_hz = 10000000.0;  // 10 MHz reference


  if (freq_hz < ref_hz * 0.99 || freq_hz > ref_hz * 1.01) {
    Serial.printf("WARNING: Measured freq = %.6f Hz is outside 99-101%% of ref_hz = %.6f Hz\r\n", 
                  freq_hz, ref_hz);
    return;
  }

  // Update running statistics using Welford's algorithm for numerical stability
  g_freq_stats.add_sample(freq_hz);
  g_last_pps_millis = millis();

  // Store frequency measurement data in PPS struct
  g_pps_data.ticks = ticks;  // ticks = freq_hz for 1 second PPS
  g_pps_data.avg_freq_hz = g_freq_stats.get_mean();
  g_pps_data.ppm_instantaneous = ((freq_hz - ref_hz) / ref_hz) * 1e6;
  g_pps_data.ppm_average = g_freq_stats.get_ppm_error(ref_hz);
  g_pps_data.has_data = true;
  

  // Display results if verbose timing is enabled
  if (!g_pause_updates && g_verbose_timing) {
    double freq_mhz = g_pps_data.ticks / 1e6;  // Calculate MHz from ticks
    double avg_freq_mhz = g_pps_data.avg_freq_hz / 1e6;  // Calculate avg MHz
    Serial.printf("t=%lus ticks=%6d latest=%.6f MHz avg=%.12f MHz ppb(lat)=%.1f ppb(avg)=%.1f\r\n",
                  (unsigned long)g_freq_stats.get_count(), g_pps_data.ticks, freq_mhz, 
                  avg_freq_mhz, g_pps_data.ppm_instantaneous * 1000.0, g_pps_data.ppm_average * 1000.0);
  }
}


void print_oscillator_status() {
  if (oscillator.isPresent()) {
    for (int i = 0; i <= 0xff; i++) {
      uint16_t value;
      oscillator.readRegister(i, value);
      if (i % 8 == 0) {
        Serial.printf("\r\n%02d: ", i);
      }
      Serial.printf("0x%04lX, ", value);
    }
    Serial.printf("\r\n");
  } else {
    Serial.println("SiT5501: Not present\r");
  }
}

void process_calibration() {
  if (g_calibration.state == CAL_IDLE) {
    return;  // No calibration in progress
  }
  
  uint32_t elapsed_ms = millis() - g_calibration.start_time;
  uint32_t elapsed_seconds = elapsed_ms / 1000;
  
  // Check for missed pulses during calibration
  uint32_t expected_pulses_now = g_calibration.expected_pulses + elapsed_seconds;
  if (g_freq_stats.get_count() < expected_pulses_now) {
    uint32_t current_missed = expected_pulses_now - g_freq_stats.get_count();
    if (current_missed > g_calibration.missed_pulses) {
      g_calibration.missed_pulses = current_missed;
      if (g_calibration.missed_pulses > 10) {
        Serial.printf("\r\nCALIBRATION ABORTED: Too many missed GPS pulses (%lu missed)\r\n", g_calibration.missed_pulses);
        Serial.println("GPS signal appears unstable. Please check GPS reception.\r");
        
        // Restore original offset
        g_frequency_offset_ppm = g_calibration.original_offset;
        oscillator.setFrequencyOffsetPPM(g_frequency_offset_ppm);
        Serial.printf("Restored original offset: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
        
        g_calibration.state = CAL_IDLE;
        return;
      }
    }
  }
  
  // Progress reporting every 10 seconds during calibration
  static uint32_t last_progress_report = 0;
  if (elapsed_seconds >= last_progress_report + 1) {
    last_progress_report = elapsed_seconds;
    uint32_t remaining_seconds = g_calibration.duration_seconds - elapsed_seconds;
    
    if (g_calibration.state == CAL_PHASE1_AVERAGING) {
      double current_avg_ppm = 0.0;
      if (g_freq_stats.get_count() > 0) {
        double current_avg_hz = g_freq_stats.get_mean();
        current_avg_ppm = ((current_avg_hz - 10000000.0) / 10000000.0) * 1e6;
      }
      Serial.printf("Calibration Phase 1: %lu seconds remaining, current avg: %.1f ppb (%lu samples)\r\n", 
                    remaining_seconds, current_avg_ppm * 1000.0, g_freq_stats.get_count());
    } else if (g_calibration.state == CAL_PHASE2_AVERAGING) {
      double current_avg_ppm = 0.0;
      if (g_freq_stats.get_count() > 0) {
        double current_avg_hz = g_freq_stats.get_mean();
        current_avg_ppm = ((current_avg_hz - 10000000.0) / 10000000.0) * 1e6;
      }
      Serial.printf("Calibration Phase 2: %lu seconds remaining, current avg: %.1f ppb (%lu samples)\r\n", 
                    remaining_seconds, current_avg_ppm * 1000.0, g_freq_stats.get_count());
    }
  }
  
  if (elapsed_seconds >= g_calibration.duration_seconds) {  // Duration completed
    if (g_calibration.state == CAL_PHASE1_AVERAGING) {
      // Phase 1 complete - calculate average and start phase 2
      if (g_freq_stats.has_samples()) {
        g_calibration.phase1_result = g_freq_stats.get_mean();
        double avg_ppm = g_freq_stats.get_ppm_error(10000000.0);
        
        Serial.printf("Phase 1 complete. Average frequency: %.12f Hz (%.1f ppb error)\r\n", 
                      g_calibration.phase1_result, avg_ppm * 1000.0);
        
        // Set new PPM offset based on phase 1 results
        g_frequency_offset_ppm = -avg_ppm;  // Negative to correct the error
        oscillator.setFrequencyOffsetPPM(g_frequency_offset_ppm);
        
        Serial.printf("Applied correction: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
        Serial.printf("Phase 2: Averaging for another %lu seconds with correction applied\r\n", g_calibration.duration_seconds);
        
        // Reset for phase 2
        reset_measurement_stats();
        g_calibration.state = CAL_PHASE2_AVERAGING;
        g_calibration.start_time = millis();
        g_calibration.expected_pulses = g_freq_stats.get_count();  // Reset expected pulse count for phase 2
        g_calibration.missed_pulses = 0;  // Reset missed pulse count for phase 2
        last_progress_report = 0;
      } else {
        Serial.println("Calibration failed: No GPS PPS data received during phase 1\r");
        g_calibration.state = CAL_IDLE;
      }
    } else if (g_calibration.state == CAL_PHASE2_AVERAGING) {
      // Phase 2 complete - report final results
      if (g_freq_stats.has_samples()) {
        double phase2_avg = g_freq_stats.get_mean();
        double phase2_ppm = g_freq_stats.get_ppm_error(10000000.0);
        
        Serial.println("\r\n=== CALIBRATION COMPLETE ===\r");
        Serial.printf("Phase 1 average: %.12f Hz (%.1f ppb error)\r\n", 
                      g_calibration.phase1_result, 
                      ((g_calibration.phase1_result - 10000000.0) / 10000000.0) * 1e6 * 1000.0);
        Serial.printf("Applied correction: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
        Serial.printf("Phase 2 average: %.12f Hz (%.1f ppb residual error)\r\n", 
                      phase2_avg, phase2_ppm * 1000.0);
        Serial.printf("Final oscillator offset: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
        
        // Save the new offset to EEPROM
        save_settings();
        Serial.println("Calibration offset saved to EEPROM\r");
      } else {
        Serial.println("Calibration failed: No GPS PPS data received during phase 2\r");
        // Restore original offset
        g_frequency_offset_ppm = g_calibration.original_offset;
        oscillator.setFrequencyOffsetPPM(g_frequency_offset_ppm);
        Serial.printf("Restored original offset: %.1f ppb\r\n", g_frequency_offset_ppm * 1000.0);
      }
      
      g_calibration.state = CAL_IDLE;
      Serial.println("Calibration procedure finished.\r\n");
    }
  }
}

void check_auto_calibration() {
  // Only check if not already calibrating and oscillator is present
  if (g_calibration.state != CAL_IDLE || !oscillator.isPresent()) {
    return;
  }
  
  // Check if we have no calibration offset (indicating no previous calibration)
  // and we have received more than 10 GPS PPS pulses
  if (g_frequency_offset_ppm == 0.0 && g_freq_stats.get_count() > 10) {
    Serial.println("\r\nAuto-calibration trigger: No calibration offset detected after 10+ GPS pulses.\r");
    start_calibration_internal(true);
  }
}

void process_nmea_messages(void) {
    while (Serial1.available()) {
	parser.encode((char)Serial1.read());
    }
}

void loop() {
  handle_serial_commands();

  // Always poll GPT2 for both capture and compare events
  gpt2_poll_capture();
  
  // Always process GPS PPS measurements (if available)
  process_frequency_measurement();
  
  // Check for auto-calibration start condition
  check_auto_calibration();
  
  process_calibration();
  process_nmea_messages();

  DisplayStatus status;
  bool pps_recent = (g_last_pps_millis != 0) && ((millis() - g_last_pps_millis) <= 10000);
  status.pps_locked = pps_recent;
  status.sample_count = g_freq_stats.get_count();
  status.ppm_error = g_pps_data.ppm_instantaneous;
  status.ppm_average = g_pps_data.ppm_average;
  status.utc_valid = g_gps_data.is_valid;
  status.uptime_seconds = millis() / 1000;  // Convert milliseconds to seconds
  if (status.utc_valid) {
    int year, month, day, hour, minute, second;
    if (sscanf(g_gps_data.timestamp, "%d-%d-%dT%d:%d:%dZ",
               &year, &month, &day, &hour, &minute, &second) == 6) {
      status.utc.year = (uint16_t)year;
      status.utc.month = (uint8_t)month;
      status.utc.day = (uint8_t)day;
      status.utc.hour = (uint8_t)hour;
      status.utc.minute = (uint8_t)minute;
      status.utc.second = (uint8_t)second;
    } else {
      status.utc_valid = false;
    }
  }
  if (g_sd_available)
      MTP.loop();
  status.output_high = gpt2_is_output_high();
  
  // Calibration status for display
  status.calibrating = (g_calibration.state != CAL_IDLE);
  status.cal_offset_ppm = g_frequency_offset_ppm;
  
  if (status.calibrating) {
    uint32_t elapsed_ms = millis() - g_calibration.start_time;
    uint32_t elapsed_seconds = elapsed_ms / 1000;
    status.cal_remaining_seconds = (elapsed_seconds < g_calibration.duration_seconds) ? 
                                   (g_calibration.duration_seconds - elapsed_seconds) : 0;
    status.cal_phase = (g_calibration.state == CAL_PHASE1_AVERAGING) ? 1 : 2;
    
    // Calculate current average PPM
    if (g_freq_stats.has_samples()) {
      status.cal_current_ppm = g_freq_stats.get_ppm_error(10000000.0);
    } else {
      status.cal_current_ppm = 0.0;
    }
  } else {
    status.cal_remaining_seconds = 0;
    status.cal_phase = 0;
    status.cal_current_ppm = 0.0;
  }
  
  display_update(status);
}
