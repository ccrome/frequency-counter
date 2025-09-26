#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Gpt2FreqMeter.h"
#include "pins.h"
#include "SiT5501.h"
#include <ArduinoNmeaParser.h>
#include "FileManager.h"
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

// GPS data structure
struct GpsData {
  String timestamp;
  String source;
  double latitude;
  double longitude;
  double speed;
  double course;
  double magnetic_variation;
  bool is_valid;
};

// Global variables for frequency measurement
static double g_sum_hz = 0.0;
static uint32_t g_sample_count = 0;

// Data structures
static PpsData g_pps_data = {0};
static GpsData g_gps_data = {0};


// SD card logging
static bool g_sd_available = false;
static bool g_pause_updates = false;
static bool g_verbose_timing = false;  // Start with verbose timing off
static String g_current_log_file = "";
static File g_log_file;


// Persistent frequency offset (stored in EEPROM)
static double g_frequency_offset_ppm = 0.0;  // Default 0.0 ppm (originally 15.26 ppb = 0.01526 ppm)

// EEPROM data structure
struct EepromData {
  uint32_t magic;              // Magic number for validity check
  uint16_t version;            // Data structure version
  double frequency_offset_ppm; // Frequency offset in PPM
  uint16_t checksum;           // Simple checksum for data integrity
};

static const uint32_t EEPROM_MAGIC = 0x12345678;
static const uint16_t EEPROM_VERSION = 1;
static const int EEPROM_DATA_ADDR = 0;


const char* rmc_source_map[] = {
    "Unknown", "GPS", "Galileo", "GLONASS", "GNSS", "BDS"
};

String float_to_json_string(double value, int precision) {
  if (isnan(value)) {
    return "null";
  }
  return String(value, precision);
}

void log_json_field(File& file, const char* name, const String& value, bool is_first = false) {
  if (!is_first) file.print(",");
  file.printf("\"%s\":%s", name, value.c_str());
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
    log_json_field(file, name, float_to_json_string(value, precision));
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
	int rmc_source_i = static_cast<int>(nmea::RmcSource::GPS);
	const char *rmc_source_s = rmc_source_map[rmc_source_i];

	// Store GPS data in struct
	g_gps_data.timestamp = String(rmc.date.year) + "-" + 
	                      String(rmc.date.month < 10 ? "0" : "") + String(rmc.date.month) + "-" + 
	                      String(rmc.date.day < 10 ? "0" : "") + String(rmc.date.day) + "T" +
	                      String(rmc.time_utc.hour < 10 ? "0" : "") + String(rmc.time_utc.hour) + ":" +
	                      String(rmc.time_utc.minute < 10 ? "0" : "") + String(rmc.time_utc.minute) + ":" +
	                      String(rmc.time_utc.second < 10 ? "0" : "") + String(rmc.time_utc.second) + "Z";
	g_gps_data.source = String(rmc_source_s);
	g_gps_data.latitude = rmc.latitude;
	g_gps_data.longitude = rmc.longitude;
	g_gps_data.speed = rmc.speed;
	g_gps_data.course = rmc.course;
	g_gps_data.magnetic_variation = rmc.magnetic_variation;
	g_gps_data.is_valid = true;

	// Create log file on first GPS message using GPS timestamp
	if (!g_log_file && g_sd_available) {
	    // Create filename from GPS timestamp (replace : with - for filesystem compatibility)
	    String filename = g_gps_data.timestamp;
	    filename.replace(":", "-");
	    filename += ".jsonl";
	    
	    g_current_log_file = filename;
	    g_log_file = SD.open(filename.c_str(), FILE_WRITE);
	    
	    if (g_log_file) {
	        Serial.printf("Created GPS-timestamped log file: %s\r\n", filename.c_str());
	    } else {
	        Serial.printf("Failed to create log file: %s\r\n", filename.c_str());
	    }
	}

	// Create combined log entry with both frequency and GPS data
	if (g_pps_data.has_data && g_log_file) {
	    // Calculate derived values
	    double freq_hz = (double)g_pps_data.ticks;  // ticks = Hz for 1 second PPS
	    
	    // Start JSON object
	    g_log_file.print("{");
	    
	    // GPS timestamp and source (always present)
	    log_json_field(g_log_file, "gps_timestamp", g_gps_data.timestamp.c_str(), true);
	    log_json_field(g_log_file, "gps_source", g_gps_data.source.c_str());
	    
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
}

void print_output_commands() {
  Serial.println("Output Generation (Always Active):\r");
  Serial.println("  f       - Re-arm GPT-driven 1 PPS output\r");
  Serial.println("  g0/g1   - Force PPS output low/high via GPIO\r");
}

void print_oscillator_commands() {
  Serial.println("SiT5501 Oscillator Control:\r");
  Serial.println("  p<ppm>  - Set frequency offset in PPM (e.g., p+5.2 or p-3.1)\r");
  Serial.println("  p0      - Reset to center frequency (0 PPM)\r");
  Serial.println("  o<ppm>  - Set persistent frequency offset in PPM (e.g., o0.01526 or o-5.0)\r");
  Serial.println("  o       - Show current persistent frequency offset\r");
  Serial.println("  e       - Enable oscillator output\r");
  Serial.println("  z       - Disable oscillator output\r");
}

void print_other_commands() {
  Serial.println("Other:\r");
  Serial.println("  h       - Show this help\r");
  Serial.println("  v       - Toggle verbose timing output (currently OFF)\r");
  Serial.println("  b       - Reboot to bootloader mode\r");
  Serial.println("  l       - List all log files on SD card\r");
  Serial.println("  d<ID>   - Download log file to console by ID (e.g., d0)\r");
  Serial.println("  x<ID>   - Download log file via XMODEM protocol by ID (e.g., x0)\r");
  Serial.println("  k       - Delete all log files except current one\r");
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
  Serial.println("=== Precision Frequency Counter (Dual Mode) ===\r");
  Serial.printf("Pin Configuration:\r\n");
  Serial.printf("  Pin %d: External clock input (optional)\r\n", GPT2_EXTCLK_PIN);
  Serial.printf("  Pin %d: GPS PPS input (always monitoring)\r\n", GPT2_CAPTURE_PIN);
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
void setup() {
  Serial1.begin(9600);
  setup_pins();
  Serial.begin(115200);
  wait_for_serial();

  print_startup_info();
  initialize_sd_card();
  initialize_gpt2();
  initialize_oscillator();
  load_frequency_offset();  // Load persistent frequency offset

  print_help();
  Serial.println("System initialized in Dual Mode. 1 PPS output active, GPS PPS monitoring enabled.\r\n");
  
  // Apply the loaded frequency offset
  oscillator.setFrequencyOffsetPPM(g_frequency_offset_ppm);
  Serial.printf("Applied frequency offset: %.6f ppm\r\n", g_frequency_offset_ppm);

  // XMODEM implementation is self-contained - no initialization needed
}


void reset_measurement_stats() {
  g_sum_hz = 0.0;
  g_sample_count = 0;
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

void load_frequency_offset() {
  EepromData data;
  EEPROM.get(EEPROM_DATA_ADDR, data);
  
  // Validate magic number and version
  if (data.magic != EEPROM_MAGIC || data.version != EEPROM_VERSION) {
    Serial.println("EEPROM data invalid or outdated, using defaults\r");
    g_frequency_offset_ppm = 0.0;  // Default to 0 ppm
    save_frequency_offset();
    return;
  }
  
  // Validate checksum
  uint16_t calculated_checksum = calculate_checksum(data);
  if (data.checksum != calculated_checksum) {
    Serial.println("EEPROM checksum invalid, using defaults\r");
    g_frequency_offset_ppm = 0.0;  // Default to 0 ppm
    save_frequency_offset();
    return;
  }
  
  // Validate frequency offset range
  if (isnan(data.frequency_offset_ppm) || data.frequency_offset_ppm < -1000.0 || data.frequency_offset_ppm > 1000.0) {
    Serial.println("EEPROM frequency offset out of range, using defaults\r");
    g_frequency_offset_ppm = 0.0;  // Default to 0 ppm
    save_frequency_offset();
    return;
  }
  
  g_frequency_offset_ppm = data.frequency_offset_ppm;
  Serial.printf("Loaded frequency offset: %.6f ppm\r\n", g_frequency_offset_ppm);
}

void save_frequency_offset() {
  EepromData data;
  data.magic = EEPROM_MAGIC;
  data.version = EEPROM_VERSION;
  data.frequency_offset_ppm = g_frequency_offset_ppm;
  data.checksum = calculate_checksum(data);
  
  EEPROM.put(EEPROM_DATA_ADDR, data);
  Serial.printf("Saved frequency offset: %.6f ppm\r\n", g_frequency_offset_ppm);
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

void cmd_set_output_frequency(const String& command) {
  (void)command;
  if (pps_release_to_gpt()) {
    Serial.println("Resumed GPT2 control of PPS output.\r");
  }
  gpt2_begin_dual_mode(1, GPT_EDGE_RISING, true);
  Serial.println("Output frequency set to 1 Hz\r");
  Serial.printf("Signal available on pin %d\r\n", GPT2_COMPARE_PIN);
}

void cmd_set_frequency_offset(const String& command) {
  String param = command.substring(1);
  param.trim();
  
  if (param.length() == 0) {
    // Show current offset
    Serial.printf("Current frequency offset: %.6f ppm\r\n", g_frequency_offset_ppm);
    return;
  }
  
  double offset = param.toFloat();
  if (offset >= -1000.0 && offset <= 1000.0) {
    g_frequency_offset_ppm = offset;
    save_frequency_offset();  // Save to EEPROM
    
    // Apply the new offset to oscillator
    oscillator.setFrequencyOffsetPPM(g_frequency_offset_ppm);
    
    Serial.printf("Frequency offset set to: %.6f ppm\r\n", g_frequency_offset_ppm);
  } else {
    Serial.println("Error: Frequency offset must be between -1000.0 and 1000.0 ppm\r");
  }
}

void show_gpt2_status() {
  Serial.println("GPT2 Mode: Dual Mode (always active)\r");
  Serial.printf("GPS PPS samples collected: %lu\r\n", g_sample_count);
  if (g_sample_count > 0) {
    double avg_hz = g_sum_hz / g_sample_count;
    Serial.printf("Average measured frequency: %.6f MHz\r\n", avg_hz / 1e6);
  } else {
    Serial.println("No GPS PPS signals received yet\r");
  }
  Serial.printf("Output frequency: 1 Hz on pin %d\r\n", GPT2_COMPARE_PIN);
  Serial.printf("GPS PPS input: pin %d\r\n", GPT2_CAPTURE_PIN);
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

void cmd_show_status() {
  Serial.println("\r\n=== System Status ===\r");
  show_gpt2_status();
  show_oscillator_status();
}

void cmd_set_oscillator_ppm(const String& command) {
  if (!oscillator.isPresent()) {
    Serial.println("Error: SiT5501 oscillator not found\r");
    return;
  }

  double ppm = command.substring(1).toFloat();
  if (oscillator.setFrequencyOffsetPPM(ppm)) {
    Serial.printf("Oscillator frequency offset set to %.3f PPM\r\n", ppm);
      } else {
    Serial.println("Error: PPM out of range (±3200 PPM max) or I2C error\r");
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

void cmd_dump_measurements() {
  Serial.println("\r\n=== SD Card File Dump ===\r");
  Serial.println("Pausing periodic updates...\r");
  g_pause_updates = true;

  if (!g_sd_available) {
    Serial.println("SD card not available.\r");
    g_pause_updates = false;
    return;
  }

  if (g_current_log_file.length() == 0) {
    Serial.println("No log file created yet.\r");
    g_pause_updates = false;
    return;
  }

  // Close current log file to ensure all data is written
  if (g_log_file) {
    g_log_file.flush();
    g_log_file.close();
  }

  // Open file for reading
  File read_file = SD.open(g_current_log_file.c_str(), FILE_READ);
  if (!read_file) {
    Serial.printf("Failed to open log file: %s\r\n", g_current_log_file.c_str());
    g_pause_updates = false;

    // Reopen for writing
    g_log_file = SD.open(g_current_log_file.c_str(), FILE_WRITE);
    return;
  }

  Serial.printf("Dumping contents of: %s\r\n", g_current_log_file.c_str());
  Serial.printf("File size: %lu bytes\r\n", read_file.size());
  Serial.println("----------------------------------------\r");

  // Dump file contents to console
  uint32_t line_count = 0;
  while (read_file.available()) {
    String line = read_file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.printf("%s\r\n", line.c_str());
      line_count++;
    }
  }

  read_file.close();

  Serial.println("----------------------------------------\r");
  Serial.printf("Total lines: %lu\r\n", line_count);
  Serial.println("=== End of File Dump ===\r");
  Serial.println("Resuming periodic updates automatically...\r");

  // Automatically resume updates
  g_pause_updates = false;

  // Reopen log file for writing
  g_log_file = SD.open(g_current_log_file.c_str(), FILE_WRITE);
}


void cmd_list_log_files() {
  Serial.println("\r\n=== SD Card Log Files ===\r");
  
  if (!g_sd_available) {
    Serial.println("SD card not available.\r");
    return;
  }
  
  list_log_files(g_current_log_file);
}

void cmd_delete_old_log_files() {
  Serial.println("\r\n=== Delete Old Log Files ===\r");
  
  if (!g_sd_available) {
    Serial.println("SD card not available.\r");
    return;
  }
  
  if (g_current_log_file.length() == 0) {
    Serial.println("No current log file - nothing to preserve.\r");
    return;
  }
  
  Serial.printf("This will delete ALL log files except current: %s\r\n", g_current_log_file.c_str());
  Serial.println("WARNING: This action cannot be undone!\r");
  Serial.println("Type 'DELETE' to confirm or any other key to cancel:");
  
  // Wait for first confirmation
  String confirmation1 = "";
  unsigned long timeout = millis() + 15000; // 15 second timeout
  
  while (millis() < timeout && confirmation1.length() == 0) {
    if (Serial.available()) {
      confirmation1 = Serial.readStringUntil('\n');
      confirmation1.trim();
      break;
    }
    delay(100);
  }
  
  if (confirmation1 != "DELETE") {
    Serial.println("Delete operation cancelled.\r");
    return;
  }
  
  // Second confirmation
  Serial.println("Are you absolutely sure? Type 'YES' to proceed:");
  
  String confirmation2 = "";
  timeout = millis() + 10000; // 10 second timeout
  
  while (millis() < timeout && confirmation2.length() == 0) {
    if (Serial.available()) {
      confirmation2 = Serial.readStringUntil('\n');
      confirmation2.trim();
      break;
    }
    delay(100);
  }
  
  if (confirmation2 != "YES") {
    Serial.println("Delete operation cancelled.\r");
    return;
  }
  
  Serial.println("Proceeding with deletion...\r");
  
  // Close current log file temporarily to avoid issues
  bool was_open = false;
  if (g_log_file) {
    g_log_file.flush();
    g_log_file.close();
    was_open = true;
  }
  
  // Use FileManager to handle the deletion
  uint32_t deleted_count = delete_old_log_files(g_current_log_file);
  
  Serial.printf("Operation complete: %lu files deleted\r\n", deleted_count);
  
  // Reopen current log file if it was open
  if (was_open && g_current_log_file.length() > 0) {
    g_log_file = SD.open(g_current_log_file.c_str(), FILE_WRITE);
    if (g_log_file) {
      Serial.printf("Reopened current log file: %s\r\n", g_current_log_file.c_str());
    } else {
      Serial.printf("Warning: Failed to reopen current log file: %s\r\n", g_current_log_file.c_str());
    }
  }
}

void cmd_download_log_file(String command) {
  Serial.println("\r\n=== Download Log File ===\r");
  Serial.println("Pausing periodic updates...\r");
  g_pause_updates = true;
  
  if (!g_sd_available) {
    Serial.println("SD card not available.\r");
    g_pause_updates = false;
    return;
  }
  
  // Parse file ID from command (e.g., "d0", "d1", etc.)
  if (command.length() < 2) {
    Serial.println("Usage: d<ID> (e.g., d0 for first file)\r");
    Serial.println("Use 'l' to list available files.\r");
    g_pause_updates = false;
    return;
  }
  
  uint32_t file_id = command.substring(1).toInt();
  
  // Get filename by ID using FileManager
  String target_filename = get_filename_by_id(file_id);
  
  if (target_filename.length() == 0) {
    uint32_t file_count = get_file_count();
    Serial.printf("File ID %lu not found. Available range: 0-%lu\r\n", file_id, file_count - 1);
    g_pause_updates = false;
    return;
  }
  
  // Close current log file if it's the same as target
  if (target_filename.equals(g_current_log_file) && g_log_file) {
    g_log_file.flush();
    g_log_file.close();
  }
  
  // Open target file for reading
  File download_file = SD.open(target_filename.c_str(), FILE_READ);
  if (!download_file) {
    Serial.printf("Failed to open file: %s\r\n", target_filename.c_str());
    g_pause_updates = false;
    
    // Reopen current log file if needed
    if (target_filename.equals(g_current_log_file)) {
      g_log_file = SD.open(g_current_log_file.c_str(), FILE_WRITE);
    }
    return;
  }
  
  Serial.printf("Downloading: %s\r\n", target_filename.c_str());
  Serial.printf("File size: %lu bytes\r\n", download_file.size());
  Serial.println("----------------------------------------\r");
  
  // Stream file contents
  uint32_t line_count = 0;
  while (download_file.available()) {
    String line = download_file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      Serial.printf("%s\r\n", line.c_str());
      line_count++;
    }
  }
  
  download_file.close();
  
  Serial.println("----------------------------------------\r");
  Serial.printf("Download complete: %lu lines\r\n", line_count);
  Serial.println("Resuming periodic updates automatically...\r");
  
  // Automatically resume updates
  g_pause_updates = false;
  
  // Reopen current log file if it was the target
  if (target_filename.equals(g_current_log_file)) {
    g_log_file = SD.open(g_current_log_file.c_str(), FILE_WRITE);
  }
}

// Simple XMODEM sender implementation
bool xmodem_send_file(File& file, Stream& serial) {
  const uint8_t SOH = 0x01;    // Start of Header
  const uint8_t EOT = 0x04;    // End of Transmission
  const uint8_t ACK = 0x06;    // Acknowledge
  const uint8_t NAK = 0x15;    // Negative Acknowledge
  const uint8_t CAN = 0x18;    // Cancel
  const uint8_t SUB = 0x1A;    // Substitute (padding)
  
  uint8_t block_num = 1;
  uint8_t buffer[128];
  
  // Wait for receiver to send NAK (start request)
  unsigned long timeout = millis() + 60000; // 60 second timeout
  while (millis() < timeout) {
    if (serial.available()) {
      uint8_t c = serial.read();
      if (c == NAK) break;
      if (c == CAN) return false;
    }
    delay(100);
  }
  
  if (millis() >= timeout) return false;
  
  // Send file blocks
  while (file.available()) {
    // Read block data
    size_t bytes_read = file.read(buffer, 128);
    
    // Pad with SUB if needed
    if (bytes_read < 128) {
      memset(buffer + bytes_read, SUB, 128 - bytes_read);
    }
    
    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 0; i < 128; i++) {
      checksum += buffer[i];
    }
    
    // Send block with retries
    bool block_acked = false;
    for (int retry = 0; retry < 10 && !block_acked; retry++) {
      // Send block header
      serial.write(SOH);
      serial.write(block_num);
      serial.write(255 - block_num);
      
      // Send data
      serial.write(buffer, 128);
      
      // Send checksum
      serial.write(checksum);
      serial.flush();
      
      // Wait for ACK/NAK
      timeout = millis() + 1000;
      while (millis() < timeout) {
        if (serial.available()) {
          uint8_t response = serial.read();
          if (response == ACK) {
            block_acked = true;
            break;
          } else if (response == NAK) {
            break; // Retry
          } else if (response == CAN) {
            return false;
          }
        }
        delay(1); 
      }
    }
    
    if (!block_acked) return false;
    block_num++;
  }
  
  // Send EOT
  for (int retry = 0; retry < 10; retry++) {
    serial.write(EOT);
    serial.flush();
    
    timeout = millis() + 3000;
    while (millis() < timeout) {
      if (serial.available()) {
        uint8_t response = serial.read();
        if (response == ACK) return true;
        if (response == CAN) return false;
        break; // Retry on NAK or other
      }
      delay(1);  // Reduced from 10ms to 1ms
    }
  }
  
  return false;
}

void cmd_xmodem_transfer(String command) {
  Serial.println("\r\n=== XMODEM File Transfer ===\r");
  Serial.println("Pausing periodic updates...\r");
  g_pause_updates = true;
  
  if (!g_sd_available) {
    Serial.println("SD card not available.\r");
    g_pause_updates = false;
    return;
  }
  
  // Parse file ID from command (e.g., "x0", "x1", etc.)
  if (command.length() < 2) {
    Serial.println("Usage: x<ID> (e.g., x0 for first file)\r");
    Serial.println("Use 'l' to list available files.\r");
    g_pause_updates = false;
    return;
  }
  
  uint32_t file_id = command.substring(1).toInt();
  
  // Get filename by ID using FileManager
  String target_filename = get_filename_by_id(file_id);
  
  if (target_filename.length() == 0) {
    uint32_t file_count = get_file_count();
    Serial.printf("File ID %lu not found. Available range: 0-%lu\r\n", file_id, file_count - 1);
    g_pause_updates = false;
    return;
  }
  
  // Close current log file if it's the same as target
  if (target_filename.equals(g_current_log_file) && g_log_file) {
    g_log_file.flush();
    g_log_file.close();
  }
  
  // Open target file for reading
  File transfer_file = SD.open(target_filename.c_str(), FILE_READ);
  if (!transfer_file) {
    Serial.printf("Failed to open file: %s\r\n", target_filename.c_str());
    g_pause_updates = false;
    
    // Reopen current log file if needed
    if (target_filename.equals(g_current_log_file)) {
      g_log_file = SD.open(g_current_log_file.c_str(), FILE_WRITE);
    }
    return;
  }
  
  Serial.printf("Starting XMODEM transfer: %s\r\n", target_filename.c_str());
  Serial.printf("File size: %lu bytes\r\n", transfer_file.size());
  Serial.println("Waiting for receiver to initiate transfer...\r");
  Serial.println("(Use XMODEM receive in your terminal program)\r");
  
  // Use our simple XMODEM implementation to send file over USB Serial
  bool success = xmodem_send_file(transfer_file, Serial);
  
  transfer_file.close();
  
  if (success) {
    Serial.println("\r\nXMODEM transfer completed successfully!\r");
  } else {
    Serial.println("\r\nXMODEM transfer failed or was cancelled.\r");
  }
  
  Serial.println("Resuming periodic updates automatically...\r");
  
  // Automatically resume updates
  g_pause_updates = false;
  
  // Reopen current log file if it was the target
  if (target_filename.equals(g_current_log_file)) {
    g_log_file = SD.open(g_current_log_file.c_str(), FILE_WRITE);
  }
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

void handle_serial_commands() {
  static String command_buffer = "";
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
      if (c == 'f' || c == 'p' || c == 'd' || c == 'x' || c == 'o' || c == 'g') {
          command_buffer = String(c);
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
        command_buffer = "";
        reading_parameter_command = false;
      } else if (c == 8 || c == 127) {  // Backspace or DEL
        if (command_buffer.length() > 0) {
          command_buffer.remove(command_buffer.length() - 1);
          Serial.print("\b \b");  // Backspace, space, backspace to erase character
        }
      } else if (c >= 32 && c <= 126) {  // Printable characters only
        command_buffer += c;
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
    case 'l': cmd_list_log_files(); break;
    case 'k': cmd_delete_old_log_files(); break;
    case 'v': 
      g_verbose_timing = !g_verbose_timing;
      Serial.printf("Verbose timing: %s\r\n", g_verbose_timing ? "ON" : "OFF");
      break;
    default:
      Serial.println("Unknown command. Type 'h' for help.\r");
      break;
  }
  Serial.println("\r");
}

void process_parameter_command(String command) {
  command.trim();
  command.toLowerCase();

  if (command.length() == 0) return;

  char cmd = command.charAt(0);

  switch (cmd) {
    case 'f':
      if (command.length() > 1) {
        Serial.println("Usage: f\r");
        break;
      }
      cmd_set_output_frequency(command);
      break;
    case 'p': cmd_set_oscillator_ppm(command); break;
    case 'd': cmd_download_log_file(command); break;
    case 'x': cmd_xmodem_transfer(command); break;
    case 'o': cmd_set_frequency_offset(command); break;
    case 'g':
      if (command.length() == 2) {
        char arg = command.charAt(1);
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
  gpt2_poll_capture();
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

  g_sum_hz += freq_hz;
  g_sample_count++;

  // Store frequency measurement data in PPS struct
  g_pps_data.ticks = ticks;  // ticks = freq_hz for 1 second PPS
  g_pps_data.avg_freq_hz = (g_sample_count == 0) ? 0.0 : (g_sum_hz / (double)g_sample_count);
  g_pps_data.ppm_instantaneous = ((freq_hz - ref_hz) / ref_hz) * 1e6;
  g_pps_data.ppm_average = ((g_pps_data.avg_freq_hz - ref_hz) / ref_hz) * 1e6;
  g_pps_data.has_data = true;
  

  // Display results if verbose timing is enabled
  if (!g_pause_updates && g_verbose_timing) {
    double freq_mhz = g_pps_data.ticks / 1e6;  // Calculate MHz from ticks
    double avg_freq_mhz = g_pps_data.avg_freq_hz / 1e6;  // Calculate avg MHz
    Serial.printf("t=%lus ticks=%6d latest=%.6f MHz avg=%.12f MHz ppm(lat)=%.3f ppm(avg)=%.6f\r\n",
                  (unsigned long)g_sample_count, g_pps_data.ticks, freq_mhz, 
                  avg_freq_mhz, g_pps_data.ppm_instantaneous, g_pps_data.ppm_average);
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

void process_nmea_messages(void) {
    while (Serial1.available()) {
	parser.encode((char)Serial1.read());
    }
}

void loop() {
  handle_serial_commands();

  // Always process GPS PPS measurements (if available)
  process_frequency_measurement();
  process_nmea_messages();
}
