#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "Gpt2FreqMeter.h"
#include "pins.h"
#include "SiT5501.h"
#include <ArduinoNmeaParser.h>
void onRmcUpdate(nmea::RmcData const rmc);

SiT5501 oscillator(0x60);
ArduinoNmeaParser parser(onRmcUpdate, nullptr);
// Global variables for frequency measurement
static double g_sum_hz = 0.0;
static uint32_t g_sample_count = 0;

// SD card logging
static bool g_sd_available = false;
static bool g_pause_updates = false;
static bool g_verbose_timing = false;  // Start with verbose timing off
static String g_current_log_file = "";
static String g_nmea_log_file = "";
static File g_log_file;
static File g_nmea_file;


const char* rmc_source_map[] = {
    "Unknown", "GPS", "Galileo", "GLONASS", "GNSS", "BDS"
};

String float_to_json_string(double value, int precision) {
  if (isnan(value)) {
    return "null";
  }
  return String(value, precision);
}

void onRmcUpdate(nmea::RmcData const rmc)
{
    if (rmc.is_valid) {
	nmea::RmcSource rmc_source = rmc.source;
	if (rmc_source > nmea::RmcSource::BDS)
	    rmc_source = nmea::RmcSource::Unknown;
	int rmc_source_i = static_cast<int>(nmea::RmcSource::GPS);
	const char *rmc_source_s = rmc_source_map[rmc_source_i];

	
	g_nmea_file.printf("{"
		      "\"timestamp\":\"%4d-%02d-%02dT%2d:%2d:%2dZ\","
		      "\"source\":\"%s\","
		      "\"latlon\":[%s, %s],"
		      "\"speed\":%s,"
		      "\"course\":%s,"
		      "\"magnetic_variation\":%.4f}\n",
		      rmc.date.year, rmc.date.month, rmc.date.day,
		      rmc.time_utc.hour, rmc.time_utc.minute, rmc.time_utc.second,
		      rmc_source_s,
		      float_to_json_string(rmc.latitude, 6).c_str(),
		      float_to_json_string(rmc.longitude, 6).c_str(),
		      float_to_json_string(rmc.speed, 4).c_str(),
		      float_to_json_string(rmc.course, 2).c_str(),
		      float_to_json_string(rmc.magnetic_variation, 2).c_str()
	    );
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
  Serial.println("  f<freq> - Set output frequency in Hz (e.g., f10 for 10 Hz)\r");
  Serial.println("  f1      - Generate 1 PPS (default)\r");
}

void print_oscillator_commands() {
  Serial.println("SiT5501 Oscillator Control:\r");
  Serial.println("  p<ppm>  - Set frequency offset in PPM (e.g., p+5.2 or p-3.1)\r");
  Serial.println("  p0      - Reset to center frequency (0 PPM)\r");
  Serial.println("  o       - Read current oscillator settings\r");
  Serial.println("  e       - Enable oscillator output\r");
  Serial.println("  z       - Disable oscillator output\r");
}

void print_other_commands() {
  Serial.println("Other:\r");
  Serial.println("  h       - Show this help\r");
  Serial.println("  v       - Toggle verbose timing output (currently OFF)\r");
  Serial.println("  b       - Reboot to bootloader mode\r");
  Serial.println("  x       - Dump current SD card log file to console\r");
  Serial.println("  l       - List all log files on SD card\r");
  Serial.println("  d<ID>   - Download specific log file by ID (e.g., d0)\r");
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
  int startup_delay = 5000;
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

  print_help();
  Serial.println("System initialized in Dual Mode. 1 PPS output active, GPS PPS monitoring enabled.\r\n");
  oscillator.setFrequencyOffsetPPM(15.26/1000.0); // 15.26ppb
}


void reset_measurement_stats() {
  g_sum_hz = 0.0;
  g_sample_count = 0;
}

bool initialize_sd_card() {
  // Teensy 4.1 built-in SD card uses BUILTIN_SDCARD
  if (SD.begin(BUILTIN_SDCARD)) {
    g_sd_available = true;
    Serial.println("SD card initialized successfully\r");

    // Create new log file with timestamp
    char filename[32];
    uint32_t file_num = 0;

    // Find next available filename
    do {
      snprintf(filename, sizeof(filename), "freq_%04lu.jsonl", file_num++);
    } while (SD.exists(filename) && file_num < 10000);

    g_current_log_file = String(filename);
    g_nmea_log_file = g_current_log_file + ".nmea";
    g_log_file = SD.open(filename, FILE_WRITE);
    g_nmea_file = SD.open(g_nmea_log_file.c_str(), FILE_WRITE);
    Serial.printf("Just opened %s as %d\r\n", g_nmea_log_file.c_str(), g_nmea_file);
    if (g_log_file) {
      Serial.printf("Created log file: %s (JSON Lines format)\r\n", filename);
      return true;
    } else {
      Serial.printf("Failed to create log file: %s\r\n", filename);
      g_sd_available = false;
      return false;
    }
  } else {
    Serial.println("SD card initialization failed\r");
    g_sd_available = false;
    return false;
  }
}

void log_measurement_to_sd(uint32_t ticks, double freq_hz, double ppm_error, double ppm_avg) {
  if (!g_sd_available || !g_log_file) return;

  // Write measurement as JSON line
  g_log_file.printf("{\"timestamp_ms\":%lu,\"ticks\":%lu,\"freq_hz\":%.6f,\"freq_mhz\":%.6f,\"ppm_error\":%.6f,\"ppm_avg\":%.6f}\n",
                    millis(),
                    ticks,
                    freq_hz,
                    freq_hz / 1e6,
                    ppm_error,
                    ppm_avg);

  // Flush after each entry to ensure immediate write
  g_log_file.flush();
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
  uint32_t freq = command.substring(1).toInt();
  if (freq > 0 && freq <= 1000000) {
    gpt2_set_output_frequency(freq);
    Serial.printf("Output frequency set to %lu Hz\r\n", freq);
    Serial.printf("Signal available on pin %d\r\n", GPT2_COMPARE_PIN);
  } else {
    Serial.println("Error: Frequency must be between 1 and 1000000 Hz\r");
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
  Serial.printf("Output frequency: %lu Hz on pin %d\r\n", gpt2_get_output_frequency(), GPT2_COMPARE_PIN);
  Serial.printf("GPS PPS input: pin %d\r\n", GPT2_CAPTURE_PIN);
  Serial.printf("GPT2 Counter: %lu\r\n", GPT2_CNT);
  Serial.printf("GPT2 Control: 0x%08lX\r\n", GPT2_CR);
  Serial.printf("Compare Register: %lu\r\n", GPT2_OCR1);
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


bool is_valid_filename(String filename) {
    return (filename.endsWith(".jsonl") ||
	    filename.endsWith(".JSONL") ||
	    filename.endsWith(".nmea")  ||
	    filename.endsWith(".NMEA")
	);
}
void cmd_list_log_files() {
  Serial.println("\r\n=== SD Card Log Files ===\r");
  
  if (!g_sd_available) {
    Serial.println("SD card not available.\r");
    return;
  }
  
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory.\r");
    return;
  }
  
  Serial.println("Available log files:\r");
  Serial.println("ID  Filename          Size (bytes)  Modified\r");
  Serial.println("--  ----------------  ------------  --------\r");
  
  uint32_t file_count = 0;
  File entry = root.openNextFile();
  while (entry) {
    String filename = entry.name();
    
    // Only show .jsonl files (our log files)
    if (is_valid_filename(filename)) {
      Serial.printf("%2lu  %-16s  %12lu  ", 
                    file_count, 
                    filename.c_str(), 
                    entry.size());
      
      // Show if this is the current active log file
      if (filename.equals(g_current_log_file)) {
        Serial.printf("(current)\r\n");
      } else {
        Serial.printf("\r\n");
      }
      file_count++;
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  
  if (file_count == 0) {
    Serial.println("No log files found.\r");
  } else {
    Serial.printf("\r\nTotal: %lu log files\r\n", file_count);
    Serial.println("Use 'd<ID>' to download a specific file (e.g., 'd0' for first file)\r");
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
  
  // Find the file by ID
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory.\r");
    g_pause_updates = false;
    return;
  }
  
  String target_filename = "";
  uint32_t current_id = 0;
  File entry = root.openNextFile();
  while (entry) {
    String filename = entry.name();
    
    // Only consider .jsonl files
    if (is_valid_filename(filename)) {
      if (current_id == file_id) {
        target_filename = filename;
        entry.close();
        break;
      }
      current_id++;
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  
  if (target_filename.length() == 0) {
    Serial.printf("File ID %lu not found. Use 'l' to list available files.\r\n", file_id);
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

        // Check if this is a parameter command that needs more input
        if (c == 'f' || c == 'p' || c == 'd') {
          command_buffer = String(c);
          reading_parameter_command = true;
          continue;
        }

      // Process single-character commands immediately
      process_single_command(c);
      } else {
      // Reading parameter for f or p commands
      if (c == '\n' || c == '\r') {
        // Process complete parameter command
        process_parameter_command(command_buffer);
        command_buffer = "";
        reading_parameter_command = false;
      } else if (c >= 32 && c <= 126) {  // Printable characters only
        command_buffer += c;
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
    case 'x': cmd_dump_measurements(); break;
    case 'l': cmd_list_log_files(); break;
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
    case 'f': cmd_set_output_frequency(command); break;
    case 'p': cmd_set_oscillator_ppm(command); break;
    case 'd': cmd_download_log_file(command); break;
    default:
      Serial.println("Unknown parameter command.\r");
      break;
  }
  Serial.println("\r");
}

void process_frequency_measurement() {
  gpt2_poll_capture();
  if (!gpt2_capture_available()) return;

  uint32_t ticks = gpt2_read_capture();
  double freq_hz = (double)ticks;
  const double ref_hz = 10000000.0;  // 10 MHz reference

    if (freq_hz < ref_hz * 0.95 || freq_hz > ref_hz * 1.05) {
    Serial.printf("WARNING: GPS PPS freq = %.6f Hz is outside 95-105%% of ref_hz = %.6f Hz\r\n", 
                  freq_hz, ref_hz);
    return;
  }

  g_sum_hz += freq_hz;
  g_sample_count++;

  display_frequency_results(ticks, ref_hz);
}

void display_frequency_results(uint32_t ticks, double ref_hz) {
    double freq_hz = (double)ticks;

  double avg_hz = (g_sample_count == 0) ? 0.0 : (g_sum_hz / (double)g_sample_count);
  uint32_t elapsed_sec = g_sample_count;

      double ppm_inst = ((freq_hz - ref_hz) / ref_hz) * 1e6;
  double ppm_avg = ((avg_hz - ref_hz) / ref_hz) * 1e6;

      double mhz_latest = freq_hz / 1e6;
  double mhz_avg = avg_hz / 1e6;

  // Only display if not paused and verbose timing is enabled
  if (!g_pause_updates && g_verbose_timing) {
    Serial.printf("t=%lus ticks=%6d latest=%.6f MHz avg=%.12f MHz ppm(lat)=%.3f ppm(avg)=%.6f\r\n",
                  (unsigned long)elapsed_sec, ticks, mhz_latest, mhz_avg, ppm_inst, ppm_avg);
  }

  // Always log to SD card regardless of pause state
  log_measurement_to_sd(ticks, freq_hz, ppm_inst, ppm_avg);
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
    g_nmea_file.flush();
}

void loop() {
  handle_serial_commands();

  // Always process GPS PPS measurements (if available)
  process_frequency_measurement();
  process_nmea_messages();
}
