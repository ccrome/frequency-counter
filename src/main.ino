#include <Arduino.h>
#include "Gpt2FreqMeter.h"
#include "pins.h"
#include "SiT5501.h"


SiT5501 oscillator(0x60);
uint32_t next_time = 0;
uint32_t interval = 5000;

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
  Serial.println("  d       - Disable oscillator output\r");
}

void print_other_commands() {
  Serial.println("Other:\r");
  Serial.println("  h       - Show this help\r");
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
  gpt2_begin_dual_mode(1, GPT_EDGE_RISING, false);  // 1 PPS output, rising edge capture, internal clock
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
  setup_pins();
  Serial.begin(115200);
  wait_for_serial();
  
  print_startup_info();
  initialize_gpt2();
  initialize_oscillator();
  
  print_help();
  Serial.println("System initialized in Dual Mode. 1 PPS output active, GPS PPS monitoring enabled.\r\n");
  next_time = millis() + interval;
  int32_t max_ppm = 0x1ffffff;
  oscillator.setFrequencyControl(max_ppm);
}


// Global variables for frequency measurement
static double g_sum_hz = 0.0;
static uint32_t g_sample_count = 0;

void reset_measurement_stats() {
  g_sum_hz = 0.0;
  g_sample_count = 0;
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
    uint32_t fc_value;
    if (oscillator.getFrequencyControl(fc_value)) {
      double ppm = SiT5501::frequencyControlToPPM(fc_value);
      Serial.printf("SiT5501: %.6f PPM (FC: 0x%08lX)\r\n", ppm, fc_value);
    }
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
  if (!oscillator.isPresent()) {
    Serial.println("Error: SiT5501 oscillator not found\r");
    return;
  }
  
  uint32_t fc_value;
  if (oscillator.getFrequencyControl(fc_value)) {
    double ppm = SiT5501::frequencyControlToPPM(fc_value);
    Serial.printf("SiT5501 current setting: %.6f PPM (FC: 0x%08lX)\r\n", ppm, fc_value);
  } else {
    Serial.println("Error reading oscillator registers\r");
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

void handle_serial_commands() {
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();
  
  if (command.length() == 0) return;
  
  char cmd = command.charAt(0);
  
  switch (cmd) {
    case 's': cmd_show_status(); break;
    case 'r': cmd_reset_measurements(); break;
    case 'c': cmd_set_capture_edge(); break;
    case 'f': cmd_set_output_frequency(command); break;
    case 'p': cmd_set_oscillator_ppm(command); break;
    case 'o': cmd_read_oscillator(); break;
    case 'e': cmd_oscillator_output_enable(true); break;
    case 'd': cmd_oscillator_output_enable(false); break;
    case 'h': print_help(); break;
    default:
      Serial.println("Unknown command. Type 'h' for help.\r");
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
  
  display_frequency_results(freq_hz, ref_hz);
}

void display_frequency_results(double freq_hz, double ref_hz) {
  double avg_hz = (g_sample_count == 0) ? 0.0 : (g_sum_hz / (double)g_sample_count);
  uint32_t elapsed_sec = g_sample_count;
  
  double ppm_inst = ((freq_hz - ref_hz) / ref_hz) * 1e6;
  double ppm_avg = ((avg_hz - ref_hz) / ref_hz) * 1e6;
  
  double mhz_latest = freq_hz / 1e6;
  double mhz_avg = avg_hz / 1e6;
  
  Serial.printf("t=%lus latest=%.6f MHz avg=%.12f MHz ppm(lat)=%.3f ppm(avg)=%.6f\r\n",
                (unsigned long)elapsed_sec, mhz_latest, mhz_avg, ppm_inst, ppm_avg);
}

void print_oscillator_status() {
  if (oscillator.isPresent()) {
    for (int i = 0; i <= 0x02; i++) {
      uint16_t value;
      oscillator.readRegister(i, value);
      Serial.printf("SiT5501: Register %d: 0x%02lX\r\n", i, value);
    }
  } else {
    Serial.println("SiT5501: Not present\r");
  }
}
bool offset = false;
void loop() {
  if (millis() >= next_time) {

    static int32_t ppm;
    if (offset) {
      ppm = 0x1ffffff;
    } else {
      ppm = 0x2000000;
    }
    offset = !offset;
    Serial.printf("Setting frequency offset to 0x%08lX PPM, %d\r\n", ppm, offset);
    oscillator.setFrequencyOffsetPPM(ppm);

    next_time += interval;
    print_oscillator_status();
  }
  handle_serial_commands();

  
  // Always process GPS PPS measurements (if available)
  process_frequency_measurement();
  
  delay(10);
}
