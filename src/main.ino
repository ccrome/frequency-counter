#include <Arduino.h>
#include "Gpt2FreqMeter.h"


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

void setup() {
  Serial.begin(115200);
  int startup_delay = 2000;
  while (startup_delay > 0) {
    Serial.printf("Waiting for serial...%d\r\n", startup_delay/1000);
    delay(1000);
    startup_delay -= 1000;
  }
  increase_peripheral_clock();
  
  // Begin with rising edge capture by default
  gpt2_begin_extclk_cap1(false, GPT_EDGE_RISING);
  check_gpt2_counter();
}

void loop() {
  // Periodic external clock status is disabled (uncomment to enable)
  static uint32_t last_status_ms = 0;
  uint32_t now_ms = millis();
  if (0) {
    if (now_ms - last_status_ms > 1000) {
      last_status_ms = now_ms;
      uint32_t a = GPT2_CNT;
      delay(10);
      uint32_t b = GPT2_CNT;
      uint32_t diff = b - a; // handles wrap
      uint32_t freq = diff * 100; // 10ms window -> Hz
      if (diff == 0) {
        Serial.println("extclk not counting\r");
      } else {
        Serial.printf("extclk ~ %lu Hz\r\n", (unsigned long)freq);
      }
    }
  }

  // Running tally averaging with reset on key press
  static double sum_hz = 0.0;
  static uint32_t sample_count = 0;
  static uint32_t start_ms = millis();

  if (Serial.available() > 0) {
    while (Serial.available() > 0) (void)Serial.read();
    sum_hz = 0.0;
    sample_count = 0;
    start_ms = millis();
  }

  gpt2_poll_capture();
  if (gpt2_available()) {
    uint32_t ticks = gpt2_read();                // ticks between PPS edges
    double freq_hz = (double)ticks;              // PPS is exactly 1 s
    const double ref_hz = 10000000.0;           // 10 MHz reference

    if (freq_hz < ref_hz * 0.95 || freq_hz > ref_hz * 1.05) {
      Serial.printf("WARNING: freq_hz = %.6f Hz is outside 95-105%% of ref_hz = %.6f Hz\r\n", freq_hz, ref_hz);
    } else {

      sum_hz += freq_hz;
      sample_count++;

      double avg_hz = (sample_count == 0) ? 0.0 : (sum_hz / (double)sample_count);
      uint32_t elapsed_sec = sample_count;         // count whole seconds (each PPS)

      double ppm_inst = ((freq_hz - ref_hz) / ref_hz) * 1e6;
      double ppm_avg  = ((avg_hz  - ref_hz) / ref_hz) * 1e6;

      double mhz_latest = freq_hz / 1e6;
      double mhz_avg    = avg_hz  / 1e6;

      // Display: averaging time (whole seconds), latest MHz, avg MHz, ppm errors
      Serial.printf("t=%lus latest=%.6f MHz avg=%.12f MHz ppm(lat)=%.3f ppm(avg)=%.6f\r\n",
                    (unsigned long)elapsed_sec, mhz_latest, mhz_avg, ppm_inst, ppm_avg);
      }
  }
}
