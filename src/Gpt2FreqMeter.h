#pragma once
#include <Arduino.h>
#include "imxrt.h"

class Gpt2FreqMeter {
public:
  // Call once to set up GPT2 with:
  //   clk_pad_mux = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02, clk_alt = 8  (GPT2_CLK on pin 14)
  //   cap_pad_mux = &IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03, cap_alt = 8  (GPT2_CAPTURE1 on pin 15)
  //   Optional: pass DAISY select registers if your core exposes them; otherwise nullptr.
  static void begin(volatile uint32_t* clk_pad_mux, uint8_t clk_alt,
                    volatile uint32_t* cap_pad_mux, uint8_t cap_alt,
                    volatile uint32_t* cap1_daisy = nullptr, uint32_t cap1_daisy_val = 0,
                    volatile uint32_t* clkin_daisy = nullptr, uint32_t clkin_daisy_val = 0);

  static bool available();   // true when a new 1-second tick count is ready
  static uint32_t read();    // ticks between last two PPS edges

private:
  static void isr();
  static volatile uint32_t last_cap_;
  static volatile uint32_t prev_cap_;
  static volatile bool have_;
};

void gpt2_begin_extclk_cap1(bool use_external_clock = true);
bool gpt2_available();
uint32_t gpt2_read();
