#include <Arduino.h>
#include "imxrt.h"
#include "Gpt2FreqMeter.h"

// Input capture variables (for GPS PPS when available)
static volatile uint32_t last_cap = 0, prev_cap = 0;
static volatile bool capture_available = false;

// System state
static volatile bool gpt2_running = false;
static volatile uint32_t current_output_freq = 1;

void gpt2_begin_dual_mode(uint32_t output_freq_hz, GptCaptureEdge capture_edge, bool use_external_clock) {
  current_output_freq = output_freq_hz;
  
  // Configure clock source
  if (use_external_clock) {
    // Use external clock input on pin 14
    uint32_t cscmr1 = CCM_CSCMR1;
    cscmr1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;
    cscmr1 &= ~(0x3F << 0);
    cscmr1 |= CCM_CSCMR1_PERCLK_PODF(1);   // PERCLK = IPG/2 = 75 MHz
    CCM_CSCMR1 = cscmr1;

    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = (1 << 4) | 8;  // SION=1, ALT8
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x1030;
    IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT =
      (IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT & ~0x3u) | 1u;
  } else {
    // Use internal clock: 10 MHz timebase
    uint32_t cscmr1 = CCM_CSCMR1;
    cscmr1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;  // use IPG root
    cscmr1 &= ~(0x3F << 0);
    cscmr1 |= CCM_CSCMR1_PERCLK_PODF(2);   // PERCLK = IPG/3 = 50 MHz
    CCM_CSCMR1 = cscmr1;
  }

  // Configure pin 15 for input capture (GPS PPS)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 8;  // ALT8 for GPT2_CAPTURE1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = 0x1030;
  IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT =
    (IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT & ~0x3u) | 1u;

  // Configure pin 41 for output compare (1 PPS output) - GPIO_AD_B1_05
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 8;  // ALT8 for GPT2_COMPARE1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05 = 0x1030;

  // Enable GPT2 clock
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

  // Reset GPT2
  delay(1);
  GPT2_CR = 0;
  GPT2_CR = GPT_CR_SWR;
  uint32_t timeout = 10000;
  while ((GPT2_CR & GPT_CR_SWR) && timeout--) {
    delayMicroseconds(1);
  }

  // Configure timer
  if (use_external_clock) {
    GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR | GPT_CR_ENMOD;
    GPT2_PR = 0;
  } else {
    GPT2_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR | GPT_CR_ENMOD;
    GPT2_PR = 4;  // PERCLK 50 MHz / (PR+1=5) = 10 MHz tick
  }

  // Configure input capture edge detection
  GPT2_CR &= ~(((uint32_t)3 << 16) | ((uint32_t)3 << 18));
  GPT2_CR |= ((uint32_t)(capture_edge & 0x3) << 16);

  // Configure output compare for toggle mode
  GPT2_CR &= ~(0x3 << 4);  // Clear OM1 bits
  GPT2_CR |= (0x1 << 4);   // Set OM1 = 01 (toggle)

  // Set output frequency
  gpt2_set_output_frequency(output_freq_hz);

  // Clear status flags
  GPT2_SR = 0x3F;
  GPT2_IR = 0;  // No IRQs, use polling

  // Start the timer
  GPT2_CR |= GPT_CR_EN;
  gpt2_running = true;
}

void gpt2_set_output_frequency(uint32_t frequency_hz) {
  if (frequency_hz == 0) frequency_hz = 1;
  current_output_freq = frequency_hz;

  // Calculate compare period for 10 MHz timebase
  // For toggle mode, we need half the period since each compare match toggles
  uint32_t compare_period = 10000000 / (2 * frequency_hz);
  GPT2_OCR1 = compare_period;
}

void gpt2_set_capture_edge(GptCaptureEdge edge) {
  // Update input capture edge detection
  GPT2_CR &= ~(((uint32_t)3 << 16));
  GPT2_CR |= ((uint32_t)(edge & 0x3) << 16);
}

bool gpt2_capture_available() { 
  return capture_available; 
}

uint32_t gpt2_read_capture() {
  capture_available = false;
  uint32_t period_ticks = last_cap - prev_cap;
  return period_ticks;
}

void gpt2_poll_capture() {
  uint32_t sr = GPT2_SR;
  if (sr & GPT_SR_IF1) {
    uint32_t cap = GPT2_ICR1;
    GPT2_SR = GPT_SR_IF1;  // clear IF1
    prev_cap = last_cap;
    last_cap = cap;
    capture_available = (prev_cap != 0);
  }
}

void gpt2_stop() {
  GPT2_CR = 0;  // Stop and disable GPT2
  gpt2_running = false;
}

bool gpt2_is_running() {
  return gpt2_running;
}

uint32_t gpt2_get_output_frequency() {
  return current_output_freq;
}
