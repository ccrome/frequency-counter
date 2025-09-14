#include <Arduino.h>
#include "imxrt.h"
#include "Gpt2FreqMeter.h"

static volatile uint32_t last_cap = 0, prev_cap = 0;
static volatile bool have = false;

void gpt2_begin_extclk_cap1(bool use_external_clock, GptCaptureEdge edge) {
  if (use_external_clock) {
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
    // For internal clock: set PERCLK = 50 MHz (IPG/3) so PR=4 gives exactly 10 MHz
    uint32_t cscmr1 = CCM_CSCMR1;
    cscmr1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;  // use IPG root
    cscmr1 &= ~(0x3F << 0);
    cscmr1 |= CCM_CSCMR1_PERCLK_PODF(2);   // PERCLK = IPG/3 = 50 MHz (IPG=150 MHz)
    CCM_CSCMR1 = cscmr1;
  }

  // Pin 15 ALT8 for CAP1
  uint32_t clk_mux14 = IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02;
  uint32_t clk_pad14 = IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02;
  uint32_t cap_mux15 = (clk_mux14 & ~0x0F) | 8;
  uint32_t cap_pad15 = clk_pad14;
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = cap_mux15;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = cap_pad15;

  // CAP1 daisy to pin 15
  IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT =
    (IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT & ~0x3u) | 1u;

  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);

  delay(1);
  GPT2_CR = 0;
  GPT2_CR = GPT_CR_SWR;
  uint32_t timeout = 10000;
  while ((GPT2_CR & GPT_CR_SWR) && timeout--) {
    delayMicroseconds(1);
  }

  if (use_external_clock) {
    GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR | GPT_CR_ENMOD;
    GPT2_PR = 0;
  } else {
    GPT2_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR | GPT_CR_ENMOD;
    GPT2_PR = 4;  // PERCLK 50 MHz / (PR+1=5) = 10 MHz tick
  }

  // IM1 bits (17:16) from edge param; IM2 (19:18) disabled
  GPT2_CR &= ~(((uint32_t)3 << 16) | ((uint32_t)3 << 18));
  GPT2_CR |=  ((uint32_t)(edge & 0x3) << 16);

  GPT2_SR = 0x3F;  // clear flags
  GPT2_IR = 0;     // no IRQs; polling

  GPT2_CR |= GPT_CR_EN;  // start
}

bool gpt2_available() { return have; }

uint32_t gpt2_read() {
  have = false;
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
    have = (prev_cap != 0);
  }
}
