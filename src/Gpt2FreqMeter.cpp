#include <Arduino.h>
#include "imxrt.h"

static volatile uint32_t last_cap = 0, prev_cap = 0;
static volatile bool have = false;

static void gpt2_isr() {
  uint32_t sr = GPT2_SR;
  if (sr & GPT_SR_IF1) {
    uint32_t cap = GPT2_ICR1;     // captured timer value (microseconds) at signal edge
    prev_cap = last_cap;
    last_cap = cap;
    have = (prev_cap != 0);       // have valid measurement after 2nd capture
    GPT2_SR = GPT_SR_IF1;         // clear IF1
  } else {
    GPT2_SR = sr;                 // clear others
  }
}

void gpt2_begin_extclk_cap1(bool use_external_clock) {
  // Pin 14 -> GPT2_CLK  (GPIO_AD_B1_02 ALT8) - external clock input
  // Pin 15 -> GPT2_CAP1 (GPIO_AD_B1_03 ALT8) - captures count on PPS edge
  
  if (use_external_clock) {
    // STEP 1: Increase PERCLK to 75MHz to support 10MHz external clock
    uint32_t cscmr1 = CCM_CSCMR1;
    cscmr1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;  // Use IPG_CLK_ROOT
    cscmr1 &= ~(0x3F << 0);  // Clear PERCLK_PODF bits
    cscmr1 |= CCM_CSCMR1_PERCLK_PODF(1);   // Divide by 2 -> 75MHz
    CCM_CSCMR1 = cscmr1;
    
    // STEP 2: Configure Pin 14 for GPT2_CLK (ALT8) with SION enabled
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = (1 << 4) | 8;  // SION=1, ALT8 = GPT2_CLK
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x1030;  // INPUT_ENABLE + pull-up
    
    // STEP 3: Set DAISY selector to exact address 0x401F876C per RM (GPT2_IPP_IND_CLKIN_SELECT_INPUT)
    volatile uint32_t* gpt2_clkin_select = (volatile uint32_t*)0x401F876C;
    *gpt2_clkin_select = 1;  // 1 = GPIO_AD_B1_02 ALT8
  }
  
  // Configure pin 15 (GPIO_AD_B1_03) for GPT2_CAPTURE1 (ALT8) with SION enabled
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = (1 << 4) | 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_03 = 0x1030;  // INPUT_ENABLE + pull-up + medium drive

  // GPT2_CAPTURE1_SELECT_INPUT register exact address 0x401F8764 per RM
  volatile uint32_t* gpt2_cap1_select = (volatile uint32_t*)0x401F8764;
  *gpt2_cap1_select = 1;  // 1 = GPIO_AD_B1_03 ALT8
  
  // Enable all GPT2 related clocks - try all possible combinations
  // Primary GPT2 clocks
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);
  
  // Additional GPT clocks that may be needed
  #ifdef CCM_CCGR1_GPT
  CCM_CCGR1 |= CCM_CCGR1_GPT(CCM_CCGR_ON);
  #endif
  #ifdef CCM_CCGR1_GPT2_BUS
  CCM_CCGR1 |= CCM_CCGR1_GPT2_BUS(CCM_CCGR_ON);
  #endif
  #ifdef CCM_CCGR1_GPT2_SERIAL
  CCM_CCGR1 |= CCM_CCGR1_GPT2_SERIAL(CCM_CCGR_ON);
  #endif
  #ifdef CCM_CCGR0_GPT2
  CCM_CCGR0 |= CCM_CCGR0_GPT2(CCM_CCGR_ON);
  #endif
  
  // Ensure peripheral clock is enabled (needed for internal clock mode)
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);  // PIT shares peripheral clock domain
  
  // Add small delay to ensure clocks are stable
  delay(1);
  // Reset and configure GPT2
  GPT2_CR = 0;
  GPT2_CR = GPT_CR_SWR;
  
  // Add timeout to prevent infinite hang
  uint32_t timeout = 10000; // 10000 attempts
  while ((GPT2_CR & GPT_CR_SWR) && timeout--) {
    delayMicroseconds(1);
  }
  // Reset completed or timed out
  if (use_external_clock) {
    GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR | GPT_CR_ENMOD;  // External clock (Pin 14), free-run
    GPT2_PR = 0;  // No prescale - count every external clock edge
  } else {
    GPT2_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR | GPT_CR_ENMOD;  // Internal clock, free-run
    GPT2_PR = 23;  // Prescale by 24 to get 1MHz timer for input capture
  }
  GPT2_SR = 0x3F;                                          // clear flags
  GPT2_IR = 0;
  GPT2_IR = GPT_IR_IF1IE;                                  // CAP1 interrupt

  attachInterruptVector(IRQ_GPT2, gpt2_isr);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  GPT2_CR |= GPT_CR_EN;                                    // start counting
}

bool gpt2_available() { return have; }

uint32_t gpt2_read() { 
  have = false; 
  uint32_t period_us = last_cap - prev_cap;  // Period in microseconds
  return period_us; 
}

// Get frequency from period measurement
uint32_t gpt2_read_frequency() {
  if (!have) return 0;
  have = false;
  uint32_t period_us = last_cap - prev_cap;
  if (period_us == 0) return 0;
  return 1000000UL / period_us;  // Convert period to frequency in Hz
}
