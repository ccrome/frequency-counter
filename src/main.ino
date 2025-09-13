#include <Arduino.h>
#include "Gpt2FreqMeter.h"
// prototypes from Gpt2FreqMeter.cpp

//

void check_gpt2_counter() {
  // Check if GPT2 counter is actually counting
  Serial.println("Checking if GPT2 counter is running...\r");
  uint32_t count1 = GPT2_CNT;
  delay(10);  // Wait 10ms
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

//

//

//

//

//

//

//

void increase_peripheral_clock() {
  Serial.println("=== INCREASING PERIPHERAL CLOCK FOR 10MHz EXTERNAL CLOCK ===\r");
  
  // Current PERCLK is 24MHz (too slow for 10MHz external clock)
  // Need PERCLK > 40MHz so that 10MHz < PERCLK/4
  // Let's set it to 60MHz to be safe
  
  Serial.printf("Current CCM_CSCMR1 = 0x%08lX\r\n", (unsigned long)CCM_CSCMR1);
  
  // PERCLK comes from IPG_CLK_ROOT (150MHz) divided by PERCLK_PODF
  // Current: IPG_CLK_ROOT / 6.25 = 24MHz, so PERCLK_PODF = 5 (divide by 6)
  // Want: IPG_CLK_ROOT / 2.5 = 60MHz, so PERCLK_PODF = 1 (divide by 2)
  
  // Set PERCLK_CLK_SEL = 0 (use IPG_CLK_ROOT) and PERCLK_PODF = 1 (divide by 2)
  uint32_t cscmr1 = CCM_CSCMR1;
  cscmr1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;  // Use IPG_CLK_ROOT (not OSC)
  cscmr1 &= ~(0x3F << 0);  // Clear PERCLK_PODF bits (bits 5:0)
  cscmr1 |= CCM_CSCMR1_PERCLK_PODF(1);     // Divide by 2 (150MHz/2 = 75MHz)
  CCM_CSCMR1 = cscmr1;
  
  Serial.printf("New CCM_CSCMR1 = 0x%08lX\r\n", (unsigned long)CCM_CSCMR1);
  Serial.println("PERCLK increased to ~75MHz (10MHz < 75MHz/4 = 18.75MHz ✅)\r");
  
  delay(10);  // Let clocks settle
}

void test_external_clock_with_fast_perclk() {
  Serial.println("=== TESTING EXTERNAL CLOCK WITH FAST PERCLK ===\r");
  
  // Configure Pin 14 for GPT2_CLK with correct settings
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = (1 << 4) | 8;  // SION=1, ALT8 = GPT2_CLK
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x1030;  // INPUT_ENABLE + pull-up + hysteresis
  
  // Set DAISY selector
  volatile uint32_t* gpt2_clkin_select = (volatile uint32_t*)0x401F876C;
  *gpt2_clkin_select = 1;
  
  // Enable GPT2 clocks
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);
  
  // Configure GPT2 for external clock
  GPT2_CR = 0;
  GPT2_CR = GPT_CR_SWR;
  while (GPT2_CR & GPT_CR_SWR) {}
  
  GPT2_PR = 0;  // No prescaling
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_ENMOD | GPT_CR_FRR;  // External clock
  GPT2_CR |= GPT_CR_EN;
  
  Serial.println("Testing external clock with increased PERCLK...\r");
  delay(100);
  
  uint32_t count1 = GPT2_CNT;
  delay(100);  // 100ms test
  uint32_t count2 = GPT2_CNT;
  
  Serial.printf("External clock test: %lu -> %lu (diff: %lu)\r\n", 
                (unsigned long)count1, (unsigned long)count2, (unsigned long)(count2 - count1));
  
  if (count2 > count1) {
    uint32_t freq = (count2 - count1) * 10;  // Extrapolate to Hz (100ms sample)
    Serial.printf("*** SUCCESS! External clock working at %lu Hz ***\r\n", (unsigned long)freq);
  } else {
    Serial.println("External clock still not working\r");
    
    // Test internal clock speed to verify PERCLK change
    Serial.println("Testing internal clock speed after PERCLK change...\r");
    GPT2_CR = 0;
    GPT2_CR = GPT_CR_SWR;
    while (GPT2_CR & GPT_CR_SWR) {}
    GPT2_PR = 23;  // Divide by 24 like before
    GPT2_CR = GPT_CR_CLKSRC(1) | GPT_CR_ENMOD | GPT_CR_FRR;  // Internal clock
    GPT2_CR |= GPT_CR_EN;
    
    delay(10);
    uint32_t int_count1 = GPT2_CNT;
    delay(10);
    uint32_t int_count2 = GPT2_CNT;
    
    uint32_t internal_freq = (int_count2 - int_count1) * 100;  // 10ms sample -> Hz
    Serial.printf("Internal clock after PERCLK change: %lu Hz\r\n", (unsigned long)internal_freq);
    Serial.printf("This means PERCLK = %lu MHz\r\n", (unsigned long)(internal_freq * 24 / 1000000));
  }
}

void try_all_daisy_addresses() {
  Serial.println("=== TRYING ALL POSSIBLE DAISY ADDRESSES ===\r");
  
  // Configure Pin 14 correctly
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;  // ALT8 = GPT2_CLK
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x1030;  // INPUT_ENABLE + pull-up
  
  // Enable clocks
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON) | CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);
  
  // Try a much wider range of DAISY addresses
  for (uint32_t addr = 0x401F8570; addr <= 0x401F85A0; addr += 4) {
    Serial.printf("Trying DAISY address 0x%08lX\r\n", (unsigned long)addr);
    
    volatile uint32_t* daisy_reg = (volatile uint32_t*)addr;
    
    for (int val = 0; val <= 7; val++) {  // Try more values
      *daisy_reg = val;
      
      // Configure GPT2 for external clock
      GPT2_CR = 0;
      GPT2_CR = GPT_CR_SWR;
      while (GPT2_CR & GPT_CR_SWR) {}
      GPT2_PR = 0;
      GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_ENMOD | GPT_CR_FRR;
      GPT2_CR |= GPT_CR_EN;
      
      delay(30);
      
      uint32_t count1 = GPT2_CNT;
      delay(30);
      uint32_t count2 = GPT2_CNT;
      
      if (count2 > count1) {
        Serial.printf("*** SUCCESS! DAISY 0x%08lX = %d WORKS! ***\r\n", (unsigned long)addr, val);
        Serial.printf("Count: %lu -> %lu (diff: %lu)\r\n", 
                      (unsigned long)count1, (unsigned long)count2, (unsigned long)(count2 - count1));
        return;
      }
    }
  }
  
  Serial.println("No DAISY address/value combination worked\r");
  
  // Last resort: Try if Pin 14 routes to GPT2_CLK through a different path
  Serial.println("Trying alternative GPT2_CLK routing approaches...\r");
  
  // Maybe the DAISY needs to select a different GPIO path
  // Try all possible GPIO pins that might route to GPT2_CLK
  volatile uint32_t* gpt2_clkin_select = (volatile uint32_t*)0x401F8574;
  
  // Test if other GPIO pins work with GPT2_CLK and see their DAISY values
  Serial.println("Testing known working GPT2_CLK pins to understand DAISY mapping...\r");
  
  // The fact that ALT1 worked earlier (even if it was TMR3) suggests the routing exists
  // Let's try ALT1 with GPT2 configuration to see if there's cross-connection
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 1;  // ALT1 (was TMR3_TIMER2)
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x1030;
  
  for (int daisy = 0; daisy <= 7; daisy++) {
    *gpt2_clkin_select = daisy;
    
    GPT2_CR = 0;
    GPT2_CR = GPT_CR_SWR;
    while (GPT2_CR & GPT_CR_SWR) {}
    GPT2_PR = 0;
    GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_ENMOD | GPT_CR_FRR;
    GPT2_CR |= GPT_CR_EN;
    
    delay(30);
    uint32_t count1 = GPT2_CNT;
    delay(30);
    uint32_t count2 = GPT2_CNT;
    
    if (count2 > count1) {
      Serial.printf("*** ALT1 + DAISY %d works with GPT2! ***\r\n", daisy);
      Serial.printf("Count: %lu -> %lu\r\n", (unsigned long)count1, (unsigned long)count2);
      return;
    }
  }
  
  Serial.println("ALT1 + GPT2 combinations also failed\r");
  
  // Since datasheet confirms ALT8 is correct, try extended DAISY values
  Serial.println("Trying extended DAISY values for GPIO_AD_B1_02 ALT8...\r");
  
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;  // Correct ALT8
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x1030;  // INPUT_ENABLE
  
  // Maybe DAISY value 1 selects GPIO_AD_B0_09, and we need a different value for GPIO_AD_B1_02
  for (int daisy = 0; daisy <= 15; daisy++) {  // Try more values
    Serial.printf("Extended DAISY test: %d\r\n", daisy);
    *gpt2_clkin_select = daisy;
    
    GPT2_CR = 0;
    GPT2_CR = GPT_CR_SWR;
    while (GPT2_CR & GPT_CR_SWR) {}
    GPT2_PR = 0;
    GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_ENMOD | GPT_CR_FRR;
    GPT2_CR |= GPT_CR_EN;
    
    delay(50);  // Longer settle time
    uint32_t count1 = GPT2_CNT;
    delay(50);  // Longer test time
    uint32_t count2 = GPT2_CNT;
    
    if (count2 > count1) {
      Serial.printf("*** SUCCESS! ALT8 + DAISY %d works! ***\r\n", daisy);
      Serial.printf("Count: %lu -> %lu (diff: %lu)\r\n", 
                    (unsigned long)count1, (unsigned long)count2, (unsigned long)(count2 - count1));
      return;
    }
  }
  
  Serial.println("Extended DAISY test failed - Pin 14 may not connect to GPT2_CLK\r");
  
  // Final test: Try completely different DAISY register addresses for GPT2_CLK
  Serial.println("Trying completely different DAISY register addresses...\r");
  
  // Maybe GPT2_CLK_SELECT_INPUT is at a different address entirely
  uint32_t alt_daisy_addresses[] = {
    0x401F8500, 0x401F8504, 0x401F8508, 0x401F850C,
    0x401F8510, 0x401F8514, 0x401F8518, 0x401F851C,
    0x401F8520, 0x401F8524, 0x401F8528, 0x401F852C,
    0x401F8530, 0x401F8534, 0x401F8538, 0x401F853C
  };
  
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;  // ALT8
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x1030;  // INPUT_ENABLE
  
  for (int addr_idx = 0; addr_idx < 16; addr_idx++) {
    volatile uint32_t* daisy_reg = (volatile uint32_t*)alt_daisy_addresses[addr_idx];
    Serial.printf("Trying DAISY address 0x%08lX\r\n", (unsigned long)alt_daisy_addresses[addr_idx]);
    
    for (int val = 0; val <= 3; val++) {
      *daisy_reg = val;
      
      GPT2_CR = 0;
      GPT2_CR = GPT_CR_SWR;
      while (GPT2_CR & GPT_CR_SWR) {}
      GPT2_PR = 0;
      GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_ENMOD | GPT_CR_FRR;
      GPT2_CR |= GPT_CR_EN;
      
      delay(30);
      uint32_t count1 = GPT2_CNT;
      delay(30);
      uint32_t count2 = GPT2_CNT;
      
      if (count2 > count1) {
        Serial.printf("*** FOUND IT! DAISY 0x%08lX = %d works! ***\r\n", 
                      (unsigned long)alt_daisy_addresses[addr_idx], val);
        return;
      }
    }
  }
  
  Serial.println("All DAISY register attempts failed - GPT2_CLK may not be functional on this chip\r");
  
  // Try the exact code from Teensy forum post that claims to work
  Serial.println("Trying exact forum post configuration...\r");
  
  // External clock pin 14 - from forum post
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8; // Set ALT8 for GPT2_CLK
  
  // Try to access the register mentioned in forum post
  // IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1; // This doesn't exist in our headers
  
  // But maybe it's at a different address - let's try some possibilities
  volatile uint32_t* possible_regs[] = {
    (volatile uint32_t*)0x401F8574,  // Our current guess
    (volatile uint32_t*)0x401F8578,  // Alternative
    (volatile uint32_t*)0x401F857C,  // Alternative  
    (volatile uint32_t*)0x401F8580,  // Alternative
  };
  
  const char* reg_names[] = {
    "0x401F8574 (current)",
    "0x401F8578", 
    "0x401F857C",
    "0x401F8580"
  };
  
  for (int i = 0; i < 4; i++) {
    Serial.printf("Trying register %s = 1\r\n", reg_names[i]);
    *possible_regs[i] = 1;
    
    // Reset and configure GPT2
    GPT2_CR = 0;
    GPT2_CR = GPT_CR_SWR;
    while (GPT2_CR & GPT_CR_SWR) {}
    GPT2_PR = 0;
    GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_ENMOD | GPT_CR_FRR;
    GPT2_CR |= GPT_CR_EN;
    
    delay(50);
    uint32_t count1 = GPT2_CNT;
    delay(50);  
    uint32_t count2 = GPT2_CNT;
    
    Serial.printf("Count: %lu -> %lu (diff: %lu)\r\n", 
                  (unsigned long)count1, (unsigned long)count2, (unsigned long)(count2 - count1));
    
    if (count2 > count1) {
      Serial.printf("*** SUCCESS! Register %s works! ***\r\n", reg_names[i]);
      return;
    }
  }
  
  Serial.println("Forum post configuration also failed\r");
}

void try_different_gpt2_pins() {
  Serial.println("Trying different pins for GPT2 external clock...\r");
  
  // Try pin 6 (GPIO_B0_10) for GPT2_CLK (ALT1)
  Serial.println("Trying pin 6 for GPT2_CLK...\r");
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 1;  // Pin 6 -> GPT2_CLK (ALT1)
  
  delay(10);
  
  // Check if GPT2 counter is counting
  uint32_t count1 = GPT2_CNT;
  delay(10);
  uint32_t count2 = GPT2_CNT;
  
  if (count2 > count1) {
    Serial.printf("SUCCESS! Pin 6 as GPT2_CLK works! Counter: %lu -> %lu\r\n", 
                  (unsigned long)count1, (unsigned long)count2);
  } else {
    Serial.printf("Pin 6 as GPT2_CLK failed. Counter stuck at: %lu\r\n", (unsigned long)count1);
  }
}

void setup() {
  Serial.begin(115200);
  int startup_delay = 2000;
  while (startup_delay > 0) {
    Serial.printf("Waiting for serial...%d\r\n", startup_delay/1000);
    delay(1000);
    startup_delay -= 1000;
  }
  // FIRST: Increase peripheral clock to handle 10MHz external clock
  increase_peripheral_clock();
  
  gpt2_begin_extclk_cap1(true);
  // Verify external clock started counting
  check_gpt2_counter();

}

void loop() {
  if (gpt2_available()) {
    uint32_t ticks = gpt2_read();      // External clock cycles between PPS edges
    Serial.printf("ticks/s = %lu\r\n", (unsigned long)ticks);
  }
}
