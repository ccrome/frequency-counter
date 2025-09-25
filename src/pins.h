

#define CLKGEN_OE 21
#define CLK_10MHZ_OE 22
#define PPS_OE 23

// GPT2 pins
#define GPT2_EXTCLK_PIN 14    // External clock input (GPIO_AD_B1_02)
#define GPT2_CAPTURE_PIN 15   // Input capture pin (GPIO_AD_B1_03)
#define GPT2_COMPARE_PIN 41   // Output compare pin (GPIO_AD_B1_05)

extern void setup_pins(void);
