#include "pins.h"
#include <Arduino.h>
#include "imxrt.h"

static bool s_pps_override_active = false;
static bool s_pps_gpio_state = false;

void setup_pins() {
    pinMode(CLKGEN_OE, OUTPUT);
    pinMode(PPS_OE, OUTPUT);
    pinMode(CLK_10MHZ_OE, OUTPUT);
    digitalWrite(CLKGEN_OE, 1);
    digitalWrite(PPS_OE, 1);
    digitalWrite(CLK_10MHZ_OE, 1);
}

void pps_force_gpio(bool high) {
    bool first_override = !s_pps_override_active;

    if (first_override) {
        s_pps_override_active = true;
        // Switch pad to GPIO function
        IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 5;  // ALT5 = GPIO1_IO21
        IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05 = 0x1030;
        pinMode(GPT2_COMPARE_PIN, OUTPUT);
    }

    digitalWrite(GPT2_COMPARE_PIN, high ? HIGH : LOW);
    s_pps_gpio_state = high;
}

bool pps_release_to_gpt() {
    if (!s_pps_override_active) {
        return false;
    }

    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_05 = 8;   // ALT8 -> GPT2_COMPARE1
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_05 = 0x1030;
    s_pps_override_active = false;
    return true;
}

bool pps_gpio_override_active() {
    return s_pps_override_active;
}

bool pps_gpio_state_high() {
    return s_pps_gpio_state;
}