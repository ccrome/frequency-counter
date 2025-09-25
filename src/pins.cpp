#include "pins.h"
#include <Arduino.h>

void setup_pins() {
    pinMode(CLKGEN_OE, OUTPUT);
    pinMode(PPS_OE, OUTPUT);
    pinMode(CLK_10MHZ_OE, OUTPUT);
    digitalWrite(CLKGEN_OE, 1);
    digitalWrite(PPS_OE, 1);
    digitalWrite(CLK_10MHZ_OE, 1);
}