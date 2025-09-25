/**
 * @file sit5501_example.ino
 * @brief Example usage of SiT5501 MEMS oscillator driver
 * 
 * This example demonstrates how to:
 * - Initialize the SiT5501 device
 * - Set frequency offsets in PPM
 * - Read current frequency control settings
 * - Enable/disable output
 */

#include <Arduino.h>
#include <Wire.h>
#include "../src/SiT5501.h"

// Create SiT5501 instance with default I2C address (0x68)
SiT5501 oscillator;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("SiT5501 MEMS Oscillator Example");
    Serial.println("================================");
    
    // Initialize I2C and check if device is present
    if (!oscillator.begin()) {
        Serial.println("ERROR: SiT5501 not found on I2C bus!");
        Serial.println("Check connections and I2C address.");
        while (1) delay(1000);
    }
    
    Serial.println("SiT5501 found and initialized successfully!");
    
    // Read current frequency control setting
    uint32_t current_fc;
    if (oscillator.getFrequencyControl(current_fc)) {
        double current_ppm = SiT5501::frequencyControlToPPM(current_fc);
        Serial.printf("Current frequency control: 0x%08X (%.3f PPM)\n", 
                     current_fc, current_ppm);
    }
    
    // Enable output
    Serial.println("Enabling output...");
    oscillator.setOutputEnable(true);
    
    Serial.println("\nReady for frequency control commands:");
    Serial.println("Commands:");
    Serial.println("  +<ppm>  - Set positive frequency offset (e.g., +10.5)");
    Serial.println("  -<ppm>  - Set negative frequency offset (e.g., -5.2)");
    Serial.println("  0       - Set to center frequency (0 PPM)");
    Serial.println("  r       - Read current setting");
    Serial.println("  e       - Enable output");
    Serial.println("  d       - Disable output");
    Serial.println();
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.length() == 0) return;
        
        char first_char = command.charAt(0);
        
        if (first_char == '+' || first_char == '-' || (first_char >= '0' && first_char <= '9')) {
            // Parse frequency offset
            double ppm_offset = command.toDouble();
            
            Serial.printf("Setting frequency offset to %.3f PPM...", ppm_offset);
            
            if (oscillator.setFrequencyOffsetPPM(ppm_offset)) {
                Serial.println(" SUCCESS");
                
                // Verify the setting
                uint32_t fc_value;
                if (oscillator.getFrequencyControl(fc_value)) {
                    double actual_ppm = SiT5501::frequencyControlToPPM(fc_value);
                    Serial.printf("Actual setting: %.6f PPM (FC: 0x%08X)\n", 
                                 actual_ppm, fc_value);
                }
            } else {
                Serial.println(" FAILED");
                Serial.println("Check PPM range (±3200 PPM max) or I2C connection.");
            }
            
        } else if (first_char == 'r' || first_char == 'R') {
            // Read current setting
            uint32_t fc_value;
            if (oscillator.getFrequencyControl(fc_value)) {
                double ppm_offset = SiT5501::frequencyControlToPPM(fc_value);
                Serial.printf("Current: %.6f PPM (FC: 0x%08X)\n", ppm_offset, fc_value);
            } else {
                Serial.println("Failed to read frequency control register");
            }
            
        } else if (first_char == 'e' || first_char == 'E') {
            // Enable output
            Serial.print("Enabling output...");
            if (oscillator.setOutputEnable(true)) {
                Serial.println(" SUCCESS");
            } else {
                Serial.println(" FAILED");
            }
            
        } else if (first_char == 'd' || first_char == 'D') {
            // Disable output
            Serial.print("Disabling output...");
            if (oscillator.setOutputEnable(false)) {
                Serial.println(" SUCCESS");
            } else {
                Serial.println(" FAILED");
            }
            
        } else {
            Serial.println("Unknown command. Use +/-<ppm>, r, e, or d");
        }
        
        Serial.println();
    }
    
    delay(10);
}
