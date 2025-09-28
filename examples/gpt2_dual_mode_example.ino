/**
 * @file gpt2_dual_mode_example.ino
 * @brief Example demonstrating GPT2 operation with always-on output
 * 
 * This example shows how to switch between:
 * - Input Capture Mode: For frequency measurement
 * - Output Compare Mode: For generating 1 PPS signals
 * 
 * Pin Usage:
 * - Pin 14: External clock input (for input capture mode)
 * - Pin 15: PPS input signal (for input capture mode) 
 * - Pin 16: 1 PPS output signal (for output compare mode)
 * 
 * Serial Commands:
 * - 'i' or 'I': Switch to Input Capture mode
 * - 'o' or 'O': Switch to Output Compare mode (1 PPS)
 * - 'f<freq>': Set output frequency in Hz (e.g., f10 for 10 Hz)
 * - 's': Show current status
 */

#include <Arduino.h>
#include "../src/Gpt2FreqMeter.h"
#include "../src/pins.h"

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("GPT2 Example\r");
    Serial.println("======================\r");
    Serial.println("\r");
    Serial.println("Pin Configuration:\r");
    Serial.printf("  Pin %d: External clock input (Input Capture mode)\n", GPT2_EXTCLK_PIN);
    Serial.printf("  Pin %d: PPS input signal (Input Capture mode)\n", GPT2_CAPTURE_PIN);
    Serial.printf("  Pin %d: 1 PPS output signal (Output Compare mode)\n", GPT2_COMPARE_PIN);
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  i - Switch to Input Capture mode");
    Serial.println("  o - Switch to Output Compare mode (1 PPS)");
    Serial.println("  f<freq> - Set output frequency (e.g., f10 for 10 Hz)");
    Serial.println("  s - Show current status");
    Serial.println();
    
    // Start in input capture mode by default
    gpt2_begin_extclk_cap1(false, GPT_EDGE_RISING);  // Internal clock, rising edge
    Serial.println("Started in Input Capture mode (internal 10 MHz clock)");
    Serial.println("Ready for commands...");
    Serial.println();
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
        
        if (command.length() == 0) return;
        
        char first_char = command.charAt(0);
        
        if (first_char == 'i') {
            // Switch to input capture mode
            Serial.println("Switching to Input Capture mode...");
            gpt2_begin_extclk_cap1(false, GPT_EDGE_RISING);  // Internal clock
            Serial.println("Input Capture mode active");
            Serial.println("Connect PPS signal to pin 15 for frequency measurement");
            
        } else if (first_char == 'o') {
            // Switch to output compare mode (1 PPS)
            Serial.println("Switching to Output Compare mode (1 PPS)...");
            gpt2_begin_output_compare(1, false);  // 1 Hz, internal clock
            Serial.println("Output Compare mode active");
            Serial.printf("1 PPS signal available on pin %d\n", GPT2_COMPARE_PIN);
            
        } else if (first_char == 'f') {
            // Set output frequency
            if (gpt2_get_mode() != GPT2_MODE_OUTPUT_COMPARE) {
                Serial.println("Error: Must be in Output Compare mode to set frequency");
                Serial.println("Use 'o' command first");
            } else {
                uint32_t freq = command.substring(1).toInt();
                if (freq > 0 && freq <= 1000000) {  // Limit to 1 MHz max
                    gpt2_set_compare_frequency(freq);
                    Serial.printf("Output frequency set to %lu Hz\n", freq);
                } else {
                    Serial.println("Error: Frequency must be between 1 and 1000000 Hz");
                }
            }
            
        } else if (first_char == 's') {
            // Show status
            Serial.println("=== GPT2 Status ===");
            if (gpt2_get_mode() == GPT2_MODE_INPUT_CAPTURE) {
                Serial.println("Mode: Input Capture");
                Serial.printf("Clock input pin: %d\n", GPT2_EXTCLK_PIN);
                Serial.printf("Capture input pin: %d\n", GPT2_CAPTURE_PIN);
                Serial.printf("GPT2 Counter: %lu\n", GPT2_CNT);
            } else {
                Serial.println("Mode: Output Compare");
                Serial.printf("Output pin: %d\n", GPT2_COMPARE_PIN);
                Serial.printf("GPT2 Counter: %lu\n", GPT2_CNT);
                Serial.printf("Compare Register: %lu\n", GPT2_OCR1);
            }
            Serial.printf("Control Register: 0x%08lX\n", GPT2_CR);
            Serial.printf("Status Register: 0x%08lX\n", GPT2_SR);
            
        } else {
            Serial.println("Unknown command. Use i, o, f<freq>, or s");
        }
        
        Serial.println();
    }
    
    // If in input capture mode, process measurements
    if (gpt2_get_mode() == GPT2_MODE_INPUT_CAPTURE) {
        gpt2_poll_capture();
        if (gpt2_available()) {
            uint32_t ticks = gpt2_read();
            double freq_hz = (double)ticks;  // Assuming 1 second PPS intervals
            
            Serial.printf("Measured frequency: %.6f Hz (%.6f MHz)\n", 
                         freq_hz, freq_hz / 1e6);
        }
    }
    
    delay(10);
}
