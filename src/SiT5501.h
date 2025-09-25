#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
 * @brief SiT5501 MEMS Precision Oscillator Driver
 * 
 * Driver for the SiTime SiT5501 Stratum 3E MEMS precision oscillator
 * with ±10 ppb stability and digital frequency control (DCTCXO variant).
 * 
 * Features:
 * - I2C interface for frequency control
 * - Digital frequency tuning up to ±3200 ppm
 * - Factory programmable or pin-selectable I2C addresses
 * - Auto address incrementing for multi-register writes
 * 
 * @see https://www.sitime.com/datasheet/SiT5501
 */
class SiT5501 {
public:
    // Register addresses
    static constexpr uint8_t REG_FC_LSW = 0x00;    // Digital Frequency Control LSW
    static constexpr uint8_t REG_FC_MSW = 0x01;    // OE Control + Digital Frequency Control MSW
    static constexpr uint8_t REG_PULL_RANGE = 0x02; // Digital Pull Range Control
    
    // Default I2C addresses (7-bit)
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x68;  // 1101000 (A1=1, A0=0)
    static constexpr uint8_t ALT_I2C_ADDR_1 = 0x60;    // 1100000 (A1=0, A0=0)
    static constexpr uint8_t ALT_I2C_ADDR_2 = 0x62;    // 1100010 (A1=0, A0=1)
    static constexpr uint8_t ALT_I2C_ADDR_3 = 0x6A;    // 1101010 (A1=1, A0=1)
    
    // Frequency control constants
    static constexpr int32_t MAX_PULL_RANGE_PPM = 3200;
    static constexpr int32_t MIN_PULL_RANGE_PPM = -3200;
    
    // Timing constants (from datasheet)
    static constexpr uint32_t FREQ_CHANGE_DELAY_US = 140;  // Max delay from reg write to freq change
    static constexpr uint32_t FREQ_SETTLE_TIME_US = 20;    // Max time to settle to 0.5% of offset
    
    // OE (Output Enable) control bits in REG_FC_MSW
    static constexpr uint16_t OE_ENABLE = 0x0000;   // Output enabled
    static constexpr uint16_t OE_DISABLE = 0x8000;  // Output disabled (bit 15 = 1)

public:
    /**
     * @brief Constructor
     * @param i2c_addr 7-bit I2C address (default: 0x68)
     * @param wire_instance Pointer to Wire instance (default: &Wire)
     */
    SiT5501(uint8_t i2c_addr = DEFAULT_I2C_ADDR, TwoWire* wire_instance = &Wire);
    
    /**
     * @brief Initialize the device
     * @return true if device responds, false otherwise
     */
    bool begin();
    
    /**
     * @brief Check if device is present on I2C bus
     * @return true if device responds, false otherwise
     */
    bool isPresent();
    
    /**
     * @brief Set frequency offset in PPM
     * @param ppm_offset Frequency offset in parts per million (-3200 to +3200)
     * @return true if successful, false if out of range or I2C error
     */
    bool setFrequencyOffsetPPM(double ppm_offset);
    
    /**
     * @brief Set frequency offset using raw 32-bit control word
     * @param fc_value 32-bit frequency control value (0 = -max, 0x80000000 = center, 0xFFFFFFFF = +max)
     * @return true if successful, false if I2C error
     */
    bool setFrequencyControl(uint32_t fc_value);
    
    /**
     * @brief Get current frequency control value
     * @param fc_value Reference to store the 32-bit frequency control value
     * @return true if successful, false if I2C error
     */
    bool getFrequencyControl(uint32_t& fc_value);
    
    /**
     * @brief Convert PPM offset to frequency control value
     * @param ppm_offset Frequency offset in PPM
     * @return 32-bit frequency control value
     */
    static uint32_t ppmToFrequencyControl(double ppm_offset);
    
    /**
     * @brief Convert frequency control value to PPM offset
     * @param fc_value 32-bit frequency control value
     * @return Frequency offset in PPM
     */
    static double frequencyControlToPPM(uint32_t fc_value);
    
    /**
     * @brief Enable or disable output
     * @param enable true to enable output, false to disable
     * @return true if successful, false if I2C error
     */
    bool setOutputEnable(bool enable);
    
    /**
     * @brief Set pull range (if supported by device variant)
     * @param pull_range_ppm Pull range in PPM (typically ±3200)
     * @return true if successful, false if I2C error
     */
    bool setPullRange(uint16_t pull_range_ppm);
    
    /**
     * @brief Read a 16-bit register
     * @param reg_addr Register address (0x00-0x02)
     * @param value Reference to store the read value
     * @return true if successful, false if I2C error
     */
    bool readRegister(uint8_t reg_addr, uint16_t& value);
    
    /**
     * @brief Write a 16-bit register
     * @param reg_addr Register address (0x00-0x02)
     * @param value Value to write
     * @return true if successful, false if I2C error
     */
    bool writeRegister(uint8_t reg_addr, uint16_t value);
    
    /**
     * @brief Write frequency control registers atomically
     * @param fc_value 32-bit frequency control value
     * @param output_enable Enable output after frequency change
     * @return true if successful, false if I2C error
     */
    bool writeFrequencyControlAtomic(uint32_t fc_value, bool output_enable = true);

private:
    uint8_t _i2c_addr;
    TwoWire* _wire;
    
    /**
     * @brief Write multiple registers using auto-increment
     * @param start_reg Starting register address
     * @param data Pointer to data array (MSB first for each 16-bit word)
     * @param num_regs Number of 16-bit registers to write
     * @return true if successful, false if I2C error
     */
    bool writeRegistersAutoIncrement(uint8_t start_reg, const uint16_t* data, uint8_t num_regs);
};
