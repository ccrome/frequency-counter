#include "SiT5501.h"

SiT5501::SiT5501(uint8_t i2c_addr, TwoWire* wire_instance) 
    : _i2c_addr(i2c_addr), _wire(wire_instance) {
}

bool SiT5501::begin() {
    _wire->begin();
    return isPresent();
}

bool SiT5501::isPresent() {
    _wire->beginTransmission(_i2c_addr);
    return (_wire->endTransmission() == 0);
}

bool SiT5501::setFrequencyOffsetPPM(double ppm_offset) {
    // Validate range
    if (ppm_offset < MIN_PULL_RANGE_PPM || ppm_offset > MAX_PULL_RANGE_PPM) {
        return false;
    }
    
    uint32_t fc_value = ppmToFrequencyControl(ppm_offset);
    return writeFrequencyControlAtomic(fc_value, true);
}

bool SiT5501::setFrequencyControl(uint32_t fc_value) {
    return writeFrequencyControlAtomic(fc_value, true);
}

bool SiT5501::getFrequencyControl(uint32_t& fc_value) {
    uint16_t lsw, msw;
    
    // Read LSW register
    if (!readRegister(REG_FC_LSW, lsw)) {
        return false;
    }
    
    // Read MSW register (contains OE bit in bit 15, FC bits in 14:0)
    if (!readRegister(REG_FC_MSW, msw)) {
        return false;
    }
    
    // Combine LSW and MSW (mask out OE bit from MSW)
    fc_value = ((uint32_t)(msw & 0x7FFF) << 16) | lsw;
    
    return true;
}

uint32_t SiT5501::ppmToFrequencyControl(double ppm_offset) {
    // Convert PPM to frequency control value
    // Center value (0 PPM) = 0x80000000
    // Full range is ±3200 PPM over 32-bit range
    
    // Clamp to valid range
    if (ppm_offset > MAX_PULL_RANGE_PPM) ppm_offset = MAX_PULL_RANGE_PPM;
    if (ppm_offset < MIN_PULL_RANGE_PPM) ppm_offset = MIN_PULL_RANGE_PPM;
    
    // Calculate offset from center
    double normalized_offset = ppm_offset / (double)MAX_PULL_RANGE_PPM;  // -1.0 to +1.0
    int64_t offset_ticks = (int64_t)(normalized_offset * 0x80000000LL);  // Scale to half range
    
    // Add to center value
    return (uint32_t)(0x80000000LL + offset_ticks);
}

double SiT5501::frequencyControlToPPM(uint32_t fc_value) {
    // Convert frequency control value back to PPM
    int64_t offset_from_center = (int64_t)fc_value - 0x80000000LL;
    double normalized_offset = (double)offset_from_center / 0x80000000LL;
    return normalized_offset * MAX_PULL_RANGE_PPM;
}

bool SiT5501::setOutputEnable(bool enable) {
    uint16_t msw_value;
    
    // Read current MSW register to preserve frequency control bits
    if (!readRegister(REG_FC_MSW, msw_value)) {
        return false;
    }
    
    // Modify OE bit (bit 15) while preserving frequency control bits (14:0)
    if (enable) {
        msw_value &= 0x7FFF;  // Clear bit 15 (enable output)
    } else {
        msw_value |= 0x8000;  // Set bit 15 (disable output)
    }
    
    return writeRegister(REG_FC_MSW, msw_value);
}

bool SiT5501::setPullRange(uint16_t pull_range_ppm) {
    // Write to pull range register if device supports it
    return writeRegister(REG_PULL_RANGE, pull_range_ppm);
}

bool SiT5501::readRegister(uint8_t reg_addr, uint16_t& value) {
    // Write register address
    _wire->beginTransmission(_i2c_addr);
    _wire->write(reg_addr);
    if (_wire->endTransmission(false) != 0) {  // Send repeated start
        return false;
    }
    
    // Read 2 bytes (MSB first)
    if (_wire->requestFrom(_i2c_addr, (uint8_t)2) != 2) {
        return false;
    }
    
    uint8_t msb = _wire->read();
    uint8_t lsb = _wire->read();
    value = ((uint16_t)msb << 8) | lsb;
    
    return true;
}

bool SiT5501::writeRegister(uint8_t reg_addr, uint16_t value) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(reg_addr);
    _wire->write((uint8_t)(value >> 8));    // MSB first
    _wire->write((uint8_t)(value & 0xFF));  // LSB
    
    return (_wire->endTransmission() == 0);
}

bool SiT5501::writeFrequencyControlAtomic(uint32_t fc_value, bool output_enable) {
    // Prepare data for atomic write using auto-increment
    uint16_t reg_data[2];
    
    // Register 0x00: FC LSW (bits 15:0)
    uint16_t lsw = (uint16_t)(fc_value & 0xFFFF);
    uint16_t msw = ((uint16_t)(fc_value >> 16)) & 0x3FF;
    msw |= (1<<10); // output enabled
    reg_data[0] = lsw;
    reg_data[1] = msw;
    // Write both registers atomically using auto-increment
    bool success = writeRegistersAutoIncrement(REG_FC_LSW, reg_data, 2);
    
    return success;
}

bool SiT5501::writeRegistersAutoIncrement(uint8_t start_reg, const uint16_t* data, uint8_t num_regs) {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(start_reg);  // Starting register address
    
    // Write data for each register (MSB first for each 16-bit word)
    for (uint8_t i = 0; i < num_regs; i++) {
        _wire->write((uint8_t)(data[i] >> 8));    // MSB
        _wire->write((uint8_t)(data[i] & 0xFF));  // LSB
    }
    
    return (_wire->endTransmission() == 0);
}
