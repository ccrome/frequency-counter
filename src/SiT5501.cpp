#include "SiT5501.h"

SiT5501::SiT5501(uint8_t i2c_addr, TwoWire* wire_instance)
    : _i2c_addr(i2c_addr), _wire(wire_instance) {
}

bool SiT5501::begin() {
    _wire->begin();
    for (int i = 0; i < N_REGISTERS; i++) {
        registers[i] = 0;
    }
    flushRegisters();
    return isPresent();
}

bool SiT5501::flushRegisters() {
    bool ok = true;
    uint16_t r_shadow[N_REGISTERS];
    for (int i = 0; i < N_REGISTERS; i++) {
        ok &= writeRegister(i, registers[i]);
        r_shadow[i] = registers[i];
    }
    Serial.printf("\r\n");
    bool dirty = false;
    for (int i = 0; i < N_REGISTERS; i++) {
        readRegister(i, registers[i]);
        if (registers[i] != r_shadow[i])
            dirty=true;
    }
    if (dirty) {
        Serial.printf("REGISTER MISMATCH!\r\n");
        Serial.printf("original registers: ");
        for (int i = 0; i < N_REGISTERS; i++)
            Serial.printf("0x%08x, ", r_shadow[i]);
        Serial.printf("\r\n");
        Serial.printf("new registers     : ");
        for (int i = 0; i < N_REGISTERS; i++)
            Serial.printf("0x%08x, ", registers[i]);
        Serial.printf("\r\n");
    }
    return ok;
}
double SiT5501::getPullRange() {
    uint16_t pull_range = registers[2] & 0xf;
    switch (pull_range) {
    case PULL_RANGE_6_25_PPM    : return    6.25;
    case PULL_RANGE_10_00_PPM   : return   10.00;
    case PULL_RANGE_12_50_PPM   : return   12.50;
    case PULL_RANGE_25_00_PPM   : return   25.00;
    case PULL_RANGE_50_00_PPM   : return   50.00;
    case PULL_RANGE_80_00_PPM   : return   80.00;
    case PULL_RANGE_100_00_PPM  : return  100.00;
    case PULL_RANGE_125_00_PPM  : return  125.00;
    case PULL_RANGE_150_00_PPM  : return  150.00;
    case PULL_RANGE_200_00_PPM  : return  200.00;
    case PULL_RANGE_400_00_PPM  : return  400.00;
    case PULL_RANGE_600_00_PPM  : return  600.00;
    case PULL_RANGE_800_00_PPM  : return  800.00;
    case PULL_RANGE_1200_00_PPM : return 1200.00;
    case PULL_RANGE_1600_00_PPM : return 1600.00;
    case PULL_RANGE_3200_00_PPM : return 3200.00;
    }
    return 0;
}
bool SiT5501::isPresent() {
    _wire->beginTransmission(_i2c_addr);
    return (_wire->endTransmission() == 0);
}

bool SiT5501::setFrequencyOffsetPPM(double ppm_offset) {
    double pull_range = getPullRange();
    if (ppm_offset < -pull_range || ppm_offset > pull_range) {
        return false;
    }

    double fc_double = 1.0 * ppm_offset * ((1<<25)-1) / pull_range;
    int32_t fc_int = (int32_t) fc_double;
    uint32_t fc_value = (uint32_t) fc_int & ((1<<25)-1);
    //Serial.printf("pull range = %f, fc_double = %f, fc_int = 0x%08x, fc_value = 0x%08x\r\n",
    //              pull_range, fc_double, fc_int, fc_value);
    return setFrequencyControl(fc_value);

}

bool SiT5501::setFrequencyControl(uint32_t fc_value) {
    uint16_t lsw = fc_value & 0xFFFF;
    uint16_t msw = (fc_value >> 16) & 0x3ff;
    registers[0] = lsw;
    registers[1] &= ~0x3fff;
    registers[1] |= msw;
    return flushRegisters();
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
    fc_value = ((uint32_t)(msw & 0x3FF) << 16) | lsw;

    return true;
}

bool SiT5501::setOutputEnable(bool enable) {
    if (enable)
        registers[1] |= (1<<10);
    else
        registers[1] &= ~(1<<10);
    return flushRegisters();
}

bool SiT5501::setPullRange(PULL_RANGE_t pull_range) {
    registers[2] = (uint32_t)pull_range;
    return flushRegisters();
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
