// as5600.cpp
// Purpose: AS5600 magnetic rotary position sensor driver implementation
// Author: crisiumnih

#include "as5600.hpp"

AS5600_RET_TYPE AS5600::IsPresent(void) {
    uint8_t data;
    return read(hinterface, as5600::REG_I2C_ADDR, as5600::REG_STATUS, &data, 1);
}

AS5600_RET_TYPE AS5600::readReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    return read(hinterface, as5600::REG_I2C_ADDR, reg, buf, len);
}

AS5600_RET_TYPE AS5600::writeReg(uint8_t reg, uint8_t* buf, uint8_t len) {
    return write(hinterface, as5600::REG_I2C_ADDR, reg, buf, len);
}

AS5600_RET_TYPE AS5600::ReadRawAngle(uint16_t* angle) {
    uint8_t buf[2];

    auto ret = readReg(as5600::REG_RAW_ANGLE_H, buf, 2);
    if (ret != AS5600_RET_OK)
        return ret;

    uint16_t v = ((uint16_t)buf[0] << 8) | buf[1];
    *angle = v & 0x0FFF;  // 12-bit mask

    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::ReadRawAngleWithOffset(uint16_t* angle) {
    uint16_t raw;
    auto ret = ReadRawAngle(&raw);
    if (ret != AS5600_RET_OK)
        return ret;

    // Apply software zero offset with wraparound for 12-bit
    int32_t offset_angle = (int32_t)raw - (int32_t)zeroOffset;
    if (offset_angle < 0) {
        offset_angle += 4096;
    }
    *angle = (uint16_t)offset_angle & 0x0FFF;

    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::ReadAngleDeg(float* deg) {
    uint16_t raw;
    auto ret = ReadRawAngle(&raw);
    if (ret != AS5600_RET_OK)
        return ret;

    *deg = (raw * 360.0f) / 4096.0f;
    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::ReadAngle01(float* normalized) {
    uint16_t raw;
    auto ret = ReadRawAngle(&raw);
    if (ret != AS5600_RET_OK)
        return ret;

    *normalized = raw / 4096.0f;
    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::GetStatus(uint8_t* status) {
    return readReg(as5600::REG_STATUS, status, 1);
}

AS5600_RET_TYPE AS5600::SetHysteresis(uint8_t val) {
    if (val > 0b11)
        return AS5600_RET_ERR;

    uint8_t conf_l;
    auto ret = readReg(as5600::REG_CONF_L, &conf_l, 1);
    if (ret != AS5600_RET_OK)
        return ret;

    conf_l = (conf_l & ~as5600::CONF_HYST_MASK) | ((val << 2) & as5600::CONF_HYST_MASK);
    return writeReg(as5600::REG_CONF_L, &conf_l, 1);
}

AS5600_RET_TYPE AS5600::SetSlowFilter(as5600::SlowFilter sf) {
    if (sf > 0b11)
        return AS5600_RET_ERR;

    uint8_t conf_h;
    auto ret = readReg(as5600::REG_CONF_H, &conf_h, 1);
    if (ret != AS5600_RET_OK)
        return ret;

    conf_h = (conf_h & ~as5600::CONF_SF_MASK) | (sf & as5600::CONF_SF_MASK);
    return writeReg(as5600::REG_CONF_H, &conf_h, 1);
}

AS5600_RET_TYPE AS5600::SetFastFilter(as5600::FastFilterThreshold fth) {
    if (fth > 0b111)
        return AS5600_RET_ERR;

    uint8_t conf_h;
    auto ret = readReg(as5600::REG_CONF_H, &conf_h, 1);
    if (ret != AS5600_RET_OK)
        return ret;

    conf_h = (conf_h & ~as5600::CONF_FTH_MASK) | ((fth << 2) & as5600::CONF_FTH_MASK);
    return writeReg(as5600::REG_CONF_H, &conf_h, 1);
}

AS5600_RET_TYPE AS5600::SetWatchdog(bool enable) {
    uint8_t conf_h;
    auto ret = readReg(as5600::REG_CONF_H, &conf_h, 1);
    if (ret != AS5600_RET_OK)
        return ret;

    if (enable) {
        conf_h |= as5600::CONF_WD_MASK;
    } else {
        conf_h &= ~as5600::CONF_WD_MASK;
    }
    return writeReg(as5600::REG_CONF_H, &conf_h, 1);
}

AS5600_RET_TYPE AS5600::Init(void) {
    auto ret = IsPresent();
    if (ret != AS5600_RET_OK)
        return ret;

    ret = SetHysteresis(1);
    if (ret != AS5600_RET_OK)
        return ret;

    ret = SetSlowFilter(as5600::SF_8X);
    if (ret != AS5600_RET_OK)
        return ret;

    ret = SetFastFilter(as5600::FTH_7_LSB);
    if (ret != AS5600_RET_OK)
        return ret;

    ret = SetWatchdog(false);
    if (ret != AS5600_RET_OK)
        return ret;

    return AS5600_RET_OK;
}

bool AS5600::MagnetDetected() {
    uint8_t status;
    if (GetStatus(&status) != AS5600_RET_OK)
        return false;
    return (status & as5600::STATUS_MD) != 0;
}

bool AS5600::MagnetTooWeak() {
    uint8_t status;
    if (GetStatus(&status) != AS5600_RET_OK)
        return false;
    return (status & as5600::STATUS_ML) != 0;
}

bool AS5600::MagnetTooStrong() {
    uint8_t status;
    if (GetStatus(&status) != AS5600_RET_OK)
        return false;
    return (status & as5600::STATUS_MH) != 0;
}

void AS5600::SetSoftwareZero(uint16_t raw) {
    zeroOffset = raw & 0x0FFF;
}

int16_t AS5600::GetDelta() {
    uint16_t curr;
    if (ReadRawAngle(&curr) != AS5600_RET_OK) {
        return 0;
    }

    int16_t delta = (int16_t)curr - (int16_t)prevAngle;

    // Wrap correction for shortest circular distance
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }

    prevAngle = curr;
    return delta;
}

AS5600_RET_TYPE AS5600::SetZeroPosition(uint16_t raw) {
    if (raw > 0x0FFF)
        return AS5600_RET_ERR;

    uint8_t buf[2];
    buf[0] = (raw >> 8) & 0x0F;
    buf[1] = raw & 0xFF;

    auto ret = writeReg(as5600::REG_ZPOS_H, &buf[0], 1);
    if (ret != AS5600_RET_OK)
        return ret;

    delayMs(hinterface, 1);

    return writeReg(as5600::REG_ZPOS_L, &buf[1], 1);
}

AS5600_RET_TYPE AS5600::SetMaxPosition(uint16_t raw) {
    if (raw > 0x0FFF)
        return AS5600_RET_ERR;

    uint8_t buf[2];
    buf[0] = (raw >> 8) & 0x0F;
    buf[1] = raw & 0xFF;

    auto ret = writeReg(as5600::REG_MPOS_H, &buf[0], 1);
    if (ret != AS5600_RET_OK)
        return ret;

    delayMs(hinterface, 1);

    return writeReg(as5600::REG_MPOS_L, &buf[1], 1);
}

AS5600_RET_TYPE AS5600::SetMaxAngle(uint16_t raw) {
    if (raw > 0x0FFF)
        return AS5600_RET_ERR;

    uint8_t buf[2];
    buf[0] = (raw >> 8) & 0x0F;
    buf[1] = raw & 0xFF;

    auto ret = writeReg(as5600::REG_MANG_H, &buf[0], 1);
    if (ret != AS5600_RET_OK)
        return ret;

    delayMs(hinterface, 1);

    return writeReg(as5600::REG_MANG_L, &buf[1], 1);
}

AS5600_RET_TYPE AS5600::GetZeroPosition(uint16_t* raw) {
    uint8_t buf[2];

    auto ret = readReg(as5600::REG_ZPOS_H, buf, 2);
    if (ret != AS5600_RET_OK)
        return ret;

    uint16_t v = ((uint16_t)buf[0] << 8) | buf[1];
    *raw = v & 0x0FFF;

    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::GetMaxPosition(uint16_t* raw) {
    uint8_t buf[2];

    auto ret = readReg(as5600::REG_MPOS_H, buf, 2);
    if (ret != AS5600_RET_OK)
        return ret;

    uint16_t v = ((uint16_t)buf[0] << 8) | buf[1];
    *raw = v & 0x0FFF;

    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::GetMaxAngle(uint16_t* raw) {
    uint8_t buf[2];

    auto ret = readReg(as5600::REG_MANG_H, buf, 2);
    if (ret != AS5600_RET_OK)
        return ret;

    uint16_t v = ((uint16_t)buf[0] << 8) | buf[1];
    *raw = v & 0x0FFF;

    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::BurnAngle() {
    // Check if magnet is detected
    if (!MagnetDetected()) {
        return AS5600_RET_NO_MAGNET;
    }

    // Check ZMCO - can only burn up to 3 times
    uint8_t zmco;
    auto ret = readReg(as5600::REG_ZMCO, &zmco, 1);
    if (ret != AS5600_RET_OK)
        return AS5600_RET_I2C_FAIL;

    if (zmco >= 3) {
        return AS5600_RET_OTP_FULL;
    }

    // Write 0x80 to BURN register to execute BURN_ANGLE command
    uint8_t burn_cmd = 0x80;
    ret = writeReg(as5600::REG_BURN, &burn_cmd, 1);
    if (ret != AS5600_RET_OK)
        return AS5600_RET_I2C_FAIL;

    delayMs(hinterface, 2);

    return AS5600_RET_OK;
}

AS5600_RET_TYPE AS5600::VerifyBurnAngle(uint16_t expected_zpos, uint16_t expected_mpos) {
    // Reload OTP shadow registers per datasheet sequence
    uint8_t cmd;

    cmd = 0x01;
    auto ret = writeReg(as5600::REG_BURN, &cmd, 1);
    if (ret != AS5600_RET_OK)
        return ret;

    delayMs(hinterface, 1);

    cmd = 0x11;
    ret = writeReg(as5600::REG_BURN, &cmd, 1);
    if (ret != AS5600_RET_OK)
        return ret;

    delayMs(hinterface, 1);

    cmd = 0x10;
    ret = writeReg(as5600::REG_BURN, &cmd, 1);
    if (ret != AS5600_RET_OK)
        return ret;

    delayMs(hinterface, 1);

    // Read back ZPOS and MPOS
    uint16_t zpos, mpos;
    ret = GetZeroPosition(&zpos);
    if (ret != AS5600_RET_OK)
        return ret;

    ret = GetMaxPosition(&mpos);
    if (ret != AS5600_RET_OK)
        return ret;

    // Verify values match
    if (zpos != expected_zpos || mpos != expected_mpos) {
        return AS5600_RET_ERR;
    }

    return AS5600_RET_OK;
}
