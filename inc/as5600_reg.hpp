// as5600_reg.hpp
// Purpose: AS5600 register definitions
// Author: crisiumnih

#ifndef AS5600_REG_HPP_
#define AS5600_REG_HPP_

#include <stdint.h>

namespace as5600 {

// I2C address (7-bit)
constexpr uint8_t REG_I2C_ADDR = 0x36;

// --- Configuration / Setup ---
constexpr uint8_t REG_ZMCO = 0x00;

constexpr uint8_t REG_ZPOS_H = 0x01;
constexpr uint8_t REG_ZPOS_L = 0x02;

constexpr uint8_t REG_MPOS_H = 0x03;
constexpr uint8_t REG_MPOS_L = 0x04;

constexpr uint8_t REG_MANG_H = 0x05;
constexpr uint8_t REG_MANG_L = 0x06;

constexpr uint8_t REG_CONF_H = 0x07;
constexpr uint8_t REG_CONF_L = 0x08;

// --- Output Registers ---
constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;
constexpr uint8_t REG_RAW_ANGLE_L = 0x0D;

constexpr uint8_t REG_ANGLE_H = 0x0E;
constexpr uint8_t REG_ANGLE_L = 0x0F;

// --- Status / Diagnostics ---
constexpr uint8_t REG_STATUS = 0x0B;
constexpr uint8_t REG_AGC = 0x1A;

constexpr uint8_t REG_MAGNITUDE_H = 0x1B;
constexpr uint8_t REG_MAGNITUDE_L = 0x1C;

// --- Burn Command ---
constexpr uint8_t REG_BURN = 0xFF;

constexpr uint8_t STATUS_MD = 1 << 5;  // Magnet Detected
constexpr uint8_t STATUS_ML = 1 << 4;  // Magnet Weak
constexpr uint8_t STATUS_MH = 1 << 3;  // Magnet Strong

// High byte (0x07)
constexpr uint8_t CONF_WD_MASK = 0b00100000;   // bit 5
constexpr uint8_t CONF_FTH_MASK = 0b00011100;  // bits 4-2
constexpr uint8_t CONF_SF_MASK = 0b00000011;   // bits 1-0

// Low byte (0x08)
constexpr uint8_t CONF_PWMF_MASK = 0b11000000;  // bits 7-6
constexpr uint8_t CONF_OUTS_MASK = 0b00110000;  // bits 5-4
constexpr uint8_t CONF_HYST_MASK = 0b00001100;  // bits 3-2
constexpr uint8_t CONF_PM_MASK = 0b00000011;    // bits 1-0

// ---------------- FILTER SETTINGS ----------------
//
// The AS5600 has two internal digital filters:
//
// 1) Slow Filter (SF)  -> always active
//    Controls base smoothing vs latency.
//    Higher value = faster response, but more noise.
//    Lower value  = smoother output, but more delay.
//
// 2) Fast Filter Threshold (FTH) -> conditional
//    If angle change > threshold, chip bypasses slow filter
//    for a couple of samples to react quickly.
//    Threshold is in LSBs of the 12-bit angle (0–4095).
//
// Typical encoder use:
//   SF_8X or SF_4X  +  FTH_7_LSB or FTH_9_LSB
//

// --- Slow Filter (SF bits: 3:2 in CONF_H) ---
enum SlowFilter : uint8_t {

    // Maximum smoothing, ~2.2 ms step delay.
    // Best stability at rest, worst responsiveness.
    SF_16X = 0b00,

    // Moderate smoothing, ~1.1 ms delay.
    // Good general-purpose default for knobs / joints.
    SF_8X = 0b01,

    // Light smoothing, ~0.55 ms delay.
    // Faster tracking, slightly noisier.
    SF_4X = 0b10,

    // Minimal smoothing, ~0.28 ms delay.
    // Fastest response, highest jitter.
    // Use for high-speed shafts.
    SF_2X = 0b11
};

// --- Fast Filter Threshold (FTH bits: 6:4 in CONF_H) ---
enum FastFilterThreshold : uint8_t {

    // Disable fast path entirely.
    // Output always governed by slow filter.
    FTH_SLOW_ONLY = 0b000,

    // Threshold ≈ 6 LSB  (~0.5°)
    // Small movements already trigger fast response.
    FTH_6_LSB = 0b001,

    // ≈ 7 LSB (~0.6°)
    // Good balance for most encoders.
    FTH_7_LSB = 0b010,

    // ≈ 9 LSB (~0.8°)
    // Slightly more smoothing before fast kick-in.
    FTH_9_LSB = 0b011,

    // ≈ 18 LSB (~1.6°)
    // Only larger motion bypasses slow filter.
    FTH_18_LSB = 0b100,

    // ≈ 21 LSB (~1.8°)
    FTH_21_LSB = 0b101,

    // ≈ 24 LSB (~2.1°)
    FTH_24_LSB = 0b110,

    // ≈ 10 LSB (~0.9°) but different return hysteresis.
    // Rarely needed; special behaviour in datasheet.
    FTH_10_LSB = 0b111
};

}  // namespace as5600

#endif  // AS5600_REG_HPP_
