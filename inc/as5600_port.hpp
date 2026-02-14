// as5600_port.hpp
// Purpose: AS5600 port layer definitions
// Author: crisiumnih

#ifndef AS5600_PORT_HPP_
#define AS5600_PORT_HPP_

#include <stdint.h>

typedef uint8_t AS5600_RET_TYPE;

constexpr AS5600_RET_TYPE AS5600_RET_OK = 0;
constexpr AS5600_RET_TYPE AS5600_RET_ERR = 1;
constexpr AS5600_RET_TYPE AS5600_RET_BUSY = 2;
constexpr AS5600_RET_TYPE AS5600_RET_NO_MAGNET = 3;
constexpr AS5600_RET_TYPE AS5600_RET_OTP_FULL = 4;
constexpr AS5600_RET_TYPE AS5600_RET_I2C_FAIL = 5;

#endif  // AS5600_PORT_HPP_
