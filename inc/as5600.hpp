// as5600.hpp
// Purpose: AS5600 magnetic rotary position sensor driver
// Author: crisiumnih

#ifndef AS5600_HPP_
#define AS5600_HPP_

#include "as5600_port.hpp"
#include "as5600_reg.hpp"

typedef AS5600_RET_TYPE (*AS5600_ReadFuncPtr)(void* hinterface, uint8_t chipAddr, uint8_t reg,
                                              uint8_t* buf, uint8_t len);
typedef AS5600_RET_TYPE (*AS5600_WriteFuncPtr)(void* hinterface, uint8_t chipAddr, uint8_t reg,
                                               uint8_t* buf, uint8_t len);
typedef AS5600_RET_TYPE (*AS5600_DelayMsFuncPtr)(void* hinterface, uint32_t delayMs);

class AS5600 {
private:
    void* hinterface;
    AS5600_ReadFuncPtr read;
    AS5600_WriteFuncPtr write;
    AS5600_DelayMsFuncPtr delayMs;

    uint16_t zeroOffset = 0;
    uint16_t prevAngle = 0;

    AS5600_RET_TYPE readReg(uint8_t reg, uint8_t* buf, uint8_t len);
    AS5600_RET_TYPE writeReg(uint8_t reg, uint8_t* buf, uint8_t len);

    AS5600_RET_TYPE GetStatus(uint8_t* status);

public:
    AS5600(void* hinterface, AS5600_ReadFuncPtr read, AS5600_WriteFuncPtr write,
           AS5600_DelayMsFuncPtr delay)
        : hinterface(hinterface), read(read), write(write), delayMs(delay) {}

    AS5600_RET_TYPE IsPresent(void);
    AS5600_RET_TYPE Init(void);

    AS5600_RET_TYPE ReadRawAngle(uint16_t* angle);
    AS5600_RET_TYPE ReadRawAngleWithOffset(uint16_t* angle);
    AS5600_RET_TYPE ReadAngleDeg(float* deg);
    AS5600_RET_TYPE ReadAngle01(float* normalized);

    AS5600_RET_TYPE SetHysteresis(uint8_t val);
    AS5600_RET_TYPE SetSlowFilter(as5600::SlowFilter sf);
    AS5600_RET_TYPE SetFastFilter(as5600::FastFilterThreshold fth);
    AS5600_RET_TYPE SetWatchdog(bool enable);

    AS5600_RET_TYPE SetZeroPosition(uint16_t raw);
    AS5600_RET_TYPE SetMaxPosition(uint16_t raw);
    AS5600_RET_TYPE SetMaxAngle(uint16_t raw);

    AS5600_RET_TYPE GetZeroPosition(uint16_t* raw);
    AS5600_RET_TYPE GetMaxPosition(uint16_t* raw);
    AS5600_RET_TYPE GetMaxAngle(uint16_t* raw);

    AS5600_RET_TYPE BurnAngle();
    AS5600_RET_TYPE VerifyBurnAngle(uint16_t expected_zpos, uint16_t expected_mpos);

    bool MagnetDetected();
    bool MagnetTooWeak();
    bool MagnetTooStrong();

    void SetSoftwareZero(uint16_t raw);

    int16_t GetDelta();
};

#endif  // AS5600_HPP_
