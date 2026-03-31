#pragma once

#include <Arduino.h>

class SingleActuatorController {
public:
    static constexpr int32_t kCountsPerRevolution = 16384;

    void setGains(float kp, float kd);
    void setIntegralGain(float ki);
    void setMaxDuty(int16_t maxDuty);
    void setVelocityAlpha(float alpha);
    void setDirection(int8_t direction);
    void setTargetDegrees(float targetDegrees);

    float kp() const;
    float ki() const;
    float kd() const;
    float velocityAlpha() const;
    int16_t maxDuty() const;
    int8_t direction() const;
    float targetDegrees() const;

    bool initialized() const;
    void reset();
    void update(uint16_t rawAngle, float dtSeconds);
    void zeroAtCurrentPosition();

    uint16_t rawAngle() const;
    int32_t unwrappedCounts() const;
    float positionDegrees() const;
    float velocityDegreesPerSecond() const;
    int16_t dutyCommand() const;

private:
    static int16_t clampDuty(float value, int16_t limit);

    bool initialized_ = false;
    uint16_t rawAngle_ = 0;
    uint16_t lastRawAngle_ = 0;
    int32_t unwrappedCounts_ = 0;
    int32_t zeroCounts_ = 0;

    float targetDegrees_ = 0.0f;
    float positionDegrees_ = 0.0f;
    float lastPositionDegrees_ = 0.0f;
    float velocityDegreesPerSecond_ = 0.0f;

    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    float velocityAlpha_ = 0.2f;
    float integralError_ = 0.0f;
    int16_t maxDuty_ = 0;
    int8_t direction_ = 1;
    int16_t dutyCommand_ = 0;
};

