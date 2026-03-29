#include "SingleActuatorController.h"

#include <math.h>

void SingleActuatorController::setGains(float kp, float kd)
{
    kp_ = kp;
    kd_ = kd;
}

void SingleActuatorController::setMaxDuty(int16_t maxDuty)
{
    if (maxDuty < 0) {
        maxDuty = static_cast<int16_t>(-maxDuty);
    }
    maxDuty_ = maxDuty;
}

void SingleActuatorController::setVelocityAlpha(float alpha)
{
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    velocityAlpha_ = alpha;
}

void SingleActuatorController::setDirection(int8_t direction)
{
    direction_ = (direction < 0) ? -1 : 1;
}

void SingleActuatorController::setTargetDegrees(float targetDegrees)
{
    targetDegrees_ = targetDegrees;
}

float SingleActuatorController::kp() const
{
    return kp_;
}

float SingleActuatorController::kd() const
{
    return kd_;
}

float SingleActuatorController::velocityAlpha() const
{
    return velocityAlpha_;
}

int16_t SingleActuatorController::maxDuty() const
{
    return maxDuty_;
}

int8_t SingleActuatorController::direction() const
{
    return direction_;
}

float SingleActuatorController::targetDegrees() const
{
    return targetDegrees_;
}

bool SingleActuatorController::initialized() const
{
    return initialized_;
}

void SingleActuatorController::reset()
{
    initialized_ = false;
    rawAngle_ = 0;
    lastRawAngle_ = 0;
    unwrappedCounts_ = 0;
    zeroCounts_ = 0;
    positionDegrees_ = 0.0f;
    lastPositionDegrees_ = 0.0f;
    velocityDegreesPerSecond_ = 0.0f;
    dutyCommand_ = 0;
}

void SingleActuatorController::update(uint16_t rawAngle, float dtSeconds)
{
    rawAngle &= 0x3FFFu;

    if (dtSeconds <= 0.0f) {
        dutyCommand_ = 0;
        return;
    }

    rawAngle_ = rawAngle;

    if (!initialized_) {
        initialized_ = true;
        lastRawAngle_ = rawAngle;
        unwrappedCounts_ = rawAngle;
        zeroCounts_ = rawAngle;
        positionDegrees_ = 0.0f;
        lastPositionDegrees_ = 0.0f;
        velocityDegreesPerSecond_ = 0.0f;
        dutyCommand_ = 0;
        return;
    }

    int32_t deltaCounts = static_cast<int32_t>(rawAngle) - static_cast<int32_t>(lastRawAngle_);
    if (deltaCounts > (kCountsPerRevolution / 2)) {
        deltaCounts -= kCountsPerRevolution;
    } else if (deltaCounts < -(kCountsPerRevolution / 2)) {
        deltaCounts += kCountsPerRevolution;
    }

    lastRawAngle_ = rawAngle;
    unwrappedCounts_ += deltaCounts;

    const int32_t signedCounts = static_cast<int32_t>(direction_) * (unwrappedCounts_ - zeroCounts_);
    positionDegrees_ = static_cast<float>(signedCounts) * (360.0f / static_cast<float>(kCountsPerRevolution));

    const float rawVelocity = (positionDegrees_ - lastPositionDegrees_) / dtSeconds;
    velocityDegreesPerSecond_ =
        (velocityAlpha_ * rawVelocity) + ((1.0f - velocityAlpha_) * velocityDegreesPerSecond_);
    lastPositionDegrees_ = positionDegrees_;

    const float positionError = targetDegrees_ - positionDegrees_;
    const float output = (kp_ * positionError) - (kd_ * velocityDegreesPerSecond_);

    dutyCommand_ = clampDuty(output, maxDuty_);
}

void SingleActuatorController::zeroAtCurrentPosition()
{
    if (!initialized_) {
        return;
    }

    zeroCounts_ = unwrappedCounts_;
    positionDegrees_ = 0.0f;
    lastPositionDegrees_ = 0.0f;
    velocityDegreesPerSecond_ = 0.0f;
    dutyCommand_ = 0;
}

uint16_t SingleActuatorController::rawAngle() const
{
    return rawAngle_;
}

int32_t SingleActuatorController::unwrappedCounts() const
{
    return unwrappedCounts_;
}

float SingleActuatorController::positionDegrees() const
{
    return positionDegrees_;
}

float SingleActuatorController::velocityDegreesPerSecond() const
{
    return velocityDegreesPerSecond_;
}

int16_t SingleActuatorController::dutyCommand() const
{
    return dutyCommand_;
}

int16_t SingleActuatorController::clampDuty(float value, int16_t limit)
{
    if (value > static_cast<float>(limit)) {
        value = static_cast<float>(limit);
    } else if (value < static_cast<float>(-limit)) {
        value = static_cast<float>(-limit);
    }

    if (value >= 0.0f) {
        return static_cast<int16_t>(value + 0.5f);
    }
    return static_cast<int16_t>(value - 0.5f);
}

