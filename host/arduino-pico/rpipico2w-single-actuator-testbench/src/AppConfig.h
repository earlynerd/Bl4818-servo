#pragma once

#include <Arduino.h>

namespace AppConfig {

constexpr uint32_t kUsbBaud = 115200;

constexpr uint8_t kRingTxPin = 4;
constexpr uint8_t kRingRxPin = 5;
constexpr uint32_t kRingBaud = 250000;
constexpr uint8_t kControlledAddress = 0;
constexpr uint16_t kDefaultTorqueLimitMa = 5000;

constexpr uint8_t kSpiMisoPin = 16;
constexpr uint8_t kSpiCsPin = 17;
constexpr uint8_t kSpiSckPin = 18;
constexpr uint8_t kSpiMosiPin = 19;
constexpr uint32_t kSpiClockHz = 2000000;
constexpr uint8_t kSpiMode = 1;
constexpr uint8_t kEncoderCount = 1;

constexpr uint32_t kControlRateHz = 2000;
constexpr uint32_t kControlPeriodUs = 1000000UL / kControlRateHz;
constexpr float kControlDt = 1.0f / static_cast<float>(kControlRateHz);
constexpr uint32_t kCommandRateHz = 800;
constexpr uint32_t kCommandPeriodUs = 1000000UL / kCommandRateHz;

constexpr uint32_t kStatusPollPeriodMs = 25;
constexpr uint32_t kTelemetryPeriodMs = 50;

constexpr float kDefaultHomeTargetDeg = 0.0f;
constexpr float kDefaultStrikeTargetDeg = 35.0f;
constexpr uint32_t kDefaultTapHoldMs = 35;

constexpr float kDefaultKp = 5.0f;
constexpr float kDefaultKi = 8.0f;
constexpr float kDefaultKd = 0.4f;
constexpr float kDefaultVelocityAlpha = 0.1f;
constexpr int16_t kDefaultMaxDuty = 1000;
constexpr int16_t kDutyDeadband = 0;
constexpr int16_t kDutySlewPerCommand = 150;
constexpr uint8_t kMaxConsecutiveCommFailures = 3;

}  // namespace AppConfig
