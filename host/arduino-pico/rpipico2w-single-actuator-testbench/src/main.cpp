#include <Arduino.h>
#include <SPI.h>
#include <BL4818RingMaster.h>
#include <stdlib.h>
#include <string.h>
#include "AppConfig.h"
#include "As5047Chain.h"
#include "SingleActuatorController.h"

namespace {
static SerialPIO target_uart(12, 11, 256);

BL4818RingMaster gRing(target_uart);
As5047Chain gEncoders(SPI, AppConfig::kSpiCsPin);
SingleActuatorController gController;

struct CaptureSample {
    float pos;
    float vel;
    int16_t duty;
};

constexpr size_t kCaptureBufferSize = 2000;
static CaptureSample gCaptureBuffer[kCaptureBufferSize];
static size_t gCaptureIndex = 0;
static bool gCaptureActive = false;

int16_t gDuties[BL4818RingMaster::kMaxDevices];
BL4818RingMaster::Status gLastStatus;
As5047Sample gLastEncoderSample;

uint8_t gDeviceCount = 0;
uint16_t gTorqueLimitMa = AppConfig::kDefaultTorqueLimitMa;

bool gRingReady = false;
bool gArmed = false;
bool gStreamTelemetry = false;
bool gTapActive = false;
bool gProbeActive = false;
bool gStrikePulseActive = false;
bool gRingTraceEnabled = false;

float gHomeTargetDeg = AppConfig::kDefaultHomeTargetDeg;
float gStrikeTargetDeg = AppConfig::kDefaultStrikeTargetDeg;
float gProbeSurfaceDeg = 0.0f;
uint32_t gTapHoldMs = AppConfig::kDefaultTapHoldMs;
uint32_t gStrikePulseEndMs = 0;
int16_t gStrikePulseDuty = 0;

uint32_t gNextControlAtUs = 0;
uint32_t gNextCommandAtUs = 0;
uint32_t gLastStatusPollAtMs = 0;
uint32_t gLastTelemetryAtMs = 0;
uint32_t gTapReturnAtMs = 0;
int16_t gAppliedDutyCommand = 0;
uint8_t gConsecutiveRingFailures = 0;

char gLineBuffer[96];
uint8_t gLineLength = 0;

bool timeReached(uint32_t now, uint32_t target)
{
    return static_cast<int32_t>(now - target) >= 0;
}

bool useAddressedDutyPath()
{
    return (gDeviceCount == 1u && AppConfig::kControlledAddress == 0u);
}

int16_t filteredDutyCommand()
{
    int16_t duty = gController.dutyCommand();

    if (duty > -AppConfig::kDutyDeadband && duty < AppConfig::kDutyDeadband) {
        duty = 0;
    }

    return duty;
}

int16_t slewLimitedDutyCommand(int16_t targetDuty)
{
    const int16_t step = AppConfig::kDutySlewPerCommand;

    if (targetDuty > gAppliedDutyCommand + step) {
        return static_cast<int16_t>(gAppliedDutyCommand + step);
    }

    if (targetDuty < gAppliedDutyCommand - step) {
        return static_cast<int16_t>(gAppliedDutyCommand - step);
    }

    return targetDuty;
}

void clearDutyArray()
{
    memset(gDuties, 0, sizeof(gDuties));
}

void applyRingTraceSetting()
{
    gRing.setTraceStream(gRingTraceEnabled ? &Serial : nullptr);
}

void printBanner()
{
    Serial.println("BL4818 single-actuator testbench");
    Serial.println("Type 'help' for commands.");
}

void printHelp()
{
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  status");
    Serial.println("  enum");
    Serial.println("  arm");
    Serial.println("  disarm");
    Serial.println("  target <deg>");
    Serial.println("  home <deg>");
    Serial.println("  strike <deg>");
    Serial.println("  tap [hold_ms]");
    Serial.println("  zero");
    Serial.println("  dir <1|-1>");
    Serial.println("  gains <kp> <kd>");
    Serial.println("  ki <gain>");
    Serial.println("  maxduty <0..1200>");
    Serial.println("  minduty <0..500>");
    Serial.println("  valpha <0.0..1.0>");
    Serial.println("  torque <mA>");
    Serial.println("  gravity <kg> <offset>");
    Serial.println("  probe");
    Serial.println("  setsurf");
    Serial.println("  strike_pulse <duty> <ms>");
    Serial.println("  capture");
    Serial.println("  dump");
    Serial.println("  stream <0|1>");
    Serial.println("  trace <0|1>");
}

void printEncoderStatus()
{
    Serial.print("encoder_raw=");
    Serial.print(gLastEncoderSample.rawAngle);
    Serial.print(" frame=0x");
    Serial.print(gLastEncoderSample.rawFrame, HEX);
    Serial.print(" pos_deg=");
    Serial.print(gController.positionDegrees(), 3);
    Serial.print(" vel_dps=");
    Serial.print(gController.velocityDegreesPerSecond(), 3);
    Serial.print(" parity_err=");
    Serial.print(gLastEncoderSample.parityError ? 1 : 0);
    Serial.print(" enc_err=");
    Serial.println(gLastEncoderSample.errorFlag ? 1 : 0);
}

void printRingStatus()
{
    Serial.print("ring_ready=");
    Serial.print(gRingReady ? 1 : 0);
    Serial.print(" devices=");
    Serial.print(gDeviceCount);
    Serial.print(" armed=");
    Serial.print(gArmed ? 1 : 0);
    Serial.print(" target_deg=");
    Serial.print(gController.targetDegrees(), 3);
    Serial.print(" duty=");
    Serial.print(gController.dutyCommand());
    Serial.print(" torque_ma=");
    Serial.print(gTorqueLimitMa);
    Serial.print(" last_fault=");
    Serial.print(gLastStatus.fault);
    Serial.print(" last_state=");
    Serial.print(gLastStatus.state);
    Serial.print(" last_current_ma=");
    Serial.print(gLastStatus.currentMa);
    Serial.print(" hall=");
    Serial.println(gLastStatus.hall);

    Serial.print("kp=");
    Serial.print(gController.kp(), 4);
    Serial.print(" ki=");
    Serial.print(gController.ki(), 4);
    Serial.print(" kd=");
    Serial.print(gController.kd(), 4);
    Serial.print(" maxduty=");
    Serial.print(gController.maxDuty());
    Serial.print(" minduty=");
    Serial.print(gController.minimumDuty());
    Serial.print(" valpha=");
    Serial.print(gController.velocityAlpha(), 4);
    Serial.print(" dir=");
    Serial.print(gController.direction());
    Serial.print(" trace=");
    Serial.println(gRingTraceEnabled ? 1 : 0);

    Serial.print("surface_deg=");
    Serial.print(gProbeSurfaceDeg, 3);
    Serial.print(" home_deg=");
    Serial.print(gHomeTargetDeg, 3);
    Serial.print(" strike_deg=");
    Serial.println(gStrikeTargetDeg, 3);
}

void printTelemetry()
{
    Serial.print("telemetry ms=");
    Serial.print(millis());
    Serial.print(" armed=");
    Serial.print(gArmed ? 1 : 0);
    Serial.print(" pos_deg=");
    Serial.print(gController.positionDegrees(), 3);
    Serial.print(" target_deg=");
    Serial.print(gController.targetDegrees(), 3);
    Serial.print(" vel_dps=");
    Serial.print(gController.velocityDegreesPerSecond(), 3);
    Serial.print(" duty=");
    Serial.print(gController.dutyCommand());
    Serial.print(" current_ma=");
    Serial.print(gLastStatus.currentMa);
    Serial.print(" fault=");
    Serial.print(gLastStatus.fault);
    Serial.print(" enc_err=");
    Serial.print(gLastEncoderSample.errorFlag ? 1 : 0);
    Serial.print(" parity_err=");
    Serial.println(gLastEncoderSample.parityError ? 1 : 0);
}

bool enumerateRing(bool verbose)
{
    uint8_t deviceCount = 0;

    if (!gRing.enumerate(deviceCount)) {
        gRingReady = false;
        gDeviceCount = 0;
        if (verbose) {
            Serial.print("enumerate failed: ");
            Serial.println(gRing.lastErrorString());
        }
        return false;
    }

    gRingReady = (deviceCount > 0);
    gDeviceCount = deviceCount;
    clearDutyArray();

    if (verbose) {
        Serial.print("enumerated devices=");
        Serial.println(gDeviceCount);
        if (gDeviceCount > 1) {
            Serial.println("only actuator 0 will be driven; others stay at zero duty");
        }
    }

    return gRingReady;
}

bool readEncoderOnce()
{
    if (!gEncoders.readAngles()) {
        return false;
    }

    gLastEncoderSample = gEncoders.sample(0);
    if (gLastEncoderSample.parityError) {
        Serial.println("encoder parity error");
        return false;
    }

    gController.update(gLastEncoderSample.rawAngle, AppConfig::kControlDt);
    return true;
}

bool applyTorqueLimit()
{
    if (!gRingReady) {
        return false;
    }

    if (!gRing.setTorqueLimit(AppConfig::kControlledAddress, gTorqueLimitMa, &gLastStatus)) {
        Serial.print("setTorqueLimit failed: ");
        Serial.println(gRing.lastErrorString());
        return false;
    }

    return true;
}

bool sendZeroDuty()
{
    clearDutyArray();
    gAppliedDutyCommand = 0;

    if (!gRingReady || gDeviceCount == 0) {
        return true;
    }

    if (useAddressedDutyPath()) {
        if (!gRing.setDuty(AppConfig::kControlledAddress, 0, &gLastStatus)) {
            Serial.print("zero-duty setDuty failed: ");
            Serial.println(gRing.lastErrorString());
            return false;
        }
        return true;
    }

    if (!gRing.broadcastDuty(gDuties, gDeviceCount)) {
        Serial.print("zero-duty broadcast failed: ");
        Serial.println(gRing.lastErrorString());
        return false;
    }

    return true;
}

void disarmController(const char *reason)
{
    if (gArmed) {
        gArmed = false;
        gTapActive = false;
        gConsecutiveRingFailures = 0;
        sendZeroDuty();
        if (gRingReady) {
            gRing.stop(AppConfig::kControlledAddress, &gLastStatus);
        }
    }

    if (reason && reason[0] != '\0') {
        Serial.print("disarmed: ");
        Serial.println(reason);
    }
}

void armController()
{
    if (!gRingReady && !enumerateRing(true)) {
        return;
    }

    if (!readEncoderOnce()) {
        Serial.println("cannot arm: encoder read failed");
        return;
    }

    if (gLastEncoderSample.errorFlag) {
        Serial.println("cannot arm: encoder reported sensor error");
        return;
    }

    if (gLastStatus.fault != 0u) {
        if (!gRing.clearFault(AppConfig::kControlledAddress, &gLastStatus)) {
            Serial.print("clearFault failed (continuing): ");
            Serial.println(gRing.lastErrorString());
        }
    }

    if (!applyTorqueLimit()) {
        return;
    }

    gController.setTargetDegrees(gHomeTargetDeg);
    if (!sendZeroDuty()) {
        return;
    }
    gConsecutiveRingFailures = 0;
    gNextCommandAtUs = micros();
    gArmed = true;
    Serial.println("armed");
}

void maybePollStatus()
{
    const uint32_t nowMs = millis();

    if (!gRingReady || !timeReached(nowMs, gLastStatusPollAtMs + AppConfig::kStatusPollPeriodMs)) {
        return;
    }

    gLastStatusPollAtMs = nowMs;
    if (!gRing.queryStatus(AppConfig::kControlledAddress, gLastStatus)) {
        disarmController("status query failed");
        return;
    }

    if (gLastStatus.fault != 0u) {
        disarmController("motor fault");
    }
}

void runControlTick()
{
    if (!readEncoderOnce()) {
        disarmController("encoder read failed");
        return;
    }

    if (gLastEncoderSample.errorFlag) {
        disarmController("encoder sensor error");
        return;
    }

    if (gProbeActive) {
        const float kProbeStepDeg = 0.1f;
        const float kProbeErrorThresholdDeg = 3.0f;

        gController.setTargetDegrees(gController.targetDegrees() + kProbeStepDeg);
        if (fabsf(gController.targetDegrees() - gController.positionDegrees()) > kProbeErrorThresholdDeg) {
            gProbeSurfaceDeg = gController.positionDegrees();
            gProbeActive = false;
            gController.setTargetDegrees(gHomeTargetDeg);
            Serial.print("probe finished: surface_deg=");
            Serial.println(gProbeSurfaceDeg, 3);

            // Auto-set strike target to 2 degrees past the surface for a positive hit
            gStrikeTargetDeg = gProbeSurfaceDeg + 2.0f;
            Serial.print("auto-set strike_deg=");
            Serial.println(gStrikeTargetDeg, 3);
        }
    }

    if (gCaptureActive && gCaptureIndex < kCaptureBufferSize) {
        gCaptureBuffer[gCaptureIndex].pos = gController.positionDegrees();
        gCaptureBuffer[gCaptureIndex].vel = gController.velocityDegreesPerSecond();
        gCaptureBuffer[gCaptureIndex].duty = gAppliedDutyCommand;
        gCaptureIndex++;
        if (gCaptureIndex >= kCaptureBufferSize) {
            gCaptureActive = false;
        }
    }

    if (gStrikePulseActive) {
        if (timeReached(millis(), gStrikePulseEndMs)) {
            gStrikePulseActive = false;
            gController.setTargetDegrees(gHomeTargetDeg);
        }
    }

    if (gTapActive && timeReached(millis(), gTapReturnAtMs)) {
        gTapActive = false;
        gController.setTargetDegrees(gHomeTargetDeg);
    }

    if (!gArmed || !gRingReady || gDeviceCount == 0) {
        return;
    }

    int16_t targetDuty = filteredDutyCommand();
    if (gStrikePulseActive) {
        targetDuty = gStrikePulseDuty;
    }

    if (useAddressedDutyPath()) {
        const uint32_t nowUs = micros();
        int16_t duty;
        
        if (gStrikePulseActive) {
            // Sharper impact: reach full power in ~2-3ms
            const int16_t strikeStep = 300;
            if (targetDuty > gAppliedDutyCommand + strikeStep) duty = gAppliedDutyCommand + strikeStep;
            else if (targetDuty < gAppliedDutyCommand - strikeStep) duty = gAppliedDutyCommand - strikeStep;
            else duty = targetDuty;
        } else {
            duty = slewLimitedDutyCommand(targetDuty);
        }

        if (!timeReached(nowUs, gNextCommandAtUs)) {
            return;
        }

        gNextCommandAtUs = nowUs + AppConfig::kCommandPeriodUs;

        if (!gRing.setDuty(AppConfig::kControlledAddress, duty, &gLastStatus)) {
            gConsecutiveRingFailures++;
            Serial.print("setDuty failed: ");
            Serial.println(gRing.lastErrorString());
            if (gConsecutiveRingFailures >= AppConfig::kMaxConsecutiveCommFailures) {
                disarmController("set duty failed");
            }
            return;
        }

        gConsecutiveRingFailures = 0;
        gAppliedDutyCommand = duty;

        if (gLastStatus.fault != 0u) {
            disarmController("motor fault");
        }
        return;
    }

    clearDutyArray();
    gDuties[AppConfig::kControlledAddress] = targetDuty;

    if (!gRing.broadcastDuty(gDuties, gDeviceCount)) {
        gConsecutiveRingFailures++;
        Serial.print("broadcastDuty failed: ");
        Serial.println(gRing.lastErrorString());
        if (gConsecutiveRingFailures >= AppConfig::kMaxConsecutiveCommFailures) {
            disarmController("broadcast duty failed");
        }
        return;
    }

    gConsecutiveRingFailures = 0;
    maybePollStatus();
}

void handleTapCommand(char *arg)
{
    if (arg && arg[0] != '\0') {
        gTapHoldMs = static_cast<uint32_t>(strtoul(arg, nullptr, 10));
    }

    // Auto-trigger capture for analysis
    gCaptureIndex = 0;
    gCaptureActive = true;

    gController.setTargetDegrees(gStrikeTargetDeg);
    gTapReturnAtMs = millis() + gTapHoldMs;
    gTapActive = true;

    Serial.print("tap hold_ms=");
    Serial.println(gTapHoldMs);
}

void handleLine(char *line)
{
    char *context = nullptr;
    char *command = strtok_r(line, " \t", &context);

    if (!command) {
        return;
    }

    if (strcmp(command, "help") == 0) {
        printHelp();
        return;
    }

    if (strcmp(command, "status") == 0) {
        printRingStatus();
        printEncoderStatus();
        return;
    }

    if (strcmp(command, "enum") == 0) {
        enumerateRing(true);
        return;
    }

    if (strcmp(command, "arm") == 0) {
        armController();
        return;
    }

    if (strcmp(command, "disarm") == 0 || strcmp(command, "stop") == 0) {
        disarmController("operator request");
        return;
    }

    if (strcmp(command, "target") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: target <deg>");
            return;
        }

        gTapActive = false;
        gController.setTargetDegrees(strtof(arg, nullptr));
        Serial.print("target_deg=");
        Serial.println(gController.targetDegrees(), 3);
        return;
    }

    if (strcmp(command, "home") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: home <deg>");
            return;
        }

        gHomeTargetDeg = strtof(arg, nullptr);
        gController.setTargetDegrees(gHomeTargetDeg);
        gTapActive = false;
        Serial.print("home_deg=");
        Serial.println(gHomeTargetDeg, 3);
        return;
    }

    if (strcmp(command, "strike") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: strike <deg>");
            return;
        }

        gStrikeTargetDeg = strtof(arg, nullptr);
        Serial.print("strike_deg=");
        Serial.println(gStrikeTargetDeg, 3);
        return;
    }

    if (strcmp(command, "tap") == 0) {
        handleTapCommand(strtok_r(nullptr, " \t", &context));
        return;
    }

    if (strcmp(command, "zero") == 0) {
        if (!gController.initialized()) {
            Serial.println("zero failed: no encoder sample yet");
            return;
        }

        gController.zeroAtCurrentPosition();
        Serial.println("zero set to current position");
        return;
    }

    if (strcmp(command, "dir") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: dir <1|-1>");
            return;
        }

        gController.setDirection(static_cast<int8_t>(strtol(arg, nullptr, 10)));
        Serial.print("direction=");
        Serial.println(gController.direction());
        return;
    }

    if (strcmp(command, "gains") == 0) {
        char *kpArg = strtok_r(nullptr, " \t", &context);
        char *kdArg = strtok_r(nullptr, " \t", &context);
        if (!kpArg || !kdArg) {
            Serial.println("usage: gains <kp> <kd>");
            return;
        }

        gController.setGains(strtof(kpArg, nullptr), strtof(kdArg, nullptr));
        Serial.print("kp=");
        Serial.print(gController.kp(), 4);
        Serial.print(" ki=");
        Serial.print(gController.ki(), 4);
        Serial.print(" kd=");
        Serial.println(gController.kd(), 4);
        return;
    }

    if (strcmp(command, "ki") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: ki <gain>");
            return;
        }

        gController.setIntegralGain(strtof(arg, nullptr));
        Serial.print("ki=");
        Serial.println(gController.ki(), 4);
        return;
    }

    if (strcmp(command, "maxduty") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: maxduty <0..1200>");
            return;
        }

        gController.setMaxDuty(static_cast<int16_t>(strtol(arg, nullptr, 10)));
        Serial.print("maxduty=");
        Serial.println(gController.maxDuty());
        return;
    }

    if (strcmp(command, "minduty") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: minduty <0..500>");
            return;
        }

        gController.setMinimumDuty(static_cast<int16_t>(strtol(arg, nullptr, 10)));
        Serial.print("minduty=");
        Serial.print(static_cast<int16_t>(strtol(arg, nullptr, 10))); // Simple display
        return;
    }

    if (strcmp(command, "valpha") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: valpha <0.0..1.0>");
            return;
        }

        gController.setVelocityAlpha(strtof(arg, nullptr));
        Serial.print("valpha=");
        Serial.println(gController.velocityAlpha(), 4);
        return;
    }

    if (strcmp(command, "torque") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: torque <mA>");
            return;
        }

        gTorqueLimitMa = static_cast<uint16_t>(strtoul(arg, nullptr, 10));
        Serial.print("torque_ma=");
        Serial.println(gTorqueLimitMa);

        if (gRingReady) {
            applyTorqueLimit();
        }
        return;
    }

    if (strcmp(command, "gravity") == 0) {
        char *kgArg = strtok_r(nullptr, " \t", &context);
        char *offsetArg = strtok_r(nullptr, " \t", &context);
        if (!kgArg || !offsetArg) {
            Serial.println("usage: gravity <kg> <offset_deg>");
            return;
        }

        gController.setGravityGain(strtof(kgArg, nullptr), strtof(offsetArg, nullptr));
        Serial.print("kg=");
        Serial.print(gController.kg(), 4);
        Serial.print(" gravity_offset_deg=");
        Serial.println(gController.gravityOffsetDegrees(), 3);
        return;
    }

    if (strcmp(command, "probe") == 0) {
        if (!gArmed) {
            Serial.println("cannot probe: controller not armed");
            return;
        }

        gProbeActive = true;
        gController.setTargetDegrees(gController.positionDegrees());
        Serial.println("probing...");
        return;
    }

    if (strcmp(command, "setsurf") == 0) {
        gProbeSurfaceDeg = gController.positionDegrees();
        gStrikeTargetDeg = gProbeSurfaceDeg + 2.0f;
        Serial.print("surface set: surface_deg=");
        Serial.print(gProbeSurfaceDeg, 3);
        Serial.print(" strike_deg=");
        Serial.println(gStrikeTargetDeg, 3);
        return;
    }

    if (strcmp(command, "strike_pulse") == 0) {
        char *dutyArg = strtok_r(nullptr, " \t", &context);
        char *msArg = strtok_r(nullptr, " \t", &context);
        if (!dutyArg || !msArg) {
            Serial.println("usage: strike_pulse <duty> <ms>");
            return;
        }

        gStrikePulseDuty = static_cast<int16_t>(strtol(dutyArg, nullptr, 10));
        gStrikePulseEndMs = millis() + static_cast<uint32_t>(strtoul(msArg, nullptr, 10));
        gStrikePulseActive = true;
        gTapActive = false;

        // Auto-trigger capture
        gCaptureIndex = 0;
        gCaptureActive = true;

        Serial.print("strike_pulse duty=");
        Serial.print(gStrikePulseDuty);
        Serial.print(" ms=");
        Serial.println(msArg);
        return;
    }

    if (strcmp(command, "capture") == 0) {
        gCaptureIndex = 0;
        gCaptureActive = true;
        Serial.println("capture started");
        return;
    }

    if (strcmp(command, "dump") == 0) {
        Serial.println("--- capture dump start ---");
        for (size_t i = 0; i < gCaptureIndex; i++) {
            Serial.print(i);
            Serial.print(",");
            Serial.print(gCaptureBuffer[i].pos, 4);
            Serial.print(",");
            Serial.print(gCaptureBuffer[i].vel, 4);
            Serial.print(",");
            Serial.println(gCaptureBuffer[i].duty);
        }
        Serial.println("--- capture dump end ---");
        return;
    }

    if (strcmp(command, "strike_ana") == 0) {
        if (gCaptureIndex < 10) {
            Serial.println("no capture data to analyze");
            return;
        }

        float startPos = gCaptureBuffer[0].pos;
        float peakPos = startPos;
        size_t peakIdx = 0;
        float peakVel = 0;

        // Use strike duty to determine which direction we are looking for the peak
        bool positiveStrike = (gStrikePulseDuty >= 0);

        for (size_t i = 0; i < gCaptureIndex; i++) {
            // Track peak velocity for characterization
            if (fabsf(gCaptureBuffer[i].vel) > fabsf(peakVel)) {
                peakVel = gCaptureBuffer[i].vel;
            }

            // Find extreme position (the physical surface)
            if (positiveStrike) {
                if (gCaptureBuffer[i].pos > peakPos) {
                    peakPos = gCaptureBuffer[i].pos;
                    peakIdx = i;
                }
            } else {
                if (gCaptureBuffer[i].pos < peakPos) {
                    peakPos = gCaptureBuffer[i].pos;
                    peakIdx = i;
                }
            }
        }

        Serial.println("--- peak-position strike analysis ---");
        Serial.print("start_pos="); Serial.println(startPos, 3);
        Serial.print("peak_vel_dps="); Serial.println(peakVel, 3);
        Serial.print("impact_ms="); Serial.println(peakIdx);
        Serial.print("impact_pos="); Serial.println(peakPos, 3);
        Serial.print("travel_dist="); Serial.println(fabsf(peakPos - startPos), 3);
        
        return;
    }

    if (strcmp(command, "stream") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: stream <0|1>");
            return;
        }

        gStreamTelemetry = (strtol(arg, nullptr, 10) != 0);
        Serial.print("stream=");
        Serial.println(gStreamTelemetry ? 1 : 0);
        return;
    }

    if (strcmp(command, "trace") == 0) {
        char *arg = strtok_r(nullptr, " \t", &context);
        if (!arg) {
            Serial.println("usage: trace <0|1>");
            return;
        }

        gRingTraceEnabled = (strtol(arg, nullptr, 10) != 0);
        applyRingTraceSetting();
        Serial.print("trace=");
        Serial.println(gRingTraceEnabled ? 1 : 0);
        return;
    }

    Serial.print("unknown command: ");
    Serial.println(command);
}

void serviceUsbSerial()
{
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value < 0) {
            return;
        }

        const char c = static_cast<char>(value);
        if (c == '\r' || c == '\n') {
            if (gLineLength == 0) {
                continue;
            }

            gLineBuffer[gLineLength] = '\0';
            handleLine(gLineBuffer);
            gLineLength = 0;
            continue;
        }

        if (gLineLength < (sizeof(gLineBuffer) - 1u)) {
            gLineBuffer[gLineLength++] = c;
        }
    }
}

}  // namespace

void setup()
{
    clearDutyArray();
    memset(&gLastStatus, 0, sizeof(gLastStatus));
    //pinMode(10, OUTPUT);
    //digitalWrite(10, HIGH);
    Serial.begin(AppConfig::kUsbBaud);
    while(!Serial);
    Serial.println("mallet servo demo");
    delay(500);

    //Serial1.setTX(AppConfig::kRingTxPin);
    //Serial1.setRX(AppConfig::kRingRxPin);
    //Serial1.begin(AppConfig::kRingBaud);
    target_uart.begin(AppConfig::kRingBaud);
    gRing.setTimeoutMs(20);
    applyRingTraceSetting();
    gEncoders.begin(AppConfig::kEncoderCount);

    gController.setGains(AppConfig::kDefaultKp, AppConfig::kDefaultKd);
    gController.setIntegralGain(AppConfig::kDefaultKi);
    gController.setMaxDuty(AppConfig::kDefaultMaxDuty);
    gController.setVelocityAlpha(AppConfig::kDefaultVelocityAlpha);
    gController.setTargetDegrees(gHomeTargetDeg);

    readEncoderOnce();
    enumerateRing(true);

    gNextControlAtUs = micros() + AppConfig::kControlPeriodUs;
    gNextCommandAtUs = micros();

    printBanner();
}

void loop()
{
    serviceUsbSerial();

    const uint32_t nowUs = micros();
    if (timeReached(nowUs, gNextControlAtUs)) {
        gNextControlAtUs = nowUs + AppConfig::kControlPeriodUs;
        runControlTick();
    }

    const uint32_t nowMs = millis();
    if (gStreamTelemetry && timeReached(nowMs, gLastTelemetryAtMs + AppConfig::kTelemetryPeriodMs)) {
        gLastTelemetryAtMs = nowMs;
        printTelemetry();
    }
}
