#include <Arduino.h>
#include <SPI.h>
#include <BL4818RingMaster.h>
#include <stdlib.h>
#include <string.h>

#include "AppConfig.h"
#include "As5047Chain.h"
#include "SingleActuatorController.h"

namespace {

BL4818RingMaster gRing(Serial1);
As5047Chain gEncoders(SPI, AppConfig::kSpiCsPin);
SingleActuatorController gController;

int16_t gDuties[BL4818RingMaster::kMaxDevices];
BL4818RingMaster::Status gLastStatus;
As5047Sample gLastEncoderSample;

uint8_t gDeviceCount = 0;
uint16_t gTorqueLimitMa = AppConfig::kDefaultTorqueLimitMa;

bool gRingReady = false;
bool gArmed = false;
bool gStreamTelemetry = false;
bool gTapActive = false;

float gHomeTargetDeg = AppConfig::kDefaultHomeTargetDeg;
float gStrikeTargetDeg = AppConfig::kDefaultStrikeTargetDeg;
uint32_t gTapHoldMs = AppConfig::kDefaultTapHoldMs;

uint32_t gNextControlAtUs = 0;
uint32_t gLastStatusPollAtMs = 0;
uint32_t gLastTelemetryAtMs = 0;
uint32_t gTapReturnAtMs = 0;

char gLineBuffer[96];
uint8_t gLineLength = 0;

bool timeReached(uint32_t now, uint32_t target)
{
    return static_cast<int32_t>(now - target) >= 0;
}

void clearDutyArray()
{
    memset(gDuties, 0, sizeof(gDuties));
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
    Serial.println("  maxduty <0..1200>");
    Serial.println("  torque <mA>");
    Serial.println("  stream <0|1>");
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
    Serial.print(" kd=");
    Serial.print(gController.kd(), 4);
    Serial.print(" maxduty=");
    Serial.print(gController.maxDuty());
    Serial.print(" dir=");
    Serial.println(gController.direction());
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

void sendZeroDuty()
{
    clearDutyArray();

    if (!gRingReady || gDeviceCount == 0) {
        return;
    }

    if (!gRing.broadcastDuty(gDuties, gDeviceCount)) {
        Serial.print("zero-duty broadcast failed: ");
        Serial.println(gRing.lastErrorString());
    }
}

void disarmController(const char *reason)
{
    if (gArmed) {
        gArmed = false;
        gTapActive = false;
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

    if (!gRing.clearFault(AppConfig::kControlledAddress, &gLastStatus)) {
        Serial.print("clearFault failed: ");
        Serial.println(gRing.lastErrorString());
        return;
    }

    if (!applyTorqueLimit()) {
        return;
    }

    gController.setTargetDegrees(gHomeTargetDeg);
    sendZeroDuty();
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

    if (gTapActive && timeReached(millis(), gTapReturnAtMs)) {
        gTapActive = false;
        gController.setTargetDegrees(gHomeTargetDeg);
    }

    if (!gArmed || !gRingReady || gDeviceCount == 0) {
        return;
    }

    clearDutyArray();
    gDuties[AppConfig::kControlledAddress] = gController.dutyCommand();

    if (!gRing.broadcastDuty(gDuties, gDeviceCount)) {
        disarmController("broadcast duty failed");
        return;
    }

    maybePollStatus();
}

void handleTapCommand(char *arg)
{
    if (arg && arg[0] != '\0') {
        gTapHoldMs = static_cast<uint32_t>(strtoul(arg, nullptr, 10));
    }

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
        Serial.print(" kd=");
        Serial.println(gController.kd(), 4);
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

    Serial.begin(AppConfig::kUsbBaud);
    delay(500);

    Serial1.setTX(AppConfig::kRingTxPin);
    Serial1.setRX(AppConfig::kRingRxPin);
    Serial1.begin(AppConfig::kRingBaud);

    gRing.setTimeoutMs(20);
    gEncoders.begin(AppConfig::kEncoderCount);

    gController.setGains(AppConfig::kDefaultKp, AppConfig::kDefaultKd);
    gController.setMaxDuty(AppConfig::kDefaultMaxDuty);
    gController.setVelocityAlpha(AppConfig::kDefaultVelocityAlpha);
    gController.setTargetDegrees(gHomeTargetDeg);

    readEncoderOnce();
    enumerateRing(true);

    gNextControlAtUs = micros() + AppConfig::kControlPeriodUs;

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
