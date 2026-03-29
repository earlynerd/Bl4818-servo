#include <Arduino.h>
#include <BL4818RingMaster.h>
#include <string.h>

BL4818RingMaster ring(Serial1);

constexpr uint8_t kRingTxPin = 4;
constexpr uint8_t kRingRxPin = 5;
constexpr uint32_t kRingBaud = 250000;

constexpr uint32_t kControlPeriodMs = 2;
constexpr uint16_t kTorqueLimitMa = 2500;

int16_t duties[BL4818RingMaster::kMaxDevices];
uint8_t deviceCount = 0;

static void printStatus(uint8_t address, const BL4818RingMaster::Status &status)
{
    Serial.print("addr=");
    Serial.print(address);
    Serial.print(" state=");
    Serial.print(status.state);
    Serial.print(" fault=");
    Serial.print(status.fault);
    Serial.print(" current_ma=");
    Serial.print(status.currentMa);
    Serial.print(" hall=");
    Serial.println(status.hall);
}

void setup()
{
    memset(duties, 0, sizeof(duties));

    Serial.begin(115200);
    delay(500);

    Serial1.setTX(kRingTxPin);
    Serial1.setRX(kRingRxPin);
    Serial1.begin(kRingBaud);

    ring.setTimeoutMs(20);

    if (!ring.enumerate(deviceCount)) {
        Serial.print("enumerate failed: ");
        Serial.println(ring.lastErrorString());
        return;
    }

    Serial.print("devices=");
    Serial.println(deviceCount);

    for (uint8_t address = 0; address < deviceCount; ++address) {
        BL4818RingMaster::Status status;
        if (ring.setTorqueLimit(address, kTorqueLimitMa, &status)) {
            printStatus(address, status);
        }
    }
}

void loop()
{
    static uint32_t lastControlMs = 0;
    static uint32_t lastMotionMs = 0;
    static uint8_t phase = 0;

    if (deviceCount == 0) {
        delay(100);
        return;
    }

    if (millis() - lastMotionMs >= 250) {
        lastMotionMs = millis();
        phase = static_cast<uint8_t>((phase + 1) & 0x03);

        for (uint8_t i = 0; i < deviceCount; ++i) {
            switch (phase) {
            case 0:
                duties[i] = 500;
                break;
            case 1:
                duties[i] = 0;
                break;
            case 2:
                duties[i] = -500;
                break;
            default:
                duties[i] = 0;
                break;
            }
        }
    }

    if (millis() - lastControlMs < kControlPeriodMs) {
        return;
    }
    lastControlMs = millis();

    if (!ring.broadcastDuty(duties, deviceCount)) {
        Serial.print("broadcast failed: ");
        Serial.println(ring.lastErrorString());
        return;
    }

    BL4818RingMaster::Status status;
    uint8_t address = 0;

    if (ring.queryNextStatus(address, status)) {
        printStatus(address, status);
    } else {
        Serial.print("status failed: ");
        Serial.println(ring.lastErrorString());
    }
}
