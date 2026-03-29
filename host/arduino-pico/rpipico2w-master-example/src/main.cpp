#include <Arduino.h>
#include <BL4818RingMaster.h>
#include <string.h>

BL4818RingMaster ring(Serial1);

constexpr uint8_t kRingTxPin = 4;
constexpr uint8_t kRingRxPin = 5;
constexpr uint32_t kRingBaud = 250000;
constexpr uint16_t kTorqueLimitMa = 2500;

int16_t duties[BL4818RingMaster::kMaxDevices];
uint8_t deviceCount = 0;

static void logStatus(uint8_t address, const BL4818RingMaster::Status &status)
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
            logStatus(address, status);
        }
    }
}

void loop()
{
    static uint32_t lastControlMs = 0;
    static uint32_t lastSwingMs = 0;
    static bool positive = true;

    if (deviceCount == 0) {
        delay(100);
        return;
    }

    if (millis() - lastSwingMs >= 250) {
        lastSwingMs = millis();
        positive = !positive;
        for (uint8_t address = 0; address < deviceCount; ++address) {
            duties[address] = positive ? 500 : -500;
        }
    }

    if (millis() - lastControlMs < 2) {
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
        logStatus(address, status);
    }
}
