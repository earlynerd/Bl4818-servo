#pragma once

#include <Arduino.h>
#include <SPI.h>

struct As5047Sample {
    uint16_t rawFrame = 0;
    uint16_t rawAngle = 0;
    bool errorFlag = false;
    bool parityError = false;
};

class As5047Chain {
public:
    static constexpr uint8_t kMaxEncoders = 16;

    As5047Chain(SPIClassRP2040 &spi, uint8_t csPin);

    void begin(uint8_t encoderCount);
    uint8_t encoderCount() const;

    bool readAngles();
    const As5047Sample &sample(uint8_t index) const;

    static uint16_t buildReadCommand(uint16_t regAddr);
    static bool parityOk(uint16_t word);

private:
    static constexpr uint16_t kAngleMask = 0x3FFFu;
    static constexpr uint16_t kErrorFlag = 0x4000u;
    static constexpr uint16_t kParityBit = 0x8000u;
    static constexpr uint16_t kReadBit = 0x4000u;
    static constexpr uint16_t kAngleRegister = 0x3FFFu;
    static constexpr uint16_t kReadAngleCommand = 0xFFFFu;

    void transferWords(const uint16_t *txWords, uint16_t *rxWords);

    SPIClassRP2040 &spi_;
    uint8_t csPin_;
    uint8_t encoderCount_;
    uint16_t txWords_[kMaxEncoders];
    uint16_t rxWords_[kMaxEncoders];
    As5047Sample samples_[kMaxEncoders];
};
