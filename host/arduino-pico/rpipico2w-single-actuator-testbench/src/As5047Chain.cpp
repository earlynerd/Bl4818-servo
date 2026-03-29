#include "As5047Chain.h"

#include "AppConfig.h"

As5047Chain::As5047Chain(SPIClassRP2040 &spi, uint8_t csPin)
    : spi_(spi),
      csPin_(csPin),
      encoderCount_(0)
{
}

void As5047Chain::begin(uint8_t encoderCount)
{
    if (encoderCount == 0) {
        encoderCount = 1;
    } else if (encoderCount > kMaxEncoders) {
        encoderCount = kMaxEncoders;
    }

    encoderCount_ = encoderCount;

    pinMode(csPin_, OUTPUT);
    digitalWrite(csPin_, HIGH);

    spi_.setRX(AppConfig::kSpiMisoPin);
    spi_.setSCK(AppConfig::kSpiSckPin);
    spi_.setTX(AppConfig::kSpiMosiPin);
    spi_.begin();

    for (uint8_t index = 0; index < encoderCount_; ++index) {
        txWords_[index] = kReadAngleCommand;
        rxWords_[index] = 0;
        samples_[index] = {};
    }
}

uint8_t As5047Chain::encoderCount() const
{
    return encoderCount_;
}

bool As5047Chain::readAngles()
{
    if (encoderCount_ == 0) {
        return false;
    }

    transferWords(txWords_, nullptr);
    transferWords(txWords_, rxWords_);

    for (uint8_t index = 0; index < encoderCount_; ++index) {
        const uint16_t raw = rxWords_[index];
        samples_[index].rawFrame = raw;
        samples_[index].rawAngle = static_cast<uint16_t>(raw & kAngleMask);
        samples_[index].errorFlag = (raw & kErrorFlag) != 0u;
        samples_[index].parityError = !parityOk(raw);
    }

    return true;
}

const As5047Sample &As5047Chain::sample(uint8_t index) const
{
    return samples_[index];
}

uint16_t As5047Chain::buildReadCommand(uint16_t regAddr)
{
    uint16_t cmd = static_cast<uint16_t>(kReadBit | (regAddr & kAngleMask));
    uint16_t folded = cmd;

    folded ^= static_cast<uint16_t>(folded >> 8);
    folded ^= static_cast<uint16_t>(folded >> 4);
    folded ^= static_cast<uint16_t>(folded >> 2);
    folded ^= static_cast<uint16_t>(folded >> 1);

    if (folded & 0x0001u) {
        cmd = static_cast<uint16_t>(cmd | kParityBit);
    }

    return cmd;
}

bool As5047Chain::parityOk(uint16_t word)
{
    uint16_t folded = word;

    folded ^= static_cast<uint16_t>(folded >> 8);
    folded ^= static_cast<uint16_t>(folded >> 4);
    folded ^= static_cast<uint16_t>(folded >> 2);
    folded ^= static_cast<uint16_t>(folded >> 1);

    return (folded & 0x0001u) == 0u;
}

void As5047Chain::transferWords(const uint16_t *txWords, uint16_t *rxWords)
{
    SPISettings settings(AppConfig::kSpiClockHz, MSBFIRST, AppConfig::kSpiMode);

    spi_.beginTransaction(settings);
    digitalWrite(csPin_, LOW);

    /*
     * Commands must be shifted from the far end of the chain back toward the
     * MCU so logical index 0 maps to the first encoder on MOSI.
     */
    for (int8_t index = static_cast<int8_t>(encoderCount_) - 1; index >= 0; --index) {
        const uint16_t txWord = txWords ? txWords[index] : buildReadCommand(kAngleRegister);
        const uint16_t rxWord = spi_.transfer16(txWord);
        if (rxWords) {
            rxWords[index] = rxWord;
        }
    }

    digitalWrite(csPin_, HIGH);
    spi_.endTransaction();
}
