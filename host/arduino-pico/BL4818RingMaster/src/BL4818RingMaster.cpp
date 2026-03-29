#include "BL4818RingMaster.h"

#include <string.h>

BL4818RingMaster::BL4818RingMaster(Stream &io)
    : io_(io),
      timeoutMs_(20),
      lastError_(Error::None),
      deviceCount_(0),
      nextQueryAddress_(0),
      enumerated_(false)
{
}

void BL4818RingMaster::setTimeoutMs(uint32_t timeoutMs)
{
    timeoutMs_ = timeoutMs;
}

uint32_t BL4818RingMaster::timeoutMs() const
{
    return timeoutMs_;
}

bool BL4818RingMaster::isEnumerated() const
{
    return enumerated_;
}

uint8_t BL4818RingMaster::deviceCount() const
{
    return deviceCount_;
}

BL4818RingMaster::Error BL4818RingMaster::lastError() const
{
    return lastError_;
}

const char *BL4818RingMaster::lastErrorString() const
{
    return errorString(lastError_);
}

const char *BL4818RingMaster::errorString(Error error)
{
    switch (error) {
    case Error::None:
        return "ok";
    case Error::InvalidArgument:
        return "invalid argument";
    case Error::NotEnumerated:
        return "ring not enumerated";
    case Error::NoDevices:
        return "no devices in ring";
    case Error::ShortWrite:
        return "short serial write";
    case Error::Timeout:
        return "serial timeout";
    case Error::BadResponse:
        return "unexpected response";
    default:
        return "unknown error";
    }
}

void BL4818RingMaster::clearInput()
{
    while (io_.available() > 0) {
        io_.read();
    }
}

bool BL4818RingMaster::enumerate(uint8_t &deviceCount)
{
    uint8_t packet[3] = {kSyncEnumerate, 0x00, 0x00};
    uint8_t response[3];

    packet[2] = crc8(packet, 2);

    clearInput();

    if (!writeExact(packet, sizeof(packet))) {
        return false;
    }

    if (!readExact(response, sizeof(response))) {
        return false;
    }

    if (response[0] != kSyncEnumerate || response[2] != crc8(response, 2)) {
        setError(Error::BadResponse);
        return false;
    }

    deviceCount = response[1];
    deviceCount_ = deviceCount;
    nextQueryAddress_ = 0;
    enumerated_ = true;
    setError(Error::None);
    return true;
}

bool BL4818RingMaster::broadcastDuty(const int16_t *duties, uint8_t count)
{
    uint8_t packet[kMaxBroadcastFrameSize];
    uint8_t response[3];
    uint8_t index;

    if (!requireEnumerated()) {
        return false;
    }

    if (!duties || count == 0 || count > kMaxDevices || count != deviceCount_) {
        setError(Error::InvalidArgument);
        return false;
    }

    packet[0] = kSyncBroadcast;
    packet[1] = count;

    for (index = 0; index < count; ++index) {
        uint16_t rawDuty = static_cast<uint16_t>(duties[index]);
        packet[2 + (2 * index)] = static_cast<uint8_t>(rawDuty >> 8);
        packet[3 + (2 * index)] = static_cast<uint8_t>(rawDuty & 0xFF);
    }

    packet[2 + (2 * count)] = crc8(packet, static_cast<size_t>(2 + (2 * count)));

    clearInput();

    if (!writeExact(packet, static_cast<size_t>(3 + (2 * count)))) {
        return false;
    }

    if (!readExact(response, sizeof(response))) {
        return false;
    }

    if (response[0] != kSyncBroadcast || response[1] != 0x00 || response[2] != crc8(response, 2)) {
        setError(Error::BadResponse);
        return false;
    }

    setError(Error::None);
    return true;
}

bool BL4818RingMaster::setDuty(uint8_t address, int16_t duty, Status *status)
{
    uint8_t payload[2];
    uint16_t rawDuty = static_cast<uint16_t>(duty);

    payload[0] = static_cast<uint8_t>(rawDuty >> 8);
    payload[1] = static_cast<uint8_t>(rawDuty & 0xFF);
    return sendAddressed(address, Command::SetDuty, payload, sizeof(payload), status);
}

bool BL4818RingMaster::setTorqueLimit(uint8_t address, uint16_t torqueLimitMa, Status *status)
{
    uint8_t payload[2];

    payload[0] = static_cast<uint8_t>(torqueLimitMa >> 8);
    payload[1] = static_cast<uint8_t>(torqueLimitMa & 0xFF);
    return sendAddressed(address, Command::SetTorqueLimit, payload, sizeof(payload), status);
}

bool BL4818RingMaster::stop(uint8_t address, Status *status)
{
    return sendAddressed(address, Command::Stop, nullptr, 0, status);
}

bool BL4818RingMaster::clearFault(uint8_t address, Status *status)
{
    return sendAddressed(address, Command::ClearFault, nullptr, 0, status);
}

bool BL4818RingMaster::queryStatus(uint8_t address, Status &status)
{
    return sendAddressed(address, Command::QueryStatus, nullptr, 0, &status);
}

bool BL4818RingMaster::queryNextStatus(uint8_t &address, Status &status)
{
    if (!requireEnumerated()) {
        return false;
    }

    address = nextQueryAddress_;
    nextQueryAddress_ = static_cast<uint8_t>((nextQueryAddress_ + 1) % deviceCount_);
    return queryStatus(address, status);
}

void BL4818RingMaster::setError(Error error)
{
    lastError_ = error;
}

bool BL4818RingMaster::requireEnumerated()
{
    if (!enumerated_) {
        setError(Error::NotEnumerated);
        return false;
    }

    if (deviceCount_ == 0) {
        setError(Error::NoDevices);
        return false;
    }

    return true;
}

bool BL4818RingMaster::validateAddress(uint8_t address) const
{
    return enumerated_ && address < deviceCount_;
}

bool BL4818RingMaster::writeExact(const uint8_t *data, size_t length)
{
    if (io_.write(data, length) != length) {
        setError(Error::ShortWrite);
        return false;
    }

    return true;
}

bool BL4818RingMaster::readExact(uint8_t *data, size_t length)
{
    size_t received = 0;
    uint32_t lastByteAt = millis();

    while (received < length) {
        if (io_.available() > 0) {
            int value = io_.read();
            if (value >= 0) {
                data[received++] = static_cast<uint8_t>(value);
                lastByteAt = millis();
            }
            continue;
        }

        if (static_cast<uint32_t>(millis() - lastByteAt) >= timeoutMs_) {
            setError(Error::Timeout);
            return false;
        }

        yield();
    }

    return true;
}

uint8_t BL4818RingMaster::crc8(const uint8_t *data, size_t length) const
{
    uint8_t crc = 0;

    while (length-- > 0) {
        uint8_t bit;

        crc ^= *data++;
        for (bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

bool BL4818RingMaster::readStatus(Status &status)
{
    uint8_t frame[kStatusFrameSize];

    if (!readExact(frame, sizeof(frame))) {
        return false;
    }

    if (frame[0] != kSyncStatus || frame[6] != crc8(frame, 6)) {
        setError(Error::BadResponse);
        return false;
    }

    status.state = frame[1];
    status.fault = frame[2];
    status.currentMa = static_cast<uint16_t>((static_cast<uint16_t>(frame[3]) << 8) | frame[4]);
    status.hall = frame[5];
    setError(Error::None);
    return true;
}

bool BL4818RingMaster::sendAddressed(uint8_t address, Command command, const uint8_t *payload, uint8_t payloadLength, Status *status)
{
    uint8_t packet[5];
    uint8_t packetLength = static_cast<uint8_t>(3 + payloadLength);
    Status localStatus;

    if (!requireEnumerated()) {
        return false;
    }

    if (!validateAddress(address) || payloadLength > 2) {
        setError(Error::InvalidArgument);
        return false;
    }

    packet[0] = static_cast<uint8_t>(kSyncAddressBase | address);
    packet[1] = static_cast<uint8_t>(command);

    if (payloadLength > 0) {
        memcpy(&packet[2], payload, payloadLength);
    }

    packet[2 + payloadLength] = crc8(packet, static_cast<size_t>(2 + payloadLength));

    clearInput();

    if (!writeExact(packet, packetLength)) {
        return false;
    }

    if (!readStatus(localStatus)) {
        return false;
    }

    if (status) {
        *status = localStatus;
    }

    return true;
}
