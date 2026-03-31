#include "BL4818RingMaster.h"

#include <string.h>

BL4818RingMaster::BL4818RingMaster(Stream &io)
    : io_(io),
      timeoutMs_(20),
      trace_(nullptr),
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

void BL4818RingMaster::setTraceStream(Stream *trace)
{
    trace_ = trace;
}

bool BL4818RingMaster::traceEnabled() const
{
    return trace_ != nullptr;
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
    uint8_t discarded[32];
    size_t discardedCount = 0;
    bool truncated = false;

    while (io_.available() > 0) {
        int value = io_.read();
        if (value < 0) {
            continue;
        }

        if (discardedCount < sizeof(discarded)) {
            discarded[discardedCount++] = static_cast<uint8_t>(value);
        } else {
            truncated = true;
        }
    }

    if (discardedCount > 0 || truncated) {
        traceBytes("drop", discarded, discardedCount, truncated);
    }
}

void BL4818RingMaster::traceBytes(const char *label, const uint8_t *data, size_t length, bool truncated)
{
    if (!trace_) {
        return;
    }

    trace_->print("ring ");
    trace_->print(label);
    trace_->print(':');

    for (size_t index = 0; index < length; ++index) {
        trace_->print(' ');
        if (data[index] < 0x10u) {
            trace_->print('0');
        }
        trace_->print(data[index], HEX);
    }

    if (truncated) {
        trace_->print(" ...");
    }

    trace_->println();
}

bool BL4818RingMaster::enumerate(uint8_t &deviceCount)
{
    uint8_t packet[3] = {kSyncEnumerate, 0x00, 0x00};
    uint8_t response[3];
    static const uint8_t kEnumeratePrefix[] = {kSyncEnumerate};

    packet[2] = crc8(packet, 2);

    clearInput();

    if (!writeExact(packet, sizeof(packet))) {
        return false;
    }

    if (!readValidFrame(response, sizeof(response), kEnumeratePrefix, sizeof(kEnumeratePrefix))) {
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
    static const uint8_t kBroadcastPrefix[] = {kSyncBroadcast, 0x00};
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

    if (!readValidFrame(response, sizeof(response), kBroadcastPrefix, sizeof(kBroadcastPrefix))) {
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
    traceBytes("tx", data, length);

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
            traceBytes("rx-timeout", data, received);
            setError(Error::Timeout);
            return false;
        }

        yield();
    }

    traceBytes("rx", data, received);
    return true;
}

bool BL4818RingMaster::readValidFrame(uint8_t *data, size_t length, const uint8_t *prefix, size_t prefixLength)
{
    size_t buffered = 0;
    uint8_t observed[64];
    size_t observedCount = 0;
    bool observedTruncated = false;
    uint32_t lastByteAt = millis();

    if (!data || length == 0 || !prefix || prefixLength == 0 || prefixLength >= length) {
        setError(Error::InvalidArgument);
        return false;
    }

    while (true) {
        if (io_.available() > 0) {
            int value = io_.read();
            if (value >= 0) {
                if (observedCount < sizeof(observed)) {
                    observed[observedCount++] = static_cast<uint8_t>(value);
                } else {
                    observedTruncated = true;
                }

                if (buffered < length) {
                    data[buffered++] = static_cast<uint8_t>(value);
                } else {
                    memmove(data, data + 1, length - 1);
                    data[length - 1] = static_cast<uint8_t>(value);
                }

                lastByteAt = millis();

                if (buffered >= length &&
                    memcmp(data, prefix, prefixLength) == 0 &&
                    data[length - 1] == crc8(data, length - 1)) {
                    traceBytes("rx", observed, observedCount, observedTruncated);
                    setError(Error::None);
                    return true;
                }
            }
            continue;
        }

        if (static_cast<uint32_t>(millis() - lastByteAt) >= timeoutMs_) {
            traceBytes("rx-timeout", observed, observedCount, observedTruncated);
            setError(Error::Timeout);
            return false;
        }

        yield();
    }
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
    static const uint8_t kStatusPrefix[] = {kSyncStatus};

    if (!readValidFrame(frame, sizeof(frame), kStatusPrefix, sizeof(kStatusPrefix))) {
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
