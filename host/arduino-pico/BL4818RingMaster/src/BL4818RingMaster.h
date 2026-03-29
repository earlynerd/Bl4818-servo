#pragma once

#include <Arduino.h>

class BL4818RingMaster {
public:
    static constexpr uint8_t kMaxDevices = 16;
    static constexpr uint8_t kSyncStatus = 0x7E;
    static constexpr uint8_t kSyncEnumerate = 0x7F;
    static constexpr uint8_t kSyncAddressBase = 0x80;
    static constexpr uint8_t kSyncBroadcast = 0xFF;

    enum class Error : uint8_t {
        None = 0,
        InvalidArgument,
        NotEnumerated,
        NoDevices,
        ShortWrite,
        Timeout,
        BadResponse,
    };

    enum class Command : uint8_t {
        SetDuty = 0x01,
        SetTorqueLimit = 0x02,
        Stop = 0x03,
        ClearFault = 0x04,
        QueryStatus = 0x10,
    };

    struct Status {
        uint8_t state = 0;
        uint8_t fault = 0;
        uint16_t currentMa = 0;
        uint8_t hall = 0;
    };

    explicit BL4818RingMaster(Stream &io);

    void setTimeoutMs(uint32_t timeoutMs);
    uint32_t timeoutMs() const;

    bool isEnumerated() const;
    uint8_t deviceCount() const;

    Error lastError() const;
    const char *lastErrorString() const;
    static const char *errorString(Error error);

    void clearInput();

    bool enumerate(uint8_t &deviceCount);

    bool broadcastDuty(const int16_t *duties, uint8_t count);

    bool setDuty(uint8_t address, int16_t duty, Status *status = nullptr);
    bool setTorqueLimit(uint8_t address, uint16_t torqueLimitMa, Status *status = nullptr);
    bool stop(uint8_t address, Status *status = nullptr);
    bool clearFault(uint8_t address, Status *status = nullptr);
    bool queryStatus(uint8_t address, Status &status);
    bool queryNextStatus(uint8_t &address, Status &status);

private:
    static constexpr uint8_t kBroadcastHeaderSize = 2;
    static constexpr uint8_t kStatusFrameSize = 6;
    static constexpr uint8_t kMaxBroadcastFrameSize = kBroadcastHeaderSize + (2 * kMaxDevices);

    Stream &io_;
    uint32_t timeoutMs_;
    Error lastError_;
    uint8_t deviceCount_;
    uint8_t nextQueryAddress_;
    bool enumerated_;

    void setError(Error error);
    bool requireEnumerated();
    bool validateAddress(uint8_t address) const;
    bool writeExact(const uint8_t *data, size_t length);
    bool readExact(uint8_t *data, size_t length);
    bool readStatus(Status &status);
    bool sendAddressed(uint8_t address, Command command, const uint8_t *payload, uint8_t payloadLength, Status *status);
};
