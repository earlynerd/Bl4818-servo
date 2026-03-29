# BL4818RingMaster

Master-side Arduino library for the BL4818 UART ring protocol.

This library targets RP2040/RP2350 boards running the Arduino-Pico core under
PlatformIO. The intended use is:

1. Enumerate the BL4818 ring once at boot.
2. Send one broadcast duty packet each control cycle.
3. Poll one device status per cycle in round-robin order.

Every production frame includes a CRC-8 trailer, and broadcast packets are only
consumed after enumeration.

## PlatformIO

For a Pico 2 W project, use the exact PlatformIO board ID `rpipico2w`.

Example `platformio.ini`:

```ini
[env:rpipico2w]
platform = raspberrypi
board = rpipico2w
framework = arduino
board_build.core = earlephilhower
monitor_speed = 115200
lib_extra_dirs = ..
```

If you vendor this library into your own project's `lib/` directory instead,
you do not need `lib_extra_dirs`.

## API

```cpp
#include <BL4818RingMaster.h>

BL4818RingMaster ring(Serial1);
```

Main calls:

- `enumerate(deviceCount)`
- `broadcastDuty(duties, deviceCount)`
- `queryStatus(address, status)`
- `queryNextStatus(address, status)`
- `setDuty(address, duty, &status)`
- `setTorqueLimit(address, limitMa, &status)`
- `stop(address, &status)`
- `clearFault(address, &status)`

Every addressed command returns the device's fixed-length status frame with CRC.

Current production firmware default:

- Ring UART baud: `250000`
- USB monitor baud in the example projects: `115200`

## Example

See [`examples/RingRoundRobin/RingRoundRobin.ino`](examples/RingRoundRobin/RingRoundRobin.ino).
