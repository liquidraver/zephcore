# ZephCore — MeshCore for Zephyr RTOS

A port of [MeshCore](https://github.com/meshcore-dev/MeshCore/) LoRa mesh firmware from Arduino to [Zephyr RTOS](https://zephyrproject.org/). Aiming for full protocol compatibility with the original Arduino firmware and the MeshCore mobile apps.

DISCLAIMER:
I did this to be a PoC for my Wio Tracker and nRF repeater.
The porting flow was this:
- Copy the meshcore companion source and hit it with a hammer named Claude until the FW works on the node
- if the FW is stable, hit it until it works with the official app
- if both goal is reached, hit the repeater fw and make it work exactly like the arduino counterpart
- Add T1000 and make LR1110 work
- try to fix a thousand bugs that the hammering did
- Add new boards that doesn't even work on arduino (that was not a choice....)
- Try to make it universal so new boards/radios/whatever can be plugged in
- Try to clean up the code and hunt down anything that I've missed <-- YOU ARE HERE NOW
- Revise everything once again, try to understand more than just a quarter of it
- Cleanup-debug--port upstream changes-anything that pops in my mind
- ???
- Profit

Wio Tracker L1 companion and Ikoka Nano 30dB repeater is working according to my tests, I can't vouch for anything else yet.
It's compatible with the current bootloaders I use, so I can roll back to arduino version easily and I have DFU.
Repeater's "touch 1200" and other methods to enter DFU aren't tested yet.



And now, the sacred AI texts:


## Why Zephyr?

The Arduino version uses a `loop()`. This port replaces that with Zephyr's event-driven primitives (`k_event_wait`, `k_poll`, `k_msgq`), so the CPU sleeps in WFI (Wait For Interrupt) between events.

Other benefits:

- **Proper driver model** -- LoRa, GNSS, display, sensors, and BLE all use Zephyr subsystem drivers rather than Arduino libraries
- **Hierarchical build configuration** -- board-specific settings compose cleanly via Kconfig and devicetree overlays
- **DFU support** -- generates Arduino compatible zip packages for OTA updates and UF2 binaries for drag-and-drop flashing
- **Back and forth compatible** -- Adapted to softdevice and adafruit's bootloader, so no bootloader re-flashing required.

## Supported Boards

| Board | MCU | Radio | Extras |
|-------|-----|-------|--------|
| **Wio Tracker L1** | nRF52840 | SX1262 | GPS (L76K), OLED display, joystick, buzzer, QSPI flash |
| **Seeed T1000-E** | nRF52840 | LR1110 | GPS (Mediatek AG3335), LEDs, button |
| **Ikoka Nano 30dBm** | XIAO nRF52840 | SX1262 (E22-900M30S, 30dBm PA) | RGB LEDs |

## Device Roles

**Companion** (default)

**Repeater**

## Building

Prerequisites: [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) and `west` installed.

```bash
# Initialize workspace (first time only)
cd %cloned folder%
west init -l zephcore
west update

# Companion (with logging)
west build -b wio_tracker_l1 zephcore --pristine

# Companion (production, no logging)
west build -b wio_tracker_l1 zephcore --pristine -- \
 -DEXTRA_CONF_FILE="boards/common/prod.conf"

# Repeater (with logging)
west build -b rak4631/nrf52840 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf"

# Repeater (production)
west build -b rak4631/nrf52840 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/prod.conf"

# Repeater with packet logging (clean RAW/RX/TX lines only, no debug spam)
west build -b rak4631/nrf52840 zephcore --pristine -- \
  -DEXTRA_CONF_FILE="boards/common/repeater.conf;boards/common/packet_logging.conf"

# Formatter (with serial logging)
west build -b wio_tracker_l1 zephcore/tools/formatter --pristine

# Companion (BLE debug logging)
west build -b rak4631/nrf52840 zephcore --pristine -- -DCONFIG_ZEPHCORE_BLE_LOG_LEVEL_DBG=y
```

Output binaries are in `build/zephyr/` -- `.hex`, `.uf2`, and DFU `.zip` as applicable.

## Architecture Overview

```
Mobile App  <--BLE (NUS)--> [ Companion ]  <--LoRa-->  Mesh Network
                                  |
                            k_event_wait()
                           /      |       \
                    LORA_RX   LORA_TX_DONE  BLE_RX
```

All code paths are event-driven. No polling loops, no busy waits.

- **LoRa RX**: Zephyr driver callback signals the mesh event loop
- **LoRa TX**: A dedicated 512-byte thread blocks on `k_poll()`, restarts RX immediately on completion, then notifies the mesh loop
- **BLE**: NUS write handler enqueues to `k_msgq` and signals the mesh loop; TX uses `bt_gatt_notify_cb()` chaining
- **Main loop**: `k_event_wait()` blocks until work arrives

### Key Differences from Arduino

| | Arduino | Zephyr |
|---|---------|--------|
| Main loop | 50ms polling | `k_event_wait()` (blocks until event) |
| LoRa TX | Blocking `transmit()` | Async thread with `k_poll()` |
| BLE queues | Ring buffer + mutex | `k_msgq` (lock-free, ISR-safe) |
| Drivers | Arduino libraries | Zephyr subsystem drivers |
| Configuration | `#define` constants | Kconfig + devicetree |
| Threading | Single-threaded `loop()` | Multi-threaded with explicit sync |
| Power | Always-on loop | WFI sleep between events |

## Power Saving

- **LoRa RX duty cycle**: CAD-based receive windowing reduces LoRa RX current from ~10-15mA to ~3-5mA (configurable via `CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE`)
- **USB disabled in production**: Saves ~2-5mA and 62KB flash when logging is off
- **GPIO-gated GPS**: Powered on only during fix acquisition

## Configuration

Key Kconfig options (set in board configs or via `-D` flags):

| Option | Default | Description |
|--------|---------|-------------|
| `CONFIG_ZEPHCORE_ROLE_COMPANION` | y | BLE companion mode |
| `CONFIG_ZEPHCORE_ROLE_REPEATER` | n | USB CLI repeater mode |
| `CONFIG_ZEPHCORE_RADIO_NATIVE` | y | SX1261/62/68/(7x?) radio |
| `CONFIG_ZEPHCORE_RADIO_LR1110` | n | LR1110/LR1120 radio |
| `CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE` | y | RX power saving |
| `CONFIG_ZEPHCORE_MAX_CONTACTS` | 350 | Contact storage slots |
| `CONFIG_ZEPHCORE_MAX_CHANNELS` | 40 | Channel slots |
| `CONFIG_ZEPHCORE_BLE_PASSKEY` | 123456 | BLE pairing PIN |
| `CONFIG_ZEPHCORE_GPS_POLL_INTERVAL_SEC` | 300 | GPS fix interval |

## Project Structure

```
zephcore/
  src/              Main entry points and core mesh protocol
  app/              Companion and repeater role implementations
  adapters/
    radio/          LoRa radio drivers (SX126x, LR1110)
    board/          GPIO, LED, power management
    datastore/      LittleFS filesystem wrapper
    sensors/        I2C sensor auto-detection
    clock/          Millisecond and RTC clocks
  helpers/
    ui/             Display, buzzer, button input
  boards/
    nrf52840/       Per-board devicetree overlays and configs
    common/         Shared Kconfig fragments and devicetree includes
  lib/              ED25519 crypto library
  patches/          Auto-applied patches to the Zephyr tree
```

## License

Same license as the upstream MeshCore project.