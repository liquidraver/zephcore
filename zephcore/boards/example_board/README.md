ZephCore — Adding a New Board
=============================

There are TWO ways to add a board, depending on whether Zephyr
already has a board definition for your hardware.


PATTERN 1: Existing Zephyr Board (overlay only)
------------------------------------------------

Use this when your board already exists in Zephyr's tree.
You only need TWO files: board.conf + board.overlay

Examples: XIAO nRF54L15, XIAO MG24, XIAO ESP32-C3, RAK4631

Directory structure:
  zephcore/boards/<platform>/<board_name>/
    board.conf       — Kconfig (name, radio type)
    board.overlay    — DT overlay (LoRa SPI, partitions, peripherals)

Steps:
  1. Create directory: boards/<platform>/<board_name>/
     Platform folders: nrf52840, nrf54l, mg24, esp32
  2. Copy board.conf and board.overlay from THIS directory
  3. Uncomment the sections matching your platform
  4. Fill in YOUR pin numbers and partition layout
  5. Add board detection to CMakeLists.txt (~line 60-75):
     Add BOARD MATCHES "your_board" to the correct platform line
  6. Build and iterate!

Build commands:
  nRF52840:  west build -b <board> zephcore --pristine
  nRF54L15:  west build -b <board>/nrf54l15/cpuapp zephcore --pristine --no-sysbuild
  MG24:      west blobs fetch hal_silabs && west build -b <board> zephcore --pristine
  ESP32-C3:  west build -b <board> zephcore --pristine


PATTERN 2: Fully Custom Board (new DTS)
-----------------------------------------

Use this when your board does NOT exist in Zephyr's tree.
You need a full board definition: .dts, pinctrl, Kconfig, etc.

Examples: Ikoka Nano 30dBm, ThinkNode M1 (custom nRF52840 designs)

Look at existing custom boards as templates:
  zephcore/boards/nrf52840/ikoka_nano_30dbm/   — minimal (LoRa only)
  zephcore/boards/nrf52840/thinknode_m1/       — full-featured (EPD, GPS, QSPI, buzzer)

A full custom board includes:
  board.conf                           — Kconfig (name, radio, overrides)
  board.overlay                        — DT overlay (usually empty if DTS is complete)
  <board>_<soc>.dts                    — Full device tree
  <board>_<soc>-pinctrl.dtsi           — Pin control definitions
  board.yml                            — Board metadata (name, arch, SoC)
  Kconfig.<board>                      — SoC selection
  <board>_<soc>_defconfig              — Minimal defconfig
  board.cmake                          — Flash runner config


What Goes in board.conf
-----------------------

Most hardware features are auto-detected from devicetree. board.conf
should ONLY contain settings that can't be inferred from hardware:

  REQUIRED (all boards):
    CONFIG_ZEPHCORE_BOARD_NAME          Human-readable name
    CONFIG_BT_DIS_MODEL_NUMBER_STR      BLE Device Information model

  REQUIRED (nRF52 only):
    CONFIG_ZEPHCORE_SD_FWID             SoftDevice firmware ID (0x00B6 or 0x0123)

  OPTIONAL (only if needed):
    CONFIG_ZEPHCORE_RADIO_LR1110        LR1110 radio (auto-selects SPI)
    CONFIG_ZEPHCORE_MAX_CONTACTS        Override for RAM-limited boards
    CONFIG_HEAP_MEM_POOL_SIZE           Override for large displays (>128x64)
    CONFIG_SEGGER_RTT_BUFFER_SIZE_UP    Shrink RTT on RAM-tight boards
    CONFIG_ESPTOOLPY_FLASHSIZE_16MB     ESP32 boards with 16MB flash
    CONFIG_ZEPHCORE_DEFAULT_TX_POWER_DBM  Boards with external PA
    CONFIG_ZEPHCORE_MAX_TX_POWER_DBM      Boards with external PA

  AUTO-DETECTED (do NOT set in board.conf):
    CONFIG_PWM                          Auto from DT buzzer nodelabel
    CONFIG_ZEPHCORE_UI_BUZZER           Auto from DT buzzer nodelabel
    CONFIG_ZEPHCORE_UI_DISPLAY          Auto from DT zephyr,display chosen
    CONFIG_SPI                          Auto from ZEPHCORE_RADIO_LR1110
    CONFIG_NORDIC_QSPI_NOR             Auto from DT nordic,qspi-nor node
    CONFIG_ZEPHCORE_LORA_RX_DUTY_CYCLE  Auto: ON for companion+SX1262, OFF for repeater/LR1110


Config Inheritance
------------------

  prj.conf                           Always loaded first
    |
  zephcore_common.conf               BLE, storage, input, LoRa, crypto, sensors
    |
  <platform>_common.conf             Platform-specific overrides only
    |                                  nrf52_common.conf  — UF2, USB CDC, DLE, GNSS, RTT
    |                                  nrf54l_common.conf — DLE, RTT
    |                                  mg24_common.conf   — SiLabs blob stacks, heap
    |                                  esp32_common.conf  — Espressif blob stacks, heap
    |
  board.conf                         Board name, radio type, board-specific

DO NOT duplicate settings from parent configs in board.conf!


Quick Reference: Wio-SX1262 XIAO Pin Mapping
---------------------------------------------

All XIAO boards use the same D-pin assignment for Wio-SX1262:

  Signal | XIAO Pin | nRF52840  | nRF54L15  | MG24      | ESP32-C3
  -------|----------|-----------|-----------|-----------|--------
  DIO1   | D1       | P0.03     | P1.05     | PC01      | GPIO3
  RESET  | D2       | P0.28     | P1.06     | PC02      | GPIO4
  BUSY   | D3       | P0.05     | P1.07     | PC03      | GPIO5
  NSS    | D4       | P0.04     | P1.10     | PC04      | GPIO6
  RXEN   | D5       | P0.29     | P1.11     | PC05      | GPIO7
  SCK    | D8       | P1.13     | P2.01     | PA03      | GPIO8
  MISO   | D9       | P1.14     | P2.04     | PA04      | GPIO9
  MOSI   | D10      | P1.15     | P2.02     | PA05      | GPIO10

Note: nRF52840 D-pin mapping varies by board (XIAO nRF52840 shown).
RAK4631 has SX1262 integrated — different pinout entirely.
