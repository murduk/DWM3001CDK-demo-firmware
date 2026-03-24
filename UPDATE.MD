# Cutebot Pro BLE Motor Control — Integration Update

## Overview

This update integrates the **ELECFREAKS Cutebot Pro BLE controller** functionality into the DWM3001CDK demo firmware. The implementation controls the Cutebot Pro robot over Bluetooth Low Energy: a BLE central (phone/tablet) connects and writes motor commands which are forwarded over I2C to the Cutebot Pro mainboard.

No existing hardware definitions (`custom_board.h`, `thisBoard.h`) were modified.

---

## New Files

All new files are located in `Src/Apps/cutebot/`.

### `cutebot_pro.h` / `cutebot_pro.c` — I2C Motor Driver

Implements the Cutebot Pro V2 I²C command protocol using **nrfx_twim1** (nRF5 SDK blocking mode).

> **Note:** TWIM0 is unavailable — it shares the same hardware peripheral and IRQ as SPIM0, which is used by the DW3000 UWB sensor. TWIM1 is used instead.

**I2C pin assignment** (free GPIOs on the DWM3001CDK expansion header, NOT in `custom_board.h`):

| Signal | Pin    |
|--------|--------|
| SDA    | P0.16  |
| SCL    | P0.17  |

To use different pins, change `CUTEBOT_I2C_SDA_PIN` / `CUTEBOT_I2C_SCL_PIN` at the top of `cutebot_pro.c`.

**Public API:**

```c
/* Initialise I2C bus and stop motors */
int cutebot_pro_init(void);

/* Set speed for both wheels: -100..100 (negative = reverse) */
int cutebot_pro_set_motors(int8_t left_speed, int8_t right_speed);

/* PID angle turn */
int cutebot_pro_angle_turn(uint16_t left_angle, uint16_t right_angle,
                           uint16_t speed, uint8_t direction);
```

---

### `ble_cutebot.h` / `ble_cutebot.c` — BLE GATT Service

Registers a custom GATT service with the nRF5 SDK SoftDevice S113.

**Custom 128-bit base UUID:** `e3a10000-1c80-4b4e-a830-7a3f6e5c9b2d`

| Characteristic | UUID suffix | Properties | Payload |
|----------------|-------------|------------|---------|
| Motor control  | `0x0001`    | Write / Write Without Response | 2 bytes: `[left_speed_i8, right_speed_i8]` |
| Angle turn     | `0x0002`    | Write / Write Without Response | 7 bytes: `[lAngH, lAngL, rAngH, rAngL, spdH, spdL, direction]` |

The device advertises as **"CutebotPro"**. Disconnecting any central automatically stops both motors for safety.

---

### `cutebot_task.h` / `cutebot_task.c` — FreeRTOS Init Task

`CutebotTaskInit()` is called from `main()` and performs:

1. Initialises the I2C Cutebot Pro driver (`cutebot_pro_init`)
2. Enables the SoftDevice S113 and BLE host stack (`nrf_sdh_enable_request` / `nrf_sdh_ble_enable`)
3. Configures the GAP device name ("CutebotPro") and preferred connection parameters
4. Registers the custom GATT service and its two write characteristics
5. Starts undirected connectable advertising
6. Creates the SoftDevice FreeRTOS event-dispatch task (`nrf_sdh_freertos_init`)

---

## Modified Files

### `Src/main.c`
- Added `#include "cutebot_task.h"`
- Added `CutebotTaskInit()` call before `osKernelStart()`

### `Src/Config/sdk_config.h`
| Setting | Old value | New value | Notes |
|---------|-----------|-----------|-------|
| `NRFX_TWIM_ENABLED` | `0` | `1` | |
| `NRFX_TWIM0_ENABLED` | `0` | `1` | Set by legacy layer; TWIM0 hardware is unavailable (shared with SPIM0) |
| `TWI1_ENABLED` | `0` | `1` | Required for legacy layer to enable NRFX_TWIM1 |
| `TWI1_USE_EASY_DMA` | `0` | `1` | Required for legacy layer to route to TWIM1 (DMA) not TWI1 (non-DMA) |
| `NRF_SDH_SOC_ENABLED` | `0` | `1` | Required when `SOFTDEVICE_PRESENT` is defined; needed by `nrf_drv_clock` |
| `NRF_SDH_ENABLED` | `0` | `1` | |
| `NRF_SDH_DISPATCH_MODEL` | `0` (interrupt) | `2` (polling — required for FreeRTOS) | |
| `NRF_SDH_BLE_ENABLED` | `0` | `1` | |
| `NRF_SDH_BLE_PERIPHERAL_LINK_COUNT` | `0` | `1` | |
| `NRF_SDH_BLE_VS_UUID_COUNT` | `0` | `1` | |

### `Src/Boards/int_priority.h`
- Added `PRIO_CutebotTask = osPriorityAboveNormal` to the priority enum.

### `DWM3001CDK-DW3_QM33_SDK_CLI-FreeRTOS.emProject`
- **Preprocessor defines:** Added `SOFTDEVICE_PRESENT`, `S113`, `BLE_STACK_SUPPORT_REQD`
- **Include paths:** Replaced `components/drivers_nrf/nrf_soc_nosd` with:
  - `components/softdevice/s113/headers`
  - `components/softdevice/s113/headers/nrf52`
  - `components/softdevice/common`
  - `components/ble/common`
  - `components/ble/ble_advertising`
  - Added `Src/Apps/cutebot`
- **Linker macros** (adjusted to place the application after the S113 SoftDevice):

  | Symbol | Old | New |
  |--------|-----|-----|
  | `FLASH_START` | `0x0` | `0x1B000` |
  | `FLASH_SIZE` | `0xE4000` | `0x65000` |
  | `RAM_START` | `0x20000000` | `0x20002000` |
  | `RAM_SIZE` | `0x20000` | `0x1E000` |
  | `DEFAULT_CONFIG_START` | `0x1D000` | `0x7D000` |
  | `FCONFIG_START` | `0x1E000` | `0x7E000` |

- **Source files added:**
  - `Src/Apps/cutebot/cutebot_pro.c`
  - `Src/Apps/cutebot/ble_cutebot.c`
  - `Src/Apps/cutebot/cutebot_task.c`
  - `nrfx_twim.c`
  - `nrf_sdh.c`, `nrf_sdh_ble.c`, `nrf_sdh_soc.c`, `nrf_sdh_freertos.c`
  - `ble_advdata.c`, `ble_srv_common.c`
  - `nrf_section_iter.c` — required by `nrf_sdh.c` for section-variable iteration

### `flash_placement.xml`

Two changes were required for the linker:

1. **Moved `.fconfig`** from the top of the FLASH section to after `.tdata`. The SEGGER linker processes sections in file order; having `.fconfig` (at `0x7E000`) before `.init` (at `0x1F000`) caused a backward-reference overlap error.

2. **Added SDH observer sections** (between `.nrf_sections` and `.log_dynamic_data`) so the linker generates the `__start_sdh_*` / `__stop_sdh_*` symbols used by `nrf_sdh.c`:
   - `.sdh_ble_observers`
   - `.sdh_soc_observers`
   - `.sdh_req_observers`
   - `.sdh_state_observers`
   - `.sdh_stack_observers`

---

## Flashing Instructions

The nRF52833 requires the **SoftDevice S113** to be present in flash before the application firmware. Flash in two steps.

### Prerequisites

- [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools) (`nrfjprog`) installed
- SEGGER J-Link connected to the DWM3001CDK debug port
- nRF5 SDK 17.1.0 installed at `/usr/local/nRF5_SDK_17.1.0_ddde560/`
- S113 SoftDevice hex file (download from [Nordic SoftDevices](https://www.nordicsemi.com/Products/Development-software/S113)):
  e.g. `s113_nrf52_7.3.0_softdevice.hex`

---

### Step 1 — Erase the device

```bash
nrfjprog --eraseall -f nrf52
```

---

### Step 2 — Flash the SoftDevice S113

```bash
nrfjprog --program /path/to/s113_nrf52_7.3.0_softdevice.hex -f nrf52 --verify
```

> **Important:** Do NOT use `--sectorerase` or `--chiperase` here — the SoftDevice occupies flash addresses `0x0000`–`0x1AFFF`. The application starts immediately after at `0x1B000`.

---

### Step 3 — Build the application

Using Docker (as defined in the Makefile):

```bash
make
```

Or directly with SEGGER Embedded Studio:

```
Open DWM3001CDK-DW3_QM33_SDK_CLI-FreeRTOS.emProject
Build → Build "Common"
```

Output hex: `Output/Common/Exe/DWM3001CDK-DW3_QM33_SDK_CLI-FreeRTOS.hex`

---

### Step 4 — Flash the application

```bash
nrfjprog --program Output/Common/Exe/DWM3001CDK-DW3_QM33_SDK_CLI-FreeRTOS.hex \
         -f nrf52 --sectorerase --verify
```

---

### Step 5 — Reset and run

```bash
nrfjprog --reset -f nrf52
```

---

### Combined one-liner (erase + SoftDevice + app + reset)

```bash
nrfjprog --eraseall -f nrf52 && \
nrfjprog --program /path/to/s113_nrf52_7.3.0_softdevice.hex -f nrf52 --verify && \
nrfjprog --program Output/Common/Exe/DWM3001CDK-DW3_QM33_SDK_CLI-FreeRTOS.hex \
         -f nrf52 --sectorerase --verify && \
nrfjprog --reset -f nrf52
```

---

## Connecting from a Phone / Tablet

1. Open a BLE scanner app (e.g. **nRF Connect for Mobile**).
2. Scan for a device named **"CutebotPro"**.
3. Connect to it.
4. Navigate to the custom service (`e3a10000-1c80-4b4e-a830-7a3f6e5c9b2d`).
5. Write to the characteristics:
   - **Motor control** (`...0001`): 2 bytes `[left_speed, right_speed]` — signed, range −100 to 100.
   - **Angle turn** (`...0002`): 7 bytes `[lAngH, lAngL, rAngH, rAngL, spdH, spdL, direction]`.

Disconnecting stops the motors automatically.

---

## Wire Connections (DWM3001CDK ↔ Cutebot Pro)

| DWM3001CDK pin | Signal | Cutebot Pro |
|----------------|--------|-------------|
| P0.16          | SDA    | I2C SDA     |
| P0.17          | SCL    | I2C SCL     |
| GND            | GND    | GND         |

> The Cutebot Pro is powered independently; only SDA, SCL and GND need to be connected.
