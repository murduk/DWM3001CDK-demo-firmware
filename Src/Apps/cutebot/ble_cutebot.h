/*
 * BLE GATT Service for Cutebot Pro Motor Control
 * Adapted from Zephyr implementation for nRF5 SDK 17.1.0 + SoftDevice S113.
 *
 * Custom 128-bit UUID base: e3a10000-1c80-4b4e-a830-7a3f6e5c9b2d
 *
 * Service and characteristic 16-bit aliases (vendor-specific portion,
 * bytes [12:13] of the base UUID in little-endian layout):
 *   Service:    0x0000  →  e3a10000-1c80-4b4e-a830-7a3f6e5c9b2d
 *   Motor char: 0x0001  →  e3a10001-1c80-4b4e-a830-7a3f6e5c9b2d
 *   Angle char: 0x0002  →  e3a10002-1c80-4b4e-a830-7a3f6e5c9b2d
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BLE_CUTEBOT_H
#define BLE_CUTEBOT_H

#include <stdint.h>
#include "ble.h"
#include "ble_err.h"
#include "ble_srv_common.h"
#include "sdk_errors.h"

/*
 * Base UUID stored as 16 bytes in little-endian order (LSB first).
 * The vendor-specific 16-bit portion occupies bytes [12:13] and is
 * set to 0x00 0x00 here; sd_ble_uuid_vs_add() fills it per-UUID.
 *
 * UUID: e3a10000-1c80-4b4e-a830-7a3f6e5c9b2d
 * Big-endian bytes: e3 a1 00 00 1c 80 4b 4e a8 30 7a 3f 6e 5c 9b 2d
 * Little-endian (index 0 = LSB):
 *   2d 9b 5c 6e 3f 7a 30 a8  4e 4b 80 1c 00 00 a1 e3
 */
#define CUTEBOT_BASE_UUID_INIT                          \
{                                                       \
    0x2d, 0x9b, 0x5c, 0x6e, 0x3f, 0x7a, 0x30, 0xa8,   \
    0x4e, 0x4b, 0x80, 0x1c, 0x00, 0x00, 0xa1, 0xe3     \
}

/* 16-bit UUID aliases for the vendor-specific characteristics */
#define CUTEBOT_SERVICE_UUID    0x0000u
/**< Motor characteristic (Write / Write Without Response): 2 bytes
 *   [left_speed_i8, right_speed_i8], range -100..100. */
#define CUTEBOT_MOTOR_UUID      0x0001u
/**< Angle-turn characteristic (Write / Write Without Response): 7 bytes
 *   [lAngH, lAngL, rAngH, rAngL, spdH, spdL, direction]. */
#define CUTEBOT_ANGLE_UUID      0x0002u

/**
 * Initialize the custom BLE GATT service.
 * Registers the vendor UUID with the SoftDevice and adds the primary
 * service plus motor-control and angle-turn write characteristics.
 *
 * Must be called after the SoftDevice and BLE stack have been enabled.
 *
 * @return NRF_SUCCESS on success, or an nRF error code on failure.
 */
ret_code_t ble_cutebot_init(void);

/**
 * Start undirected connectable advertising as "CutebotPro".
 * The service UUID is included in the advertising payload; the device
 * name is placed in the scan response.
 *
 * Must be called after ble_cutebot_init().
 *
 * @return NRF_SUCCESS on success, or an nRF error code on failure.
 */
ret_code_t ble_cutebot_advertising_start(void);

#endif /* BLE_CUTEBOT_H */
