/*
 * Cutebot BLE Motor Control Task
 *
 * Initialises the nRF5 SDK SoftDevice S113 BLE stack, registers the
 * Cutebot Pro GATT service and starts advertising, then hands off BLE
 * event processing to the nrf_sdh_freertos dispatch task.
 *
 * Design note
 * -----------
 * This follows the same high-level structure as the Zephyr main.c:
 *   main()
 *     cutebot_pro_init()   ←→  cutebot_pro_init()   (nrfx_twim variant)
 *     ble_cutebot_init()   ←→  ble_cutebot_init()   (nRF5 SDK variant)
 *
 * The SoftDevice S113 binary must be flashed to the device before the
 * application firmware.  Refer to the Nordic nRF5 SDK documentation for
 * S113 programming instructions (nrfjprog --program s113_nrf52_7.x.x.hex).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_freertos.h"
#include "ble_gap.h"
#include "app_error.h"
#include "app_util.h"

#include "cutebot_pro.h"
#include "ble_cutebot.h"
#include "cutebot_task.h"

/* Connection-configuration tag - must match the tag used in
 * ble_cutebot_advertising_start() (sd_ble_gap_adv_start). */
#define APP_BLE_CONN_CFG_TAG    1u

/* Device name advertised over BLE - matches Zephyr "CutebotPro" */
#define DEVICE_NAME             "CutebotPro"

/* GAP preferred connection parameters */
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(100,  UNIT_1_25_MS)
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(200,  UNIT_1_25_MS)
#define SLAVE_LATENCY           0u
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)

/* --------------------------------------------------------
 * Private helpers
 * ------------------------------------------------------ */

/**
 * Enable the SoftDevice and BLE stack.
 * nrf_sdh_ble_default_cfg_set() fills ram_start with the minimum
 * RAM address the application may use; nrf_sdh_ble_enable() verifies
 * that the linker-provided __app_ram_start__ satisfies that requirement.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

/**
 * Set the GAP device name and preferred connection parameters.
 * Security mode: open (no bonding required for motor commands).
 */
static void gap_params_init(void)
{
    ret_code_t err_code;

    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          sizeof(DEVICE_NAME) - 1u);
    APP_ERROR_CHECK(err_code);

    ble_gap_conn_params_t gap_conn_params;
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/* --------------------------------------------------------
 * Public API
 * ------------------------------------------------------ */

void CutebotTaskInit(void)
{
    /* 1. Initialise the I2C Cutebot Pro motor driver.
     *    cutebot_pro_init() configures nrfx_twim0, enables the bus and
     *    sends a "stop motors" command as a safe initial state. */
    int rc = cutebot_pro_init();
    APP_ERROR_CHECK(rc == 0 ? NRF_SUCCESS : NRF_ERROR_INTERNAL);

    /* 2. Enable the SoftDevice S113 and the BLE host stack. */
    ble_stack_init();

    /* 3. Configure GAP parameters (device name + preferred conn params). */
    gap_params_init();

    /* 4. Add the custom GATT service and its two write characteristics. */
    ret_code_t err_code = ble_cutebot_init();
    APP_ERROR_CHECK(err_code);

    /* 5. Start undirected connectable advertising as "CutebotPro". */
    err_code = ble_cutebot_advertising_start();
    APP_ERROR_CHECK(err_code);

    /* 6. Create the SoftDevice FreeRTOS event-dispatch task.
     *    This task calls sd_app_evt_wait() for low-power event pending
     *    and nrf_sdh_evts_poll() to dispatch queued SoftDevice events
     *    to registered observers (including ble_evt_handler in
     *    ble_cutebot.c).  No hook function is needed here. */
    nrf_sdh_freertos_init(NULL, NULL);
}
