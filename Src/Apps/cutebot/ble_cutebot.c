/*
 * BLE GATT Service for Cutebot Pro Motor Control
 * Adapted from Zephyr implementation for nRF5 SDK 17.1.0 + SoftDevice S113.
 *
 * BLE event handling pattern mirrors the Zephyr BT_CONN_CB_DEFINE /
 * BT_GATT_SERVICE_DEFINE approach using nRF5 SDK equivalents:
 *   - NRF_SDH_BLE_OBSERVER  ↔  BT_CONN_CB_DEFINE + write callbacks
 *   - sd_ble_gatts_*        ↔  BT_GATT_SERVICE_DEFINE macros
 *   - sd_ble_gap_adv_*      ↔  bt_le_adv_start
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "nrf_sdh_ble.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "app_error.h"
#include "app_util.h"

#include "ble_cutebot.h"
#include "cutebot_pro.h"

/* Observer priority - must be lower than the SoftDevice's internal priority */
#define APP_BLE_OBSERVER_PRIO           3

/* Advertising interval: 40 ms (unit = 0.625 ms) */
#define ADV_INTERVAL_MS                 40u
#define ADV_INTERVAL_UNITS              MSEC_TO_UNITS(ADV_INTERVAL_MS, UNIT_0_625_MS)

/* GAP connection parameters (accepted from any central) */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100,  UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200,  UNIT_1_25_MS)
#define SLAVE_LATENCY                   0
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)

/* --------------------------------------------------------
 * Module-level state
 * ------------------------------------------------------ */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

/* Static buffers for encoded advertising / scan-response payloads */
static uint8_t m_enc_adv[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static uint8_t m_enc_sr[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

static ble_gap_adv_data_t m_adv_data =
{
    .adv_data      = { .p_data = m_enc_adv, .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX },
    .scan_rsp_data = { .p_data = m_enc_sr,  .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX },
};

static uint16_t             m_conn_handle       = BLE_CONN_HANDLE_INVALID;
static uint8_t              m_uuid_type;            /**< Registered vendor UUID type */
static uint16_t             m_service_handle;
static ble_gatts_char_handles_t m_motor_char_handles;
static ble_gatts_char_handles_t m_angle_char_handles;

/* --------------------------------------------------------
 * BLE event handler (registered via NRF_SDH_BLE_OBSERVER)
 * ------------------------------------------------------ */

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context);

NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            /* Safety: stop motors whenever the central disconnects */
            (void)cutebot_pro_set_motors(0, 0);
            /* Restart advertising so a new central can reconnect */
            (void)ble_cutebot_advertising_start();
            break;

        case BLE_GATTS_EVT_WRITE:
        {
            ble_gatts_evt_write_t const *p_write =
                &p_ble_evt->evt.gatts_evt.params.write;

            if (p_write->handle == m_motor_char_handles.value_handle &&
                p_write->len == 2u)
            {
                /* [0] = left_speed (int8), [1] = right_speed (int8) */
                int8_t left_speed  = (int8_t)p_write->data[0];
                int8_t right_speed = (int8_t)p_write->data[1];
                (void)cutebot_pro_set_motors(left_speed, right_speed);
            }
            else if (p_write->handle == m_angle_char_handles.value_handle &&
                     p_write->len == 7u)
            {
                /* [lAngH, lAngL, rAngH, rAngL, spdH, spdL, dir] */
                uint16_t left_angle  = ((uint16_t)p_write->data[0] << 8) |
                                        p_write->data[1];
                uint16_t right_angle = ((uint16_t)p_write->data[2] << 8) |
                                        p_write->data[3];
                uint16_t speed       = ((uint16_t)p_write->data[4] << 8) |
                                        p_write->data[5];
                uint8_t  direction   = p_write->data[6];
                (void)cutebot_pro_angle_turn(left_angle, right_angle,
                                             speed, direction);
            }
            break;
        }

        default:
            break;
    }
}

/* --------------------------------------------------------
 * Public API
 * ------------------------------------------------------ */

ret_code_t ble_cutebot_init(void)
{
    ret_code_t err_code;

    /* Register the vendor-specific 128-bit base UUID with the SoftDevice.
     * After this call m_uuid_type holds the assigned type identifier.
     * The SoftDevice expands a ble_uuid_t {.uuid=X, .type=m_uuid_type}
     * to the full 128-bit UUID by substituting X into bytes [12:13]. */
    ble_uuid128_t base_uuid = { .uuid128 = CUTEBOT_BASE_UUID_INIT };
    err_code = sd_ble_uuid_vs_add(&base_uuid, &m_uuid_type);
    VERIFY_SUCCESS(err_code);

    /* Add primary service */
    ble_uuid_t service_uuid = { .uuid = CUTEBOT_SERVICE_UUID,
                                 .type = m_uuid_type };
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &m_service_handle);
    VERIFY_SUCCESS(err_code);

    /* ---- Motor control characteristic ----
     * Properties : Write + Write Without Response
     * Value      : 2 bytes, no fixed initial value
     * Security   : open (no pairing required)           */
    {
        ble_gatts_char_md_t char_md;
        memset(&char_md, 0, sizeof(char_md));
        char_md.char_props.write         = 1;
        char_md.char_props.write_wo_resp = 1;

        ble_uuid_t char_uuid = { .uuid = CUTEBOT_MOTOR_UUID,
                                  .type = m_uuid_type };

        ble_gatts_attr_md_t attr_md;
        memset(&attr_md, 0, sizeof(attr_md));
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;

        ble_gatts_attr_t attr;
        memset(&attr, 0, sizeof(attr));
        attr.p_uuid    = &char_uuid;
        attr.p_attr_md = &attr_md;
        attr.max_len   = 2;
        attr.init_len  = 0;
        attr.p_value   = NULL;

        err_code = sd_ble_gatts_characteristic_add(m_service_handle,
                                                   &char_md,
                                                   &attr,
                                                   &m_motor_char_handles);
        VERIFY_SUCCESS(err_code);
    }

    /* ---- Angle turn characteristic ----
     * Properties : Write + Write Without Response
     * Value      : 7 bytes, no fixed initial value
     * Security   : open                                 */
    {
        ble_gatts_char_md_t char_md;
        memset(&char_md, 0, sizeof(char_md));
        char_md.char_props.write         = 1;
        char_md.char_props.write_wo_resp = 1;

        ble_uuid_t char_uuid = { .uuid = CUTEBOT_ANGLE_UUID,
                                  .type = m_uuid_type };

        ble_gatts_attr_md_t attr_md;
        memset(&attr_md, 0, sizeof(attr_md));
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;

        ble_gatts_attr_t attr;
        memset(&attr, 0, sizeof(attr));
        attr.p_uuid    = &char_uuid;
        attr.p_attr_md = &attr_md;
        attr.max_len   = 7;
        attr.init_len  = 0;
        attr.p_value   = NULL;

        err_code = sd_ble_gatts_characteristic_add(m_service_handle,
                                                   &char_md,
                                                   &attr,
                                                   &m_angle_char_handles);
        VERIFY_SUCCESS(err_code);
    }

    return NRF_SUCCESS;
}

ret_code_t ble_cutebot_advertising_start(void)
{
    ret_code_t err_code;

    /* ---- Advertising data: flags + full 128-bit service UUID ---- */
    ble_advdata_t advdata;
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type = BLE_ADVDATA_NO_NAME;
    advdata.flags     = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuid = { .uuid = CUTEBOT_SERVICE_UUID, .type = m_uuid_type };
    ble_advdata_uuid_list_t uuid_list = { .uuid_cnt = 1, .p_uuids = &adv_uuid };
    advdata.uuids_complete = uuid_list;

    /* ---- Scan response: device name ---- */
    ble_advdata_t srdata;
    memset(&srdata, 0, sizeof(srdata));
    srdata.name_type = BLE_ADVDATA_FULL_NAME;

    /* Encode payloads into static buffers */
    m_adv_data.adv_data.len      = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    m_adv_data.scan_rsp_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

    err_code = ble_advdata_encode(&advdata,
                                  m_adv_data.adv_data.p_data,
                                  &m_adv_data.adv_data.len);
    VERIFY_SUCCESS(err_code);

    err_code = ble_advdata_encode(&srdata,
                                  m_adv_data.scan_rsp_data.p_data,
                                  &m_adv_data.scan_rsp_data.len);
    VERIFY_SUCCESS(err_code);

    /* ---- Advertising parameters ---- */
    ble_gap_adv_params_t adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = ADV_INTERVAL_UNITS;
    adv_params.duration        = BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED;
    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle,
                                            &m_adv_data,
                                            &adv_params);
    VERIFY_SUCCESS(err_code);

    /* conn_cfg_tag = 1 must match the tag used in nrf_sdh_ble_default_cfg_set */
    err_code = sd_ble_gap_adv_start(m_adv_handle, 1u);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
