/*
 * Cutebot Pro I2C Motor Control Driver
 * Adapted from Zephyr implementation for nRF5 SDK 17.1.0 using nrfx_twim.
 *
 * I2C pins are intentionally kept outside the board definition files so
 * they can be assigned to any free PMOD/header GPIO on the DWM3001CDK
 * without disturbing the existing DW3000 / UART / LED pin assignments.
 *
 * Default pin assignment (free GPIOs on DWM3001CDK expansion header):
 *   SDA = P0.16
 *   SCL = P0.17
 * Change CUTEBOT_I2C_SDA_PIN / CUTEBOT_I2C_SCL_PIN below if needed.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>

#include "nrfx_twim.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"

#include "cutebot_pro.h"

/* I2C pins - free GPIO on DWM3001CDK PMOD expansion header.
 * These pins are NOT defined in custom_board.h (hardware definitions
 * of this repo are left unchanged). */
#define CUTEBOT_I2C_SDA_PIN     NRF_GPIO_PIN_MAP(0, 16)
#define CUTEBOT_I2C_SCL_PIN     NRF_GPIO_PIN_MAP(0, 17)

/* V2 protocol frame header bytes */
#define FRAME_HDR0              0xFF
#define FRAME_HDR1              0xF9

/* Command opcodes */
#define CMD_MOTOR_CONTROL       0x10
#define CMD_ANGLE_TURN          0x86

/* Maximum frame size: 4 header bytes + 7 payload bytes (angle turn) */
#define CMD_BUF_MAX             11u

static const nrfx_twim_t m_twim = NRFX_TWIM_INSTANCE(1);
static bool              m_initialized = false;

/**
 * Build and transmit a V2 command frame over I2C.
 * Frame: [0xFF][0xF9][CMD][PARAM_LEN][...params]
 */
static int cutebot_send_cmd(uint8_t cmd, const uint8_t *params, uint8_t param_len)
{
    uint8_t buf[CMD_BUF_MAX];
    uint8_t total_len = (uint8_t)(4u + param_len);

    buf[0] = FRAME_HDR0;
    buf[1] = FRAME_HDR1;
    buf[2] = cmd;
    buf[3] = param_len;
    if (param_len > 0u && params != NULL)
    {
        memcpy(&buf[4], params, param_len);
    }

    nrfx_twim_xfer_desc_t xfer = NRFX_TWIM_XFER_DESC_TX(CUTEBOT_PRO_ADDR, buf, total_len);
    nrfx_err_t err = nrfx_twim_xfer(&m_twim, &xfer, 0);
    if (err != NRFX_SUCCESS)
    {
        return -1;
    }

    /* Protocol requires 1 ms inter-command delay */
    nrf_delay_ms(1);
    return 0;
}

int cutebot_pro_init(void)
{
    nrfx_twim_config_t config = {
        .scl                = CUTEBOT_I2C_SCL_PIN,
        .sda                = CUTEBOT_I2C_SDA_PIN,
        .frequency          = NRF_TWIM_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW,
        .hold_bus_uninit    = false,
    };

    /* Passing NULL as event_handler enables blocking (polling) mode */
    nrfx_err_t err = nrfx_twim_init(&m_twim, &config, NULL, NULL);
    if (err != NRFX_SUCCESS)
    {
        return -1;
    }

    nrfx_twim_enable(&m_twim);
    m_initialized = true;

    /* Stop both motors as a safe initial state */
    return cutebot_pro_set_motors(0, 0);
}

int cutebot_pro_set_motors(int8_t left_speed, int8_t right_speed)
{
    if (!m_initialized)
    {
        return -1;
    }

    /* Clamp speeds to [-100, 100] */
    if (left_speed  >  100) { left_speed  =  100; }
    if (left_speed  < -100) { left_speed  = -100; }
    if (right_speed >  100) { right_speed =  100; }
    if (right_speed < -100) { right_speed = -100; }

    uint8_t dir_flags = 0;
    if (left_speed  < 0) { dir_flags |= 0x01u; }
    if (right_speed < 0) { dir_flags |= 0x02u; }

    uint8_t params[4] = {
        CUTEBOT_WHEEL_BOTH,
        (uint8_t)abs(left_speed),
        (uint8_t)abs(right_speed),
        dir_flags,
    };

    return cutebot_send_cmd(CMD_MOTOR_CONTROL, params, sizeof(params));
}

int cutebot_pro_angle_turn(uint16_t left_angle, uint16_t right_angle,
                           uint16_t speed, uint8_t direction)
{
    if (!m_initialized)
    {
        return -1;
    }

    uint8_t params[7] = {
        (uint8_t)(left_angle  >> 8),
        (uint8_t)(left_angle  & 0xFFu),
        (uint8_t)(right_angle >> 8),
        (uint8_t)(right_angle & 0xFFu),
        (uint8_t)(speed       >> 8),
        (uint8_t)(speed       & 0xFFu),
        direction,
    };

    return cutebot_send_cmd(CMD_ANGLE_TURN, params, sizeof(params));
}
