/*
 * Cutebot Pro I2C Motor Control Driver
 * Adapted from Zephyr implementation for nRF5 SDK 17.1.0.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CUTEBOT_PRO_H
#define CUTEBOT_PRO_H

#include <stdint.h>

/* I2C address of the Cutebot Pro mainboard */
#define CUTEBOT_PRO_ADDR        0x10

/* Wheel selection */
#define CUTEBOT_WHEEL_LEFT      0
#define CUTEBOT_WHEEL_RIGHT     1
#define CUTEBOT_WHEEL_BOTH      2

/**
 * Initialize the Cutebot Pro I2C interface.
 * Stops both motors as part of initialization.
 *
 * @return 0 on success, -1 on failure.
 */
int cutebot_pro_init(void);

/**
 * Set motor speeds for both wheels.
 *
 * @param left_speed  Left wheel speed, -100 to 100 (negative = reverse).
 * @param right_speed Right wheel speed, -100 to 100 (negative = reverse).
 * @return 0 on success, -1 on failure.
 */
int cutebot_pro_set_motors(int8_t left_speed, int8_t right_speed);

/**
 * Execute a PID angle turn.
 *
 * @param left_angle  Left wheel angle in degrees (0-65535).
 * @param right_angle Right wheel angle in degrees (0-65535).
 * @param speed       Speed value (0-65535).
 * @param direction   Direction byte (0 = forward, 1 = reverse).
 * @return 0 on success, -1 on failure.
 */
int cutebot_pro_angle_turn(uint16_t left_angle, uint16_t right_angle,
                           uint16_t speed, uint8_t direction);

#endif /* CUTEBOT_PRO_H */
