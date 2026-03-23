/*
 * Cutebot BLE Motor Control Task - Header
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CUTEBOT_TASK_H
#define CUTEBOT_TASK_H

/**
 * Initialize and launch the Cutebot BLE motor control subsystem.
 *
 * Sequence performed:
 *  1. Initialize the nrfx_twim I2C driver and Cutebot Pro motor driver.
 *  2. Enable the SoftDevice S113 and BLE stack.
 *  3. Configure GAP device name and preferred connection parameters.
 *  4. Register the custom GATT service (motor + angle characteristics).
 *  5. Start undirected connectable advertising as "CutebotPro".
 *  6. Create the SoftDevice FreeRTOS event-dispatch task.
 *
 * Call this function in main() after BoardInit() and before osKernelStart().
 */
void CutebotTaskInit(void);

#endif /* CUTEBOT_TASK_H */
