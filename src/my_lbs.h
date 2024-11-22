/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_LBS_H_
#define BT_LBS_H_

/**@file
 * @defgroup bt_lbs LED Button Service API
 * @{
 * @brief API for the LED Button Service (LBS).
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/** @brief LBS Service UUID. */
#define BT_UUID_LBS_VAL BT_UUID_128_ENCODE(0xa5c298c0,0xa235,0x4a32,0xa4e9,0x5b42f6bd50e5)
#define BT_UUID_LBS BT_UUID_DECLARE_128(BT_UUID_LBS_VAL)

/* Assign a UUID to the MYSENSOR characteristic */
/** @brief LED Characteristic UUID. */
#define BT_UUID_LBS_MYSENSOR_VAL BT_UUID_128_ENCODE(0xa5c298c1,0xa235,0x4a32,0xa4e9,0x5b42f6bd50e5)
/* Convert the array to a generic UUID */
#define BT_UUID_LBS_MYSENSOR BT_UUID_DECLARE_128(BT_UUID_LBS_MYSENSOR_VAL)

/** @brief Send the sensor value as notification.
 *
 * This function sends an uint32_t  value, typically the value
 * of a simulated sensor to all connected peers.
 *
 * @param[in] sensor_value The value of the simulated sensor.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a (negative) error code is returned.
 */
int my_lbs_send_sensor_notify(uint32_t* sensor_value, size_t data_size);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_LBS_H_ */
