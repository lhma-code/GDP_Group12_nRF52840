/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief LED Button Service (LBS) sample
 */



#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "my_lbs.h"

static bool notify_mysensor_enabled;

/* Define the configuration change callback function for the MYSENSOR characteristic */
static void mylbsbc_ccc_mysensor_cfg_changed(const struct bt_gatt_attr *attr,uint16_t value)
{
	notify_mysensor_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(
	my_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS),
	/* Create and add the MYSENSOR characteristic and its CCCD  */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_MYSENSOR,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL,
			       NULL),

	BT_GATT_CCC(mylbsbc_ccc_mysensor_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

);


/* Define the function to send notifications for the MYSENSOR characteristic */
int my_lbs_send_sensor_notify(uint32_t* sensor_value, size_t data_size)
{
	if (!notify_mysensor_enabled) {
		return -EACCES;
	}

	

	return bt_gatt_notify(NULL, &my_lbs_svc.attrs[2], 
			      sensor_value,
			      data_size);
}
