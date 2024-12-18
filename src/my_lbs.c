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
#include <zephyr/sys/printk.h> // Extra to print out received data. Not logging

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

static uint8_t writable_value[20]; // Me: Buffer for the characteristic value
static ssize_t received_app_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags){
	
	
	uint8_t *value = attr -> user_data;

	if(offset + len > sizeof(writable_value)){
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	printk("Characteristic written, new value: ");
	for(int i=0; i<len; i++){
		printk("%d", ((uint8_t *)buf)[i]);
	}
	printk("\n");

	return len;
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

	/* Characteristic that receives the app data */
	BT_GATT_CHARACTERISTIC(BT_UUID_APP_DATA, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, received_app_data, writable_value),
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
