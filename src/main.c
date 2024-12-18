/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* All libraries for the BLE, Logging, LBS*/
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <dk_buttons_and_leds.h>
#include "my_lbs.h"
#include <zephyr/bluetooth/hci.h> // Me: Host Controller Interface for BLE and NFC purposes

/* Include NFC libraries */
#include <nfc_t4t_lib.h>
#include <nfc/t4t/ndef_file.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/ch.h>
#include <nfc/ndef/ch_msg.h>
#include <nfc/ndef/le_oob_rec.h>
#include <nfc/ndef/le_oob_rec_parser.h>
#include <nfc/tnep/tag.h>
#include <nfc/tnep/ch.h>

/* Include Settings libraries for storing non-volatile data like keys*/
#include <zephyr/settings/settings.h>

/* Include header for nrfx drivers */
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

LOG_MODULE_REGISTER(Group12_Test, LOG_LEVEL_DBG);


/* Fixed SAADC Macros */
#define SAADC_SAMPLE_INTERVAL_US 500 // Define the SAADC sample interval in microseconds
#define SAADC_BUFFER_SIZE 4000 // Define the SAADC buffer size for the SAADC
#define NRFX_SAADC_CHANNEL_COUNT 2 // Me: Number of SAADCs

/* Fixed BLE Macros*/
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define USER_LED DK_LED3
#define USER_BUTTON DK_BTN1_MSK
#define STACKSIZE 1024
#define PRIORITY 7
#define RUN_LED_BLINK_INTERVAL 1000
#define NOTIFY_INTERVAL 20 // Me: Notification interval in miliseconds


/* Fixed NFC pairing Macros*/
#define K_POOL_EVENTS_CNT (NFC_TNEP_EVENTS_NUMBER + 1) // Me: NFC_TNEP_EVENTS_NUMBER = 2, i.e., NFC TNEP library event count
#define NDEF_MSG_BUF_SIZE 256 // Me: NFC Data Exchange Format (NDEF) Buffer size
#define AUTH_SC_FLAG 0x08 // Me: Authentication Secure Connection flag
#define KEY_BOND_REMOVE_MASK DK_BTN4_MSK // Me: Button 4 triggers the removal of all previous bonds
#define NFC_NDEF_LE_OOB_REC_PARSER_BUFF_SIZE 150 
#define NFC_TNEP_BUFFER_SIZE 1024


/* BLE: variables and threading*/
static uint16_t ble_tx_max_len; // Me: Self-defined, maximum transmission payload length
static struct bt_gatt_exchange_params exchange_params;
static bool mtu_exchanged; // Me: flag to know if the MTU has been exchanged before notifications can be sent
static K_SEM_DEFINE(data_ready_sem, 0, 1);
static const struct bt_data ad[] = { // Adverting data
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};


/* NFC pairing: variables and threading */
static struct bt_le_oob oob_local;
static struct k_work adv_work;
static uint8_t conn_cnt;
static uint8_t tk_value[NFC_NDEF_LE_OOB_REC_TK_LEN]; // Me: Temporary key value. Its length is 16 uint8_t
static uint8_t remote_tk_value[NFC_NDEF_LE_OOB_REC_TK_LEN]; // Me: Remote temporary key value. Its length is 16 uint8_t
static struct bt_le_oob oob_remote;
K_MSGQ_DEFINE(bonds_queue, // Me: name of message queue defined in this MACRO
	      sizeof(bt_addr_le_t), // Me: size of the message queue buffer in bytes
	      CONFIG_BT_MAX_PAIRED, // Me: Maximum number of messages that can be queued. Minimum and default is 1
	      4); /* Bonded address queue. */ // Me: A message queue is a kernel object that implements a simple message queue, allowing threads and ISRs to asynchronously send and receive fixed-size data items
static struct k_poll_signal pair_signal;
static struct k_poll_event events[K_POOL_EVENTS_CNT];
static uint8_t tnep_buffer[NFC_TNEP_BUFFER_SIZE];
static uint8_t tnep_swap_buffer[NFC_TNEP_BUFFER_SIZE];
static bool use_remote_tk;
static bool adv_permission;


/* SAADC: Variables */
const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(2); // Declaring an instance of nrfx_timer for TIMER2.
static int16_t saadc_sample_buffer[2][SAADC_BUFFER_SIZE]; // Declare the buffers for the SAADC
static uint32_t saadc_current_buffer = 0; // Declare variable used to keep track of which buffer was last assigned to the SAADC driver



/* NFC pairing functions, callback structures */

/* Generate a Temporary Key for legacy pairing */
static int tk_value_generate(void)
{
	int err;

	err = bt_rand(tk_value, sizeof(tk_value)); // Me: Generate random data
	if (err) {
		printk("Random TK value generation failed: %d\n", err);
	}

	return err;
}
/* Initializes the polling mechanism for generating pairing keys */
static void pair_key_generate_init(void)
{
	k_poll_signal_init(&pair_signal); // Me: Initialise poll signal object pair_signal as a global
	k_poll_event_init(&events[NFC_TNEP_EVENTS_NUMBER], // Me: Initialise specific event 2
			  K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, // Me: K_POLL_TYPE_SIGNAL = Kernel object to monitor (poll) is a signal. K_POLL_MODE_NOTIFY_ONLY = thread notified when even occurs but not action required
			  &pair_signal); 
}
/* Fetches local BLE OOB data and generates TK */
static int paring_key_generate(void)
{
	int err;

	printk("Generating new pairing keys\n");

	err = bt_le_oob_get_local(BT_ID_DEFAULT, &oob_local); // Me: Get the local info useful for OOB pairing or connection creation. Save it in the global variable, oob_local
	if (err) {
		printk("Error while fetching local OOB data: %d\n", err);
	}

	return tk_value_generate(); // Me: Call the tk_value_generate function
}
/* Monitors if there is an event to regenerate pairing keys and processes them accordingly */
static void paring_key_process(void)
{
	int err;

	if (events[NFC_TNEP_EVENTS_NUMBER].state == K_POLL_STATE_SIGNALED) { // Me: If event 2's state is in a signal polling state, i.e., event was signalled. 
		err = paring_key_generate(); // Me: Generate a key
		if (err) {
			printk("Pairing key generation error: %d\n", err);
		}

		k_poll_signal_reset(events[NFC_TNEP_EVENTS_NUMBER].signal); // Me: Reset the event signal to indicate that it has been handled, i.e., event no longer in signaled state
		events[NFC_TNEP_EVENTS_NUMBER].state = K_POLL_STATE_NOT_READY; // Me: The event state is set to not read indicating it is no longer active or waiting for processing
	}
}
/* Searches for stored BLE bonds and queues them for reconnection */
static void bond_find(const struct bt_bond_info *info, void *user_data)
{
	int err;
	struct bt_conn *conn;

	/* Filter already connected peers. */
	conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &info->addr);
	if (conn) {
		bt_conn_unref(conn);
		return;
	}

	err = k_msgq_put(&bonds_queue, (void *) &info->addr, K_NO_WAIT); // Me: save the connection info into the bonds_queue for later directed advertising
	if (err) {
		printk("No space in the queue for the bond\n");
	}
}
/* Starts BLE advertising with bond information */
static void advertising_start(void)
{
	k_msgq_purge(&bonds_queue); // Me: clear any existing bonded device addresses from the message queue
	bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL); // Me: iterate over all the bonded devices for a specific BLE identity (the device) for connection

	k_work_submit(&adv_work);// Me: submit the advertising work item to the system work queue
}
/* Handles NFC events such as field detection (NFC_T4T_EVENT_FIELD_ON), field loss(NFC_T4T_EVENT_FIELD_OFF), and NDEF message updates (NFC_T4T_EVENT_NDEF_UPDATED) */
static void nfc_callback(void *context, nfc_t4t_event_t event, const uint8_t *data, size_t data_length, uint32_t flags)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(flags);

	switch (event) {
	case NFC_T4T_EVENT_FIELD_ON: // Me: Field detection
		nfc_tnep_tag_on_selected();
		//dk_set_led_on(NFC_FIELD_LED);

		adv_permission = true;

		break;

	case NFC_T4T_EVENT_FIELD_OFF: // Me: Field loss
		nfc_tnep_tag_on_selected();
		//dk_set_led_off(NFC_FIELD_LED);
		break;

	case NFC_T4T_EVENT_NDEF_READ: // Me: NDEF message read
		if (adv_permission) {
			advertising_start();
			adv_permission = false;
		}

		break;

	case NFC_T4T_EVENT_NDEF_UPDATED: // Me: NDEF message updated
		if (data_length > 0) {
			nfc_tnep_tag_rx_msg_indicate(nfc_t4t_ndef_file_msg_get(data),
						     data_length);
		}

	default:
		break;
	}
}
/* Continues BLE advertising after NFC triggers it */
static void advertising_continue(void)
{
	struct bt_le_adv_param adv_param;

	bt_addr_le_t addr;

	if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) { // Me: Wait until bonds_queue thread receives a message, save the address data. k_msq_get returns 0 when message received
		char addr_buf[BT_ADDR_LE_STR_LEN]; // Me: 30, recommended length of user string buffer for BLE address

		adv_param = *BT_LE_ADV_CONN_DIR(&addr); // Me: Peer address is put into the adv_param for directed
		adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA; // Me: Directed advertising, resolvable private address

		int err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0); // Me: Start advertising

		if (err) {
			printk("Directed advertising failed to start\n");
			return;
		}

		bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN); // Me: Put the address into addr_buf as a string
		printk("Direct advertising to %s started\n", addr_buf);
	} else {
		int err;

		adv_param = *BT_LE_ADV_CONN; // Me: Connectable advertising
		adv_param.options |= BT_LE_ADV_OPT_ONE_TIME; // Me: Advertise one time only. If either a connection is established, or bt_le_adv_stop manually stop advertising, advertise
		err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
				      NULL, 0);

		/* Re-enabling advertising when it is already active
		 * is not an error here.
		 */
		if (err && (err != -EALREADY)) {
			printk("Advertising failed to start (err %d)\n", err);
            LOG_DBG("Connection count is %d", conn_cnt); // Test
			return;
		}

		printk("Regular advertising started\n");
	}
}
/* Helper function to manage advertising */
static void adv_handler(struct k_work *work)
{
	advertising_continue();
}
/* Handles cancellation of a BLE pairing process iniitated via NFC or other methods. Called when pairing is interrupted or aborted by the user or device */
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}
/* Configures BLE pairing using Secure Connection (SC) OOB data */
static void lesc_oob_data_set(struct bt_conn *conn, struct bt_conn_oob_info *oob_info)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn_info info;

	err = bt_conn_get_info(conn, &info); // Me: retrieve the BLE connection (e.g., local and remote addresses)
	if (err) {
		return;
	}

	// Me: Determine which OOB data to use, either local or remote
	struct bt_le_oob_sc_data *oob_data_local =
		oob_info->lesc.oob_config != BT_CONN_OOB_REMOTE_ONLY
					  ? &oob_local.le_sc_data
					  : NULL;
	struct bt_le_oob_sc_data *oob_data_remote =
		oob_info->lesc.oob_config != BT_CONN_OOB_LOCAL_ONLY
					  ? &oob_remote.le_sc_data
					  : NULL;

	// Me: Validate the remote OOB data. Check if the remote device's address matches the expected OOB data
	if (oob_data_remote &&
	    bt_addr_le_cmp(info.le.remote, &oob_remote.addr)) {
		bt_addr_le_to_str(info.le.remote, addr, sizeof(addr));
		printk("No OOB data available for remote %s", addr);
		bt_conn_auth_cancel(conn);
		return;
	}

	// Me: Validate the local OOB data. Ensure the local device's address matches the expected OOB data
	if (oob_data_local &&
	    bt_addr_le_cmp(info.le.local, &oob_local.addr)) {
		bt_addr_le_to_str(info.le.local, addr, sizeof(addr));
		printk("No OOB data available for local %s", addr);
		bt_conn_auth_cancel(conn);
		return;
	}

	// Me: Set the OOB secure connection data. Configure the BLE stack to use the local and/or remote OOB data for pairing
	err = bt_le_oob_set_sc_data(conn, oob_data_local, oob_data_remote);
	if (err) {
		printk("Error while setting OOB data: %d\n", err);
	}
}
/* Sets the TK value for legacy BLE pairing */
static void legacy_tk_value_set(struct bt_conn *conn)
{
	int err;
	const uint8_t *tk = use_remote_tk ? remote_tk_value : tk_value;

	err = bt_le_oob_set_legacy_tk(conn, tk);
	if (err) {
		printk("TK value set error: %d\n", err);
	}

	use_remote_tk = false;
}
/* Responds to BLE pairing requests by providing OOB data or TK */
static void auth_oob_data_request(struct bt_conn *conn, struct bt_conn_oob_info *info)
{
	if (info->type == BT_CONN_OOB_LE_SC) { // Me: If we're using Secure Connection
		printk("LESC OOB data requested\n");
		lesc_oob_data_set(conn, info);
	}

	if (info->type == BT_CONN_OOB_LE_LEGACY) { // Me: If we're using Legacy Pairing
		printk("Legacy TK value requested\n");
		legacy_tk_value_set(conn);
	}
}
/* Handles successful BLE pairing events */
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);

	k_poll_signal_raise(&pair_signal, 0);
	bt_le_oob_set_sc_flag(false); // Me: After pairing is complete, reset the OOB secure connection for the next connection
	bt_le_oob_set_legacy_flag(false); // Me: After pairing is complete, reset for the next connection
}
/* Handles BLE pairing failures */
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d\n", addr, reason);

	k_poll_signal_raise(&pair_signal, 0);
	bt_le_oob_set_sc_flag(false);
	bt_le_oob_set_legacy_flag(false);
}
/* Custom logic for accepting BLE pairing requests */
static enum bt_security_err pairing_accept(struct bt_conn *conn, const struct bt_conn_pairing_feat *const feat)
{
	if (feat->oob_data_flag && (!(feat->auth_req & AUTH_SC_FLAG))) {
		bt_le_oob_set_legacy_flag(true);
	}

	return BT_SECURITY_ERR_SUCCESS; // Me: Either Secure Connection is successful or unsuccessful

}
/* Structure that holds BLE connection callbacks for authentication */
static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
	.oob_data_request = auth_oob_data_request,
	.pairing_accept = pairing_accept,
};
/* Structure that holds BLE/NFC connection callbacks for pairing */
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
/* Logs changes in BLE security levels */
static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
			err);
	}
}
/* Encodes the initial NFC Tag NDEF message to share device capabilities and Out of Band (OOB) information */
static int tnep_initial_msg_encode(struct nfc_ndef_msg_desc *msg)
{
	int err;
	struct nfc_ndef_ch_msg_records ch_records; // Me: Structure holding the records (carrier and alternative carrier) for the Connection Handover message
	static struct nfc_ndef_le_oob_rec_payload_desc rec_payload; // Me: Describes payload for BLE Out-of-Band record

	NFC_NDEF_LE_OOB_RECORD_DESC_DEF(oob_rec, '0', &rec_payload); // Me: Defines LE OOB record (i.e., primary data source for Bluetooth pairing)
	NFC_NDEF_CH_AC_RECORD_DESC_DEF(oob_ac, NFC_AC_CPS_ACTIVE, 1, "0", 0); // Me: Defines the Alternative Carrier record
	NFC_NDEF_CH_HS_RECORD_DESC_DEF(hs_rec, NFC_NDEF_CH_MSG_MAJOR_VER, // Me: Defines the Handover Select record, which groups the available carriers
				       NFC_NDEF_CH_MSG_MINOR_VER, 1);

	memset(&rec_payload, 0, sizeof(rec_payload)); // Me: Initialise the Record Payload. This clears the structure to avoid undefined values

	// Me: populate the LE OOB Record Payload above
	rec_payload.addr = &oob_local.addr;
	rec_payload.le_sc_data = &oob_local.le_sc_data;
	rec_payload.tk_value = tk_value;
	rec_payload.local_name = bt_get_name();
	rec_payload.le_role = NFC_NDEF_LE_OOB_REC_LE_ROLE(
		NFC_NDEF_LE_OOB_REC_LE_ROLE_PERIPH_ONLY);
	rec_payload.appearance = NFC_NDEF_LE_OOB_REC_APPEARANCE(
		CONFIG_BT_DEVICE_APPEARANCE);
	rec_payload.flags = NFC_NDEF_LE_OOB_REC_FLAGS(BT_LE_AD_NO_BREDR);

	// Me: configure the connection handover records
	ch_records.ac = &NFC_NDEF_CH_AC_RECORD_DESC(oob_ac); // Me: Points to the Alternative Carrier record
	ch_records.carrier = &NFC_NDEF_LE_OOB_RECORD_DESC(oob_rec); // Me: Points to the LE OOB record
	ch_records.cnt = 1;

	err =  nfc_ndef_ch_msg_hs_create(msg, // Me: Create handover select message containing carriers
					 &NFC_NDEF_CH_RECORD_DESC(hs_rec),
					 &ch_records);
	if (err) {
		return err;
	}

	return nfc_tnep_initial_msg_encode(msg, NULL, 0);
}
/* Validates the carrier record in an NFC handover message to ensure it supports BLE OOB pairing */
static int check_oob_carrier(const struct nfc_tnep_ch_record *ch_record, const struct nfc_ndef_record_desc **oob_data)
{
	const struct nfc_ndef_ch_ac_rec *ac_rec = NULL;

	for (size_t i = 0; i < ch_record->count; i++) {
		if (nfc_ndef_le_oob_rec_check(ch_record->carrier[i])) {
			*oob_data = ch_record->carrier[i];
		}
	}

	if (!oob_data) {
		printk("Connection Handover Requester not supporting OOB BLE\n");
		return -EINVAL;
	}

	/* Look for the corresponding Alternative Carrier Record. */
	for (size_t i = 0; i < ch_record->count; i++) {
		if (((*oob_data)->id_length == ch_record->ac[i].carrier_data_ref.length) &&
		    (memcmp((*oob_data)->id,
			    ch_record->ac[i].carrier_data_ref.data,
			    (*oob_data)->id_length) == 0)) {
			ac_rec = &ch_record->ac[i];
		}
	}

	if (!ac_rec) {
		printk("No Alternative Carrier Record for OOB LE carrier\n");
		return -EINVAL;
	}

	/* Check carrier state */
	if ((ac_rec->cps != NFC_AC_CPS_ACTIVE) &&
	    (ac_rec->cps != NFC_AC_CPS_ACTIVATING)) {
		printk("LE OBB Carrier inactive\n");
		return -EINVAL;
	}

	return 0;
}
/* Processes an OOB carrier record to extract BLE pairing information like device address, Temporary Key (TK), or Secure Connection (SC) data */
static int oob_le_data_handle(const struct nfc_ndef_record_desc *rec, bool request)
{
	int err;
	const struct nfc_ndef_le_oob_rec_payload_desc *oob;
	uint8_t desc_buf[NFC_NDEF_LE_OOB_REC_PARSER_BUFF_SIZE];
	uint32_t desc_buf_len = sizeof(desc_buf);

	err = nfc_ndef_le_oob_rec_parse(rec, desc_buf,
					&desc_buf_len);
	if (err) {
		printk("Error during NDEF LE OOB Record parsing, err: %d.\n",
			err);
	}

	oob = (struct nfc_ndef_le_oob_rec_payload_desc *) desc_buf;

	nfc_ndef_le_oob_rec_printout(oob);

	if ((*oob->le_role != NFC_NDEF_LE_OOB_REC_LE_ROLE_CENTRAL_ONLY) &&
	    (*oob->le_role != NFC_NDEF_LE_OOB_REC_LE_ROLE_CENTRAL_PREFFERED)) {
		printk("Unsupported Device LE Role\n");
		return -EINVAL;
	}

	if (oob->le_sc_data) {
		bt_le_oob_set_sc_flag(true);
		oob_remote.le_sc_data = *oob->le_sc_data;
		bt_addr_le_copy(&oob_remote.addr, oob->addr);
	}

	if (oob->tk_value) {
		bt_le_oob_set_legacy_flag(true);
		memcpy(remote_tk_value, oob->tk_value, sizeof(remote_tk_value));
		use_remote_tk = request;
	}

	advertising_start();

	return 0;
}
/* Prepares a carrier record for NFC communication. This is needed for NFC pairing handover */
static int carrier_prepare(void)
{
	static struct nfc_ndef_le_oob_rec_payload_desc rec_payload;

	NFC_NDEF_LE_OOB_RECORD_DESC_DEF(oob_rec, '0', &rec_payload);
	NFC_NDEF_CH_AC_RECORD_DESC_DEF(oob_ac, NFC_AC_CPS_ACTIVE, 1, "0", 0);

	memset(&rec_payload, 0, sizeof(rec_payload));

	rec_payload.addr = &oob_local.addr;
	rec_payload.le_sc_data = &oob_local.le_sc_data;
	rec_payload.tk_value = tk_value;
	rec_payload.local_name = bt_get_name();
	rec_payload.le_role = NFC_NDEF_LE_OOB_REC_LE_ROLE(
		NFC_NDEF_LE_OOB_REC_LE_ROLE_PERIPH_ONLY);
	rec_payload.appearance = NFC_NDEF_LE_OOB_REC_APPEARANCE(
		CONFIG_BT_DEVICE_APPEARANCE);
	rec_payload.flags = NFC_NDEF_LE_OOB_REC_FLAGS(BT_LE_AD_NO_BREDR);

	return nfc_tnep_ch_carrier_set(&NFC_NDEF_CH_AC_RECORD_DESC(oob_ac),
				       &NFC_NDEF_LE_OOB_RECORD_DESC(oob_rec),
				       1);
}
/* Processes received NFC TNEP (Tag NDEF Exchange Protocol) connection handover. Ensures NFC request properly processed and prepares system to handle BLE pairing or reconnecting with OOB data */
static int tnep_ch_request_received(const struct nfc_tnep_ch_request *ch_req) // Me: pointer to structure representing received Connection Handover Request message.
{
	int err;
	const struct nfc_ndef_record_desc *oob_data = NULL;

	if (!ch_req->ch_record.count) { // Me: check if the request contains carriers (i.e., message for how connection should be established)
		return -EINVAL;
	}

	err = check_oob_carrier(&ch_req->ch_record, &oob_data); // Me: validates the Connection Handover record and identifies the OOB carrier, putting it into oob_data
	if (err) {
		return err;
	}

	bt_le_adv_stop(); // Me: stop BLE advertising

	err = oob_le_data_handle(oob_data, true); // Me: process the extracted OOB record (e.g, extract useful pairing info, configure BLE stack, etc)
	if (err) {
		return err;
	}

	return carrier_prepare(); // Me: reconfigure the NFC message with the updated carrier info making the system reqdy for subsequent NFC or BLE interactions
}
/* Structure for the Connection Handover callbacks*/
static struct nfc_tnep_ch_cb ch_cb = {
#if defined(CONFIG_NFC_TAG_CH_REQUESTER)
	.request_msg_prepare = tnep_ch_request_prepare,
	.select_msg_recv = tnep_ch_select_received,
#endif
	.request_msg_recv = tnep_ch_request_received // Me: Connection Handover Request Message received callback (i.e., NFC data exchanged)
};
/* Initializes NFC functionality, including TNEP support, NDEF, NFC field sensing */
static void nfc_init(void)
{
	int err;

	/* TNEP init */
	err = nfc_tnep_tag_tx_msg_buffer_register(tnep_buffer, tnep_swap_buffer, sizeof(tnep_buffer)); // Me: Register two buffers for sending and receiving TNEP messages. They store messages exchanged between NFC tag and NFC reader
	if (err) {
		printk("Cannot register tnep buffer, err: %d\n", err);
		return;
	}

	err = nfc_tnep_tag_init(events, NFC_TNEP_EVENTS_NUMBER, nfc_t4t_ndef_rwpayload_set); // Me: Initialize the TNEP stack for the NFC tag, allows dynamic exchange of data over NFC
	if (err) {
		printk("Cannot initialize TNEP protocol, err: %d\n", err);
		return;
	}

	/* Set up NFC */
	err = nfc_t4t_setup(nfc_callback, NULL); // Me: Configures the NFC Type 4 Tag library
	if (err) {
		printk("Cannot setup NFC T4T library!\n");
		return;
	}

	err = nfc_tnep_tag_initial_msg_create(2, tnep_initial_msg_encode); // Me: Prepares the initial message to be presented to an NFC reader when it interacts with a tag
	if (err) {
		printk("Cannot create initial TNEP message, err: %d\n", err);
	}

	err = nfc_tnep_ch_service_init(&ch_cb); // Me: Initialize TNEP Connection Handover (CH) service which facilitates pairing process. Links the global nfc_tnep_ch_cb structure
	if (err) {
		printk("TNEP CH Service init error: %d\n", err);
		return;
	}

	/* Start sensing NFC field */
	err = nfc_t4t_emulation_start(); // Me: Starts the NFC field sensing process. The tag can now detect NFC readers, and respond to read/write requests from devices
	if (err) {
		printk("Cannot start emulation!\n");
		return;
	}

	printk("NFC configuration done\n");
}
/* Button changed callback*/
void button_changed(uint32_t button_state, uint32_t has_changed)
{
	int err;
	uint32_t buttons = button_state & has_changed;

	if (buttons & KEY_BOND_REMOVE_MASK) {
		err = bt_unpair(BT_ID_DEFAULT, NULL);
		if (err) {
			printk("Bond remove failed err: %d\n", err);
		} else {
			printk("All bond removed\n");
		}
	}
}



/* BLE functions, callback structures, and threading */

/* Define the send thread function  */
void send_data_thread(void)
{
	while(1){

        k_sem_take(&data_ready_sem, K_FOREVER); // Me: wait until data is ready
        
        for(int i=0; i<SAADC_BUFFER_SIZE; i +=  ble_tx_max_len/sizeof(int16_t)){

            int chunk_size = ((i + ble_tx_max_len / sizeof(int16_t)) <= SAADC_BUFFER_SIZE) 
                             ? (ble_tx_max_len / sizeof(int16_t)) 
                             : (SAADC_BUFFER_SIZE - i);
 
            uint16_t packet_data[chunk_size];

            for(int j=0; j<chunk_size; j++){
                packet_data[j] = saadc_sample_buffer[saadc_current_buffer % 2][i+j];
            }
            my_lbs_send_sensor_notify(packet_data, sizeof(packet_data));
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
        }
		
	}
}
/* Define and initialize the thread "send_data_thread" to send data periodically */
K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread, NULL, NULL,NULL, PRIORITY, 0, 0);
/* Forward declaration of exchange_func(): */
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);
/* Change the BLE data length to max*/
static void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX, // Me: 251 bytes 
        .tx_max_time = BT_GAP_DATA_TIME_MAX, // Me: 17040 us
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len); // Previously my_conn
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}
/* Update the new MTU length based on data length */
static void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}
/* Connection callback when a BLE connection is established */
static void on_connected(struct bt_conn *conn, uint8_t err){
    // Me: Get the address of the BLE connection
    char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    // Me: Connection failure or timeout
	if (err) {
		if (err == BT_HCI_ERR_ADV_TIMEOUT) {
			printk("Direct advertising to %s timed out\n", addr);
			k_work_submit(&adv_work);
		} else {
			printk("Failed to connect to %s (%u)\n", addr, err);
		}

		return;
	}

	// Me: Get the information of the current connection
    struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}

	/* Add the connection parameters to your log */
	double connection_interval = info.le.interval*1.25; // in ms
	uint16_t supervision_timeout = info.le.timeout*10; // in ms
	LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, info.le.latency, supervision_timeout);

    /* Update the data length and MTU */
	update_data_length(conn); // Previously my_conn
	update_mtu(conn); // Previously my_conn

	mtu_exchanged = false;

    // Me: Advertise only if the number of connection is less than the max allowable
    if (conn_cnt < 2) { // Test: previously CONFIG_BT_MAX_CONN
		advertising_start();
	}

	conn_cnt++;

	printk("Connected\n");
	dk_set_led_on(CON_STATUS_LED);

    k_thread_start(send_data_thread_id); // Me: Start the thread used to send data when buffer is full
}
/* Disconnection callback when a BLE connection is terminated */
static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	bt_le_adv_stop(); // Test

    char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	conn_cnt--;

    // Me: Turn off the LED if no connections are active
	if (!conn_cnt) {
		dk_set_led_off(CON_STATUS_LED);
	}
	printk("Disconnected from %s (reason %u)\n", addr, reason);
}
/* Callback for connection parameter updates */
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval*1.25;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms
    LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}
/* Callback for data length updates */
void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    uint16_t tx_len     = info->tx_max_len; 
    uint16_t tx_time    = info->tx_max_time;
    uint16_t rx_len     = info->rx_max_len;
    uint16_t rx_time    = info->rx_max_time;
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}
/* Structure holding all connection-related callbacks */
struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
    /* Add the callback for connection parameter updates */
	.le_param_updated       = on_le_param_updated,
    /* Add the callback for data length updates */
	.le_data_len_updated    = on_le_data_len_updated,
    /* Add the callback for security changed */
    .security_changed = security_changed,
};
/* Callback function for MTU exchange between peripheral and central */
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params)
{
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);

        ble_tx_max_len = payload_mtu; // Me: Let it be globally known payload length
    }
	mtu_exchanged = true;
}


/* SAADC Functions and channel structure*/

/* Structure that holds the SAADC channels and their configuration */
static nrfx_saadc_channel_t multiple_channels[] = {
    NRFX_SAADC_DEFAULT_CHANNEL_DIFFERENTIAL(NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1, 0),
    NRFX_SAADC_DEFAULT_CHANNEL_DIFFERENTIAL(NRF_SAADC_INPUT_AIN2, NRF_SAADC_INPUT_AIN4, 1)
};
/* Configure the timer used for the sampling */
static void configure_timer(void)
{
    nrfx_err_t err;
    /* Declaring timer config and intialize nrfx_timer instance. */
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000); // Me: Frequecy of the timer is 1MHz
    err = nrfx_timer_init(&timer_instance, &timer_config, NULL); // Me: initialise the timer globally
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: %08x", err);
        return;
    }

    /* Set compare channel 0 to generate event every SAADC_SAMPLE_INTERVAL_US. */
    uint32_t timer_ticks = nrfx_timer_us_to_ticks(&timer_instance, SAADC_SAMPLE_INTERVAL_US); // Me: convert time from microseconds to timer ticks
    nrfx_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, timer_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

}
/* What happens at each sample event*/
static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t err;
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_READY:
        
           /* Buffer is ready, timer (and sampling) can be started. */
            nrfx_timer_enable(&timer_instance);

            break;                        
            
        case NRFX_SAADC_EVT_BUF_REQ:

            // Me: alternate the buffers
            err = nrfx_saadc_buffer_set(saadc_sample_buffer[(saadc_current_buffer++)%2], SAADC_BUFFER_SIZE);
            if (err != NRFX_SUCCESS) {
                LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
                return;
            }

            break;

        case NRFX_SAADC_EVT_DONE:

            /* Buffer has been filled. Do something with the data and proceed */
            int64_t average_AIN0 = 0;
            int16_t max_AIN0 = INT16_MIN;
            int16_t min_AIN0 = INT16_MAX;

            int64_t average_AIN1 = 0;
            int16_t max_AIN1 = INT16_MIN;
            int16_t min_AIN1 = INT16_MAX;

            int16_t current_value;

            // Data in the buffer is interleaved for the channels
            for (int i = 0; i < p_event->data.done.size; i++) {

                // Channel 0 value
                current_value = ((int16_t *)(p_event->data.done.p_buffer))[i];

                average_AIN0 += current_value;
                if (current_value > max_AIN0) {
                    max_AIN0 = current_value;
                }
                if (current_value < min_AIN0) {
                    min_AIN0 = current_value;
                }

                // Channel 1 value
                i++;

                // First value in the buffer
                current_value = ((int16_t *)(p_event->data.done.p_buffer))[i];

                average_AIN1 += current_value;
                if (current_value > max_AIN1) {
                    max_AIN1 = current_value;
                }
                if (current_value < min_AIN1) {
                    min_AIN1 = current_value;
                }

            }

            average_AIN0 = average_AIN0 / (p_event->data.done.size/NRFX_SAADC_CHANNEL_COUNT);

            average_AIN1 = average_AIN1 / (p_event->data.done.size/NRFX_SAADC_CHANNEL_COUNT);


            //LOG_INF("SAADC buffer at 0x%x filled with %d samples", (uint32_t)p_event->data.done.p_buffer, p_event->data.done.size);
            //LOG_INF("Channel 0: AVG=%d, MIN=%d, MAX=%d", (int16_t)average_AIN0, min_AIN0, max_AIN0);
            //LOG_INF("Channel 1: AVG=%d, MIN=%d, MAX=%d", (int16_t)average_AIN1, min_AIN1, max_AIN1);
            //LOG_INF("First=%d", ((int16_t *)(p_event->data.done.p_buffer))[0]);

			if(mtu_exchanged)
            	k_sem_give(&data_ready_sem);

            break;

        default:
            LOG_INF("Unhandled SAADC evt %d", p_event->type);
            break;
    }
}
/* Configure the SAADC */
static void configure_saadc(void)
{
    nrfx_err_t err;

    /* Connect ADC interrupt to nrfx interrupt handler */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
            DT_IRQ(DT_NODELABEL(adc), priority),
            nrfx_isr, nrfx_saadc_irq_handler, 0);

    
    /* Initialize the nrfx_SAADC driver */
    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_init error: %08x", err);
        return;
    }

    /* Change gain config in default config and apply channel configuration */
    multiple_channels[0].channel_config.gain = NRF_SAADC_GAIN1_6;
    multiple_channels[1].channel_config.gain = NRF_SAADC_GAIN1_6;

    err = nrfx_saadc_channels_config(&multiple_channels, NRFX_SAADC_CHANNEL_COUNT);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_channels_config error: %08x", err);
        return;
    }

    uint32_t channels_mask = nrfx_saadc_channels_configured_get();

    /* Configure channel in advanced mode with event handler (non-blocking mode) */
    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;

    err = nrfx_saadc_advanced_mode_set(channels_mask, // Me: Enable channel  1 and 0 with the mask
                                    NRF_SAADC_RESOLUTION_12BIT,
                                    &saadc_adv_config,
                                    saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_advanced_mode_set error: %08x", err);
        return;
    }
                                            
    /* Configure two buffers to make use of double-buffering feature of SAADC for each channel */
    err = nrfx_saadc_buffer_set(saadc_sample_buffer[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }
    err = nrfx_saadc_buffer_set(saadc_sample_buffer[1], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }
    

    /* Trigger the SAADC. This will not start sampling, but will prepare buffer for sampling triggered through PPI */
    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
        return;
    }

}
/* Configure the Programmable Peripheral Interface, to clock the SAADC */
static void configure_ppi(void)
{
    nrfx_err_t err;
    /* Declare variables used to hold the (D)PPI channel number */
    uint8_t m_saadc_sample_ppi_channel;
    uint8_t m_saadc_start_ppi_channel;

    /* Trigger task sample from timer */
    err = nrfx_gppi_channel_alloc(&m_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }

    err = nrfx_gppi_channel_alloc(&m_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }


    /* Trigger task sample from timer */
    nrfx_gppi_channel_endpoints_setup(m_saadc_sample_ppi_channel,
				  nrfx_timer_compare_event_address_get(&timer_instance,
								       NRF_TIMER_CC_CHANNEL0),
				  nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));


    /* Trigger task start from end event */
    nrfx_gppi_channel_endpoints_setup(m_saadc_start_ppi_channel,
				  nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
				  nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    /* Enable both (D)PPI channels */ 
    nrfx_gppi_channels_enable(BIT(m_saadc_sample_ppi_channel));
    nrfx_gppi_channels_enable(BIT(m_saadc_start_ppi_channel));
}


int main(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("LEDs init failed (err %d)\n", err);
		return -1;
	}

    /* Configure buttons */
	err = dk_buttons_init(button_changed);
	if (err) {
		printk("Cannot init buttons (err %d\n", err);
	}

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		printk("Failed to register authorization callbacks.\n");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

    LOG_INF("Bluetooth initialized\n");

    // Me: Initialize Settings subsystem after enabling BLE. Else advertising of stored bonds fails with err -11 cause it wasn't ready
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_load();
        if (err) {
            printk("Failed to load settings (err %d)\n", err);
        } else {
            printk("Settings loaded\n");
        }
    }

    // Me: Generate the paring key
    err = paring_key_generate();
	if (err) {
		return 0;
	}

    // Me: Start the adv_handler working
	k_work_init(&adv_work, adv_handler);
	pair_key_generate_init();
	nfc_init();

	bt_conn_cb_register(&connection_callbacks);

    configure_timer();
    configure_saadc();  
    configure_ppi();

	for (;;) {
        k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		nfc_tnep_tag_process();
		paring_key_process();
	}

}

