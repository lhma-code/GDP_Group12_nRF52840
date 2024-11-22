/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

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

LOG_MODULE_REGISTER(Group12_Test, LOG_LEVEL_DBG);

/* Include header for nrfx drivers */
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

/* Define the SAADC sample interval in microseconds */
#define SAADC_SAMPLE_INTERVAL_US 500

/* Define the buffer size for the SAADC */
#define SAADC_BUFFER_SIZE 4000

/* Bluetooth LE */

static K_SEM_DEFINE(data_ready_sem, 0, 1);

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONNECTABLE |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	800, /* Min Advertising Interval 500ms (800*0.625ms) */
	801, /* Max Advertising Interval 500.625ms (801*0.625ms) */
	NULL); /* Set to NULL for undirected advertising */

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define USER_LED DK_LED3
#define USER_BUTTON DK_BTN1_MSK

#define STACKSIZE 1024
#define PRIORITY 7

#define RUN_LED_BLINK_INTERVAL 1000
#define NOTIFY_INTERVAL 20 // Me: Notification interval

/* Advertising and Scan Data*/
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

/* Forward declaration of exchange_func(): */
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);

/* Define the data you want to stream over Bluetooth LE */
//static int16_t app_sensor_value = 100; //static int16_t app_sensor_value[SAADC_BUFFER_SIZE];

static uint16_t ble_tx_max_len; // Me: Self-defined, maximum transmission payload length

struct bt_conn *my_conn = NULL; // Me: Change connection parameters globally

static struct bt_gatt_exchange_params exchange_params;

/* Declaring an instance of nrfx_timer for TIMER2. */
const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(2);
/* Declare the buffers for the SAADC */
static int16_t saadc_sample_buffer[2][SAADC_BUFFER_SIZE];
/* Declare variable used to keep track of which buffer was last assigned to the SAADC driver */
static uint32_t saadc_current_buffer = 0;

//static uint32_t app_sensor_value = 100;

/* Define the send thread function  */
void send_data_thread(void)
{
	while(1){

        k_sem_take(&data_ready_sem, K_FOREVER); // Me: wait until data is ready
        
        //my_lbs_send_sensor_notify(app_sensor_value);
        //k_sleep(K_MSEC(NOTIFY_INTERVAL));

        
        for(int i=0; i<SAADC_BUFFER_SIZE; i +=  ble_tx_max_len/sizeof(int16_t)){

            int chunk_size = ((i + ble_tx_max_len / sizeof(int16_t)) <= SAADC_BUFFER_SIZE) 
                             ? (ble_tx_max_len / sizeof(int16_t)) 
                             : (SAADC_BUFFER_SIZE - i);
 
            //uint16_t packet_data[2] = {3435, 3460};
            uint16_t packet_data[chunk_size];

            for(int j=0; j<chunk_size; j++){
                packet_data[j] = saadc_sample_buffer[saadc_current_buffer % 2][i+j];
            }

            //uint16_t packet_data_test[5] = {-3, -158, -900, -1758, -2007};

            my_lbs_send_sensor_notify(packet_data, sizeof(packet_data));
            k_sleep(K_MSEC(NOTIFY_INTERVAL));
        }
		
	}
}

/* Define and initialize a thread to send data periodically */
K_THREAD_DEFINE(send_data_thread_id, STACKSIZE, send_data_thread, NULL, NULL,NULL, PRIORITY, 0, 0);

static void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX, // Me: 251 bytes 
        .tx_max_time = BT_GAP_DATA_TIME_MAX, // Me: 17040 us
    };
    err = bt_conn_le_data_len_update(my_conn, &my_data_len);
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

/* Define the function to update the connection's MTU */
static void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    // Me: Connection housekeeping
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}
	printk("Connected\n");
	dk_set_led_on(CON_STATUS_LED);

    my_conn = bt_conn_ref(conn); // Me: make the connection global

    // Me: Get connection information
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
	update_data_length(my_conn);
	update_mtu(my_conn);

    k_thread_start(send_data_thread_id); // Me: Start the thread used to send data when buffer is full
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	dk_set_led_off(CON_STATUS_LED);
}

/* Add the callback for connection parameter updates */
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval*1.25;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms
    LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}

void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    uint16_t tx_len     = info->tx_max_len; 
    uint16_t tx_time    = info->tx_max_time;
    uint16_t rx_len     = info->rx_max_len;
    uint16_t rx_time    = info->rx_max_time;
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
    /* Add the callback for connection parameter updates */
	.le_param_updated       = on_le_param_updated,
    /* Add the callback for data length updates */
	.le_data_len_updated    = on_le_data_len_updated,
};

/* Implement callback function for MTU exchange */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
			  struct bt_gatt_exchange_params *params)
{
	LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);

        ble_tx_max_len = payload_mtu; // Me: Let it be globally known payload length
    }
}


/* Successive Approximation ADC */

static void configure_timer(void)
{
    nrfx_err_t err;
    /* STEP 3.3 - Declaring timer config and intialize nrfx_timer instance. */
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
    err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: %08x", err);
        return;
    }

    /* STEP 3.4 - Set compare channel 0 to generate event every SAADC_SAMPLE_INTERVAL_US. */
    uint32_t timer_ticks = nrfx_timer_us_to_ticks(&timer_instance, SAADC_SAMPLE_INTERVAL_US);
    nrfx_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, timer_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

}

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
        
            /* Set up the next available buffer. Alternate between buffer 0 and 1 */
            err = nrfx_saadc_buffer_set(saadc_sample_buffer[(saadc_current_buffer++)%2], SAADC_BUFFER_SIZE);
            if (err != NRFX_SUCCESS) {
                LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
                return;
            }

            break;

        case NRFX_SAADC_EVT_DONE:

            /* Buffer has been filled. Do something with the data and proceed */
            int64_t average = 0;
            int16_t max = INT16_MIN;
            int16_t min = INT16_MAX;
            int16_t current_value;

            //LOG_INF("New Log");
            for (int i = 0; i < p_event->data.done.size; i++) {
                current_value = ((int16_t *)(p_event->data.done.p_buffer))[i];
                average += current_value;
                //LOG_INF("Current Value=%d", current_value);
                if (current_value > max) {
                    max = current_value;
                }
                if (current_value < min) {
                    min = current_value;
                }
            }
            average = average / p_event->data.done.size;
            //LOG_INF("SAADC buffer at 0x%x filled with %d samples", (uint32_t)p_event->data.done.p_buffer, p_event->data.done.size);
            LOG_INF("AVG=%d, MIN=%d, MAX=%d", (int16_t)average, min, max);
            //LOG_INF("First=%d", ((int16_t *)(p_event->data.done.p_buffer))[0]);

            k_sem_give(&data_ready_sem);

            break;

        default:
            LOG_INF("Unhandled SAADC evt %d", p_event->type);
            break;
    }
}

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

    
    /* Declare the struct to hold the configuration for the SAADC channel used to sample the battery voltage */
    nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_DIFFERENTIAL(NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1, 0);

    /* Change gain config in default config and apply channel configuration */
    channel.channel_config.gain = NRF_SAADC_GAIN1_6;
    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_channels_config error: %08x", err);
        return;
    }

    /* Configure channel 0 in advanced mode with event handler (non-blocking mode) */
    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(BIT(0),
                                    NRF_SAADC_RESOLUTION_12BIT,
                                    &saadc_adv_config,
                                    saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_advanced_mode_set error: %08x", err);
        return;
    }
                                            
    /* Configure two buffers to make use of double-buffering feature of SAADC */
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

static void configure_ppi(void)
{
    nrfx_err_t err;
    /* STEP 6.1 - Declare variables used to hold the (D)PPI channel number */
    uint8_t m_saadc_sample_ppi_channel;
    uint8_t m_saadc_start_ppi_channel;

    /* STEP 6.2 - Trigger task sample from timer */
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


    /* STEP 6.3 - Trigger task sample from timer */
    nrfx_gppi_channel_endpoints_setup(m_saadc_sample_ppi_channel,
				  nrfx_timer_compare_event_address_get(&timer_instance,
								       NRF_TIMER_CC_CHANNEL0),
				  nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));


    /* STEP 6.4 - Trigger task start from end event */
    nrfx_gppi_channel_endpoints_setup(m_saadc_start_ppi_channel,
				  nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
				  nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    /* STEP 6.5 - Enable both (D)PPI channels */ 
    nrfx_gppi_channels_enable(BIT(m_saadc_sample_ppi_channel));
    nrfx_gppi_channels_enable(BIT(m_saadc_start_ppi_channel));
}


int main(void)
{
    int blink_status = 0;
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("LEDs init failed (err %d)\n", err);
		return -1;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}
	bt_conn_cb_register(&connection_callbacks);

	LOG_INF("Bluetooth initialized\n");
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return -1;
	}

	LOG_INF("Advertising successfully started\n");

    k_sleep(K_MSEC(100));

    configure_timer();
    configure_saadc();  
    configure_ppi();

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}

}

