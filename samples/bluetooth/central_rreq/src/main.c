/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <bluetooth/services/ras.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

NET_BUF_SIMPLE_DEFINE_STATIC(ranging_data, 5500);
static K_SEM_DEFINE(setup_sem, 0, 1);
static K_SEM_DEFINE(rd_ready_sem, 0, 1);
static K_SEM_DEFINE(rd_complete_sem, 0, 1);

static struct bt_conn *default_conn;
static uint16_t most_recent_ranging_counter;

void ranging_data_get_complete_cb(int err, uint16_t ranging_counter)
{
	if (err) {
		printk("Error %d, when getting ranging data with ranging counter %d\n", err,
		       ranging_counter);
	} else {
		printk("Ranging data get completed for ranging counter %d\n", ranging_counter);
	}

	k_sem_give(&rd_complete_sem);
}

static void ranging_data_ready_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	printk("Ranging data ready %i\n", ranging_counter);
	most_recent_ranging_counter = ranging_counter;
	k_sem_give(&rd_ready_sem);
}

static void ranging_data_overwritten_cb(struct bt_conn *conn, uint16_t ranging_counter)
{
	printk("Ranging data overwritten %i\n", ranging_counter);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing completed: %s, bonded: %d\n", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing failed conn: %s, reason %d %s\n", addr, reason,
	       bt_security_err_to_str(reason));
}

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {.pairing_complete = pairing_complete,
							       .pairing_failed = pairing_failed};

static void discovery_completed_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	printk("The discovery procedure succeeded\n");

	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);

	bt_gatt_dm_data_print(dm);

	err = bt_ras_rreq_alloc_and_assign_handles(dm, conn);
	if (err) {
		printk("RAS RREQ alloc init failed, err %d\n", err);
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		printk("Could not release the discovery data, err %d\n", err);
	}

	k_sem_give(&setup_sem);
}

static void disconnect(struct bt_conn *conn)
{
	int err;

	err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	if (err) {
		printk("Disconnect failed, err %d\n", err);
	}
}

static void discovery_service_not_found_cb(struct bt_conn *conn, void *context)
{
	printk("The service could not be found during the discovery, disconnecting\n");
	disconnect(conn);
}

static void discovery_error_found_cb(struct bt_conn *conn, int err, void *context)
{
	printk("The discovery procedure failed, err %d\n", err);
	disconnect(conn);
}

static struct bt_gatt_dm_cb discovery_cb = {
	.completed = discovery_completed_cb,
	.service_not_found = discovery_service_not_found_cb,
	.error_found = discovery_error_found_cb,
};

static void gatt_discover(struct bt_conn *conn)
{
	int err;

	printk("Starting GATT service discovery");

	err = bt_gatt_dm_start(conn, BT_UUID_RANGING_SERVICE, &discovery_cb, NULL);
	if (err) {
		printk("Could not start the discovery procedure, err %d\n", err);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Error connecting, err %d\n", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected %s\n", addr);

	default_conn = bt_conn_ref(conn);
	k_sem_give(&setup_sem);
}

static void start_scanning(void)
{
	int err;
	err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
	if (err) {
		printk("Scanning failed to start, err %i\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected: %s (reason 0x%02x)", addr, reason);

	bt_ras_rreq_free(conn);

	default_conn = NULL;
	bt_conn_unref(conn);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match, bool connectable)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

	printk("Filters matched. Address: %s connectable: %d", addr, connectable);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed, restarting scanning");

	start_scanning();
}

static void scan_connecting(struct bt_scan_device_info *device_info, struct bt_conn *conn)
{
	printk("Connecting");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

static void scan_init(void)
{
	int err;

	struct bt_scan_init_param param = {
		.scan_param = NULL, .conn_param = BT_LE_CONN_PARAM_DEFAULT, .connect_if_match = 1};

	bt_scan_init(&param);
	bt_scan_cb_register(&scan_cb);

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_RANGING_SERVICE);
	if (err) {
		printk("Scanning filters cannot be set, err %d\n", err);
	}

	err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
	if (err) {
		printk("Filters cannot be turned on, err %d\n", err);
	}
}

int main(void)
{
	int err;

	printk("Starting Bluetooth Central RREQ example\n");

	bt_conn_auth_cb_register(&auth_cb_display);

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed, err %d\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	scan_init();

	start_scanning();

	err = k_sem_take(&setup_sem, K_FOREVER);
	__ASSERT_NO_MSG(!err);

	gatt_discover(default_conn);

	err = k_sem_take(&setup_sem, K_SECONDS(5));
	if (err) {
		printk("Timeout waiting for gatt discovery, err %d\n", err);
		return 0;
	}

	err = bt_ras_rreq_on_demand_ranging_data_subscribe_all(default_conn, ranging_data_ready_cb,
							       ranging_data_overwritten_cb);
	if (err) {
		printk("RAS RREQ On-demand ranging data subscribe all failed, err %d\n", err);
		return 0;
	}

	while (true) {
		err = k_sem_take(&rd_ready_sem, K_SECONDS(5));
		if (err) {
			printk("Timeout waiting for ranging data ready, err %d\n", err);
			return 0;
		}

		net_buf_simple_reset(&ranging_data);

		err = bt_ras_rreq_cp_get_ranging_data(default_conn, &ranging_data,
						      most_recent_ranging_counter,
						      ranging_data_get_complete_cb);
		if (err) {
			printk("Get ranging data, err %d\n", err);
		}

		err = k_sem_take(&rd_complete_sem, K_SECONDS(5));
		if (err) {
			printk("Timeout waiting for ranging data complete, err %d\n", err);
			return 0;
		}
	}
}
