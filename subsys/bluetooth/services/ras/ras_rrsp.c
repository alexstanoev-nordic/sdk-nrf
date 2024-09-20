/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/ead.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/check.h>
#include <zephyr/net/buf.h>
#include <bluetooth/services/ras.h>

#include "ras_internal.h"

LOG_MODULE_REGISTER(ras, CONFIG_BT_RAS_LOG_LEVEL);

static uint32_t m_ras_features;

static ssize_t ras_features_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &m_ras_features, sizeof(m_ras_features));
}

static ssize_t ras_cp_write(struct bt_conn *conn, struct bt_gatt_attr const *attr,
			    void const *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("RAS-CP write");

	if (!bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		return BT_GATT_ERR(RAS_ATT_ERROR_CCC_CONFIG);
	}

	uint8_t const * cmd = (uint8_t const *)buf;

	LOG_DBG("ras_cp_write len %d", len);
	LOG_HEXDUMP_DBG(buf, len, "ras_cp_write");

	return rrsp_rasp_cmd_handle(conn, cmd, len);
}

static void ondemand_rd_ccc_cfg_changed(struct bt_gatt_attr const *attr, uint16_t value)
{
	LOG_DBG("On-demand Ranging Data CCCD changed: %u", value);
}

static void ras_cp_ccc_cfg_changed(struct bt_gatt_attr const *attr, uint16_t value)
{
	LOG_DBG("RAS-CP CCCD changed: %u", value);
}

static void rd_ready_ccc_cfg_changed(struct bt_gatt_attr const *attr, uint16_t value)
{
	LOG_DBG("Ranging Data Ready CCCD changed: %u", value);
}

static void rd_overwritten_ccc_cfg_changed(struct bt_gatt_attr const *attr, uint16_t value)
{
	LOG_DBG("Ranging Data Overwritten CCCD changed: %u", value);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_DBG("Connected %s\n", addr);

	/* TODO: Initialize RRSP instance for this connection */
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_DBG("Disconnected: %s (reason 0x%02x)", addr, reason);

	/* TODO: Free RRSP instance for this connection */
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	/* TODO: CS callback to call bt_ras_rrsp_subevent_store */
};

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_DBG("Updated MTU: TX: %d RX: %d bytes", tx, rx);
	/* TODO: Use MTU for segment size */
}

static struct bt_gatt_cb gatt_callbacks = {.att_mtu_updated = mtu_updated};

int bt_ras_rrsp_subevent_store(void)
{
	/* TODO: Call bt_ras_rd_buffer_subevent_append */
	/* TODO: Trigger workqueue to keep feeding data to clients */
	/* TODO: Trigger workqueue from notification sent / indication ACKd callback */

	return 0;
}

int bt_ras_rrsp_init(void)
{
	m_ras_features = 0;
#if defined(CONFIG_BT_RAS_REALTIME_RANGING_DATA)
	m_ras_features |= 1 << RAS_FEAT_REALTIME_RD_BIT;
#endif

	bt_conn_cb_register(&conn_callbacks);
	bt_gatt_cb_register(&gatt_callbacks);

	return 0;
}

BT_GATT_SERVICE_DEFINE(rrsp_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_RANGING_SERVICE),
	/* RAS Features */
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_FEATURES, BT_GATT_CHRC_READ,
				BT_GATT_PERM_READ_ENCRYPT, ras_features_read, NULL, NULL),
#if defined(CONFIG_BT_RAS_REALTIME_RANGING_DATA)
	/* Real-time Ranging Data */
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_REALTIME_RD, BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_NOTIFY,
				BT_GATT_PERM_NONE,
				NULL, NULL, NULL),
	BT_GATT_CCC(realtime_rd_ccc_cfg_changed,
				BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
#endif
	/* On-demand Ranging Data */
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_ONDEMAND_RD, BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_NOTIFY,
				BT_GATT_PERM_NONE,
				NULL, NULL, NULL),
	BT_GATT_CCC(ondemand_rd_ccc_cfg_changed,
				BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	/* RAS-CP */
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_CP, BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_INDICATE,
				BT_GATT_PERM_WRITE_ENCRYPT,
				NULL, ras_cp_write, NULL),
	BT_GATT_CCC(ras_cp_ccc_cfg_changed,
				BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	/* Ranging Data Ready */
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_RD_READY, BT_GATT_CHRC_INDICATE,
				BT_GATT_PERM_NONE,
				NULL, NULL, NULL),
	BT_GATT_CCC(rd_ready_ccc_cfg_changed,
				BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	/* Ranging Data Overwritten */
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_RD_OVERWRITTEN, BT_GATT_CHRC_INDICATE,
				BT_GATT_PERM_NONE,
				NULL, NULL, NULL),
	BT_GATT_CCC(rd_overwritten_ccc_cfg_changed,
				BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
);

static int uuid_indicate(struct bt_conn *conn, const struct bt_uuid *uuid, const struct bt_gatt_attr *attr, const void *data, uint16_t len)
{
	struct bt_gatt_indicate_params ind_params;
	ind_params.uuid = uuid;
	ind_params.attr = attr;
	ind_params.func = NULL;
	ind_params.destroy = NULL;
	ind_params.data = data;
	ind_params.len = len;
	return bt_gatt_indicate(conn, &ind_params);
}

int rrsp_ondemand_rd_notify_or_indicate(struct bt_conn *conn, const void *data, uint16_t len)
{
	struct bt_gatt_attr *attr = bt_gatt_find_by_uuid(rrsp_svc.attrs, 1, BT_UUID_RAS_ONDEMAND_RD);
	__ASSERT_NO_MSG(attr != NULL);

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		return uuid_indicate(conn, BT_UUID_RAS_ONDEMAND_RD, attr, data, len);
	} else if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		return bt_gatt_notify_uuid(conn, BT_UUID_RAS_ONDEMAND_RD, attr, data, len);
	} else {
		return -EINVAL;
	}
}

int rrsp_ras_cp_indicate(struct bt_conn *conn, const void *data, uint16_t len)
{
	return uuid_indicate(conn, BT_UUID_RAS_CP, rrsp_svc.attrs, data, len);
}

int rrsp_rd_ready_indicate(struct bt_conn *conn, uint16_t ranging_counter)
{
	return uuid_indicate(conn, BT_UUID_RAS_RD_READY, rrsp_svc.attrs, &ranging_counter, sizeof(ranging_counter));
}

int rrsp_rd_overwritten_indicate(struct bt_conn *conn, uint16_t ranging_counter)
{
	return uuid_indicate(conn, BT_UUID_RAS_RD_OVERWRITTEN, rrsp_svc.attrs, &ranging_counter, sizeof(ranging_counter));
}
