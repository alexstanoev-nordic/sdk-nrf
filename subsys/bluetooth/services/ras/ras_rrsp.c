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

static struct bt_ras_rrsp m_rrsp_pool[CONFIG_BT_RAS_MAX_ACTIVE_RRSP];
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

	LOG_DBG("ras_cp_write len %d", len);
	LOG_HEXDUMP_DBG(buf, len, "ras_cp_write");

	struct net_buf_simple req;
	net_buf_simple_init_with_data(&req, (uint8_t *)buf, len);

	rrsp_rascp_cmd_handle(conn, &req);

	return len;
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

#if defined(CONFIG_BT_RAS_AUTO_ALLOC_RRSP_INSTANCE)
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_DBG("Allocating RRSP for %s\n", addr);

	bt_ras_rrsp_alloc(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_DBG("Freeing RRSP for %s (reason 0x%02x)", addr, reason);

	bt_ras_rrsp_free(conn);
}
#endif

static struct bt_conn_cb conn_callbacks = {
#if defined(CONFIG_BT_RAS_AUTO_ALLOC_RRSP_INSTANCE)
	.connected = connected,
	.disconnected = disconnected,
#endif
};

static int rrsp_chunk_send(struct bt_ras_rrsp *rrsp)
{
	int err;
	__ASSERT_NO_MSG(rrsp->conn);

	/* By spec, up to (ATT_MTU-4) octets of the data to be transported
	 * shall be used to fill the characteristic message.
	 * An extra byte is reserved for the segment header */
	uint16_t max_data_len = bt_gatt_get_mtu(rrsp->conn) - (4 + sizeof(struct ras_seg_header));

	NET_BUF_SIMPLE_DEFINE(buf, (sizeof(struct ras_seg_header) + max_data_len));

	struct ras_segment *ras_segment = net_buf_simple_add(&buf, sizeof(struct ras_segment) + max_data_len);
	if (!ras_segment) {
		LOG_ERR("Cannot allocate segment buffer");
		return -ENOMEM;
	}

	uint16_t actual_data_len = bt_ras_rd_buffer_segment_get(rrsp, ras_segment->data, max_data_len);

	if (actual_data_len) {
		ras_segment->header.first_seg = true; /* TODO */
		ras_segment->header.last_seg = true;
		ras_segment->header.seg_counter = rrsp->segment_counter & BIT_MASK(6);

		rrsp->segment_counter++;

		(void)net_buf_simple_remove_mem(&buf, (max_data_len - actual_data_len));

		err = rrsp_ondemand_rd_notify_or_indicate(rrsp->conn, &buf);
		if (err) {
			LOG_WRN("rrsp_ondemand_rd_notify_or_indicate failed err %d", err);

			return err;
		}

		LOG_DBG("Chunk with RSC %d sent", rrsp->segment_counter);
	}

	uint16_t bytes_left = bt_ras_rd_buffer_bytes_left_get(rrsp);
	if (bytes_left) {
		k_work_submit(&rrsp->send_data_work);
	} else {
		LOG_DBG("all chunks sent");
		rrsp_rascp_send_complete_rd_rsp(rrsp->conn, rrsp->streaming_ranging_counter);
		rrsp->streaming = false;
		k_work_cancel(&rrsp->send_data_work);
	}

	return 0;
}

static void send_data_work_handler(struct k_work *work)
{
	struct bt_ras_rrsp *rrsp = CONTAINER_OF(work, struct bt_ras_rrsp, send_data_work);
	LOG_DBG("rrsp %p", rrsp);

	if (!rrsp->streaming) {
		return;
	}

	int err = rrsp_chunk_send(rrsp);
	if (err) {
		/* TODO: handle error */
		LOG_ERR("rrsp_chunk_send failed: %d", err);
	}

}

struct bt_ras_rrsp *bt_ras_rrsp_find(struct bt_conn *conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(m_rrsp_pool); i++) {
		struct bt_ras_rrsp *rrsp = &m_rrsp_pool[i];

		if (rrsp->conn == conn) {
			return rrsp;
		}
	}

	return NULL;
}

int bt_ras_rrsp_alloc(struct bt_conn *conn)
{
	struct bt_ras_rrsp *rrsp = NULL;

	if (bt_ras_rrsp_find(conn) != NULL) {
		return -EALREADY;
	}

	for (size_t i = 0; i < ARRAY_SIZE(m_rrsp_pool); i++) {
		if (m_rrsp_pool[i].conn == NULL) {
			rrsp = &m_rrsp_pool[i];
			break;
		}
	}

	if (rrsp == NULL) {
		return -ENOMEM;
	}

	LOG_DBG("conn %p new rrsp %p", (void *)conn, (void *)rrsp);

	memset(rrsp, 0, sizeof(struct bt_ras_rrsp));
	rrsp->conn = bt_conn_ref(conn);

	k_work_init(&rrsp->send_data_work, send_data_work_handler);

	// A maximum timeout of 10 seconds, or a shorter implementation configurable timeout value, has
	// elapsed since the RRSP indicated a Complete Ranging Data indication and the RREQ did not
	// respond with an ACK_Ranging_Data command or a Retrieve_Lost_Ranging_Data command.
	//k_timer_init(&my_timer, my_expiry_function, NULL);

	return 0;
}

void bt_ras_rrsp_free(struct bt_conn *conn)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		LOG_DBG("conn %p rrsp %p", (void *)rrsp->conn, (void *)rrsp);

		bt_conn_unref(rrsp->conn);
		rrsp->conn = NULL;

		(void)k_work_cancel(&rrsp->send_data_work);
	}
}

int bt_ras_rrsp_subevent_store(struct bt_conn *conn, struct net_buf_simple *subevent_buf)
{
	/* TODO: Call bt_ras_rd_buffer_subevent_append */
	/* TODO: Trigger workqueue to keep feeding data to clients */

	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	__ASSERT(rrsp, "Non-existing RRSP");

	// if overwritten: rrsp_rd_overwritten_indicate(conn, 0);
	// if ready: rrsp_rd_ready_indicate(conn, 1);

	k_work_submit(&rrsp->send_data_work);

	return 0;
}

int bt_ras_rrsp_init(void)
{
	m_ras_features = 0;
#if defined(CONFIG_BT_RAS_REALTIME_RANGING_DATA)
	m_ras_features |= 1 << RAS_FEAT_REALTIME_RD_BIT;
#endif

	bt_conn_cb_register(&conn_callbacks);

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

static void ondemand_rd_notify_sent_cb(struct bt_conn *conn, void *user_data)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		LOG_DBG("");
		k_work_submit(&rrsp->send_data_work);
	}
}

static void ondemand_rd_indicate_sent_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		LOG_DBG("");
		k_work_submit(&rrsp->send_data_work);
	}
}

int rrsp_ondemand_rd_notify_or_indicate(struct bt_conn *conn, struct net_buf_simple *buf)
{
	struct bt_gatt_attr *attr = bt_gatt_find_by_uuid(rrsp_svc.attrs, 1, BT_UUID_RAS_ONDEMAND_RD);
	__ASSERT_NO_MSG(attr);

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		struct bt_gatt_indicate_params params = {0};

		params.attr = attr;
		params.uuid = NULL;
		params.data = buf->data;
		params.len = buf->len;
		params.func = ondemand_rd_indicate_sent_cb;
		params.destroy = NULL;

		return bt_gatt_indicate(conn, &params);
	} else if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		struct bt_gatt_notify_params params = {0};

		params.attr = attr;
		params.uuid = NULL;
		params.data = buf->data;
		params.len = buf->len;
		params.func = ondemand_rd_notify_sent_cb;

		return bt_gatt_notify_cb(conn, &params);
	} else {
		return -EINVAL;
	}
}

int rrsp_rascp_indicate(struct bt_conn *conn, struct net_buf_simple *rsp)
{
	struct bt_gatt_attr *attr = bt_gatt_find_by_uuid(rrsp_svc.attrs, 1, BT_UUID_RAS_CP);
	__ASSERT_NO_MSG(attr);

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		struct bt_gatt_indicate_params params = {0};

		params.attr = attr;
		params.uuid = NULL;
		params.data = rsp->data;
		params.len = rsp->len;
		params.func = NULL;
		params.destroy = NULL;

		return bt_gatt_indicate(conn, &params);
	}

	return -EINVAL;
}

static int rd_status_notify_or_indicate(struct bt_conn *conn, const struct bt_uuid *uuid, uint16_t ranging_counter)
{
	struct bt_gatt_attr *attr = bt_gatt_find_by_uuid(rrsp_svc.attrs, 1, uuid);
	__ASSERT_NO_MSG(attr);

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		return bt_gatt_notify(conn, attr, &ranging_counter, sizeof(ranging_counter));
	} else if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		struct bt_gatt_indicate_params params = {0};

		params.attr = attr;
		params.uuid = NULL;
		params.data = &ranging_counter;
		params.len = sizeof(ranging_counter);
		params.func = NULL;
		params.destroy = NULL;

		return bt_gatt_indicate(conn, &params);
	} else {
		return -EINVAL;
	}
}

int rrsp_rd_ready_indicate(struct bt_conn *conn, uint16_t ranging_counter)
{
	return rd_status_notify_or_indicate(conn, BT_UUID_RAS_RD_READY, ranging_counter);
}

int rrsp_rd_overwritten_indicate(struct bt_conn *conn, uint16_t ranging_counter)
{
	return rd_status_notify_or_indicate(conn, BT_UUID_RAS_RD_OVERWRITTEN, ranging_counter);
}
