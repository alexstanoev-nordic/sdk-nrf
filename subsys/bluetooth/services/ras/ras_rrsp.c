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

#define RRSP_WQ_STACK_SIZE 5120
#define RRSP_WQ_PRIORITY 5

K_THREAD_STACK_DEFINE(rrsp_wq_stack_area, RRSP_WQ_STACK_SIZE);

static struct bt_ras_rrsp rrsp_pool[CONFIG_BT_RAS_MAX_ACTIVE_RRSP];
static uint32_t ras_features;
struct k_work_q rrsp_wq;

/* TODO define static net buf or see if one can be used from bt, mtu size
 * instead of defining one on the stack
 */

static int rd_status_notify_or_indicate(struct bt_conn *conn, const struct bt_uuid *uuid, uint16_t ranging_counter);

static ssize_t ras_features_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ras_features, sizeof(ras_features));
}

static ssize_t rd_ready_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	__ASSERT_NO_MSG(rrsp);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &rrsp->ready_ranging_counter, sizeof(&rrsp->ready_ranging_counter));
}

static ssize_t rd_overwritten_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	__ASSERT_NO_MSG(rrsp);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &rrsp->overwritten_ranging_counter, sizeof(&rrsp->overwritten_ranging_counter));
}

static ssize_t ras_cp_write(struct bt_conn *conn, struct bt_gatt_attr const *attr,
			    void const *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_HEXDUMP_DBG(buf, len, "Write request");

	if (!bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		LOG_DBG("Not subscribed");
		return BT_GATT_ERR(RAS_ATT_ERROR_CCC_CONFIG);
	}

	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	__ASSERT_NO_MSG(rrsp);

	if (k_work_is_pending(&rrsp->rascp_work) || len > RASCP_WRITE_MAX_LEN) {
		LOG_DBG("Write rejected");
		return BT_GATT_ERR(RAS_ATT_ERROR_WRITE_REQ_REJECTED);
	}

	uint16_t buf_len = MIN(len, RASCP_WRITE_MAX_LEN);
	memcpy(rrsp->rascp_cmd_buf, buf, buf_len);
	rrsp->rascp_cmd_len = (uint8_t) buf_len;

	k_work_submit_to_queue(&rrsp_wq, &rrsp->rascp_work);

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

static struct bt_ras_rd_buffer_cb rd_buffer_callbacks = {
	.new_ranging_data_received = rrsp_rd_ready_indicate,
	.ranging_data_overwritten = rrsp_rd_overwritten_indicate,
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

	bool first_seg = (rrsp->active_buf->read_cursor == 0);
	uint16_t actual_data_len = bt_ras_rd_buffer_bytes_pull(rrsp->active_buf, ras_segment->data, max_data_len);
	bool last_seg = (actual_data_len < max_data_len); /* TODO handle case when last seg is max_data_len sized */

	LOG_DBG("%u %u", actual_data_len, max_data_len);

	if (actual_data_len) {
		ras_segment->header.first_seg = first_seg;
		ras_segment->header.last_seg = last_seg;
		ras_segment->header.seg_counter = rrsp->segment_counter & BIT_MASK(6);

		(void)net_buf_simple_remove_mem(&buf, (max_data_len - actual_data_len));

		err = rrsp_ondemand_rd_notify_or_indicate(rrsp->conn, &buf);
		if (err) {
			LOG_WRN("rrsp_ondemand_rd_notify_or_indicate failed err %d", err);

			/* Keep retrying */
			bt_ras_rd_buffer_rewind(rrsp->active_buf, actual_data_len);

			return err;
		}

		rrsp->segment_counter++;

		LOG_DBG("Chunk with RSC %d sent", rrsp->segment_counter);
	}

	if (!last_seg) {
		k_work_submit_to_queue(&rrsp_wq, &rrsp->send_data_work);
	} else {
		LOG_DBG("all chunks sent");
		rrsp_rascp_send_complete_rd_rsp(rrsp->conn, rrsp->active_buf->ranging_counter);
		rrsp->streaming = false;
		k_work_cancel(&rrsp->send_data_work);
	}

	return 0;
}

static void send_data_work_handler(struct k_work *work)
{
	struct bt_ras_rrsp *rrsp = CONTAINER_OF(work, struct bt_ras_rrsp, send_data_work);
	//LOG_DBG("rrsp %p", rrsp);

	if (!rrsp->streaming || !rrsp->active_buf) {
		return;
	}

	int err = rrsp_chunk_send(rrsp);
	if (err) {
		/* TODO: handle error */
		LOG_ERR("rrsp_chunk_send failed: %d", err);
	}

}

static void rascp_work_handler(struct k_work *work)
{
	struct bt_ras_rrsp *rrsp = CONTAINER_OF(work, struct bt_ras_rrsp, rascp_work);
	LOG_DBG("rrsp %p", rrsp);

	rrsp_rascp_cmd_handle(rrsp);
}

static void status_work_handler(struct k_work *work)
{
	struct bt_ras_rrsp *rrsp = CONTAINER_OF(work, struct bt_ras_rrsp, status_work);
	LOG_DBG("rrsp %p", rrsp);

	if (rrsp->notify_overwritten) {
		int err = rd_status_notify_or_indicate(rrsp->conn, BT_UUID_RAS_RD_OVERWRITTEN, rrsp->overwritten_ranging_counter);
		if (err) {
			LOG_WRN("overwritten failed %d", err);
		}

		rrsp->notify_overwritten = false;
	}

	if (rrsp->notify_ready) {
		int err = rd_status_notify_or_indicate(rrsp->conn, BT_UUID_RAS_RD_READY, rrsp->ready_ranging_counter);
		if (err) {
			LOG_WRN("ready failed %d", err);
		}

		rrsp->notify_ready = false;
	}
}

static void rascp_timeout_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	LOG_DBG("");
}

struct bt_ras_rrsp *bt_ras_rrsp_find(struct bt_conn *conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(rrsp_pool); i++) {
		struct bt_ras_rrsp *rrsp = &rrsp_pool[i];

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

	for (size_t i = 0; i < ARRAY_SIZE(rrsp_pool); i++) {
		if (rrsp_pool[i].conn == NULL) {
			rrsp = &rrsp_pool[i];
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
	k_work_init(&rrsp->rascp_work, rascp_work_handler);
	k_work_init(&rrsp->status_work, status_work_handler);
	k_timer_init(&rrsp->rascp_timeout, rascp_timeout_handler, NULL);

	return 0;
}

void bt_ras_rrsp_free(struct bt_conn *conn)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		LOG_DBG("conn %p rrsp %p", (void *)rrsp->conn, (void *)rrsp);

		(void)k_work_cancel(&rrsp->send_data_work);
		(void)k_work_cancel(&rrsp->rascp_work);
		(void)k_work_cancel(&rrsp->status_work);
		k_timer_stop(&rrsp->rascp_timeout);

		k_work_queue_drain(&rrsp_wq, false);

		bt_conn_unref(rrsp->conn);
		rrsp->conn = NULL;
	}
}

int bt_ras_rrsp_init(void)
{
	k_work_queue_init(&rrsp_wq);
	k_work_queue_start(&rrsp_wq, rrsp_wq_stack_area,
	                   K_THREAD_STACK_SIZEOF(rrsp_wq_stack_area), RRSP_WQ_PRIORITY,
	                   NULL);

	ras_features = 0;
#if defined(CONFIG_BT_RAS_REALTIME_RANGING_DATA)
	ras_features |= BIT(RAS_FEAT_REALTIME_RD_BIT);
#endif

	bt_ras_rd_buffer_init();
	bt_ras_rd_buffer_cb_register(&rd_buffer_callbacks);

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
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_RD_READY, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_NOTIFY,
				BT_GATT_PERM_READ_ENCRYPT,
				rd_ready_read, NULL, NULL),
	BT_GATT_CCC(rd_ready_ccc_cfg_changed,
				BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
	/* Ranging Data Overwritten */
	BT_GATT_CHARACTERISTIC(BT_UUID_RAS_RD_OVERWRITTEN, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_NOTIFY,
				BT_GATT_PERM_READ_ENCRYPT,
				rd_overwritten_read, NULL, NULL),
	BT_GATT_CCC(rd_overwritten_ccc_cfg_changed,
				BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
);

static void ondemand_rd_notify_sent_cb(struct bt_conn *conn, void *user_data)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		LOG_DBG("");
		k_work_submit_to_queue(&rrsp_wq, &rrsp->send_data_work);
	}
}

static void ondemand_rd_indicate_sent_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		LOG_DBG("");
		k_work_submit_to_queue(&rrsp_wq, &rrsp->send_data_work);
	}
}

int rrsp_ondemand_rd_notify_or_indicate(struct bt_conn *conn, struct net_buf_simple *buf)
{
	struct bt_gatt_attr *attr = bt_gatt_find_by_uuid(rrsp_svc.attrs, 1, BT_UUID_RAS_ONDEMAND_RD);
	__ASSERT_NO_MSG(attr);

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) {
		struct bt_gatt_notify_params params = {0};

		params.attr = attr;
		params.uuid = NULL;
		params.data = buf->data;
		params.len = buf->len;
		params.func = ondemand_rd_notify_sent_cb;

		/* TODO this can fail if there are no resources in the host */
		return bt_gatt_notify_cb(conn, &params);
	} else if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
		__ASSERT_NO_MSG(rrsp);

		rrsp->ondemand_ind_params.attr = attr;
		rrsp->ondemand_ind_params.uuid = NULL;
		rrsp->ondemand_ind_params.data = buf->data;
		rrsp->ondemand_ind_params.len = buf->len;
		rrsp->ondemand_ind_params.func = ondemand_rd_indicate_sent_cb;
		rrsp->ondemand_ind_params.destroy = NULL; /* TODO */

		return bt_gatt_indicate(conn, &rrsp->ondemand_ind_params);
	} else {
		return -EINVAL;
	}
}

int rrsp_rascp_indicate(struct bt_conn *conn, struct net_buf_simple *rsp)
{
	struct bt_gatt_attr *attr = bt_gatt_find_by_uuid(rrsp_svc.attrs, 1, BT_UUID_RAS_CP);
	__ASSERT_NO_MSG(attr);

	if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
		__ASSERT_NO_MSG(rrsp);

		rrsp->rascp_ind_params.attr = attr;
		rrsp->rascp_ind_params.uuid = NULL;
		rrsp->rascp_ind_params.data = rsp->data;
		rrsp->rascp_ind_params.len = rsp->len;
		rrsp->rascp_ind_params.func = NULL;
		rrsp->rascp_ind_params.destroy = NULL;

		return bt_gatt_indicate(conn, &rrsp->rascp_ind_params);
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
		struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
		__ASSERT_NO_MSG(rrsp);

		rrsp->rd_status_params.attr = attr;
		rrsp->rd_status_params.uuid = NULL;
		rrsp->rd_status_params.data = &ranging_counter;
		rrsp->rd_status_params.len = sizeof(ranging_counter);
		rrsp->rd_status_params.func = NULL;
		rrsp->rd_status_params.destroy = NULL;

		return bt_gatt_indicate(conn, &rrsp->rd_status_params);
	} else {
		return -EINVAL;
	}
}

void rrsp_rd_ready_indicate(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		rrsp->ready_ranging_counter = ranging_counter;
		rrsp->notify_ready = true;
		k_work_submit_to_queue(&rrsp_wq, &rrsp->status_work);
	}
}

void rrsp_rd_overwritten_indicate(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct bt_ras_rrsp *rrsp = bt_ras_rrsp_find(conn);
	if (rrsp) {
		rrsp->overwritten_ranging_counter = ranging_counter;
		rrsp->notify_overwritten = true;
		k_work_submit_to_queue(&rrsp_wq, &rrsp->status_work);
	}
}
