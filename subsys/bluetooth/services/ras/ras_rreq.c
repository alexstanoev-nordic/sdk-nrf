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
#include <zephyr/net_buf.h>
#include <bluetooth/services/ras.h>
#include <bluetooth/gatt_dm.h>
#include <stdint.h>
#include <errno.h>

#include "ras_internal.h"
#include "zephyr/sys/__assert.h"

LOG_MODULE_DECLARE(ras, CONFIG_BT_RAS_LOG_LEVEL);
static struct bt_ras_rreq m_rreq_pool[CONFIG_BT_RAS_MAX_ACTIVE_RREQ];

static struct bt_ras_rreq *ras_rreq_find(struct bt_conn *conn)
{
	for (size_t i = 0; i < ARRAY_SIZE(m_rreq_pool); i++) {
		struct bt_ras_rreq *rreq = &m_rreq_pool[i];

		if (rreq->conn == conn) {
			return rreq;
		}
	}

	return NULL;
}

static int ras_rreq_alloc(struct bt_conn *conn)
{
	struct bt_ras_rreq *rreq = NULL;

	if (ras_rreq_find(conn) != NULL) {
		return -EALREADY;
	}

	for (size_t i = 0; i < ARRAY_SIZE(m_rreq_pool); i++) {
		if (m_rreq_pool[i].conn == NULL) {
			rreq = &m_rreq_pool[i];
			break;
		}
	}

	if (rreq == NULL) {
		return -ENOMEM;
	}

	LOG_DBG("conn %p new rreq %p", (void *)conn, (void *)rreq);

	memset(rreq, 0, sizeof(struct bt_ras_rreq));
	rreq->conn = bt_conn_ref(conn);

	return 0;
}

static uint8_t ranging_data_ready_notify_func(struct bt_conn *conn,
						  struct bt_gatt_subscribe_params *params,
						  const void *data, uint16_t length)
{

	if (length != sizeof(uint16_t)) {
		LOG_DBG("Ranging Data Ready Indication size error");
		return BT_GATT_ITER_CONTINUE;
	}

	struct bt_ras_rreq *rreq = ras_rreq_find(conn);

	uint16_t ranging_counter = *(uint16_t *)data;

	rreq->rd_ready.cb(conn, ranging_counter);

	return BT_GATT_ITER_CONTINUE;
}

static void data_receive_finished(struct bt_ras_rreq *rreq)
{
	if (!rreq->on_demand_rd.last_segment_received) {
		LOG_DBG("Ranging data completed with missing segments");
		rreq->on_demand_rd.error_with_data_receive = true;
	}

	int error_code = rreq->on_demand_rd.error_with_data_receive ? -EINVAL : 0;
	rreq->on_demand_rd.data_get_in_progress = false;

	rreq->on_demand_rd.cb(error_code, rreq->on_demand_rd.counter_in_progress);
}

static uint8_t ranging_data_overwritten_notify_func(struct bt_conn *conn,
							struct bt_gatt_subscribe_params *params,
							const void *data, uint16_t length)
{

	if (length != sizeof(uint16_t)) {
		LOG_DBG("Ranging Data Ready Indication size error");
		return BT_GATT_ITER_CONTINUE;
	}

	struct bt_ras_rreq *rreq = ras_rreq_find(conn);

	uint16_t ranging_counter = *(uint16_t *)data;

	if (rreq->on_demand_rd.data_get_in_progress &&
	    rreq->on_demand_rd.counter_in_progress == ranging_counter) {
		if (rreq->cp.state != BT_RAS_RREQ_CP_STATE_NONE) {
			LOG_DBG("Overwritten received while writing to RAS-CP, will continue "
				"waiting for RAS-CP response");
			return BT_GATT_ITER_CONTINUE;
		}

		LOG_DBG("Ranging counter %d overwritten whilst receiving", ranging_counter);
		rreq->on_demand_rd.error_with_data_receive = true;
		data_receive_finished(rreq);

		return BT_GATT_ITER_CONTINUE;
	}

	rreq->rd_overwritten.cb(conn, ranging_counter);

	return BT_GATT_ITER_CONTINUE;
}

static int ack_ranging_data(struct bt_ras_rreq *rreq)
{
	int err;

	struct ras_ack_ranging_data ack_ranging_data;
	ack_ranging_data.opcode = RASCP_OPCODE_ACK_RD;
	ack_ranging_data.ranging_counter = rreq->on_demand_rd.counter_in_progress;

	err = bt_gatt_write_without_response(rreq->conn, rreq->cp.subscribe_params.value_handle,
					     &ack_ranging_data, sizeof(ack_ranging_data), false);
	if (err) {
		LOG_DBG("CP ACK ranging data written failed, err %d", err);
		return err;
	}

	rreq->cp.state = BT_RAS_RREQ_CP_STATE_ACK_RD_WRITTEN;
	LOG_DBG("Ack Ranging data for counter %d", rreq->on_demand_rd.counter_in_progress);

	return 0;
}

static void handle_rsp_code(uint8_t rsp_code, struct bt_ras_rreq *rreq)
{
	switch (rreq->cp.state) {
	case BT_RAS_RREQ_CP_STATE_NONE: {
		if (rreq->on_demand_rd.data_get_in_progress &&
		    rsp_code == RASCP_RESPONSE_PROCEDURE_NOT_COMPLETED) {
			LOG_DBG("Ranging counter %d aborted whilst receiving", rreq->on_demand_rd.counter_in_progress);
			rreq->on_demand_rd.error_with_data_receive = true;
			data_receive_finished(rreq);
			break;
		}

		LOG_DBG("Unexpected Response code received %d", rsp_code);
		break;
	}
	case BT_RAS_RREQ_CP_STATE_GET_RD_WRITTEN: {
		__ASSERT_NO_MSG(rreq->on_demand_rd.data_get_in_progress);
		rreq->cp.state = BT_RAS_RREQ_CP_STATE_NONE;

		if (rsp_code != RASCP_RESPONSE_SUCCESS) {
			LOG_DBG("Get Ranging Data returned an error %d", rsp_code);
			rreq->on_demand_rd.error_with_data_receive = true;
			data_receive_finished(rreq);
			break;
		}

		LOG_DBG("Get Ranging Data Success");

		break;
	}
	case BT_RAS_RREQ_CP_STATE_ACK_RD_WRITTEN: {
		__ASSERT_NO_MSG(rreq->on_demand_rd.data_get_in_progress);
		rreq->cp.state = BT_RAS_RREQ_CP_STATE_NONE;
		if (rsp_code != RASCP_RESPONSE_SUCCESS) {
			LOG_DBG("ACK Ranging Data returned an error %d", rsp_code);
		}

		data_receive_finished(rreq);
		break;
	}
	default:
		LOG_DBG("Unexpected Response code received %d", rsp_code);
		break;
	}
}

static uint8_t ras_cp_notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
				      const void *data, uint16_t length)
{
	struct bt_ras_rreq *rreq = ras_rreq_find(conn);

	struct net_buf_simple rsp;
	net_buf_simple_init_with_data(&rsp, (uint8_t *)data, length);

	uint8_t opcode = net_buf_simple_pull_u8(&rsp);

	switch (opcode) {
	case RASCP_RSP_OPCODE_COMPLETE_RD_RSP: {
		if (rsp.len != RASCP_RSP_OPCODE_COMPLETE_RD_RSP_LEN) {
			LOG_DBG("RAS-CP Complete RD RSP incorrect length: %d", length);
			break;
		}

		uint16_t ranging_counter = net_buf_simple_pull_le16(&rsp);
		if (!rreq->on_demand_rd.data_get_in_progress ||
		    rreq->on_demand_rd.counter_in_progress != ranging_counter) {
			LOG_DBG("RAS-CP Complete RD RSP unexpected ranging counter %d",
				ranging_counter);
			break;
		}

		ack_ranging_data(rreq);
		break;
	}
	case RASCP_RSP_OPCODE_RSP_CODE: {
		if (rsp.len != RASCP_RSP_OPCODE_RSP_CODE_LEN) {
			LOG_DBG("RAS-CP RSP Code incorrect length: %d", length);
			break;
		}

		uint8_t rsp_code = net_buf_simple_pull_u8(&rsp);
		handle_rsp_code(rsp_code, rreq);

		break;
	}
	default: {
		LOG_DBG("Unkown RAS-CP RSP opcode: %d", opcode);
		break;
	}
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t ras_on_demand_ranging_data_notify_func(struct bt_conn *conn,
						      struct bt_gatt_subscribe_params *params,
						      const void *data, uint16_t length)
{
	LOG_DBG("On-demand Ranging Data notification received");

	struct bt_ras_rreq *rreq = ras_rreq_find(conn);

	if (!rreq->on_demand_rd.data_get_in_progress) {
		LOG_DBG("Unexpected On-demand Ranging Data notification received");
		return BT_GATT_ITER_CONTINUE;
	}

	if (length < 2) {
		LOG_DBG("On-demand Ranging Data notification received invalid length");
		rreq->on_demand_rd.error_with_data_receive = true;
		return BT_GATT_ITER_CONTINUE;
	}

	if (rreq->on_demand_rd.last_segment_received) {
		LOG_DBG("On-demand Ranging Data notification received after last segment");
		return BT_GATT_ITER_CONTINUE;
	}

	if (rreq->on_demand_rd.error_with_data_receive) {
		/* Already had an error receiving this ranging counter, so exit here. */
		return BT_GATT_ITER_CONTINUE;
	}

	struct net_buf_simple segment;
	net_buf_simple_init_with_data(&segment, (uint8_t *)data, length);

	uint8_t segmentation_header = net_buf_simple_pull_u8(&segment);

	bool first_segment = segmentation_header & BIT(0);
	bool last_segment = segmentation_header & BIT(1);
	uint8_t rolling_segment_counter = segmentation_header >> 2;

	if (first_segment && rolling_segment_counter != 0) {
		LOG_DBG("On-demand Ranging Data notification received invalid "
			"rolling_segment_counter %d",
			rolling_segment_counter);
		rreq->on_demand_rd.error_with_data_receive = true;
		return BT_GATT_ITER_CONTINUE;
	}

	if (rreq->on_demand_rd.next_expected_segment_counter != rolling_segment_counter) {
		LOG_DBG("No support for receiving segments out of order");
		rreq->on_demand_rd.error_with_data_receive = true;
		return BT_GATT_ITER_CONTINUE;
	}

	uint16_t ranging_data_segment_length = segment.len;

	if (net_buf_simple_tailroom(rreq->on_demand_rd.ranging_data_out) <
	    ranging_data_segment_length) {
		LOG_DBG("Ranging data out buffer not large enough for next segment");
		rreq->on_demand_rd.error_with_data_receive = true;
		return BT_GATT_ITER_CONTINUE;
	}

	uint8_t *ranging_data_segment =
		net_buf_simple_pull_mem(&segment, ranging_data_segment_length);
	net_buf_simple_add_mem(rreq->on_demand_rd.ranging_data_out, ranging_data_segment,
			       ranging_data_segment_length);

	if (last_segment) {
		rreq->on_demand_rd.last_segment_received = true;
	}

	/* Segment counter is between 0-63. */
	rreq->on_demand_rd.next_expected_segment_counter =
		(rolling_segment_counter + 1) & BIT_MASK(6);

	return BT_GATT_ITER_CONTINUE;
}

static void subscribed_func(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_subscribe_params *params)
{
	if (err) {
		LOG_DBG("Subscribe to ccc_handle %d failed, err %d", params->ccc_handle, err);
	}
}

static int ras_cp_subscribe_params_populate(struct bt_gatt_dm *dm, struct bt_ras_rreq *rreq)
{
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	/* RAS-CP characteristic (mandatory) */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_RAS_CP);
	if (!gatt_chrc) {
		return -EINVAL;
	}

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_RAS_CP);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->cp.subscribe_params.value_handle = gatt_desc->handle;

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->cp.subscribe_params.ccc_handle = gatt_desc->handle;

	rreq->cp.subscribe_params.notify = ras_cp_notify_func;
	rreq->cp.subscribe_params.value = BT_GATT_CCC_INDICATE;
	rreq->cp.subscribe_params.subscribe = subscribed_func;

	return 0;
}

static int ras_rreq_cp_subscribe(struct bt_ras_rreq *rreq)
{
	int err;
	err = bt_gatt_subscribe(rreq->conn, &rreq->cp.subscribe_params);
	if (err) {
		LOG_DBG("RAS-CP subscribe failed (err %d)", err);
		return err;
	}

	return 0;
}

static int ondemand_rd_subscribe_params_populate(struct bt_gatt_dm *dm, struct bt_ras_rreq *rreq)
{
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	/* RAS On-demand ranging data characteristic (mandatory) */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_RAS_ONDEMAND_RD);
	if (!gatt_chrc) {
		return -EINVAL;
	}

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_RAS_ONDEMAND_RD);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->on_demand_rd.subscribe_params.value_handle = gatt_desc->handle;

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->on_demand_rd.subscribe_params.ccc_handle = gatt_desc->handle;
	rreq->on_demand_rd.subscribe_params.notify = ras_on_demand_ranging_data_notify_func;
	rreq->on_demand_rd.subscribe_params.value = BT_GATT_CCC_NOTIFY | BT_GATT_CCC_INDICATE;
	rreq->on_demand_rd.subscribe_params.subscribe = subscribed_func;

	return 0;
}

static int rd_ready_subscribe_params_populate(struct bt_gatt_dm *dm, struct bt_ras_rreq *rreq)
{
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	/* RAS Ranging Data Ready characteristic (mandatory) */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_RAS_RD_READY);
	if (!gatt_chrc) {
		return -EINVAL;
	}

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_RAS_RD_READY);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->rd_ready.subscribe_params.value_handle = gatt_desc->handle;

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->rd_ready.subscribe_params.ccc_handle = gatt_desc->handle;
	rreq->rd_ready.subscribe_params.notify = ranging_data_ready_notify_func;
	rreq->rd_ready.subscribe_params.value = BT_GATT_CCC_NOTIFY | BT_GATT_CCC_INDICATE;
	rreq->rd_ready.subscribe_params.subscribe = subscribed_func;

	return 0;
}

static int rd_overwritten_subscribe_params_populate(struct bt_gatt_dm *dm, struct bt_ras_rreq *rreq)
{
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	/* RAS Ranging Data Ready characteristic (mandatory) */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_RAS_RD_OVERWRITTEN);
	if (!gatt_chrc) {
		return -EINVAL;
	}

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_RAS_RD_OVERWRITTEN);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->rd_overwritten.subscribe_params.value_handle = gatt_desc->handle;

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc, BT_UUID_GATT_CCC);
	if (!gatt_desc) {
		return -EINVAL;
	}
	rreq->rd_overwritten.subscribe_params.ccc_handle = gatt_desc->handle;
	rreq->rd_overwritten.subscribe_params.notify = ranging_data_overwritten_notify_func;
	rreq->rd_overwritten.subscribe_params.value = BT_GATT_CCC_NOTIFY | BT_GATT_CCC_INDICATE;
	rreq->rd_overwritten.subscribe_params.subscribe = subscribed_func;

	return 0;
}

static int on_demand_ranging_data_subscribe(struct bt_ras_rreq *rreq)
{
	int err;

	err = bt_gatt_subscribe(rreq->conn, &rreq->on_demand_rd.subscribe_params);
	if (err) {
		LOG_DBG("On-demand ranging data subscribe failed (err %d)", err);
		return err;
	}

	LOG_DBG("On-demand ranging data subscribed");

	return 0;
}

static int ranging_data_ready_subscribe(struct bt_ras_rreq *rreq, bt_ras_rreq_rd_ready_cb_t cb)
{
	int err;

	err = bt_gatt_subscribe(rreq->conn, &rreq->rd_ready.subscribe_params);
	if (err) {
		LOG_DBG("Ranging data ready subscribe failed (err %d)", err);
		return err;
	}

	LOG_DBG("Ranging data ready subscribed");
	rreq->rd_ready.cb = cb;

	return 0;
}

static int ranging_data_overwritten_subscribe(struct bt_ras_rreq *rreq,
					      bt_ras_rreq_rd_overwritten_cb_t cb)
{
	int err;

	err = bt_gatt_subscribe(rreq->conn, &rreq->rd_overwritten.subscribe_params);
	if (err) {
		LOG_DBG("Ranging data overwritten subscribe failed (err %d)", err);
		return err;
	}

	LOG_DBG("Ranging data overwritten subscribed");
	rreq->rd_overwritten.cb = cb;

	return 0;
}

void bt_ras_rreq_free(struct bt_conn *conn)
{
	struct bt_ras_rreq *rreq = ras_rreq_find(conn);
	if (rreq) {
		LOG_DBG("Free rreq %p for conn %p", (void *)rreq->conn, (void *)rreq);

		bt_conn_unref(rreq->conn);
		rreq->conn = NULL;
	}
}

int bt_ras_rreq_on_demand_ranging_data_subscribe_all(
	struct bt_conn *conn, bt_ras_rreq_rd_ready_cb_t rd_ready_cb,
	bt_ras_rreq_rd_overwritten_cb_t rd_overwritten_cb)
{
	int err;
	struct bt_ras_rreq *rreq = ras_rreq_find(conn);

	if (rreq == NULL) {
		return -EINVAL;
	}

	err = ras_rreq_cp_subscribe(rreq);
	if (err) {
		return err;
	}

	err = on_demand_ranging_data_subscribe(rreq);
	if (err) {
		return err;
	}

	err = ranging_data_ready_subscribe(rreq, rd_ready_cb);
	if (err) {
		return err;
	}

	err = ranging_data_overwritten_subscribe(rreq, rd_overwritten_cb);
	if (err) {
		return err;
	}

	return 0;
}

int bt_ras_rreq_alloc_and_assign_handles(struct bt_gatt_dm *dm, struct bt_conn *conn)
{
	int err;
	struct bt_ras_rreq *rreq = NULL;

	if (dm == NULL || conn == NULL) {
		return -EINVAL;
	}

	err = ras_rreq_alloc(conn);
	if (err) {
		return err;
	}

	rreq = ras_rreq_find(conn);
	__ASSERT_NO_MSG(rreq);

	err = ondemand_rd_subscribe_params_populate(dm, rreq);
	if (err) {
		return err;
	}

	err = rd_ready_subscribe_params_populate(dm, rreq);
	if (err) {
		return err;
	}

	err = rd_overwritten_subscribe_params_populate(dm, rreq);
	if (err) {
		return err;
	}

	err = ras_cp_subscribe_params_populate(dm, rreq);
	if (err) {
		return err;
	}

	return 0;
}

int bt_ras_rreq_cp_get_ranging_data(struct bt_conn *conn, struct net_buf_simple *ranging_data_out,
				    uint16_t ranging_counter,
				    bt_ras_rreq_ranging_data_get_complete_t cb)
{
	int err;
	struct bt_ras_rreq *rreq = ras_rreq_find(conn);

	if (rreq == NULL || ranging_data_out == NULL) {
		return -EINVAL;
	}

	if (rreq->cp.state != BT_RAS_RREQ_CP_STATE_NONE ||
	    rreq->on_demand_rd.data_get_in_progress) {
		return -EBUSY;
	}

	rreq->on_demand_rd.data_get_in_progress = true;
	rreq->on_demand_rd.ranging_data_out = ranging_data_out;
	rreq->on_demand_rd.counter_in_progress = ranging_counter;
	rreq->on_demand_rd.cb = cb;
	rreq->on_demand_rd.next_expected_segment_counter = 0;
	rreq->on_demand_rd.last_segment_received = false;
	rreq->on_demand_rd.error_with_data_receive = false;

	struct ras_get_ranging_data get_ranging_data;
	get_ranging_data.opcode = RASCP_OPCODE_GET_RD;
	get_ranging_data.ranging_counter = ranging_counter;

	err = bt_gatt_write_without_response(conn, rreq->cp.subscribe_params.value_handle,
					     &get_ranging_data, sizeof(get_ranging_data), false);
	if (err) {
		LOG_DBG("CP Get ranging data written failed, err %d", err);
		return err;
	}

	rreq->cp.state = BT_RAS_RREQ_CP_STATE_GET_RD_WRITTEN;

	return 0;
}