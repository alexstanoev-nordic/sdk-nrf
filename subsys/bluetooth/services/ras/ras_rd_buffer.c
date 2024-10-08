/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>

#include "ras_internal.h"

/* Receives CS subevents from the local controller.
 * Reassembles into RAS ranging data format.
 */

LOG_MODULE_DECLARE(ras, CONFIG_BT_RAS_LOG_LEVEL);

#define SEND_DUMMY_DATA

#define RD_BUFFER_COUNT (CONFIG_BT_RAS_MAX_ACTIVE_RRSP * CONFIG_BT_RAS_RD_BUFFERS_PER_CONN)

static struct ras_rd_buffer rd_buffer_pool[RD_BUFFER_COUNT];
static sys_slist_t callback_list = SYS_SLIST_STATIC_INIT(&callback_list);
static struct step_data_context {
	uint16_t step_data_len;
	uint8_t  current_step;
	uint8_t *step_mode_buffer;
	uint8_t *step_data_buffer;
} step_data_context;

static void notify_new_rd_stored(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct bt_ras_rd_buffer_cb *cb;
	LOG_DBG("%u", ranging_counter);

	SYS_SLIST_FOR_EACH_CONTAINER(&callback_list, cb, node) {
		if (cb->new_ranging_data_received) {
			cb->new_ranging_data_received(conn, ranging_counter);
		}
	}
}

static void notify_rd_overwritten(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct bt_ras_rd_buffer_cb *cb;
	LOG_DBG("%u", ranging_counter);

	SYS_SLIST_FOR_EACH_CONTAINER(&callback_list, cb, node) {
		if (cb->ranging_data_overwritten) {
			cb->ranging_data_overwritten(conn, ranging_counter);
		}
	}
}

static struct ras_rd_buffer *buffer_get(struct bt_conn *conn, uint16_t ranging_counter, bool ready, bool busy)
{
	for (uint8_t i = 0; i < RD_BUFFER_COUNT; i++) {
		if (rd_buffer_pool[i].conn == conn
		    && rd_buffer_pool[i].ranging_counter == ranging_counter
		    && rd_buffer_pool[i].ready == ready
		    && rd_buffer_pool[i].busy == busy)
		{
			return &rd_buffer_pool[i];
		}
	}

	return NULL;
}

static void rd_buffer_init(struct bt_conn *conn, struct ras_rd_buffer *buf, uint16_t ranging_counter)
{
	buf->conn            = bt_conn_ref(conn);
	buf->ranging_counter = ranging_counter;
	buf->ready           = false;
	buf->busy            = true;
	buf->refcount        = 0;
	buf->subevent_cursor = 0;
	buf->read_cursor     = 0;
}

static void rd_buffer_free(struct ras_rd_buffer *buf)
{
	if (buf->conn) {
		bt_conn_unref(buf->conn);
	}

	buf->conn            = NULL;
	buf->ready           = false;
	buf->busy            = false;
	buf->refcount        = 0;
	buf->subevent_cursor = 0;
	buf->read_cursor     = 0;
}

static struct ras_rd_buffer *rd_buffer_alloc(struct bt_conn *conn, uint16_t ranging_counter)
{
	uint16_t conn_buffer_count = 0;
	uint16_t oldest_ranging_counter = UINT16_MAX;
	struct ras_rd_buffer *available_free_buffer = NULL;
	struct ras_rd_buffer *available_oldest_buffer = NULL;

	for (uint8_t i = 0; i < RD_BUFFER_COUNT; i++) {
		if (rd_buffer_pool[i].conn == conn) {
			conn_buffer_count++;

			/* Only overwrite buffers that have ranging data stored and are not being read. */
			if (rd_buffer_pool[i].ready
			    && !rd_buffer_pool[i].busy
			    && rd_buffer_pool[i].refcount == 0
			    && rd_buffer_pool[i].ranging_counter < oldest_ranging_counter) {
				oldest_ranging_counter = rd_buffer_pool[i].ranging_counter;
				available_oldest_buffer = &rd_buffer_pool[i];
			}
		}

		if (available_free_buffer == NULL && rd_buffer_pool[i].conn == NULL) {
			available_free_buffer = &rd_buffer_pool[i];
		}
	}

	/* Allocate the buffer straight away if the connection has not reached
	 * the maximum number of buffers allocated. */
	if (conn_buffer_count < CONFIG_BT_RAS_RD_BUFFERS_PER_CONN) {
		rd_buffer_init(conn, available_free_buffer, ranging_counter);

		return available_free_buffer;
	}

	/* Overwrite the oldest stored ranging buffer that is not in use */
	if (available_oldest_buffer != NULL) {
		notify_rd_overwritten(conn, oldest_ranging_counter);
		rd_buffer_free(available_oldest_buffer);

		rd_buffer_init(conn, available_oldest_buffer, ranging_counter);

		return available_oldest_buffer;
	}

	/* Could not allocate a buffer */
	return NULL;
}

static bool process_step_data(struct bt_le_cs_subevent_step *step)
{
	step_data_context.step_mode_buffer[step_data_context.current_step] = step->mode;
	/* TODO: what about step->channel? */
	memcpy(&step_data_context.step_data_buffer[step_data_context.step_data_len], step->data, step->data_len);
	step_data_context.step_data_len += step->data_len;
	step_data_context.current_step++;

	return true;
}

static void subevent_data_available(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	struct ras_rd_buffer *buf = buffer_get(conn, result->header.procedure_counter, false, true);
	if (!buf) {
		/* First subevent - allocate a buffer */
		buf = rd_buffer_alloc(conn, result->header.procedure_counter);

		if (!buf) {
			LOG_ERR("Failed to allocate buffer for procedure %u", result->header.procedure_counter);
			return;
		}

		buf->procedure.ranging_header.ranging_counter = result->header.procedure_counter;
		buf->procedure.ranging_header.config_id = result->header.config_id;
		buf->procedure.ranging_header.selected_tx_power = 0;  /* TODO */
		buf->procedure.ranging_header.antenna_paths_mask = 1; /* TODO */
	}

	struct ras_subevent_header *hdr = (struct ras_subevent_header *) &buf->procedure.subevents[buf->subevent_cursor];
	buf->subevent_cursor += sizeof(struct ras_subevent_header);

	hdr->start_acl_conn_event = result->header.start_acl_conn_event;
	hdr->freq_compensation = result->header.frequency_compensation;
	hdr->ranging_done_status = result->header.procedure_done_status;
	hdr->subevent_done_status = result->header.subevent_done_status;
	hdr->ranging_abort_reason = result->header.procedure_abort_reason;
	hdr->subevent_abort_reason = result->header.subevent_abort_reason;
	hdr->ref_power_level = result->header.reference_power_level;
	hdr->num_steps_reported = result->header.num_steps_reported;

	uint8_t *step_modes = &buf->procedure.subevents[buf->subevent_cursor];
	buf->subevent_cursor += hdr->num_steps_reported * BT_RAS_STEP_MODE_LEN;

	uint8_t *step_data = &buf->procedure.subevents[buf->subevent_cursor];

	if (result->step_data_buf) {
		step_data_context.current_step = 0;
		step_data_context.step_data_len = 0;
		step_data_context.step_mode_buffer = step_modes;
		step_data_context.step_data_buffer = step_data;

		bt_le_cs_step_data_parse(result->step_data_buf, process_step_data);

		buf->subevent_cursor += step_data_context.step_data_len;
	}

	if (hdr->ranging_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		buf->ready = true;
		buf->busy = false;
		notify_new_rd_stored(conn, result->header.procedure_counter);
	}
}

#ifdef SEND_DUMMY_DATA
static struct bt_conn *curr_conn;
NET_BUF_SIMPLE_DEFINE_STATIC(tmp_step_buf, 2100);

void data_timer_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);
	static int cnt = 0;
	cnt++;

	net_buf_simple_reset(&tmp_step_buf);

	const int steps = 10;
	const int step_len = 200;

	for (int j = 0; j < steps; j++) {
		net_buf_simple_add_u8(&tmp_step_buf, 1);  // mode
		net_buf_simple_add_u8(&tmp_step_buf, 2);  // channel
		net_buf_simple_add_u8(&tmp_step_buf, step_len); // data len

		for(int i = 0; i < step_len; i++) {
			net_buf_simple_add_u8(&tmp_step_buf, (uint8_t) (cnt+i)); // data
		}
	}

	struct bt_conn_le_cs_subevent_result data;
	data.header.procedure_counter = cnt;
	data.header.num_steps_reported = steps;
	data.header.procedure_done_status = BT_CONN_LE_CS_PROCEDURE_COMPLETE;
	data.step_data_buf = &tmp_step_buf;
	subevent_data_available(curr_conn, &data);
}

K_TIMER_DEFINE(data_timer, data_timer_handler, NULL);

static void connected(struct bt_conn *conn, uint8_t err) {
	k_timer_start(&data_timer, K_SECONDS(5), K_MSEC(500));
	curr_conn = conn;
}
#endif

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(reason);
	k_timer_stop(&data_timer);

	for (uint8_t i = 0; i < RD_BUFFER_COUNT; i++) {
		if (rd_buffer_pool[i].conn == conn) {
			rd_buffer_free(&rd_buffer_pool[i]);
		}
	}
}

static struct bt_conn_cb conn_callbacks = {
	.le_cs_subevent_data_available = subevent_data_available,
	.disconnected = disconnected,
#ifdef SEND_DUMMY_DATA
	.connected = connected,
#endif
};

void bt_ras_rd_buffer_init(void)
{
	bt_conn_cb_register(&conn_callbacks);
}

void bt_ras_rd_buffer_cb_register(struct bt_ras_rd_buffer_cb *cb)
{
	sys_slist_append(&callback_list, &cb->node);
}

bool bt_ras_rd_buffer_ready_check(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct ras_rd_buffer *buf = buffer_get(conn, ranging_counter, true, false);
	return buf != NULL;
}

struct ras_rd_buffer *bt_ras_rd_buffer_claim(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct ras_rd_buffer *buf = buffer_get(conn, ranging_counter, true, false);
	if (buf) {
		buf->refcount++;
		return buf;
	}

	return NULL;
}

int bt_ras_rd_buffer_release(struct ras_rd_buffer *buf)
{
	if (!buf || buf->refcount == 0) {
		return -EINVAL;

	}

	buf->refcount--;

	/* Not freeing the buffer as it may be requested again by the app. */

	return 0;
}

int bt_ras_rd_buffer_bytes_pull(struct ras_rd_buffer *buf, uint8_t *out_buf, uint16_t max_data_len) {
	if (!buf->ready) {
		return 0;
	}

	uint16_t buf_len = sizeof(struct ras_ranging_header) + buf->subevent_cursor;
	__ASSERT_NO_MSG(buf->read_cursor <= buf_len);
	uint16_t remaining = buf_len - buf->read_cursor;
	uint16_t pull_bytes = MIN(max_data_len, remaining);

	memcpy(out_buf, &buf->procedure.buf[buf->read_cursor], pull_bytes);
	buf->read_cursor += pull_bytes;

	return pull_bytes;
}

void bt_ras_rd_buffer_rewind(struct ras_rd_buffer *buf, uint16_t data_len) {
	if (!buf->ready) {
		return;
	}

	__ASSERT_NO_MSG(buf->read_cursor >= data_len);
	buf->read_cursor -= data_len;

}
