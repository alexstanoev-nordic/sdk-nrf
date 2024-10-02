/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/bluetooth/cs.h>
#include <bluetooth/services/ras.h>

#include "ras_internal.h"

/* Receives CS subevents from the local controller.
 * Reassembles into RAS ranging data format.
 */

LOG_MODULE_DECLARE(ras, CONFIG_BT_RAS_LOG_LEVEL);

#define RD_BUFFERS_PER_CONN 1 /* TODO Kconfig */
#define RD_BUFFER_COUNT (CONFIG_BT_RAS_MAX_ACTIVE_RRSP * RD_BUFFERS_PER_CONN)

struct ras_rd_buffer m_rd_buffer_pool[RD_BUFFER_COUNT];
static sys_slist_t callback_list = SYS_SLIST_STATIC_INIT(&callback_list);

static void notify_new_rd_stored(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct ras_rd_buffer_cb *cb;

	SYS_SLIST_FOR_EACH_CONTAINER(&callback_list, cb, node) {
		if (cb->new_ranging_data_received) {
			cb->new_ranging_data_received(conn, ranging_counter);
		}
	}
}

static void notify_rd_overwritten(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct ras_rd_buffer_cb *cb;

	SYS_SLIST_FOR_EACH_CONTAINER(&callback_list, cb, node) {
		if (cb->ranging_data_overwritten) {
			cb->ranging_data_overwritten(conn, ranging_counter);
		}
	}
}

static struct ras_rd_buffer *buffer_get(struct bt_conn *conn, uint16_t ranging_counter, bool ready, bool busy)
{
	for (uint8_t i = 0; i < RD_BUFFER_COUNT; i++) {
		if (m_rd_buffer_pool[i].conn == conn
		    && m_rd_buffer_pool[i].ready == ready
		    && m_rd_buffer_pool[i].busy == busy)
		{
			return &m_rd_buffer_pool[i];
		}
	}

	return NULL;
}

static void rd_buffer_init(struct bt_conn *conn, struct ras_rd_buffer *buf)
{
	bt_conn_ref(conn);
	available_free_buffer->conn          = conn;
	available_free_buffer->ready         = false;
	available_free_buffer->busy          = true;
	available_free_buffer->refcount      = 0;
	available_free_buffer->bytes_written = 0;
}

static struct ras_rd_buffer *rd_buffer_alloc(struct bt_conn *conn)
{
	int conn_buffer_count = 0;
	struct ras_rd_buffer *available_free_buffer = NULL;
	struct ras_rd_buffer *available_oldest_buffer = NULL;
	uint16_t oldest_ranging_counter = UINT16_MAX;

	for (uint8_t i = 0; i < RD_BUFFER_COUNT; i++) {
		if (m_rd_buffer_pool[i].conn == conn) {
			conn_buffer_count++;

			/* Only overwrite buffers that have ranging data stored and are not being read. */
			if (m_rd_buffer_pool[i].ready
			    && !m_rd_buffer_pool[i].busy
			    && m_rd_buffer_pool[i].refcount == 0
			    && m_rd_buffer_pool[i].procedure.header.ranging_counter < oldest_ranging_counter) {
				oldest_ranging_counter = m_rd_buffer_pool[i].procedure.header.ranging_counter;
				available_oldest_buffer = &m_rd_buffer_pool[i];
			}
		}

		if (available_free_buffer == NULL && m_rd_buffer_pool[i].conn == NULL) {
			available_free_buffer = &m_rd_buffer_pool[i];
		}
	}

	/* Allocate the buffer straight away if the connection has not reached
	 * the maximum number of buffers allocated. */
	if (conn_buffer_count < RD_BUFFERS_PER_CONN) {
		rd_buffer_init(conn, available_free_buffer);

		return available_free_buffer;
	}

	/* Overwrite the oldest stored ranging buffer that is not in use */
	if (available_oldest_buffer != NULL) {
		notify_rd_overwritten(conn, oldest_ranging_counter);

		rd_buffer_init(conn, available_oldest_buffer);

		return available_oldest_buffer;
	}

	/* Could not allocate a buffer */
	return NULL;
}

/* TODO this is not nice */
static uint8_t current_step;
static uint8_t *step_mode_buffer;
static uint8_t *step_data_buffer;
static uint16_t step_data_len;

static bool process_step_data(struct bt_le_cs_subevent_step *step)
{
	step_mode_buffer[current_step] = step->mode;
	/* TODO: what about step->channel? */
	memcpy(&step_data_buffer[step_data_len], step->data, step->data_len);
	step_data_len += step->data_len;
	current_step++;
}

static void subevent_data_available(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	struct ras_rd_buffer buf = buffer_get(conn, result->procedure_counter, false, true);
	if (!buf) {
		/* First subevent - allocate a buffer */
		buf = rd_buffer_alloc(conn);

		if (!buf) {
			LOG_ERR("Failed to allocate buffer for procedure %u", result->procedure_counter);
			return;
		}

		buf->procedure.header.ranging_counter = result->procedure_counter;
		buf->procedure.ranging_header.config_id = result->config_id;
		buf->procedure.ranging_header.selected_tx_power = 0; /* TODO */
		buf->procedure.ranging_header.antenna_paths_mask = 1; /* TODO */
	}

	struct ras_subevent_header *hdr = &buf->procedure.subevents[buf->subevent_cursor];
	buf->subevent_cursor += sizeof(struct ras_subevent_header);

	hdr->start_acl_conn_event = result->start_acl_conn_event;
	hdr->freq_compensation = result->frequency_compensation;
	hdr->ranging_done_status = result->procedure_done_status; /* TODO HCI->RAS */
	hdr->subevent_done_status = result->subevent_done_status; /* TODO HCI->RAS */
	hdr->ranging_abort_reason = result->procedure_abort_reason; /* TODO HCI->RAS */
	hdr->subevent_abort_reason = result->subevent_abort_reason; /* TODO HCI->RAS */
	hdr->ref_power_level = result->reference_power_level;
	hdr->num_steps_reported = result->num_steps_reported;

	uint8_t *step_modes = &buf->procedure.subevents[buf->subevent_cursor];
	buf->subevent_cursor += hdr->num_steps_reported * BT_RAS_STEP_MODE_LEN;

	uint8_t *step_data = &buf->procedure.subevents[buf->subevent_cursor];

	current_step = 0;
	step_data_len = 0;
	step_mode_buffer = step_modes;
	step_data_buffer = step_data;

	if (result->step_data_buf) {
		bt_le_cs_step_data_parse(result->step_data_buf, process_step_data);
		buf->subevent_cursor += step_data_len;
	}

	if (hdr->ranging_done_status == BT_CONN_LE_CS_PROCEDURE_COMPLETE) {
		buf->ready = true;
		buf->busy = false;
		notify_new_rd_stored(conn, result->procedure_counter);
	}
}

static struct bt_conn_cb cs_data_callbacks = {
	.le_cs_subevent_data_available = subevent_data_available,
};

void bt_ras_rd_buffer_init(void)
{
	bt_conn_cb_register(&cs_data_callbacks);
}

void bt_ras_rd_buffer_cb_register(struct ras_rd_buffer_cb *cb)
{
	sys_slist_append(&callback_list, &cb->node);
}

bool bt_ras_rd_buffer_ready_check(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct ras_rd_buffer buf = buffer_get(conn, ranging_counter, true, false);
	return buf != NULL;
}

struct ras_rd_buffer *bt_ras_rd_buffer_claim(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct ras_rd_buffer buf = buffer_get(conn, ranging_counter, true, false);
	if (buf) {
		buf->refcount++;
		return buf;
	}

	return NULL;
}

int bt_ras_rd_buffer_release(struct bt_conn *conn, uint16_t ranging_counter)
{
	struct ras_rd_buffer buf = buffer_get(conn, ranging_counter, true, false);
	if (buf) {
		if (buf->refcount == 0) {
			return -1; /* TODO errcode */
		}

		buf->refcount--;
		return 0;
	}

	return -2; /* TODO errcode */
}
