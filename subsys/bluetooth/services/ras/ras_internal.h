/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>

#include <stdint.h>

enum ras_feat {
	RAS_FEAT_REALTIME_RD          = BIT(0),
	RAS_FEAT_RETRIEVE_LOST_RD_SEG = BIT(1),
	RAS_FEAT_ABORT_OP             = BIT(2),
	RAS_FEAT_FILTER_RD            = BIT(3),
};

enum ras_reporting_mode {
	RAS_REPORTING_MODE_OFF,
	RAS_REPORTING_MODE_ONDEMAND,
	RAS_REPORTING_MODE_REALTIME,
};

enum ras_att_error {
	RAS_ATT_ERROR_CCC_CONFIG         = 0xFD,
	RAS_ATT_ERROR_WRITE_REQ_REJECTED = 0xFC,
};

struct ras_seg_header {
	bool    first_seg   : 1;
	bool    last_seg    : 1;
	uint8_t seg_counter : 6;
} __packed;

struct ras_segment {
	struct ras_seg_header header;
	uint8_t               data[];
} __packed;

struct bt_ras_rrsp {
	struct bt_conn *conn;
	struct net_buf_simple *rd_buf;
	struct k_work send_data_work;
	struct k_timer data_req_timeout_timer; /* TODO */

	bool streaming; /* TODO: atomic */
	uint16_t streaming_ranging_counter;
	uint16_t writing_ranging_counter;
	uint16_t overwritten_ranging_counter;
	uint16_t segment_counter;

	uint16_t bytes_sent;
	uint16_t bytes_written;
};

struct bt_ras_rrsp *bt_ras_rrsp_find(struct bt_conn *conn);

void rrsp_rascp_cmd_handle(struct bt_conn *conn, struct net_buf_simple *req);
void rrsp_rascp_send_complete_rd_rsp(struct bt_conn *conn, uint16_t ranging_counter);

int rrsp_ondemand_rd_notify_or_indicate(struct bt_conn *conn, struct net_buf_simple *buf);
int rrsp_rascp_indicate(struct bt_conn *conn, struct net_buf_simple *rsp);
int rrsp_rd_ready_indicate(struct bt_conn *conn, uint16_t ranging_counter);
int rrsp_rd_overwritten_indicate(struct bt_conn *conn, uint16_t ranging_counter);

bool bt_ras_rd_buffer_ranging_counter_check(struct bt_ras_rrsp *rrsp, uint16_t ranging_counter);
bool bt_ras_rd_buffer_ranging_counter_free(struct bt_ras_rrsp *rrsp, uint16_t ranging_counter);
int bt_ras_rd_buffer_bytes_left_get(struct bt_ras_rrsp *rrsp);
int bt_ras_rd_buffer_segment_get(struct bt_ras_rrsp *rrsp, uint8_t * data_out, uint16_t seg_size);
