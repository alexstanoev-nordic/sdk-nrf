/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_RAS_INTERNAL_H_
#define BT_RAS_INTERNAL_H_

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/ras.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RASCP_CMD_OPCODE_LEN     1
#define RASCP_CMD_OPCODE_OFFSET  0
#define RASCP_CMD_PARAMS_OFFSET  RASCP_CMD_OPCODE_LEN
#define RASCP_CMD_PARAMS_MAX_LEN 4
#define RASCP_WRITE_MAX_LEN (RASCP_CMD_OPCODE_LEN + RASCP_CMD_PARAMS_MAX_LEN)

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

	struct ras_rd_buffer *active_buf;
	struct k_work send_data_work;
	struct k_work rascp_work;
	struct k_work status_work;
	struct k_timer rascp_timeout;

	struct bt_gatt_indicate_params ondemand_ind_params;
	struct bt_gatt_indicate_params rascp_ind_params;
	struct bt_gatt_indicate_params rd_status_params;

	uint8_t rascp_cmd_buf[RASCP_WRITE_MAX_LEN];
	uint8_t rascp_cmd_len;

	uint16_t ready_ranging_counter;
	uint16_t overwritten_ranging_counter;
	uint16_t segment_counter;

	bool streaming; /* TODO: atomic */
	bool notify_ready; /* TODO handle overwriting older */
	bool notify_overwritten; /* TODO handle overwriting older */
};

struct bt_ras_rrsp *bt_ras_rrsp_find(struct bt_conn *conn);

void rrsp_rascp_cmd_handle(struct bt_ras_rrsp *rrsp);
void rrsp_rascp_send_complete_rd_rsp(struct bt_conn *conn, uint16_t ranging_counter);

int rrsp_ondemand_rd_notify_or_indicate(struct bt_conn *conn, struct net_buf_simple *buf);
int rrsp_rascp_indicate(struct bt_conn *conn, struct net_buf_simple *rsp);
void rrsp_rd_ready_indicate(struct bt_conn *conn, uint16_t ranging_counter);
void rrsp_rd_overwritten_indicate(struct bt_conn *conn, uint16_t ranging_counter);

#ifdef __cplusplus
}
#endif

#endif /* BT_RAS_INTERNAL_H_ */
