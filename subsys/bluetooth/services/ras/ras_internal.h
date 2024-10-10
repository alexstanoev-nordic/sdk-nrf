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

/**@brief RAS Control Point opcodes. */
enum rascp_opcode {
	RASCP_OPCODE_GET_RD                    = 0x00,
	RASCP_OPCODE_ACK_RD                    = 0x01,
	RASCP_OPCODE_RETRIEVE_LOST_RD_SEGMENTS = 0x02,
	RASCP_OPCODE_ABORT_OP                  = 0x03,
	RASCP_OPCODE_SET_FILTER                = 0x04,
};

/**@brief RAS Control Point Response opcodes. */
enum rascp_rsp_opcode {
	RASCP_RSP_OPCODE_COMPLETE_RD_RSP          = 0x00,
	RASCP_RSP_OPCODE_COMPLETE_LOST_RD_SEG_RSP = 0x01,
	RASCP_RSP_OPCODE_RSP_CODE                 = 0x02,
};

/**@brief RAS Control Point Response length. */
enum rascp_rsp_opcode_len {
	RASCP_RSP_OPCODE_COMPLETE_RD_RSP_LEN          = 2,
	RASCP_RSP_OPCODE_COMPLETE_LOST_RD_SEG_RSP_LEN = 4,
	RASCP_RSP_OPCODE_RSP_CODE_LEN                 = 1,
};

/**@brief RAS Control Point response codes. */
enum rascp_rsp_code {
	RASCP_RESPONSE_RESERVED                = 0x00,
	RASCP_RESPONSE_SUCCESS                 = 0x01,
	RASCP_RESPONSE_OPCODE_NOT_SUPPORTED    = 0x02,
	RASCP_RESPONSE_INVALID_PARAMETER       = 0x03,
	RASCP_RESPONSE_SUCCESS_PERSISTED       = 0x04,
	RASCP_RESPONSE_ABORT_UNSUCCESSFUL      = 0x05,
	RASCP_RESPONSE_PROCEDURE_NOT_COMPLETED = 0x06,
	RASCP_RESPONSE_SERVER_BUSY             = 0x07,
	RASCP_RESPONSE_NO_RECORDS_FOUND        = 0x08,
};

#define RASCP_CMD_OPCODE_LEN     1
#define RASCP_CMD_OPCODE_OFFSET  0
#define RASCP_CMD_PARAMS_OFFSET  RASCP_CMD_OPCODE_LEN
#define RASCP_CMD_PARAMS_MAX_LEN 4

struct ras_get_ranging_data {
	uint8_t opcode;
	uint16_t ranging_counter;
} __packed;

struct ras_ack_ranging_data {
	uint8_t opcode;
	uint16_t ranging_counter;
} __packed;

struct ras_features {
	uint8_t real_time_support : 1;
	uint8_t retrieve_lost_rd_support : 1;
	uint8_t abort_op_support : 1;
	uint8_t filter_rd_support : 1;
	uint32_t rfu : 28;
} __packed;

struct bt_ras_rrsp *bt_ras_rrsp_find(struct bt_conn *conn);

void rrsp_rascp_cmd_handle(struct bt_ras_rrsp *rrsp);
void rrsp_rascp_send_complete_rd_rsp(struct bt_conn *conn, uint16_t ranging_counter);

int rrsp_ondemand_rd_notify_or_indicate(struct bt_conn *conn, struct net_buf_simple *buf);
int rrsp_rascp_indicate(struct bt_conn *conn, struct net_buf_simple *rsp);
void rrsp_rd_ready_indicate(struct bt_conn *conn, uint16_t ranging_counter);
void rrsp_rd_overwritten_indicate(struct bt_conn *conn, uint16_t ranging_counter);

enum bt_ras_rreq_cp_state {
	BT_RAS_RREQ_CP_STATE_NONE,
	BT_RAS_RREQ_CP_STATE_GET_RD_WRITTEN,
	BT_RAS_RREQ_CP_STATE_ACK_RD_WRITTEN,
};

struct bt_ras_rreq_features {
	struct bt_gatt_read_params read_params;
	struct ras_features remote_features;
};

struct bt_ras_rreq_cp {
	struct bt_gatt_subscribe_params subscribe_params;
	enum bt_ras_rreq_cp_state state;
};
struct bt_ras_on_demand_rd {
	struct bt_gatt_subscribe_params subscribe_params;
	struct net_buf_simple * ranging_data_out;
	bt_ras_rreq_ranging_data_get_complete_t cb;
	uint16_t counter_in_progress;
	uint8_t next_expected_segment_counter;
	bool data_get_in_progress;
	bool last_segment_received;
	bool error_with_data_receive;
};

struct bt_ras_rd_ready {
	struct bt_gatt_subscribe_params subscribe_params;
	bt_ras_rreq_rd_ready_cb_t cb;
};

struct bt_ras_rd_overwritten {
	struct bt_gatt_subscribe_params subscribe_params;
	bt_ras_rreq_rd_overwritten_cb_t cb;
};

struct bt_ras_rreq {
	/** Connection object. */
	struct bt_conn *conn;

	struct bt_ras_rreq_features features;

	struct bt_ras_rreq_cp cp;

	struct bt_ras_on_demand_rd on_demand_rd;

	struct bt_ras_rd_ready rd_ready;

	struct bt_ras_rd_overwritten rd_overwritten;
};

#ifdef __cplusplus
}
#endif

#endif /* BT_RAS_INTERNAL_H_ */
