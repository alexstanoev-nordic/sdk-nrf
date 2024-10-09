/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/ras.h>
#include <stdint.h>

#include "ras_internal.h"

LOG_MODULE_DECLARE(ras, CONFIG_BT_RAS_LOG_LEVEL);

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

void rrsp_rascp_send_complete_rd_rsp(struct bt_conn *conn, uint16_t ranging_counter)
{
	LOG_DBG("%u", ranging_counter);
	NET_BUF_SIMPLE_DEFINE(rsp, RASCP_CMD_OPCODE_LEN + RASCP_RSP_OPCODE_COMPLETE_RD_RSP_LEN);

	net_buf_simple_add_u8(&rsp, RASCP_RSP_OPCODE_COMPLETE_RD_RSP);
	net_buf_simple_add_le16(&rsp, ranging_counter);
	int err = rrsp_rascp_indicate(conn, &rsp); /* TODO handle error */
	if (err) {
		LOG_WRN("error %d", err);
	}
}

static void send_rsp_code(struct bt_conn *conn, enum rascp_rsp_code rsp_code)
{
	LOG_DBG("%d", rsp_code);
	NET_BUF_SIMPLE_DEFINE(rsp, RASCP_CMD_OPCODE_LEN + RASCP_RSP_OPCODE_RSP_CODE_LEN);

	net_buf_simple_add_u8(&rsp, RASCP_RSP_OPCODE_RSP_CODE);
	net_buf_simple_add_u8(&rsp, rsp_code);

	int err = rrsp_rascp_indicate(conn, &rsp); /* TODO handle error */
	if (err) {
		LOG_WRN("error %d", err);
	}
}

static void start_streaming(struct bt_ras_rrsp *rrsp, uint16_t ranging_counter)
{
	LOG_DBG("");
	rrsp->active_buf = bt_ras_rd_buffer_claim(rrsp->conn, ranging_counter);
	rrsp->segment_counter = 0;
	rrsp->streaming = true;

	extern struct k_work_q rrsp_wq; // FIXME
	k_work_submit_to_queue(&rrsp_wq, &rrsp->send_data_work);
}

void rrsp_rascp_cmd_handle(struct bt_ras_rrsp *rrsp)
{
	int err;
	ARG_UNUSED(err);

	struct net_buf_simple req;
	net_buf_simple_init_with_data(&req, rrsp->rascp_cmd_buf, rrsp->rascp_cmd_len);

	uint8_t opcode = net_buf_simple_pull_u8(&req);
	uint8_t param_len = MIN(req.len, RASCP_CMD_PARAMS_MAX_LEN);

	/* TODO: Handle RASCP_OPCODE_ABORT_OP */
	if (rrsp->streaming) {
		send_rsp_code(rrsp->conn, RASCP_RESPONSE_SERVER_BUSY);
		return;
	}

	switch (opcode) {
		case RASCP_OPCODE_GET_RD:
		{
			if (param_len != sizeof(uint16_t)) {
				LOG_DBG("invalid %d", param_len);
				send_rsp_code(rrsp->conn, RASCP_RESPONSE_INVALID_PARAMETER);
				return;
			}
			// TODO check subscription to ondemand rd
			uint16_t ranging_counter = net_buf_simple_pull_le16(&req);
			LOG_DBG("GET_RD %d", ranging_counter);

			if (rrsp->active_buf) {
				/* Disallow getting new ranging data until the current one has been ACKed. */
				send_rsp_code(rrsp->conn, RASCP_RESPONSE_SERVER_BUSY);
				return;
			}

			if (!bt_ras_rd_buffer_ready_check(rrsp->conn, ranging_counter)) {
				send_rsp_code(rrsp->conn, RASCP_RESPONSE_NO_RECORDS_FOUND);
				return;
			}

			send_rsp_code(rrsp->conn, RASCP_RESPONSE_SUCCESS);
			start_streaming(rrsp, ranging_counter);

			break;
		}
		case RASCP_OPCODE_ACK_RD:
		{
			if (param_len != sizeof(uint16_t)) {
				send_rsp_code(rrsp->conn, RASCP_RESPONSE_INVALID_PARAMETER);
				return;
			}

			uint16_t ranging_counter = net_buf_simple_pull_le16(&req);
			LOG_DBG("ACK_RD %d", ranging_counter);

			if (!rrsp->active_buf || rrsp->active_buf->ranging_counter != ranging_counter) {
				/* Only allow ACKing the currently requested ranging counter. */
				send_rsp_code(rrsp->conn, RASCP_RESPONSE_NO_RECORDS_FOUND);
				return;
			}

			rrsp->active_buf->acked = true;
			err = bt_ras_rd_buffer_release(rrsp->active_buf);
			__ASSERT_NO_MSG(!err);
			rrsp->active_buf = NULL;

			send_rsp_code(rrsp->conn, RASCP_RESPONSE_SUCCESS);

			break;
		}
		default:
		{
			LOG_INF("Opcode %x invalid or unsupported", opcode);
			send_rsp_code(rrsp->conn, RASCP_RESPONSE_OPCODE_NOT_SUPPORTED);
			break;
		}
	}
}
