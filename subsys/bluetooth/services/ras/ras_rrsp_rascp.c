/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
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

#define RASCP_CMD_OPCODE_OFFSET  0
#define RASCP_CMD_PARAMS_OFFSET  1
#define RASCP_CMD_PARAMS_MAX_LEN 4

static void send_rsp_code(struct bt_conn *conn, enum rascp_rsp_code rsp_code)
{
	uint8_t rsp[2] = { RASCP_RSP_OPCODE_RSP_CODE, rsp_code };
	(void)rrsp_ras_cp_indicate(conn, rsp, sizeof(rsp));
}

ssize_t rrsp_rasp_cmd_handle(struct bt_conn *conn, uint8_t const * command, uint16_t len)
{
	uint8_t opcode = command[RASCP_CMD_OPCODE_OFFSET];
	uint8_t param_len = MIN(len, RASCP_CMD_PARAMS_MAX_LEN) - 1;
	(void)param_len;

	switch (opcode) {
		case RASCP_OPCODE_GET_RD:
			LOG_DBG("GET_RD");
			break;

		case RASCP_OPCODE_ACK_RD:
			LOG_DBG("ACK_RD");
			break;

		default:
			LOG_INF("Opcode %x invalid or unsupported", opcode);
			send_rsp_code(conn, RASCP_RESPONSE_OPCODE_NOT_SUPPORTED);
			break;
	}

	return len;
}
