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

ssize_t rrsp_rasp_cmd_handle(struct bt_conn *conn, uint8_t const * command, uint16_t len);

int rrsp_ondemand_rd_notify_or_indicate(struct bt_conn *conn, const void *data, uint16_t len);
int rrsp_ras_cp_indicate(struct bt_conn *conn, const void *data, uint16_t len);
int rrsp_rd_ready_indicate(struct bt_conn *conn, uint16_t ranging_counter);
int rrsp_rd_overwritten_indicate(struct bt_conn *conn, uint16_t ranging_counter);
