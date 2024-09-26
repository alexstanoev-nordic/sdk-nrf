/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <bluetooth/services/ras.h>

#include "ras_internal.h"

LOG_MODULE_DECLARE(ras, CONFIG_BT_RAS_LOG_LEVEL);

/* Ranging Data stored by a RAS Server may contain zero or more CS Subevent Data elements, up to a
 * maximum of 32 elements. The CS Subevent Data can have 1-3 Mode 0 Step results and multiple non-
 * Mode 0 Step results, where the maximum number of CS Steps per CS Subevent is 160 and per CS
 * Procedure is 256
 */

/* HCI Events */

int bt_ras_rd_buffer_ranging_header_set(uint8_t buffer_idx, uint16_t ranging_counter)
{
	/* TODO: Allocate ranging data buffer and set header */
	/* TODO: Set rolling segment counter to 0 */
	/* TODO: Send ranging data overwriten indication if not all data was read */

	return 0;
}

int bt_ras_rd_buffer_subevent_append(uint8_t buffer_idx, uint16_t ranging_counter)
{
	/* TODO: Append subevent to buffer if header has been set */
	return 0;
}

/* GATT TX */

bool bt_ras_rd_buffer_ranging_counter_check(struct bt_ras_rrsp *rrsp, uint16_t ranging_counter)
{
	/* TODO: True if ranging_counter exists, otherwise false */
	return false;
}

bool bt_ras_rd_buffer_ranging_counter_free(struct bt_ras_rrsp *rrsp, uint16_t ranging_counter)
{
	/* TODO: Free counter, return false if it does not exist */
	return false;
}

int bt_ras_rd_buffer_bytes_left_get(struct bt_ras_rrsp *rrsp)
{
	/* TODO: Return number of unread bytes left from currently active buffer or 0 */
	return 0;
}

int bt_ras_rd_buffer_segment_get(struct bt_ras_rrsp *rrsp, uint8_t * data_out, uint16_t seg_size)
{
	/* TODO: Return pointer to ranging data segment */
	/* TODO: Return size of ranging data segment */
	return 0;
}

/* GATT RX */

/* TODO: To be populated when implementing RAS RREQ client */
