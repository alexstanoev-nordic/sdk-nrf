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

/* TODO: buffer per connection */

/* HCI Events */

int bt_ras_rd_buffer_ranging_header_set(uint8_t buffer_idx, uint16_t ranging_counter)
{
	/* TODO: Rewind randing data buffer and set header */
	/* TODO: Set rolling segment counter to 0 */
	/* TODO: Send ranging data overwriten indication if not all data was read */

	return 0;
}

int bt_ras_rd_buffer_subevent_append(uint8_t buffer_idx, uint16_t ranging_counter, uint8_t * data, uint16_t len)
{
	/* TODO: Append subevent to buffer if header has been set */
	return 0;
}

/* GATT TX */

int bt_ras_rd_buffer_segment_count_get(uint8_t buffer_idx, uint16_t seg_size)
{
	/* TODO: Return number of segments required to transmit stored data */
	return 0;
}

int bt_ras_rd_buffer_segment_get(uint8_t buffer_idx, uint16_t seg_size, uint8_t * data_out, uint8_t * seg_header_out)
{
	/* TODO: Return pointer to ranging data segment and populate segmentation header */
	/* TODO: Return size of ranging data segment */
	return 0;
}

/* GATT RX */

/* TODO: To be populated when implementing RAS RREQ client */
