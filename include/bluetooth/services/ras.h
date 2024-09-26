/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

/** @brief UUID of the Ranging Service. **/
#define BT_UUID_RANGING_SERVICE_VAL (0x185B)

/** @brief UUID of the RAS Features Characteristic. **/
#define BT_UUID_RAS_FEATURES_VAL (0x2C14)

/** @brief UUID of the Real-time Ranging Data Characteristic. **/
#define BT_UUID_RAS_REALTIME_RD_VAL (0x2C15)

/** @brief UUID of the On-demand Ranging Data Characteristic. **/
#define BT_UUID_RAS_ONDEMAND_RD_VAL (0x2C16)

/** @brief UUID of the RAS Control Point Characteristic. **/
#define BT_UUID_RAS_CP_VAL (0x2C17)

/** @brief UUID of the Ranging Data Ready Characteristic. **/
#define BT_UUID_RAS_RD_READY_VAL (0x2C18)

/** @brief UUID of the Ranging Data Overwritten Characteristic. **/
#define BT_UUID_RAS_RD_OVERWRITTEN_VAL (0x2C19)

#define BT_UUID_RANGING_SERVICE    BT_UUID_DECLARE_16(BT_UUID_RANGING_SERVICE_VAL)
#define BT_UUID_RAS_FEATURES       BT_UUID_DECLARE_16(BT_UUID_RAS_FEATURES_VAL)
#define BT_UUID_RAS_REALTIME_RD    BT_UUID_DECLARE_16(BT_UUID_RAS_REALTIME_RD_VAL)
#define BT_UUID_RAS_ONDEMAND_RD    BT_UUID_DECLARE_16(BT_UUID_RAS_ONDEMAND_RD_VAL)
#define BT_UUID_RAS_CP             BT_UUID_DECLARE_16(BT_UUID_RAS_CP_VAL)
#define BT_UUID_RAS_RD_READY       BT_UUID_DECLARE_16(BT_UUID_RAS_RD_READY_VAL)
#define BT_UUID_RAS_RD_OVERWRITTEN BT_UUID_DECLARE_16(BT_UUID_RAS_RD_OVERWRITTEN_VAL)

struct ras_ranging_header {
	uint16_t ranging_counter : 12;
	uint8_t  config_id       : 4;
	int8_t   selected_tx_power;
	uint8_t  antenna_paths_mask;
} __packed;

struct ras_subevent_header {
	uint16_t start_acl_conn_event;
	int16_t freq_compensation;
	uint8_t ranging_done_status   : 4;
	uint8_t subevent_done_status  : 4;
	uint8_t ranging_abort_reason  : 4;
	uint8_t subevent_abort_reason : 4;
	int8_t  ref_power_level;
	uint8_t num_steps_reported;
} __packed;

struct ras_subevent {
	struct ras_subevent_header header;
	uint8_t * data;
} __packed;

int bt_ras_rrsp_init(void);
int bt_ras_rrsp_alloc(struct bt_conn *conn);
void bt_ras_rrsp_free(struct bt_conn *conn);
