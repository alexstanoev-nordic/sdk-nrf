/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef BT_RAS_H_
#define BT_RAS_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#define BT_RAS_MAX_SUBEVENTS_PER_PROCEDURE 32
#define BT_RAS_MAX_STEPS_PER_PROCEDURE     256

#define BT_RAS_RANGING_HEADER_LEN  (sizeof(struct ras_ranging_header))
#define BT_RAS_SUBEVENT_HEADER_LEN (sizeof(struct ras_subevent_header))
#define BT_RAS_STEP_MODE_LEN       1
#define BT_RAS_MAX_STEP_DATA_LEN   35

/* TODO: This is most likely overestimating the size */
#define BT_RAS_PROCEDURE_MEM (BT_RAS_RANGING_HEADER_LEN \
			     + (BT_RAS_MAX_SUBEVENTS_PER_PROCEDURE * BT_RAS_SUBEVENT_HEADER_LEN) \
			     + (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_STEP_MODE_LEN) \
			     + (BT_RAS_MAX_STEPS_PER_PROCEDURE * BT_RAS_MAX_STEP_DATA_LEN))

/** @brief RAS Ranging Data Buffer callback structure. */
struct bt_ras_rd_buffer_cb {
	/** @brief New ranging data has been received from the local controller.
	 *
	 *  This callback notifies the application that the ranging data buffer
	 *  has reassembled a complete ranging procedure from the local controller.
	 *
	 *  @param conn Connection object.
	 *  @param ranging_counter Ranging counter of the stored procedure.
	 */
	void (*new_ranging_data_received)(struct bt_conn *conn, uint16_t ranging_counter);

	/** @brief Ranging data has been overwritten.
	 *
	 *  This callback notifies the application that the ranging data buffer
	 *  has overwritten a stored procedure due to running out of buffers
	 *  to store a newer procedure from the local controller.
	 *
	 *  @param conn Connection object.
	 *  @param ranging_counter Ranging counter of the overwritten procedure.
	 */
	void (*ranging_data_overwritten)(struct bt_conn *conn, uint16_t ranging_counter);

	sys_snode_t node;
};

/** @brief Ranging Header as defined in Ranging Service Specification */
struct ras_ranging_header {
	uint16_t ranging_counter : 12;
	uint8_t  config_id       : 4;
	int8_t   selected_tx_power;
	uint8_t  antenna_paths_mask;
} __packed;
BUILD_ASSERT(sizeof(struct ras_ranging_header) == 4);

/** @brief Subevent Header as defined in Ranging Service Specification */
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
BUILD_ASSERT(sizeof(struct ras_subevent_header) == 8);

/** @brief Ranging Data buffer
 * Ranging data buffer as defined in RAS Section 3.2
 *
 * @note The following fields are not included in this buffer:
 *       - Subevent count
 *       - Subevent length
 *       - Step channel
 */
struct ras_rd_buffer {
	struct bt_conn *conn;
	uint16_t ranging_counter;
	uint16_t subevent_cursor;
	uint16_t read_cursor;
	uint8_t ready : 1; /* All ranging data has been written */
	uint8_t busy : 1; /* Buffer is receiving data from HCI */
	uint8_t acked : 1; /* Buffer has been ACKed, do not notify overwritten */
	uint8_t refcount; /* TODO atomic */
	union {
		uint8_t buf[BT_RAS_PROCEDURE_MEM];
		struct {
			struct ras_ranging_header ranging_header;
			uint8_t subevents[];
		} __packed;
	} procedure;
};

/* Ranging service API */

/** @brief Initialize RAS Ranging Responder service.
 *
 * This will initialize the Ranging Service in the Ranging Responder role.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_ras_rrsp_init(void);

/** @brief Allocate Ranging Responder instance for connection.
 *
 * This will allocate an instance of the Ranging Responder service for the given connection.
 *
 *  @note The number of supported instances can be set using CONFIG_BT_RAS_MAX_ACTIVE_RRSP
 *
 *  @param conn Connection instance.
 *
 *  @return Zero in case of success and error code in case of error.
 */
int bt_ras_rrsp_alloc(struct bt_conn *conn);

/** @brief Free Ranging Responder instance for connection.
 *
 * This will free an allocated instance of the Ranging Responder service for
 * the given connection, if one has been allocated.
 * If the connection has no instance allocated, this method has no effect.
 *
 *  @param conn Connection instance.
 *
 */
void bt_ras_rrsp_free(struct bt_conn *conn);

/* Ranging data buffer API */

void bt_ras_rd_buffer_init(void);
void bt_ras_rd_buffer_cb_register(struct bt_ras_rd_buffer_cb *cb);
bool bt_ras_rd_buffer_ready_check(struct bt_conn *conn, uint16_t ranging_counter);
struct ras_rd_buffer *bt_ras_rd_buffer_claim(struct bt_conn *conn, uint16_t ranging_counter);
int bt_ras_rd_buffer_release(struct ras_rd_buffer *buf);
int bt_ras_rd_buffer_bytes_pull(struct ras_rd_buffer *buf, uint8_t *out_buf, uint16_t max_data_len);
void bt_ras_rd_buffer_rewind(struct ras_rd_buffer *buf, uint16_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* BT_RAS_H_ */
