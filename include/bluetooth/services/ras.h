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
#include <bluetooth/gatt_dm.h>

/** @file
 *  @defgroup bt_ras Ranging Service API
 *  @{
 *  @brief API for the Ranging Service (RAS).
 */

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

#define BT_RAS_RANGING_HEADER_LEN  4
#define BT_RAS_SUBEVENT_HEADER_LEN 8
#define BT_RAS_STEP_MODE_LEN       1
#define BT_RAS_MAX_STEP_DATA_LEN   35

/** @brief Ranging Header structure as defined in RAS Specification, Table 3.7. */
struct ras_ranging_header {
	/** Ranging Counter is lower 12-bits of CS Procedure_Counter provided by the Core Controller
	 *  (Core Specification, Volume 4, Part E, Section 7.7.65.44).
	 */
	uint16_t ranging_counter : 12;
	/** CS configuration identifier. Range: 0 to 3. */
	uint8_t  config_id       : 4;
	/** Transmit power level used for the CS Procedure. Range: -127 to 20. Units: dBm. */
	int8_t   selected_tx_power;
	/** Antenna paths that are reported:
	 *  Bit0: 1 if Antenna Path_1 included; 0 if not.
	 *  Bit1: 1 if Antenna Path_2 included; 0 if not.
	 *  Bit2: 1 if Antenna Path_3 included; 0 if not.
	 *  Bit3: 1 if Antenna Path_4 included; 0 if not.
	 *  Bits 4-7: RFU
	 */
	uint8_t  antenna_paths_mask;
} __packed;
BUILD_ASSERT(sizeof(struct ras_ranging_header) == BT_RAS_RANGING_HEADER_LEN);

/** @brief Subevent Header structure as defined in RAS Specification, Table 3.8. */
struct ras_subevent_header {
	/** Starting ACL connection event count for the results reported in the event */
	uint16_t start_acl_conn_event;
	/** Frequency compensation value in units of 0.01 ppm (15-bit signed integer).
	 *  Note this value can be BT_HCI_LE_CS_SUBEVENT_RESULT_FREQ_COMPENSATION_NOT_AVAILABLE
	 *  if the role is not the initiator, or the frequency compensation value is unavailable.
	 */
	uint16_t freq_compensation;
	/** Ranging Done Status:
	 *  0x0: All results complete for the CS Procedure
	 *  0x1: Partial results with more to follow for the CS procedure
	 *  0xF: All subsequent CS Procedures aborted
	 *  All other values: RFU
	 */
	uint8_t ranging_done_status   : 4;
	/** Subevent Done Status:
	 *  0x0: All results complete for the CS Subevent
	 *  0xF: Current CS Subevent aborted.
	 *  All other values: RFU
	 */
	uint8_t subevent_done_status  : 4;
	/** Indicates the abort reason when Procedure_Done Status received from the Core Controller
	 *  (Core Specification, Volume 4, Part 4, Section 7.7.65.44) is set to 0xF,
	 *  otherwise the value is set to zero.
	 *  0x0: Report with no abort
	 *  0x1: Abort because of local Host or remote request
	 *  0x2: Abort because filtered channel map has less than 15 channels
	 *  0x3: Abort because the channel map update instant has passed
	 *  0xF: Abort because of unspecified reasons
	 *  All other values: RFU
	 */
	uint8_t ranging_abort_reason  : 4;
	/** Indicates the abort reason when Subevent_Done_Status received from the Core Controller
	 * (Core Specification, Volume 4, Part 4, Section 7.7.65.44) is set to 0xF,
	 * otherwise the default value is set to zero.
	 * 0x0: Report with no abort
	 * 0x1: Abort because of local Host or remote request
	 * 0x2: Abort because no CS_SYNC (mode 0) received
	 * 0x3: Abort because of scheduling conflicts or limited resources
	 * 0xF: Abort because of unspecified reasons
	 * All other values: RFU
	 */
	uint8_t subevent_abort_reason : 4;
	/** Reference power level. Range: -127 to 20. Units: dBm */
	int8_t  ref_power_level;
	/** Number of steps in the CS Subevent for which results are reported.
	 *  If the Subevent is aborted, then the Number Of Steps Reported can be set to zero
	 */
	uint8_t num_steps_reported;
} __packed;
BUILD_ASSERT(sizeof(struct ras_subevent_header) == BT_RAS_SUBEVENT_HEADER_LEN);

/** @brief Ranging data ready callback. Called when peer has ranging data available.
 *
 * @param[in] conn            Connection Object.
 * @param[in] ranging_counter Ranging counter ready to be requested.
 */
typedef void (*bt_ras_rreq_rd_ready_cb_t)(struct bt_conn *conn, uint16_t ranging_counter);

/** @brief Ranging data overwritted callback. Called when peer has overwritten previously available
 * ranging data.
 *
 * @param[in] conn            Connection Object.
 * @param[in] ranging_counter Ranging counter which has been overwritten.
 */
typedef void (*bt_ras_rreq_rd_overwritten_cb_t)(struct bt_conn *conn, uint16_t ranging_counter);

/** @brief Ranging data get complete callback. Called when ranging data get procedure has completed.
 *
 * @param[in] conn            Connection Object.
 * @param[in] ranging_counter Ranging counter which has been overwritten.
 * @param[in] err             Error code, 0 if the ranging data get was successful. Otherwise a
 * negative error code.
 */
typedef void (*bt_ras_rreq_ranging_data_get_complete_t)(struct bt_conn *conn,
							uint16_t ranging_counter, int err);

/** @brief Allocate a RREQ context and assign GATT handles. Takes a reference to the connection.
 *
 * @param[in] dm   Discovery Object.
 * @param[in] conn Connection Object.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_ras_rreq_alloc_and_assign_handles(struct bt_gatt_dm *dm, struct bt_conn *conn);

/** @brief Get ranging data for given ranging counter.
 *
 * @note This should only be called after receiving a ranging data ready callback and
 * when subscribed to ondemand ranging data and RAS-CP.
 *
 * @param[in] conn                 Connection Object.
 * @param[in] ranging_data_out     Simple buffer to store received ranging data.
 * @param[in] ranging_counter      Ranging counter to get.
 * @param[in] data_get_complete_cb Callback called when get ranging data completes.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_ras_rreq_cp_get_ranging_data(struct bt_conn *conn, struct net_buf_simple *ranging_data_out,
				    uint16_t ranging_counter,
				    bt_ras_rreq_ranging_data_get_complete_t data_get_complete_cb);

/** @brief Free RREQ context for connection. Should be called from disconnected callback. This will
 * unsubscribe from any remaining subscriptions.
 *
 * @param[in] conn Connection Object.
 */
void bt_ras_rreq_free(struct bt_conn *conn);

/** @brief Subscribe to all required on-demand ranging data subscriptions. Subscribes to ranging
 * data ready, ranging data overwritten, on-demand ranging data and RAS-CP.
 *
 * @param[in] conn              Connection Object.
 * @param[in] rd_ready_cb       Callback called when ranging data ready notification has been
 * received.
 * @param[in] rd_overwritten_cb Callback called when ranging data overwritten notification has been
 * received, unless get ranging data has already been called for this ranging counter where
 * data_get_complete_cb will be called instead.
 *
 * @retval 0 If the operation was successful.
 *           Otherwise, a negative error code is returned.
 */
int bt_ras_rreq_on_demand_ranging_data_subscribe_all(
	struct bt_conn *conn, bt_ras_rreq_rd_ready_cb_t rd_ready_cb,
	bt_ras_rreq_rd_overwritten_cb_t rd_overwritten_cb);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_RAS_H_ */
