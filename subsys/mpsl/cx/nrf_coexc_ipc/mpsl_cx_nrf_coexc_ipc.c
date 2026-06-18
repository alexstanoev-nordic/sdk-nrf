/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 *   Nordic COEXC IPC coexistence interface implementation.
 *
 *   Two short-range COEXC clients are used: one for RX and one for TX.
 *   Each client uses two IPCT channels:
 *     - REQUEST is asserted by triggering an IPCT SEND task on the request
 *       channel; GRANT arrives as an IPCT RECEIVE event on the same channel.
 *     - RELEASE is asserted by triggering an IPCT SEND task on the release
 *       channel; REVOKE arrives as an IPCT RECEIVE event on the same channel.
 */

#include <mpsl_cx_abstract_interface.h>

#include <stddef.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <hal/nrf_ipct.h>

#if DT_NODE_EXISTS(DT_NODELABEL(nrf_radio_coex))
#define CX_NODE DT_NODELABEL(nrf_radio_coex)
#else
#error No enabled coex nodes registered in DTS.
#endif

#if !(DT_NODE_HAS_COMPAT(CX_NODE, nordic_nrf_coexc_ipc))
#error Selected coex node is not compatible with nordic,nrf-coexc-ipc.
#endif

#define COEXC_IPCT_NODE DT_PHANDLE(CX_NODE, nordic_ipct)
#define COEXC_IPCT_REG	((NRF_IPCT_Type *)DT_REG_ADDR(COEXC_IPCT_NODE))

#define COEXC_IPCT_IRQ_IDX   DT_PROP(CX_NODE, nordic_ipct_irq_index)
#define COEXC_IPCT_INT_GROUP COEXC_IPCT_IRQ_IDX
#define COEXC_IPCT_IRQN	     DT_IRQ_BY_IDX(COEXC_IPCT_NODE, COEXC_IPCT_IRQ_IDX, irq)
#define COEXC_IPCT_IRQ_PRIO  DT_IRQ_BY_IDX(COEXC_IPCT_NODE, COEXC_IPCT_IRQ_IDX, priority)

#define RX_REQUEST_IPCT_CH DT_PROP_BY_IDX(CX_NODE, rx_channels, 0)
#define RX_RELEASE_IPCT_CH DT_PROP_BY_IDX(CX_NODE, rx_channels, 1)
#define TX_REQUEST_IPCT_CH DT_PROP_BY_IDX(CX_NODE, tx_channels, 0)
#define TX_RELEASE_IPCT_CH DT_PROP_BY_IDX(CX_NODE, tx_channels, 1)

#define IPCT_NUM_CHANNELS DT_PROP(COEXC_IPCT_NODE, channels)

#define IPCT_REVOKE_INT_MASK ((1UL << RX_RELEASE_IPCT_CH) | (1UL << TX_RELEASE_IPCT_CH))

BUILD_ASSERT(DT_PROP_LEN(CX_NODE, rx_channels) == 2,
	     "rx-channels must be <request-channel release-channel>");
BUILD_ASSERT(DT_PROP_LEN(CX_NODE, tx_channels) == 2,
	     "tx-channels must be <request-channel release-channel>");

BUILD_ASSERT(RX_REQUEST_IPCT_CH < IPCT_NUM_CHANNELS && RX_RELEASE_IPCT_CH < IPCT_NUM_CHANNELS &&
		     TX_REQUEST_IPCT_CH < IPCT_NUM_CHANNELS &&
		     TX_RELEASE_IPCT_CH < IPCT_NUM_CHANNELS,
	     "COEXC IPC channel index out of range for the referenced IPCT");

BUILD_ASSERT(RX_REQUEST_IPCT_CH != RX_RELEASE_IPCT_CH && TX_REQUEST_IPCT_CH != TX_RELEASE_IPCT_CH &&
		     RX_REQUEST_IPCT_CH != TX_REQUEST_IPCT_CH &&
		     RX_REQUEST_IPCT_CH != TX_RELEASE_IPCT_CH &&
		     RX_RELEASE_IPCT_CH != TX_REQUEST_IPCT_CH &&
		     RX_RELEASE_IPCT_CH != TX_RELEASE_IPCT_CH,
	     "COEXC IPC channels must be distinct");

static atomic_t m_requested_ops;
static atomic_t m_granted_ops;
static mpsl_cx_cb_t m_callback;

static inline NRF_IPCT_Type *coexc_ipct(void)
{
	return COEXC_IPCT_REG;
}

static inline void ipct_send(uint8_t channel)
{
	nrf_ipct_task_trigger(coexc_ipct(), nrf_ipct_send_task_get(channel));
}

static inline void ipct_recv_clear(uint8_t channel)
{
	nrf_ipct_event_clear(coexc_ipct(), nrf_ipct_receive_event_get(channel));
}

static inline bool ipct_recv_check(uint8_t channel)
{
	return nrf_ipct_event_check(coexc_ipct(), nrf_ipct_receive_event_get(channel));
}

static int32_t request(const mpsl_cx_request_t *p_req_params)
{
	mpsl_cx_op_map_t requested_ops;

	if (p_req_params == NULL) {
		return -EINVAL;
	}

	requested_ops = p_req_params->ops & (MPSL_CX_OP_RX | MPSL_CX_OP_TX);

	atomic_and(&m_granted_ops, ~(atomic_val_t)requested_ops);
	atomic_set(&m_requested_ops, (atomic_val_t)requested_ops);

	if (requested_ops & MPSL_CX_OP_RX) {
		ipct_recv_clear(RX_REQUEST_IPCT_CH);
		ipct_recv_clear(RX_RELEASE_IPCT_CH);
		ipct_send(RX_REQUEST_IPCT_CH);
	}
	if (requested_ops & MPSL_CX_OP_TX) {
		ipct_recv_clear(TX_REQUEST_IPCT_CH);
		ipct_recv_clear(TX_RELEASE_IPCT_CH);
		ipct_send(TX_REQUEST_IPCT_CH);
	}

	return 0;
}

static int32_t release(void)
{
	mpsl_cx_op_map_t prev = (mpsl_cx_op_map_t)atomic_set(&m_requested_ops, 0);

	if (prev == 0) {
		return -EALREADY;
	}

	atomic_set(&m_granted_ops, 0);

	if (prev & MPSL_CX_OP_RX) {
		ipct_send(RX_RELEASE_IPCT_CH);
	}
	if (prev & MPSL_CX_OP_TX) {
		ipct_send(TX_RELEASE_IPCT_CH);
	}

	return 0;
}

static int32_t granted_ops_get(mpsl_cx_op_map_t *p_granted_ops)
{
	mpsl_cx_op_map_t requested;
	mpsl_cx_op_map_t observed;
	mpsl_cx_op_map_t newly_observed = 0;
	mpsl_cx_op_map_t granted = MPSL_CX_OP_IDLE_LISTEN;

	if (p_granted_ops == NULL) {
		return -EINVAL;
	}

	requested = (mpsl_cx_op_map_t)atomic_get(&m_requested_ops);
	observed = (mpsl_cx_op_map_t)atomic_get(&m_granted_ops);

	if ((requested & MPSL_CX_OP_RX) && ipct_recv_check(RX_REQUEST_IPCT_CH)) {
		granted |= MPSL_CX_OP_RX;
		newly_observed |= (MPSL_CX_OP_RX & ~observed);
	}
	if ((requested & MPSL_CX_OP_TX) && ipct_recv_check(TX_REQUEST_IPCT_CH)) {
		granted |= MPSL_CX_OP_TX;
		newly_observed |= (MPSL_CX_OP_TX & ~observed);
	}

	if (newly_observed) {
		atomic_or(&m_granted_ops, (atomic_val_t)newly_observed);
	}

	*p_granted_ops = granted;
	return 0;
}

static uint32_t req_grant_delay_get(void)
{
	return 0U;
}

static int32_t register_callback(mpsl_cx_cb_t cb)
{
	NRF_IPCT_Type *p_reg = coexc_ipct();

	m_callback = cb;
	if (cb != NULL) {
		nrf_ipct_int_enable(p_reg, COEXC_IPCT_INT_GROUP, IPCT_REVOKE_INT_MASK);
	} else {
		nrf_ipct_int_disable(p_reg, COEXC_IPCT_INT_GROUP, IPCT_REVOKE_INT_MASK);
	}
	return 0;
}

static const mpsl_cx_interface_t m_mpsl_cx_methods = {
	.p_request = request,
	.p_release = release,
	.p_granted_ops_get = granted_ops_get,
	.p_req_grant_delay_get = req_grant_delay_get,
	.p_register_callback = register_callback,
};

static void coexc_ipct_isr(const void *arg)
{
	ARG_UNUSED(arg);

	mpsl_cx_op_map_t revoked = 0;

	if (ipct_recv_check(RX_RELEASE_IPCT_CH)) {
		ipct_recv_clear(RX_RELEASE_IPCT_CH);
		ipct_recv_clear(RX_REQUEST_IPCT_CH);
		revoked |= MPSL_CX_OP_RX;
	}
	if (ipct_recv_check(TX_RELEASE_IPCT_CH)) {
		ipct_recv_clear(TX_RELEASE_IPCT_CH);
		ipct_recv_clear(TX_REQUEST_IPCT_CH);
		revoked |= MPSL_CX_OP_TX;
	}

	if (revoked == 0) {
		return;
	}

	mpsl_cx_op_map_t notify_mask = revoked & (mpsl_cx_op_map_t)atomic_get(&m_granted_ops);

	if (notify_mask == 0) {
		return;
	}

	atomic_val_t cb_ops =
		atomic_and(&m_granted_ops, ~(atomic_val_t)notify_mask) & ~(atomic_val_t)notify_mask;

	mpsl_cx_cb_t cb = m_callback;

	if (cb != NULL) {
		cb((mpsl_cx_op_map_t)cb_ops | MPSL_CX_OP_IDLE_LISTEN);
	}
}

static int mpsl_cx_init(void)
{
	int32_t ret;
	NRF_IPCT_Type *p_reg = coexc_ipct();

	NRF_SPU10->PERIPH[13].PERM = (SPU_PERIPH_PERM_SECATTR_Secure << SPU_PERIPH_PERM_SECATTR_Pos);

	ret = mpsl_cx_interface_set(&m_mpsl_cx_methods);
	if (ret != 0) {
		return ret;
	}

	nrf_ipct_int_disable(p_reg, COEXC_IPCT_INT_GROUP, 0xFFFFFFFFUL);
	ipct_recv_clear(RX_REQUEST_IPCT_CH);
	ipct_recv_clear(RX_RELEASE_IPCT_CH);
	ipct_recv_clear(TX_REQUEST_IPCT_CH);
	ipct_recv_clear(TX_RELEASE_IPCT_CH);

	IRQ_CONNECT(COEXC_IPCT_IRQN, COEXC_IPCT_IRQ_PRIO, coexc_ipct_isr, NULL, 0);
	irq_enable(COEXC_IPCT_IRQN);

	return 0;
}

SYS_INIT(mpsl_cx_init, POST_KERNEL, CONFIG_MPSL_CX_INIT_PRIORITY);
