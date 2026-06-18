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

#if defined(CONFIG_NRF71_SR_COEX_DRIVER)
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/sys_io.h>

#include <nrf71_sr_coex_api.h>
#endif

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

/* Without the Wi-Fi coexistence driver enabling coexistence, every request is granted. */
static atomic_t m_coex_enabled;

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

/* COEXC does not answer a REQUEST from a client that already holds the GRANT, so a client that
 * stays requested keeps its pending GRANT event instead of having it cleared and re-requested.
 */
static void client_request(bool already_requested, uint8_t request_ch, uint8_t release_ch)
{
	if (!already_requested) {
		ipct_recv_clear(request_ch);
		ipct_recv_clear(release_ch);
	}
	if (!ipct_recv_check(request_ch)) {
		ipct_send(request_ch);
	}
}

#if defined(CONFIG_NRF71_SR_COEX_DRIVER)

#define SR_RX_CLIENT	  4U
#define SR_TX_CLIENT	  5U
#define CCCONF_REG_STRIDE 4U

#define CCCONF_ADDR(client)  (ABS_COEXC_CCCONF_0 + (CCCONF_REG_STRIDE * (client)))
#define CCCONF_SR_MODE	     0U

enum {
	PTI_NORMAL = 0,
	PTI_CRITICAL = 1,
};

/* Priorities resolved from the ranges, indexed by [criticality][0 = RX, 1 = TX]. */
static atomic_t m_sr_pti[2][2];

/* The highest priority of the range is used when the level is left unspecified. */
static bool m_range_value(const unsigned char range[NUM_ELEMENTS_IN_CCCONF_PTI_RANGE],
			  unsigned char level, unsigned int *p_out)
{
	const unsigned int start = range[0];
	const unsigned int end = range[1];
	const unsigned int step = range[2];

	if ((step == 0U) || (start < end)) {
		return false;
	}

	const unsigned int count = ((start - end) / step) + 1U;
	unsigned int index = count - 1U;

	if (level != COEX_SR_PRIORITY_LEVEL_UNUSED) {
		if (level >= count) {
			return false;
		}
		index = level;
	}

	*p_out = start - (index * step);
	return true;
}

static void m_ccconf_priority_set(uint32_t client, uint32_t pti)
{
	sys_write32((pti << COEXC_CCCONF_0_PRIORITY_SHIFT) |
			    (CCCONF_SR_MODE << COEXC_CCCONF_0_MODE_SHIFT),
		    CCCONF_ADDR(client));
}

/* COEXC arbitrates on the CCCONF priority of the requesting client, so it is set before the
 * REQUEST is signalled.
 */
static void sr_priority_apply(mpsl_cx_op_map_t ops, bool critical)
{
	const int kind = critical ? PTI_CRITICAL : PTI_NORMAL;

	if (ops & MPSL_CX_OP_RX) {
		m_ccconf_priority_set(SR_RX_CLIENT, (uint32_t)atomic_get(&m_sr_pti[kind][0]));
	}
	if (ops & MPSL_CX_OP_TX) {
		m_ccconf_priority_set(SR_TX_CLIENT, (uint32_t)atomic_get(&m_sr_pti[kind][1]));
	}
}

static unsigned int sr_coex_enable(unsigned int enable_coex)
{
	unsigned int key = irq_lock();
	const mpsl_cx_op_map_t requested = (mpsl_cx_op_map_t)atomic_get(&m_requested_ops);
	const bool was_enabled = atomic_set(&m_coex_enabled, enable_coex != 0U) != 0;

	if ((enable_coex != 0U) && !was_enabled) {
		/* Grants were assumed until now; claim the clients for the ongoing request. */
		if (requested & MPSL_CX_OP_RX) {
			client_request(false, RX_REQUEST_IPCT_CH, RX_RELEASE_IPCT_CH);
		}
		if (requested & MPSL_CX_OP_TX) {
			client_request(false, TX_REQUEST_IPCT_CH, TX_RELEASE_IPCT_CH);
		}
	} else if ((enable_coex == 0U) && was_enabled) {
		if (requested & MPSL_CX_OP_RX) {
			ipct_send(RX_RELEASE_IPCT_CH);
		}
		if (requested & MPSL_CX_OP_TX) {
			ipct_send(TX_RELEASE_IPCT_CH);
		}
		ipct_recv_clear(RX_REQUEST_IPCT_CH);
		ipct_recv_clear(RX_RELEASE_IPCT_CH);
		ipct_recv_clear(TX_REQUEST_IPCT_CH);
		ipct_recv_clear(TX_RELEASE_IPCT_CH);
	}

	irq_unlock(key);
	return 1U;
}

static unsigned int sr_client_priority_set(const struct coex_sr_priority_range_t *sr_priority_range)
{
	unsigned int resolved[2][2];

	if (sr_priority_range == NULL) {
		return 0U;
	}

	const unsigned char level = sr_priority_range->client_priority_level;

	if (!m_range_value(sr_priority_range->sr_rx_client_ccconf_pti_range, level,
			   &resolved[PTI_NORMAL][0]) ||
	    !m_range_value(sr_priority_range->sr_tx_client_ccconf_pti_range, level,
			   &resolved[PTI_NORMAL][1]) ||
	    !m_range_value(sr_priority_range->sr_rx_client_critical_ccconf_pti_range, level,
			   &resolved[PTI_CRITICAL][0]) ||
	    !m_range_value(sr_priority_range->sr_tx_client_critical_ccconf_pti_range, level,
			   &resolved[PTI_CRITICAL][1])) {
		return 0U;
	}

	for (int kind = 0; kind < 2; kind++) {
		for (int dir = 0; dir < 2; dir++) {
			atomic_set(&m_sr_pti[kind][dir], (atomic_val_t)resolved[kind][dir]);
		}
	}

	return 1U;
}

static const struct coex_sr_driver_cb m_sr_driver_cb = {
	.enable = sr_coex_enable,
	.set_client_priority = sr_client_priority_set,
};

static int sr_driver_cb_register(void)
{
	return coex_cd_register_sr_driver_cb(&m_sr_driver_cb);
}

#else

static inline void sr_priority_apply(mpsl_cx_op_map_t ops, bool critical)
{
	ARG_UNUSED(ops);
	ARG_UNUSED(critical);
}

static inline int sr_driver_cb_register(void)
{
	return 0;
}

#endif /* CONFIG_NRF71_SR_COEX_DRIVER */

static int32_t request(const mpsl_cx_request_t *p_req_params)
{
	mpsl_cx_op_map_t requested_ops;
	mpsl_cx_op_map_t prev_ops;
	mpsl_cx_op_map_t withdrawn_ops;

	if (p_req_params == NULL) {
		return -EINVAL;
	}

	requested_ops = p_req_params->ops & (MPSL_CX_OP_RX | MPSL_CX_OP_TX);

	prev_ops = (mpsl_cx_op_map_t)atomic_set(&m_requested_ops, (atomic_val_t)requested_ops);
	withdrawn_ops = prev_ops & ~requested_ops;

	atomic_and(&m_granted_ops, ~(atomic_val_t)withdrawn_ops);

	if (!atomic_get(&m_coex_enabled)) {
		return 0;
	}

	/* Each request overrides the previous one, so any client that was part of the
	 * previous request but is not requested now must be released. A COEXC client
	 * that was granted keeps holding the resource (and its GRANTED state used for
	 * arbitration) until the client sends RELEASE; a still-pending request must
	 * likewise be withdrawn with RELEASE.
	 */
	if (withdrawn_ops & MPSL_CX_OP_RX) {
		ipct_send(RX_RELEASE_IPCT_CH);
		ipct_recv_clear(RX_REQUEST_IPCT_CH);
		ipct_recv_clear(RX_RELEASE_IPCT_CH);
	}
	if (withdrawn_ops & MPSL_CX_OP_TX) {
		ipct_send(TX_RELEASE_IPCT_CH);
		ipct_recv_clear(TX_REQUEST_IPCT_CH);
		ipct_recv_clear(TX_RELEASE_IPCT_CH);
	}

	sr_priority_apply(requested_ops, p_req_params->prio > (UINT8_MAX / 2));

	if (requested_ops & MPSL_CX_OP_RX) {
		client_request(prev_ops & MPSL_CX_OP_RX, RX_REQUEST_IPCT_CH, RX_RELEASE_IPCT_CH);
	}
	if (requested_ops & MPSL_CX_OP_TX) {
		client_request(prev_ops & MPSL_CX_OP_TX, TX_REQUEST_IPCT_CH, TX_RELEASE_IPCT_CH);
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

	if (!atomic_get(&m_coex_enabled)) {
		return 0;
	}

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
	mpsl_cx_op_map_t granted = MPSL_CX_OP_IDLE_LISTEN;

	if (p_granted_ops == NULL) {
		return -EINVAL;
	}

	requested = (mpsl_cx_op_map_t)atomic_get(&m_requested_ops);

	if (!atomic_get(&m_coex_enabled)) {
		granted |= requested & (MPSL_CX_OP_RX | MPSL_CX_OP_TX);
		atomic_or(&m_granted_ops, (atomic_val_t)(granted & (MPSL_CX_OP_RX | MPSL_CX_OP_TX)));
		*p_granted_ops = granted;
		return 0;
	}

	/* This function runs in the radio timeslot context, which has higher priority than the
	 * COEXC IPCT (revoke) interrupt. Sampling a GRANT (RECEIVE event on the request channel)
	 * and recording it in m_granted_ops therefore cannot be preempted by the revoke ISR, so
	 * no explicit synchronization is required; individual accesses use atomics for coherency.
	 */
	if ((requested & MPSL_CX_OP_RX) && ipct_recv_check(RX_REQUEST_IPCT_CH)) {
		granted |= MPSL_CX_OP_RX;
	}
	if ((requested & MPSL_CX_OP_TX) && ipct_recv_check(TX_REQUEST_IPCT_CH)) {
		granted |= MPSL_CX_OP_TX;
	}

	atomic_or(&m_granted_ops, (atomic_val_t)(granted & (MPSL_CX_OP_RX | MPSL_CX_OP_TX)));

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

	/* The IPCT sink holds back further signals on a channel until it is flushed, so without
	 * these shortcuts only the first GRANT and REVOKE per channel would raise RECEIVE.
	 */
	nrf_ipct_shorts_enable(p_reg, BIT(IPCT_SHORTS_RECEIVE0_FLUSH0_Pos + RX_REQUEST_IPCT_CH) |
					      BIT(IPCT_SHORTS_RECEIVE0_FLUSH0_Pos + RX_RELEASE_IPCT_CH) |
					      BIT(IPCT_SHORTS_RECEIVE0_FLUSH0_Pos + TX_REQUEST_IPCT_CH) |
					      BIT(IPCT_SHORTS_RECEIVE0_FLUSH0_Pos + TX_RELEASE_IPCT_CH));
	nrf_ipct_task_trigger(p_reg, nrf_ipct_flush_task_get(RX_REQUEST_IPCT_CH));
	nrf_ipct_task_trigger(p_reg, nrf_ipct_flush_task_get(RX_RELEASE_IPCT_CH));
	nrf_ipct_task_trigger(p_reg, nrf_ipct_flush_task_get(TX_REQUEST_IPCT_CH));
	nrf_ipct_task_trigger(p_reg, nrf_ipct_flush_task_get(TX_RELEASE_IPCT_CH));

	ipct_recv_clear(RX_REQUEST_IPCT_CH);
	ipct_recv_clear(RX_RELEASE_IPCT_CH);
	ipct_recv_clear(TX_REQUEST_IPCT_CH);
	ipct_recv_clear(TX_RELEASE_IPCT_CH);

	IRQ_CONNECT(COEXC_IPCT_IRQN, COEXC_IPCT_IRQ_PRIO, coexc_ipct_isr, NULL, 0);
	irq_enable(COEXC_IPCT_IRQN);

	return sr_driver_cb_register();
}

SYS_INIT(mpsl_cx_init, POST_KERNEL, CONFIG_MPSL_CX_INIT_PRIORITY);
