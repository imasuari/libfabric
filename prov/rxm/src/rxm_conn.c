/*
 * Copyright (c) 2016-2021 Intel Corporation, Inc.  All rights reserved.
 * Copyright (c) 2019 Amazon.com, Inc. or its affiliates. All rights reserved.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include <poll.h>

#include <ofi.h>
#include <ofi_util.h>
#include "rxm.h"

static void rxm_flush_msg_cq(struct rxm_ep *rxm_ep);


/* castable to fi_eq_cm_entry - we can't use fi_eq_cm_entry directly
 * here because of a compiler error with a 0-sized array
 */
struct rxm_eq_cm_entry {
	fid_t fid;
	struct fi_info *info;
	union rxm_cm_data data;
};


static void rxm_close_conn(struct rxm_conn *conn)
{
	struct rxm_deferred_tx_entry *tx_entry;
	struct fi_peer_rx_entry *rx_entry;
	struct rxm_rx_buf *buf;

	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL, "closing conn %p\n", conn);

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));
	/* All deferred transfers are internally generated */
	while (!dlist_empty(&conn->deferred_tx_queue)) {
		tx_entry = container_of(conn->deferred_tx_queue.next,
				     struct rxm_deferred_tx_entry, entry);
		rxm_dequeue_deferred_tx(tx_entry);
		free(tx_entry);
	}

	while (!dlist_empty(&conn->deferred_sar_segments)) {
		buf = container_of(conn->deferred_sar_segments.next,
				   struct rxm_rx_buf, unexp_entry);
		dlist_remove(&buf->unexp_entry);
	}

	while (!dlist_empty(&conn->deferred_sar_msgs)) {
		rx_entry = (struct fi_peer_rx_entry*)conn->deferred_sar_msgs.next;
		rx_entry->srx->owner_ops->free_entry(rx_entry);
	}
	conn->connected_msg_eps = 0;
	conn->lazy_connecting = false;
	conn->failed_slot = -1;

	if (conn->msg_eps) {
		for (uint8_t i = 0; i < conn->num_msg_eps; i++) {
			if (conn->msg_eps[i]) {
				fi_close(&conn->msg_eps[i]->fid);
				conn->msg_eps[i] = NULL;
			}
		}
	}
	rxm_flush_msg_cq(conn->ep);
	dlist_remove_init(&conn->loopback_entry);

	if (conn->state == RXM_CM_CONNECTING || conn->state == RXM_CM_ACCEPTING)
		conn->ep->connecting_cnt--;
	assert(conn->ep->connecting_cnt >= 0);
	conn->state = RXM_CM_IDLE;
}

static int rxm_bind_comp(struct rxm_ep *ep, struct fid_ep *msg_ep)
{
	struct rxm_cntr *cntr;
	int ret;

	ret = fi_ep_bind(msg_ep, &ep->msg_cq->fid, FI_TRANSMIT | FI_RECV);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
		return ret;
	}

	if (!rxm_passthru_info(ep->rxm_info))
		return 0;

	if (ep->util_ep.cntrs[CNTR_TX]) {
		cntr = container_of(ep->util_ep.cntrs[CNTR_TX], struct rxm_cntr,
				    util_cntr);
		ret = fi_ep_bind(msg_ep, &cntr->msg_cntr->fid, FI_SEND);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
			return ret;
		}
	}

	if (ep->util_ep.cntrs[CNTR_RX]) {
		cntr = container_of(ep->util_ep.cntrs[CNTR_RX], struct rxm_cntr,
				    util_cntr);
		ret = fi_ep_bind(msg_ep, &cntr->msg_cntr->fid, FI_RECV);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
			return ret;
		}
	}

	if (ep->util_ep.cntrs[CNTR_RD]) {
		cntr = container_of(ep->util_ep.cntrs[CNTR_RD], struct rxm_cntr,
				    util_cntr);
		ret = fi_ep_bind(msg_ep, &cntr->msg_cntr->fid, FI_READ);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
			return ret;
		}
	}

	if (ep->util_ep.cntrs[CNTR_WR]) {
		cntr = container_of(ep->util_ep.cntrs[CNTR_WR], struct rxm_cntr,
				    util_cntr);
		ret = fi_ep_bind(msg_ep, &cntr->msg_cntr->fid, FI_WRITE);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
			return ret;
		}
	}

	if (ep->util_ep.cntrs[CNTR_REM_RD]) {
		cntr = container_of(ep->util_ep.cntrs[CNTR_REM_RD], struct rxm_cntr,
				    util_cntr);
		ret = fi_ep_bind(msg_ep, &cntr->msg_cntr->fid, FI_REMOTE_READ);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
			return ret;
		}
	}

	if (ep->util_ep.cntrs[CNTR_REM_WR]) {
		cntr = container_of(ep->util_ep.cntrs[CNTR_REM_WR], struct rxm_cntr,
				    util_cntr);
		ret = fi_ep_bind(msg_ep, &cntr->msg_cntr->fid, FI_REMOTE_WRITE);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
			return ret;
		}
	}

	return 0;
}

/* Allocate conn->msg_eps and conn->slots on the primary-open path.
 * Slots 1..N-1 remain NULL until lazy-connected. Idempotent across
 * close/reopen cycles: rxm_close_conn leaves the arrays allocated (to
 * keep fid->context pointers valid for late EQ events) with every
 * msg_eps[i] nulled; this function reuses them when it finds arrays
 * already present.
 */
static int rxm_alloc_slot_arrays(struct rxm_conn *conn)
{
	if (conn->msg_eps && conn->slots) {
		for (uint8_t i = 0; i < conn->num_msg_eps; i++) {
			assert(!conn->msg_eps[i]);
			conn->slots[i].conn = conn;
			conn->slots[i].slot_idx = i;
		}
		return 0;
	}
	assert(!conn->msg_eps && !conn->slots);
	conn->msg_eps = calloc(conn->num_msg_eps, sizeof(*conn->msg_eps));
	conn->slots = calloc(conn->num_msg_eps, sizeof(*conn->slots));
	if (!conn->msg_eps || !conn->slots) {
		free(conn->msg_eps);
		conn->msg_eps = NULL;
		free(conn->slots);
		conn->slots = NULL;
		return -FI_ENOMEM;
	}
	for (uint8_t i = 0; i < conn->num_msg_eps; i++) {
		conn->slots[i].conn = conn;
		conn->slots[i].slot_idx = i;
	}
	return 0;
}

/* Open one msg_ep for a given slot and make it ready to connect/accept.
 * Binds EQ/SRX/CQ/cntrs, enables, and preposts receives. The returned ep
 * is NOT yet assigned to conn->msg_eps[slot_idx] — the caller owns that.
 */
static int rxm_open_slot(struct rxm_conn *conn, struct fi_info *msg_info,
			 uint8_t slot_idx, struct fid_ep **out_ep)
{
	struct rxm_domain *domain;
	struct rxm_ep *ep;
	struct fid_ep *msg_ep;
	int ret;

	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL, "open msg ep %p slot %u\n", conn,
	       slot_idx);

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));
	assert(slot_idx < conn->num_msg_eps && conn->slots);
	ep = conn->ep;
	domain = container_of(ep->util_ep.domain, struct rxm_domain,
			      util_domain);

	ret = fi_endpoint(domain->msg_domain, msg_info, &msg_ep,
			  &conn->slots[slot_idx]);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_endpoint", ret);
		return ret;
	}

	ret = fi_ep_bind(msg_ep, &ep->msg_eq->fid, 0);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
		goto err;
	}

	if (ep->msg_srx) {
		ret = fi_ep_bind(msg_ep, &ep->msg_srx->fid, 0);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_ep_bind", ret);
			goto err;
		}
	}

	ret = rxm_bind_comp(ep, msg_ep);
	if (ret)
		goto err;

	ret = fi_enable(msg_ep);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_enable", ret);
		goto err;
	}

	/* Primary slot determines flow_ctrl availability for the conn;
	 * secondaries inherit and are enabled in lockstep with primary. */
	if (slot_idx == 0)
		conn->flow_ctrl = domain->flow_ctrl_ops->available(msg_ep);

	if (!ep->msg_srx) {
		ret = rxm_prepost_recv(ep, msg_ep);
		if (ret)
			goto err;
	}

	*out_ep = msg_ep;
	return 0;
err:
	fi_close(&msg_ep->fid);
	return ret;
}

/* Primary-open path: allocate slot arrays + open slot 0. On open failure
 * leave the arrays allocated (rxm_free_conn will release them) and just
 * ensure msg_eps[0] is NULL — the arrays are reusable across close/reopen
 * cycles and may have been inherited from a prior open by the idempotent
 * rxm_alloc_slot_arrays.
 */
static int rxm_open_primary(struct rxm_conn *conn, struct fi_info *msg_info)
{
	int ret;

	ret = rxm_alloc_slot_arrays(conn);
	if (ret)
		return ret;
	ret = rxm_open_slot(conn, msg_info, 0, &conn->msg_eps[0]);
	if (ret)
		conn->msg_eps[0] = NULL;
	return ret;
}

/* We send passive endpoint's port to the server as connection request
 * would be from a different one.
 */
static int rxm_init_connect_data(struct rxm_conn *conn,
				 union rxm_cm_data *cm_data)
{
	size_t cm_data_size = 0;
	size_t opt_size = sizeof(cm_data_size);
	int ret;

	memset(cm_data, 0, sizeof(*cm_data));
	cm_data->connect.version = RXM_CM_DATA_VERSION;
	cm_data->connect.ctrl_version = RXM_CTRL_VERSION;
	cm_data->connect.op_version = RXM_OP_VERSION;
	cm_data->connect.endianness = ofi_detect_endianness();
	cm_data->connect.eager_limit = (uint32_t) conn->ep->eager_limit;
	cm_data->connect.rx_size = (uint32_t) conn->ep->msg_info->rx_attr->size;
	cm_data->connect.flow_ctrl = conn->flow_ctrl ?
						RXM_CM_FLOW_CTRL_PEER_ON :
						RXM_CM_FLOW_CTRL_PEER_OFF;

	ret = fi_getopt(&conn->ep->msg_pep->fid, FI_OPT_ENDPOINT,
			FI_OPT_CM_DATA_SIZE, &cm_data_size, &opt_size);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_getopt", ret);
		return ret;
	}

	if (cm_data_size < sizeof(*cm_data)) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "cm data too small\n");
		return -FI_EOTHER;
	}

	cm_data->connect.port = ofi_addr_get_port(&conn->ep->addr.sa);
	cm_data->connect.client_conn_id = rxm_conn_id(conn->peer->index);
	return 0;
}

static int rxm_send_connect(struct rxm_conn *conn)
{
	union rxm_cm_data cm_data;
	struct fi_info *info;
	int ret;

	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL, "connecting %p\n", conn);
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	info = conn->ep->msg_info;
	info->dest_addrlen = conn->ep->msg_info->src_addrlen;

	free(info->dest_addr);
	info->dest_addr = mem_dup(&conn->peer->addr, info->dest_addrlen);
	if (!info->dest_addr)
		return -FI_ENOMEM;

	ret = rxm_open_primary(conn, info);
	if (ret)
		return ret;

	ret = rxm_init_connect_data(conn, &cm_data);
	if (ret)
		goto err;

	/* slot_idx == 0: the primary connect that creates the rxm_conn on
	 * the accepting side. Lazy secondaries set this field to their
	 * slot index. */
	cm_data.connect.slot_idx = 0;

	ret = fi_connect(conn->msg_eps[0], info->dest_addr, &cm_data,
			 sizeof(cm_data));
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_connect", ret);
		goto err;
	}
	conn->state = RXM_CM_CONNECTING;
	conn->ep->connecting_cnt++;
	return 0;

err:
	/* Leave the slot arrays allocated; rxm_free_conn is the single owner
	 * of that memory and a late EQ event for the just-closed ep still
	 * dereferences fid->context into conn->slots[]. */
	if (conn->msg_eps && conn->msg_eps[0]) {
		fi_close(&conn->msg_eps[0]->fid);
		conn->msg_eps[0] = NULL;
	}
	return ret;
}

/* Bring up msg_eps[slot_idx] as a secondary slot on an already-CONNECTED
 * rxm_conn. Mirrors rxm_send_connect but:
 *   - does not touch conn->state (primary stays CONNECTED);
 *   - does not bump ep->connecting_cnt (per-slot progress is tracked
 *     by conn->lazy_connecting alone);
 *   - on any failure, sets conn->failed_slot = slot_idx without tearing
 *     down the primary.
 * Preconditions checked by the caller: state==CONNECTED,
 * slot_idx==connected_msg_eps, !lazy_connecting, slot_idx<num_msg_eps,
 * and (failed_slot<0 || slot_idx<failed_slot).
 */
static int rxm_send_secondary_connect(struct rxm_conn *conn, uint8_t slot_idx)
{
	union rxm_cm_data cm_data;
	struct fid_ep *new_ep = NULL;
	struct fi_info *info;
	int ret;

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));
	assert(conn->state == RXM_CM_CONNECTED);
	assert(!conn->lazy_connecting);
	assert(slot_idx == conn->connected_msg_eps);
	assert(slot_idx < conn->num_msg_eps);
	assert(conn->failed_slot < 0 || slot_idx < (uint8_t) conn->failed_slot);

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
		"lazy secondary connect %p slot %u\n", conn, slot_idx);

	info = conn->ep->msg_info;
	info->dest_addrlen = conn->ep->msg_info->src_addrlen;
	free(info->dest_addr);
	info->dest_addr = mem_dup(&conn->peer->addr, info->dest_addrlen);
	if (!info->dest_addr) {
		conn->failed_slot = (int8_t) slot_idx;
		return -FI_ENOMEM;
	}

	ret = rxm_open_slot(conn, info, slot_idx, &new_ep);
	if (ret)
		goto fail;

	ret = rxm_init_connect_data(conn, &cm_data);
	if (ret)
		goto fail_close;

	cm_data.connect.slot_idx = slot_idx;

	ret = fi_connect(new_ep, info->dest_addr, &cm_data, sizeof(cm_data));
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_connect", ret);
		goto fail_close;
	}

	conn->msg_eps[slot_idx] = new_ep;
	conn->lazy_connecting = true;
	return 0;

fail_close:
	fi_close(&new_ep->fid);
fail:
	conn->failed_slot = (int8_t) slot_idx;
	return ret;
}

void rxm_maybe_start_secondary_connect(struct rxm_conn *conn)
{
	uint8_t next;

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	/* Fast negative paths first — this runs on the TX hot path. */
	if (conn->state != RXM_CM_CONNECTED)
		return;
	if (conn->lazy_connecting)
		return;
	if (conn->connected_msg_eps >= conn->num_msg_eps)
		return;
	if (conn->failed_slot >= 0 &&
	    conn->connected_msg_eps >= (uint8_t) conn->failed_slot)
		return;
	/* Either side may initiate a secondary connect. Simultaneous-initiate
	 * races are tie-broken by address at accept time in
	 * rxm_process_secondary_connreq, mirroring the primary
	 * simultaneous-connect handling in rxm_process_connreq. The one case
	 * excluded here is loopback/self (addr cmp == 0): the loopback conn
	 * lives on ep->loopback_list rather than in conn_idx_map, so a
	 * secondary connreq would be mis-routed.
	 */
	if (ofi_addr_cmp(&rxm_prov, &conn->peer->addr.sa,
			 &conn->ep->addr.sa) == 0)
		return;

	next = conn->connected_msg_eps;
	(void) rxm_send_secondary_connect(conn, next);
}

static int rxm_connect(struct rxm_conn *conn)
{
	int ret;

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	switch (conn->state) {
	case RXM_CM_IDLE:
		ret = rxm_send_connect(conn);
		if (ret)
			return ret;
		break;
	case RXM_CM_CONNECTING:
	case RXM_CM_ACCEPTING:
		break;
	case RXM_CM_CONNECTED:
		return 0;
	default:
		assert(0);
		conn->state = RXM_CM_IDLE;
		break;
	}

	return -FI_EAGAIN;
}

static void rxm_free_conn(struct rxm_conn *conn)
{
	struct rxm_av *av;

	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL, "free conn %p\n", conn);
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	if (conn->flags & RXM_CONN_INDEXED)
		ofi_idm_clear(&conn->ep->conn_idx_map, conn->peer->index);

	if (conn->selector && conn->selector->destroy)
		conn->selector->destroy(conn->selector);
	conn->selector = NULL;

	/* Final release of the slot arrays. rxm_close_conn deliberately leaves
	 * these alive so that late EQ events whose fid->context points into
	 * conn->slots[] remain safe; this is the only site that frees them.
	 */
	free(conn->msg_eps);
	conn->msg_eps = NULL;
	free(conn->slots);
	conn->slots = NULL;

	util_put_peer(conn->peer);
	av = container_of(conn->ep->util_ep.av, struct rxm_av, util_av);
	rxm_av_free_conn(av, conn);
}

void rxm_freeall_conns(struct rxm_ep *ep)
{
	struct rxm_conn *conn;
	struct dlist_entry *tmp;
	struct rxm_av *av;
	int i, cnt;

	if (!ep->util_ep.av)
		return;

	av = container_of(ep->util_ep.av, struct rxm_av, util_av);
	ofi_genlock_lock(&ep->util_ep.lock);

	/* We can't have more connections than the current number of
	 * possible peers.
	 */
	cnt = (int) rxm_av_max_peers(av);
	for (i = 0; i < cnt; i++) {
		conn = ofi_idm_lookup(&ep->conn_idx_map, i);
		if (!conn)
			continue;

		if (conn->state != RXM_CM_IDLE)
			rxm_close_conn(conn);
		rxm_free_conn(conn);
	}

	dlist_foreach_container_safe(&ep->loopback_list, struct rxm_conn,
				     conn, loopback_entry, tmp) {
		rxm_close_conn(conn);
		rxm_free_conn(conn);
	}

	ofi_genlock_unlock(&ep->util_ep.lock);
}

static struct rxm_conn *
rxm_alloc_conn(struct rxm_ep *ep, struct util_peer_addr *peer)
{
	struct rxm_conn *conn;
	struct rxm_av *av;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	av = container_of(ep->util_ep.av, struct rxm_av, util_av);
	conn = rxm_av_alloc_conn(av);
	if (!conn) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "rxm_av_alloc_conn", -FI_ENOMEM);
		return NULL;
	}

	conn->ep = ep;
	conn->state = RXM_CM_IDLE;
	conn->remote_index = -1;
	conn->flags = 0;
	conn->flow_ctrl = false;
	conn->peer_flow_ctrl = false;
	conn->connected_msg_eps = 0;
	conn->lazy_connecting = false;
	conn->failed_slot = -1;
	dlist_init(&conn->deferred_entry);
	dlist_init(&conn->deferred_tx_queue);
	dlist_init(&conn->deferred_sar_msgs);
	dlist_init(&conn->deferred_sar_segments);
	dlist_init(&conn->loopback_entry);

	conn->peer = peer;
	rxm_ref_peer(peer);

	conn->num_msg_eps = (uint8_t) rxm_num_msg_eps;
	if (conn->num_msg_eps > 1 &&
	    strncasecmp(ep->msg_info->fabric_attr->prov_name, "verbs",
			strlen("verbs"))) {
		FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
			"num_msg_eps > 1 not supported over %s, clamping to 1\n",
			ep->msg_info->fabric_attr->prov_name);
		conn->num_msg_eps = 1;
	}
	conn->msg_eps = NULL;
	conn->slots = NULL;

	if (conn->num_msg_eps > 1) {
		conn->selector = rxm_rr_selector_alloc();
		if (!conn->selector) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "rxm_rr_selector_alloc",
				     -FI_ENOMEM);
			util_put_peer(peer);
			rxm_av_free_conn(av, conn);
			return NULL;
		}
	} else {
		conn->selector =
			(struct rxm_qp_selector *) &rxm_selector_single_qp;
	}

	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL, "allocated conn %p\n", conn);
	return conn;
}

static struct rxm_conn *
rxm_add_conn(struct rxm_ep *ep, struct util_peer_addr *peer)
{
	struct rxm_conn *conn;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	conn = ofi_idm_lookup(&ep->conn_idx_map, peer->index);
	if (conn)
		return conn;

	conn = rxm_alloc_conn(ep, peer);
	if (!conn)
		return NULL;

	if (ofi_idm_set(&ep->conn_idx_map, peer->index, conn) < 0) {
		rxm_free_conn(conn);
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "ofi_idm_set", -FI_ENOMEM);
		return NULL;
	}

	conn->flags |= RXM_CONN_INDEXED;
	return conn;
}

/* The returned conn is only valid if the function returns success. */
ssize_t rxm_get_conn(struct rxm_ep *ep, fi_addr_t addr, struct rxm_conn **conn)
{
	struct util_peer_addr **peer;
	ssize_t ret;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	peer = ofi_av_addr_context(ep->util_ep.av, addr);
	*conn = rxm_add_conn(ep, *peer);
	if (!*conn)
		return -FI_ENOMEM;

	if ((*conn)->state == RXM_CM_CONNECTED) {
		if (!dlist_empty(&(*conn)->deferred_tx_queue)) {
			rxm_ep_do_progress(&ep->util_ep);
			if (!dlist_empty(&(*conn)->deferred_tx_queue))
				return -FI_EAGAIN;
		}
		return 0;
	}

	if ((*peer)->firewall_addr)
		return -FI_EFIREWALLADDR;

	ret = rxm_connect(*conn);

	/* If the progress function encounters an error trying to establish
	 * the connection, it may free the connection object.  This resets
	 * the connection process to restart from the beginning.
	 */
	if (ret == -FI_EAGAIN)
		rxm_conn_progress(ep);
	return ret;
}

static void rxm_set_peer_flow_ctrl(struct rxm_conn *conn, int cm_flow_ctrl_flag)
{
	switch (cm_flow_ctrl_flag) {
	case RXM_CM_FLOW_CTRL_LOCAL:
		/*
		 * For backward compatibility: the flag maps to 0 paddings in
		 * old protocol. Old protocol doesn't negotiate flow control
		 * capability. Decision is made locally.
		 */
		conn->peer_flow_ctrl = conn->flow_ctrl;
		break;

	case RXM_CM_FLOW_CTRL_PEER_ON:
		conn->peer_flow_ctrl = true;
		break;

	case RXM_CM_FLOW_CTRL_PEER_OFF:
		conn->peer_flow_ctrl = false;
		break;
	}
}

void rxm_process_connect(struct rxm_eq_cm_entry *cm_entry)
{
	struct rxm_ep_slot *slot = cm_entry->fid->context;
	struct rxm_conn *conn = slot->conn;
	uint8_t slot_idx = slot->slot_idx;
	struct rxm_domain *domain;

	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL,
	       "processing connected for handle: %p slot %u\n", conn, slot_idx);

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	if (slot_idx > 0) {
		/* Secondary slot came up. Primary stays CONNECTED; we just
		 * widen the selector's usable range by one. Growth to slot
		 * k+1 is not auto-chained: the next TX that observes a
		 * wants_spread op and sees connected < num will re-trigger
		 * rxm_maybe_start_secondary_connect.
		 */
		assert(conn->state == RXM_CM_CONNECTED);
		assert(conn->lazy_connecting);
		assert(slot_idx == conn->connected_msg_eps);
		if (conn->flow_ctrl && conn->peer_flow_ctrl) {
			domain = container_of(conn->ep->util_ep.domain,
					      struct rxm_domain, util_domain);
			domain->flow_ctrl_ops->enable(
				conn->msg_eps[slot_idx],
				conn->ep->msg_info->rx_attr->size / 2);
		}
		conn->connected_msg_eps = slot_idx + 1;
		conn->lazy_connecting = false;
		return;
	}

	/* Primary (slot 0) connect. */
	if (conn->state == RXM_CM_CONNECTING) {
		conn->remote_index = rxm_peer_index(cm_entry->data.accept.
						    server_conn_id);
		conn->remote_pid = rxm_peer_pid(cm_entry->data.accept.
						server_conn_id);
		rxm_set_peer_flow_ctrl(conn, cm_entry->data.accept.flow_ctrl);
	}

	if (conn->flow_ctrl && conn->peer_flow_ctrl) {
		domain = container_of(conn->ep->util_ep.domain,
				      struct rxm_domain, util_domain);
		domain->flow_ctrl_ops->enable(conn->msg_eps[0],
					      conn->ep->msg_info->rx_attr->size / 2);
	}

	conn->ep->connecting_cnt--;
	assert(conn->ep->connecting_cnt >= 0);
	conn->state = RXM_CM_CONNECTED;
	conn->connected_msg_eps = 1;
}

/* Tear down a secondary slot that failed. Closes the slot's msg_ep,
 * caps connected_msg_eps at failed_slot, clears lazy_connecting. The
 * primary slot (and the rxm_conn as a whole) stays alive.
 */
static void
rxm_fail_secondary_slot(struct rxm_conn *conn, uint8_t slot_idx)
{
	assert(slot_idx > 0);
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
		"secondary slot %u failed on conn %p\n", slot_idx, conn);

	if (conn->msg_eps && conn->msg_eps[slot_idx]) {
		fi_close(&conn->msg_eps[slot_idx]->fid);
		conn->msg_eps[slot_idx] = NULL;
	}
	if (conn->failed_slot < 0 || slot_idx < (uint8_t) conn->failed_slot)
		conn->failed_slot = (int8_t) slot_idx;
	if (conn->connected_msg_eps > slot_idx)
		conn->connected_msg_eps = slot_idx;
	conn->lazy_connecting = false;
}

/* For simultaneous connection requests, if the peer won the coin
 * flip (reject EALREADY), our connection request is discarded.
 */
static void
rxm_process_reject(struct rxm_conn *conn, uint8_t slot_idx,
		   struct fi_eq_err_entry *entry)
{
	union rxm_cm_data *cm_data;
	uint8_t reason;

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
	       "Processing reject for handle: %p slot %u\n", conn, slot_idx);
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	if (entry->err_data_size >= sizeof(cm_data->reject)) {
		cm_data = entry->err_data;
		if (cm_data->reject.version != RXM_CM_DATA_VERSION) {
			FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "invalid reject version\n");
			reason = RXM_REJECT_ECONNREFUSED;
		} else {
			reason = cm_data->reject.reason;
		}
	} else {
		reason = RXM_REJECT_ECONNREFUSED;
	}

	if (slot_idx > 0) {
		/* Secondary reject. EALREADY means the peer's connreq for
		 * this slot won and will fill the slot shortly — drop our
		 * pending ep but don't mark the slot dead. Any other reason
		 * retires the slot permanently. */
		if (conn->msg_eps && conn->msg_eps[slot_idx]) {
			fi_close(&conn->msg_eps[slot_idx]->fid);
			conn->msg_eps[slot_idx] = NULL;
		}
		if (reason == RXM_REJECT_EALREADY) {
			conn->lazy_connecting = false;
		} else {
			if (conn->failed_slot < 0 ||
			    slot_idx < (uint8_t) conn->failed_slot)
				conn->failed_slot = (int8_t) slot_idx;
			conn->lazy_connecting = false;
		}
		return;
	}

	switch (conn->state) {
	case RXM_CM_IDLE:
		/* Unlikely, but can occur if our request was rejected, and
		 * there was a failure trying to accept the peer's.
		 */
		break;
	case RXM_CM_CONNECTING:
		rxm_close_conn(conn);
		if (reason != RXM_REJECT_EALREADY)
			rxm_free_conn(conn);
		else
			FI_INFO(&rxm_prov, FI_LOG_EP_CTRL, "rejected, already connected\n");
		break;
	case RXM_CM_ACCEPTING:
	case RXM_CM_CONNECTED:
		/* Our request was rejected, but we accepted the peer's. */
		break;
	default:
		assert(0);
		break;
	}
}

static int
rxm_verify_connreq(struct rxm_ep *ep, union rxm_cm_data *cm_data)
{
	if (cm_data->connect.version != RXM_CM_DATA_VERSION) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "cm version mismatch");
		return -FI_EINVAL;
	}

	if (cm_data->connect.endianness != ofi_detect_endianness()) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "endianness mismatch");
		return -FI_EINVAL;
	}

	if (cm_data->connect.ctrl_version != RXM_CTRL_VERSION) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "cm ctrl_version mismatch");
		return -FI_EINVAL;
	}

	if (cm_data->connect.op_version != RXM_OP_VERSION) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "cm op_version mismatch");
		return -FI_EINVAL;
	}

	if (cm_data->connect.eager_limit != ep->eager_limit) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "eager_limit mismatch");
		return -FI_EINVAL;
	}

	return FI_SUCCESS;
}

static void
rxm_reject_connreq(struct rxm_ep *ep, struct rxm_eq_cm_entry *cm_entry,
		   uint8_t reason)
{
	union rxm_cm_data cm_data;
	int ret;

	cm_data.reject.version = RXM_CM_DATA_VERSION;
	cm_data.reject.reason = reason;

	ret = fi_reject(ep->msg_pep, cm_entry->info->handle,
			&cm_data.reject, sizeof(cm_data.reject));
	if (ret)
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_reject", ret);
}

static int
rxm_accept_connreq(struct rxm_conn *conn, struct rxm_eq_cm_entry *cm_entry,
		   uint8_t slot_idx)
{
	union rxm_cm_data cm_data;
	int ret;

	cm_data.accept.server_conn_id = rxm_conn_id(conn->peer->index);
	cm_data.accept.rx_size = (uint32_t) cm_entry->info->rx_attr->size;
	cm_data.accept.flow_ctrl = conn->flow_ctrl ? RXM_CM_FLOW_CTRL_PEER_ON :
						     RXM_CM_FLOW_CTRL_PEER_OFF;
	cm_data.accept.align_pad[0] = 0;
	cm_data.accept.align_pad[1] = 0;
	cm_data.accept.align_pad[2] = 0;

	ret = fi_accept(conn->msg_eps[slot_idx], &cm_data.accept,
			sizeof(cm_data.accept));
	if (ret)
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_accept", ret);
	return ret;
}

/* Handle an FI_CONNREQ with slot_idx > 0 (lazy secondary). Looks up the
 * existing rxm_conn to the same peer; a secondary for a nonexistent or
 * not-yet-CONNECTED conn is a protocol violation or race and gets
 * rejected. On success, opens msg_eps[slot_idx], accepts, and leaves
 * the primary state untouched.
 */
static void
rxm_process_secondary_connreq(struct rxm_ep *ep,
			      struct rxm_eq_cm_entry *cm_entry,
			      struct util_peer_addr *peer,
			      uint8_t slot_idx)
{
	struct rxm_conn *conn;
	struct fid_ep *new_ep = NULL;
	uint8_t reject_reason = RXM_REJECT_ECONNREFUSED;
	int ret;

	assert(slot_idx > 0);
	conn = ofi_idm_lookup(&ep->conn_idx_map, peer->index);
	if (!conn)
		goto reject;

	if (conn->state != RXM_CM_CONNECTED) {
		/* Peer initiated a secondary before we observed primary
		 * CONNECTED. EALREADY tells them to back off; their next
		 * TX will retry once both sides see primary up. */
		reject_reason = RXM_REJECT_EALREADY;
		goto reject;
	}
	if (slot_idx != conn->connected_msg_eps)
		goto reject;
	if (conn->failed_slot >= 0 && slot_idx >= (uint8_t) conn->failed_slot)
		goto reject;
	if (conn->lazy_connecting) {
		/* Both sides raced a secondary connect on slot k. Deterministic
		 * tie-break on address: the higher-address side keeps its
		 * in-flight connect and rejects the peer's with EALREADY; the
		 * lower-address side tears down its in-flight msg_ep and
		 * accepts the peer's. Mirrors the primary tie-break in
		 * rxm_process_connreq.
		 *
		 * cmp = ofi_addr_cmp(peer, ep); cmp < 0 means peer < ep i.e.
		 * we are higher-addr → reject. cmp == 0 is unreachable here —
		 * rxm_maybe_start_secondary_connect suppresses loopback, and
		 * the only other caller path is a remote peer.
		 *
		 * EALREADY (not ECONNREFUSED) is load-bearing: it clears the
		 * peer's lazy_connecting without marking failed_slot, so the
		 * peer can re-observe our connreq via the normal accept path.
		 */
		int cmp = ofi_addr_cmp(&rxm_prov, &peer->addr.sa,
				       &ep->addr.sa);
		if (cmp <= 0) {
			reject_reason = RXM_REJECT_EALREADY;
			goto reject;
		}
		if (conn->msg_eps[slot_idx])
			fi_close(&conn->msg_eps[slot_idx]->fid);
		conn->msg_eps[slot_idx] = NULL;
		conn->lazy_connecting = false;
	}

	ret = rxm_open_slot(conn, cm_entry->info, slot_idx, &new_ep);
	if (ret) {
		conn->failed_slot = (int8_t) slot_idx;
		goto reject;
	}
	conn->msg_eps[slot_idx] = new_ep;

	ret = rxm_accept_connreq(conn, cm_entry, slot_idx);
	if (ret) {
		fi_close(&new_ep->fid);
		conn->msg_eps[slot_idx] = NULL;
		conn->failed_slot = (int8_t) slot_idx;
		goto reject;
	}
	conn->lazy_connecting = true;
	util_put_peer(peer);
	fi_freeinfo(cm_entry->info);
	return;

reject:
	util_put_peer(peer);
	rxm_reject_connreq(ep, cm_entry, reject_reason);
	fi_freeinfo(cm_entry->info);
}

static void
rxm_process_connreq(struct rxm_ep *ep, struct rxm_eq_cm_entry *cm_entry)
{
	union ofi_sock_ip peer_addr;
	struct util_peer_addr *peer;
	struct rxm_conn *conn;
	struct rxm_av *av;
	uint8_t slot_idx;
	ssize_t ret;
	int cmp;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	if (rxm_verify_connreq(ep, &cm_entry->data))
		goto reject;

	memcpy(&peer_addr, cm_entry->info->dest_addr,
	       cm_entry->info->dest_addrlen);
	ofi_addr_set_port(&peer_addr.sa, cm_entry->data.connect.port);

	av = container_of(ep->util_ep.av, struct rxm_av, util_av);
	peer = util_get_peer(av, &peer_addr, 0);
	if (!peer) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "util_get_peer", -FI_ENOMEM);
		goto reject;
	}

	slot_idx = cm_entry->data.connect.slot_idx;
	if (slot_idx > 0) {
		rxm_process_secondary_connreq(ep, cm_entry, peer, slot_idx);
		return;
	}

	conn = rxm_add_conn(ep, peer);
	if (!conn)
		goto remove;

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL, "connreq for %p\n", conn);
	switch (conn->state) {
	case RXM_CM_IDLE:
		break;
	case RXM_CM_CONNECTING:
		/* simultaneous connections */
		cmp = ofi_addr_cmp(&rxm_prov, &peer_addr.sa, &ep->addr.sa);
		if (cmp < 0) {
			/* let our request finish */
			FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
				"simultaneous, reject peer %p\n", conn);
			rxm_reject_connreq(ep, cm_entry,
					   RXM_REJECT_EALREADY);
			goto put;
		} else if (cmp > 0) {
			/* accept peer's request */
			FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
				"simultaneous, accept peer %p\n", conn);
			rxm_close_conn(conn);
		} else {
			/* connecting to ourself, create loopback conn */
			FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
				"loopback conn %p\n", conn);
			conn = rxm_alloc_conn(ep, peer);
			if (!conn)
				goto remove;

			dlist_insert_tail(&conn->loopback_entry, &ep->loopback_list);
			break;
		}
		break;
	case RXM_CM_ACCEPTING:
	case RXM_CM_CONNECTED:
		if (conn->remote_pid &&
		    (conn->remote_pid == rxm_peer_pid(cm_entry->data.connect.
		    				      client_conn_id))) {
			FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
				"simultaneous, reject peer\n");
			rxm_reject_connreq(ep, cm_entry,
					   RXM_REJECT_EALREADY);
			goto put;
		} else {
			FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
				"old connection exists, replacing %p\n", conn);
			rxm_close_conn(conn);
		}
		break;
	default:
		assert(0);
		break;
	}

	conn->remote_pid = rxm_peer_pid(cm_entry->data.connect.client_conn_id);
	conn->remote_index = rxm_peer_index(cm_entry->data.connect.client_conn_id);
	ret = rxm_open_primary(conn, cm_entry->info);
	if (ret)
		goto free;

	rxm_set_peer_flow_ctrl(conn, cm_entry->data.connect.flow_ctrl);

	ret = rxm_accept_connreq(conn, cm_entry, 0);
	if (ret)
		goto close;

	conn->state = RXM_CM_ACCEPTING;
	conn->ep->connecting_cnt++;
put:
	util_put_peer(peer);
	fi_freeinfo(cm_entry->info);
	return;

close:
	rxm_close_conn(conn);
free:
	rxm_free_conn(conn);
remove:
	util_put_peer(peer);
reject:
	rxm_reject_connreq(ep, cm_entry, RXM_REJECT_ECONNREFUSED);
	fi_freeinfo(cm_entry->info);
}

static void rxm_process_shutdown_slot(struct rxm_conn *conn, uint8_t slot_idx)
{
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
		"shutdown conn %p slot %u (state %d)\n", conn, slot_idx,
		conn->state);

	if (slot_idx > 0) {
		rxm_fail_secondary_slot(conn, slot_idx);
		return;
	}

	switch (conn->state) {
	case RXM_CM_IDLE:
		break;
	case RXM_CM_CONNECTING:
	case RXM_CM_ACCEPTING:
	case RXM_CM_CONNECTED:
		rxm_close_conn(conn);
		rxm_free_conn(conn);
		break;
	default:
		break;
	}
}

static void rxm_handle_error(struct rxm_ep *ep)
{
	struct fi_eq_err_entry entry = {0};
	ssize_t ret;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	ret = fi_eq_readerr(ep->msg_eq, &entry, 0);
	if (ret != sizeof(entry)) {
		if (ret != -FI_EAGAIN)
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_eq_readerr", ret);
		return;
	}

	if (entry.err == FI_ECONNREFUSED) {
		FI_LOG_SPARSE(&rxm_prov, FI_LOG_WARN, FI_LOG_CQ,
			"fi_eq_readerr: err: %s (%d), prov_err: %s (%d)\n",
			fi_strerror(entry.err), entry.err,
			fi_eq_strerror(ep->msg_eq, entry.prov_errno,
					entry.err_data, NULL, 0),
			entry.prov_errno);
	} else {
		FI_WARN(&rxm_prov, FI_LOG_CQ,
			"fi_eq_readerr: err: %s (%d), prov_err: %s (%d)\n",
			fi_strerror(entry.err), entry.err,
			fi_eq_strerror(ep->msg_eq, entry.prov_errno,
					entry.err_data, NULL, 0),
			entry.prov_errno);
	}

	if (!entry.fid || entry.fid->fclass != FI_CLASS_EP)
		return;

	struct rxm_ep_slot *slot = entry.fid->context;
	if (entry.err == ECONNREFUSED) {
		rxm_process_reject(slot->conn, slot->slot_idx, &entry);
	} else if (slot->slot_idx > 0) {
		rxm_fail_secondary_slot(slot->conn, slot->slot_idx);
	} else {
		rxm_process_shutdown_slot(slot->conn, 0);
	}
}

static void
rxm_handle_event(struct rxm_ep *ep, uint32_t event,
		 struct rxm_eq_cm_entry *cm_entry, size_t len)
{
	assert(ofi_genlock_held(&ep->util_ep.lock));
	switch (event) {
	case FI_NOTIFY:
		break;
	case FI_CONNREQ:
		rxm_process_connreq(ep, cm_entry);
		break;
	case FI_CONNECTED:
		rxm_process_connect(cm_entry);
		break;
	case FI_SHUTDOWN: {
		struct rxm_ep_slot *slot = cm_entry->fid->context;
		rxm_process_shutdown_slot(slot->conn, slot->slot_idx);
		break;
	}
	default:
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL,
			"Unknown event: %u\n", event);
		break;
	}
}

void rxm_conn_progress(struct rxm_ep *ep)
{
	struct rxm_eq_cm_entry cm_entry;
	uint32_t event;
	ssize_t ret;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	do {
		ret = fi_eq_read(ep->msg_eq, &event, &cm_entry,
				 sizeof(cm_entry), 0);
		if (ret > 0) {
			rxm_handle_event(ep, event, &cm_entry, ret);
		} else if (ret == -FI_EAVAIL) {
			rxm_handle_error(ep);
			ret = 1;
		}
	} while (ret > 0);
}

void rxm_stop_listen(struct rxm_ep *ep)
{
	struct fi_eq_entry entry = {0};
	ssize_t size_ret;
	int ret;

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL, "stopping CM thread\n");
	if (!ep->cm_thread)
		return;

	ofi_genlock_lock(&ep->util_ep.lock);
	ep->do_progress = false;
	ofi_genlock_unlock(&ep->util_ep.lock);

	size_ret = fi_eq_write(ep->msg_eq, FI_NOTIFY, &entry, sizeof(entry), 0);
	if (size_ret != sizeof(entry)) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL, "Unable to signal\n");
		return;
	}

	ret = pthread_join(ep->cm_thread, NULL);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "pthread_join", -ret);
	}
}

static void rxm_flush_msg_cq(struct rxm_ep *ep)
{
	struct fi_cq_data_entry comp;
	ssize_t ret;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	do {
		ret = fi_cq_read(ep->msg_cq, &comp, 1);
		if (ret > 0) {
			ret = ep->handle_comp(ep, &comp);
			if (ret) {
				rxm_cq_write_error_all(ep, (int) ret);
			} else {
				ret = 1;
			}
		} else if (ret == -FI_EAVAIL) {
			ep->handle_comp_error(ep);
			ret = 1;
		} else if (ret < 0 && ret != -FI_EAGAIN) {
			rxm_cq_write_error_all(ep, (int) ret);
		}
	} while (ret > 0);
}

static void *rxm_cm_data_progress(void *arg)
{
	struct rxm_ep *ep = container_of(arg, struct rxm_ep, util_ep);
	struct rxm_fabric *fabric;
	struct fid *fids[2] = {
		&ep->msg_eq->fid,
		&ep->msg_cq->fid,
	};
	struct pollfd fds[2] = {
		{.events = POLLIN},
		{.events = POLLIN},
	};
	int ret;

	fabric = container_of(ep->util_ep.domain->fabric,
			      struct rxm_fabric, util_fabric);
	ret = fi_control(&ep->msg_eq->fid, FI_GETWAIT, &fds[0].fd);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_control", ret);
		return NULL;
	}

	ret = fi_control(&ep->msg_cq->fid, FI_GETWAIT, &fds[1].fd);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_control", ret);
		return NULL;
	}

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL, "Starting auto-progress thread\n");
	ofi_genlock_lock(&ep->util_ep.lock);
	while (ep->do_progress) {
		ofi_genlock_unlock(&ep->util_ep.lock);
		ret = fi_trywait(fabric->msg_fabric, fids, 2);

		if (!ret) {
			ret = poll(fds, 2, -1);
			if (ret == -1) {
				RXM_WARN_ERR(FI_LOG_EP_CTRL, "poll", -errno);
			}
		}
		ep->util_ep.progress(&ep->util_ep);
		ofi_genlock_lock(&ep->util_ep.lock);
		rxm_conn_progress(ep);
	}
	ofi_genlock_unlock(&ep->util_ep.lock);

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL, "Stopping auto progress thread\n");
	return NULL;
}

int rxm_start_listen(struct rxm_ep *ep)
{
	size_t addr_len;
	int ret;

	ret = fi_listen(ep->msg_pep);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_listen", ret);
		return ret;
	}

	addr_len = sizeof(ep->addr);
	ret = fi_getname(&ep->msg_pep->fid, &ep->addr, &addr_len);
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_getname", ret);
		return ret;
	}

	/* Update src_addr that will be used for active endpoints.
	 * Zero out the port to avoid address conflicts, as we will
	 * create multiple msg ep's for a single rdm ep.
	 */
	if (ep->msg_info->src_addr) {
		free(ep->msg_info->src_addr);
		ep->msg_info->src_addr = NULL;
		ep->msg_info->src_addrlen = 0;
	}

	ep->msg_info->src_addr = mem_dup(&ep->addr, addr_len);
	if (!ep->msg_info->src_addr)
		return -FI_ENOMEM;

	ep->msg_info->src_addrlen = addr_len;
	ofi_addr_set_port(ep->msg_info->src_addr, 0);

	if (ep->util_ep.domain->data_progress == FI_PROGRESS_AUTO ||
	    force_auto_progress) {
		assert(ep->util_ep.domain->threading == FI_THREAD_SAFE);
		ep->do_progress = true;
		ret = pthread_create(&ep->cm_thread, 0, rxm_cm_data_progress,
				     ep);
		if (ret) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "pthread_create", -ret);
			return -ret;
		}
	}
	return 0;
}

void rxm_av_remove_handler(struct util_ep *util_ep, struct util_peer_addr *peer)
{
	struct rxm_ep *ep;
	struct rxm_conn *conn;

	ep = container_of(util_ep, struct rxm_ep, util_ep);
	ofi_genlock_lock(&ep->util_ep.lock);
	conn = ofi_idm_lookup(&ep->conn_idx_map, peer->index);
	if (conn) {
		rxm_close_conn(conn);
		rxm_free_conn(conn);
	}
	ofi_genlock_unlock(&ep->util_ep.lock);
}
