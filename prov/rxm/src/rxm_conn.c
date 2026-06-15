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
#include <unistd.h>
#include <sys/types.h>

#include <ofi.h>
#include <ofi_util.h>
#include "rxm.h"

#define RXM_DBG(fmt, ...) do { \
	if (getenv("RXM_DBG")) \
		fprintf(stderr, "[RXM_DBG pid=%d] " fmt "\n", \
			(int) getpid(), ##__VA_ARGS__); \
} while (0)

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
	if (conn->msg_eps) {
		for (uint8_t i = 0; i < conn->num_msg_eps; i++) {
			if (conn->msg_eps[i])
				fi_close(&conn->msg_eps[i]->fid);
		}
	}
	rxm_flush_msg_cq(conn->ep);
	dlist_remove_init(&conn->loopback_entry);
	free(conn->msg_eps);
	conn->msg_eps = NULL;

	for (uint8_t i = 0; i < conn->num_msg_eps; i++) {
		if (conn->states[i] == RXM_CM_CONNECTING ||
		    conn->states[i] == RXM_CM_ACCEPTING)
			conn->ep->connecting_cnt--;
		conn->states[i] = RXM_CM_IDLE;
	}
	assert(conn->ep->connecting_cnt >= 0);
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

static int rxm_open_msg_ep(struct rxm_conn *conn, uint8_t idx,
			   struct fi_info *msg_info)
{
	struct rxm_domain *domain;
	struct rxm_ep *ep;
	struct fid_ep *msg_ep;
	int ret;

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));
	ep = conn->ep;
	domain = container_of(ep->util_ep.domain, struct rxm_domain,
			      util_domain);
	RXM_DBG("open_msg_ep ENTER idx=%u msg_info=%p handle=%p src_addr=%p dest_addr=%p src_addrlen=%zu dest_addrlen=%zu",
		idx, (void*)msg_info, (void*)msg_info->handle,
		(void*)msg_info->src_addr, (void*)msg_info->dest_addr,
		msg_info->src_addrlen, msg_info->dest_addrlen);
	ret = fi_endpoint(domain->msg_domain, msg_info, &msg_ep, conn);
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

	if (!ep->msg_srx) {
		RXM_DBG("open_msg_ep idx=%u prepost_recv (no SRX) msg_ep=%p",
			idx, (void*)msg_ep);
		ret = rxm_prepost_recv(ep, msg_ep);
		if (ret)
			goto err;
	} else {
		RXM_DBG("open_msg_ep idx=%u SRX bound (no per-ep prepost) msg_ep=%p msg_srx=%p",
			idx, (void*)msg_ep, (void*)ep->msg_srx);
	}

	conn->msg_eps[idx] = msg_ep;
	return 0;
err:
	fi_close(&msg_ep->fid);
	return ret;
}

static int rxm_open_conn(struct rxm_conn *conn, struct fi_info *msg_info)
{
	struct rxm_domain *domain;
	int ret;

	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL, "open msg ep %p\n", conn);
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	conn->msg_eps = calloc(conn->num_msg_eps, sizeof(*conn->msg_eps));
	if (!conn->msg_eps) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "calloc msg_eps", -FI_ENOMEM);
		return -FI_ENOMEM;
	}

	ret = rxm_open_msg_ep(conn, 0, msg_info);
	if (ret) {
		free(conn->msg_eps);
		conn->msg_eps = NULL;
		return ret;
	}

	domain = container_of(conn->ep->util_ep.domain, struct rxm_domain,
			      util_domain);
	conn->flow_ctrl = domain->flow_ctrl_ops->available(conn->msg_eps[0]);

	return 0;
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

	ret = rxm_open_conn(conn, info);
	if (ret)
		return ret;

	ret = rxm_init_connect_data(conn, &cm_data);
	if (ret)
		goto err;

	{
		const struct sockaddr_in *sin_dst = (const struct sockaddr_in *) info->dest_addr;
		const struct sockaddr_in *sin_src = (const struct sockaddr_in *) info->src_addr;
		RXM_DBG("send_connect EP0 calling fi_connect conn=%p src_addrlen=%zu dest_addrlen=%zu src_fam=%u src_port=%u dst_fam=%u dst_port=%u dst_ip=0x%08x",
			(void*)conn,
			info->src_addrlen, info->dest_addrlen,
			sin_src ? sin_src->sin_family : 0,
			sin_src ? ntohs(sin_src->sin_port) : 0,
			sin_dst ? sin_dst->sin_family : 0,
			sin_dst ? ntohs(sin_dst->sin_port) : 0,
			sin_dst ? ntohl(sin_dst->sin_addr.s_addr) : 0);
	}
	ret = fi_connect(conn->msg_eps[0], info->dest_addr, &cm_data,
			 sizeof(cm_data));
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_connect", ret);
		goto err;
	}
	conn->states[0] = RXM_CM_CONNECTING;
	conn->ep->connecting_cnt++;
	RXM_DBG("send_connect EP0 sent conn=%p msg_ep=%p num_eps=%u",
		(void*)conn, (void*)conn->msg_eps[0], conn->num_msg_eps);
	return 0;

err:
	if (conn->msg_eps && conn->msg_eps[0])
		fi_close(&conn->msg_eps[0]->fid);
	free(conn->msg_eps);
	conn->msg_eps = NULL;
	conn->states[0] = RXM_CM_IDLE;
	return ret;
}

int rxm_lazy_connect(struct rxm_conn *conn, uint8_t idx)
{
	union rxm_cm_data cm_data;
	struct fi_info *info;
	int ret;

	assert(idx > 0 && idx < conn->num_msg_eps);
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));
	RXM_DBG("lazy_connect ENTER conn=%p idx=%u num_eps=%u state[idx]=%d",
		(void*)conn, idx, conn->num_msg_eps, conn->states[idx]);

	/* Per-ep deep-copied info: ep 0's connect mutated msg_info->dest_addr
	 * and the verbs core may stash residual CM state on the shared struct
	 * (handle, src_addr ownership) that confuses a second active connect
	 * to the same peer. Each lazy connect starts from a fresh dup. */
	info = fi_dupinfo(conn->ep->msg_info);
	if (!info) {
		ret = -FI_ENOMEM;
		goto truncate;
	}

	/* The active side must not carry a passive-ep / connreq handle into
	 * fi_endpoint, or vrb_open_ep takes the wrong creation branch. */
	info->handle = NULL;

	free(info->dest_addr);
	info->dest_addrlen = conn->ep->msg_info->src_addrlen;
	info->dest_addr = mem_dup(&conn->peer->addr, info->dest_addrlen);
	if (!info->dest_addr) {
		ret = -FI_ENOMEM;
		goto free_info;
	}

	ret = rxm_open_msg_ep(conn, idx, info);
	if (ret)
		goto free_info;

	ret = rxm_init_connect_data(conn, &cm_data);
	if (ret)
		goto close_ep;
	cm_data.connect.ep_idx = idx;

	{
		const struct sockaddr_in *sin_dst = (const struct sockaddr_in *) info->dest_addr;
		const struct sockaddr_in *sin_src = (const struct sockaddr_in *) info->src_addr;
		RXM_DBG("lazy_connect calling fi_connect conn=%p idx=%u msg_ep=%p src_addrlen=%zu dest_addrlen=%zu src_fam=%u src_port=%u dst_fam=%u dst_port=%u dst_ip=0x%08x",
			(void*)conn, idx, (void*)conn->msg_eps[idx],
			info->src_addrlen, info->dest_addrlen,
			sin_src ? sin_src->sin_family : 0,
			sin_src ? ntohs(sin_src->sin_port) : 0,
			sin_dst ? sin_dst->sin_family : 0,
			sin_dst ? ntohs(sin_dst->sin_port) : 0,
			sin_dst ? ntohl(sin_dst->sin_addr.s_addr) : 0);
	}
	ret = fi_connect(conn->msg_eps[idx], info->dest_addr, &cm_data,
			 sizeof(cm_data));
	if (ret) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_connect (lazy)", ret);
		RXM_DBG("lazy_connect fi_connect FAILED conn=%p idx=%u ret=%d",
			(void*)conn, idx, ret);
		goto close_ep;
	}

	conn->states[idx] = RXM_CM_CONNECTING;
	conn->ep->connecting_cnt++;
	RXM_DBG("lazy_connect FI_CONNECT_SENT conn=%p idx=%u msg_ep=%p ep_idx_in_cm_data=%u connecting_cnt=%d",
		(void*)conn, idx, (void*)conn->msg_eps[idx],
		cm_data.connect.ep_idx, conn->ep->connecting_cnt);
	fi_freeinfo(info);
	return 0;

close_ep:
	fi_close(&conn->msg_eps[idx]->fid);
	conn->msg_eps[idx] = NULL;
free_info:
	fi_freeinfo(info);
truncate:
	FI_WARN(&rxm_prov, FI_LOG_EP_CTRL,
		"lazy connect of ep %u failed (%s); truncating conn %p to %u eps\n",
		idx, fi_strerror(-ret), conn, idx);
	RXM_DBG("lazy_connect TRUNCATE conn=%p idx=%u ret=%d",
		(void*)conn, idx, ret);
	conn->num_msg_eps = idx;
	return ret;
}

static int rxm_connect(struct rxm_conn *conn)
{
	int ret;

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	switch (conn->states[0]) {
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
		conn->states[0] = RXM_CM_IDLE;
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

	free(conn->states);
	conn->states = NULL;

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

		if (conn->states[0] != RXM_CM_IDLE)
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
	conn->remote_index = -1;
	conn->flags = 0;
	conn->flow_ctrl = false;
	conn->peer_flow_ctrl = false;
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

	conn->states = calloc(conn->num_msg_eps, sizeof(*conn->states));
	if (!conn->states) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "calloc states", -FI_ENOMEM);
		util_put_peer(peer);
		rxm_av_free_conn(av, conn);
		return NULL;
	}

	if (conn->num_msg_eps > 1) {
		conn->selector = rxm_rr_selector_alloc();
		if (!conn->selector) {
			RXM_WARN_ERR(FI_LOG_EP_CTRL, "rxm_rr_selector_alloc",
				     -FI_ENOMEM);
			free(conn->states);
			util_put_peer(peer);
			rxm_av_free_conn(av, conn);
			return NULL;
		}
	} else {
		conn->selector =
			(struct rxm_ep_selector *) &rxm_selector_single_ep;
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

	if ((*conn)->states[0] == RXM_CM_CONNECTED) {
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
	struct rxm_conn *conn;
	struct rxm_domain *domain;

	int idx;

	conn = cm_entry->fid->context;
	idx = rxm_get_ep_idx(conn, cm_entry->fid);
	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL,
	       "processing connected for handle: %p ep %d\n", conn, idx);
	RXM_DBG("process_connect FI_CONNECTED conn=%p idx=%d num_eps=%u prev_state=%d",
		(void*)conn, idx, conn->num_msg_eps,
		(idx >= 0 && idx < conn->num_msg_eps) ? conn->states[idx] : -99);

	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	if (idx == 0 && conn->states[0] == RXM_CM_CONNECTING) {
		conn->remote_index = rxm_peer_index(cm_entry->data.accept.
						    server_conn_id);
		conn->remote_pid = rxm_peer_pid(cm_entry->data.accept.
						server_conn_id);
		rxm_set_peer_flow_ctrl(conn, cm_entry->data.accept.flow_ctrl);
	}

	if (conn->flow_ctrl && conn->peer_flow_ctrl) {
		domain = container_of(conn->ep->util_ep.domain,
				      struct rxm_domain, util_domain);
		domain->flow_ctrl_ops->enable(conn->msg_eps[idx],
					      conn->ep->msg_info->rx_attr->size / 2);
	}

	conn->ep->connecting_cnt--;
	assert(conn->ep->connecting_cnt >= 0);
	conn->states[idx] = RXM_CM_CONNECTED;
	RXM_DBG("process_connect CONNECTED conn=%p idx=%d connecting_cnt=%d",
		(void*)conn, idx, conn->ep->connecting_cnt);
}

/* Sibling ep (idx > 0) failure that arrived while the ep was still
 * mid-handshake (states[idx] != CONNECTED). Close just that ep and
 * truncate num_msg_eps to drop it and any higher eps from rotation.
 * The conn stays alive on ep 0 and any lower CONNECTED siblings.
 *
 * Caller must already have decided this is a sibling-ep pre-CONNECTED
 * failure; we don't dispatch on idx here. */
static void
rxm_drop_sibling_ep(struct rxm_conn *conn, uint8_t idx)
{
	assert(idx > 0 && idx < conn->num_msg_eps);
	assert(conn->states[idx] != RXM_CM_CONNECTED);

	if (conn->msg_eps[idx]) {
		fi_close(&conn->msg_eps[idx]->fid);
		conn->msg_eps[idx] = NULL;
	}
	if (conn->states[idx] == RXM_CM_CONNECTING ||
	    conn->states[idx] == RXM_CM_ACCEPTING)
		conn->ep->connecting_cnt--;
	conn->states[idx] = RXM_CM_IDLE;
	conn->num_msg_eps = idx;
}

/* For simultaneous connection requests, if the peer won the coin
 * flip (reject EALREADY), our connection request is discarded.
 */
static void
rxm_process_reject(struct rxm_conn *conn, uint8_t idx,
		   struct fi_eq_err_entry *entry)
{
	union rxm_cm_data *cm_data;
	uint8_t reason;

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
	       "Processing reject for handle: %p ep %u\n", conn, idx);
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

	if (idx > 0) {
		/* Sibling ep rejected during lazy open — drop it without
		 * tearing down the conn. Ep 0 is unaffected. */
		rxm_drop_sibling_ep(conn, idx);
		return;
	}

	switch (conn->states[0]) {
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
rxm_accept_connreq(struct rxm_conn *conn, uint8_t idx,
		   struct rxm_eq_cm_entry *cm_entry)
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

	ret = fi_accept(conn->msg_eps[idx], &cm_data.accept,
			sizeof(cm_data.accept));
	if (ret)
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "fi_accept", ret);
	return ret;
}

/* Acceptor side of a sibling ep's lazy open: a connreq arrives with
 * ep_idx > 0 for an already-CONNECTED conn. Open the ep, fi_accept,
 * and let rxm_process_connect flip states[idx] = CONNECTED later. No
 * remote_index/pid update — ep 0 already established those.
 *
 * Validation rules: the conn must already exist (we do not create one
 * for a sibling connreq), ep 0 must be CONNECTED (peer must have its
 * own ep 0 up before lazily opening siblings), idx must be in range,
 * and the ep must still be IDLE. Any failure rejects the connreq —
 * the conn itself stays alive on ep 0. */
static int
rxm_process_sibling_connreq(struct rxm_ep *ep,
			    struct rxm_eq_cm_entry *cm_entry, uint8_t idx)
{
	union ofi_sock_ip peer_addr;
	struct util_peer_addr *peer;
	struct rxm_conn *conn;
	struct rxm_av *av;
	int ret;

	RXM_DBG("sibling_connreq ENTER idx=%u", idx);
	memcpy(&peer_addr, cm_entry->info->dest_addr,
	       cm_entry->info->dest_addrlen);
	ofi_addr_set_port(&peer_addr.sa, cm_entry->data.connect.port);

	av = container_of(ep->util_ep.av, struct rxm_av, util_av);
	peer = util_get_peer(av, &peer_addr, 0);
	if (!peer) {
		RXM_DBG("sibling_connreq idx=%u util_get_peer FAILED", idx);
		return -FI_ENOMEM;
	}

	conn = ofi_idm_lookup(&ep->conn_idx_map, peer->index);
	if (!conn || conn->states[0] != RXM_CM_CONNECTED ||
	    idx >= conn->num_msg_eps ||
	    conn->states[idx] != RXM_CM_IDLE) {
		FI_WARN(&rxm_prov, FI_LOG_EP_CTRL,
			"sibling connreq idx=%u rejected: conn=%p states[0]=%d num_eps=%u\n",
			idx, conn, conn ? conn->states[0] : -1,
			conn ? conn->num_msg_eps : 0);
		RXM_DBG("sibling_connreq REJECT idx=%u conn=%p states[0]=%d num_eps=%u state[idx]=%d",
			idx, (void*)conn, conn ? conn->states[0] : -1,
			conn ? conn->num_msg_eps : 0,
			conn && idx < conn->num_msg_eps ? conn->states[idx] : -99);
		ret = -FI_ENOENT;
		goto put;
	}

	ret = rxm_open_msg_ep(conn, idx, cm_entry->info);
	if (ret) {
		RXM_DBG("sibling_connreq idx=%u open_msg_ep FAILED ret=%d", idx, (int) ret);
		goto put;
	}

	ret = rxm_accept_connreq(conn, idx, cm_entry);
	if (ret) {
		RXM_DBG("sibling_connreq idx=%u fi_accept FAILED ret=%d", idx, (int) ret);
		fi_close(&conn->msg_eps[idx]->fid);
		conn->msg_eps[idx] = NULL;
		goto put;
	}

	conn->states[idx] = RXM_CM_ACCEPTING;
	conn->ep->connecting_cnt++;
	RXM_DBG("sibling_connreq ACCEPTING conn=%p idx=%u msg_ep=%p connecting_cnt=%d",
		(void*)conn, idx, (void*)conn->msg_eps[idx],
		conn->ep->connecting_cnt);
put:
	util_put_peer(peer);
	return ret;
}

static void
rxm_process_connreq(struct rxm_ep *ep, struct rxm_eq_cm_entry *cm_entry)
{
	union ofi_sock_ip peer_addr;
	struct util_peer_addr *peer;
	struct rxm_conn *conn;
	struct rxm_av *av;
	ssize_t ret;
	int cmp;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	RXM_DBG("process_connreq ENTER ep_idx_in_data=%u",
		cm_entry->data.connect.ep_idx);
	if (rxm_verify_connreq(ep, &cm_entry->data)) {
		RXM_DBG("process_connreq verify FAILED ep_idx=%u",
			cm_entry->data.connect.ep_idx);
		goto reject;
	}

	if (cm_entry->data.connect.ep_idx > 0) {
		if (rxm_process_sibling_connreq(ep, cm_entry,
						cm_entry->data.connect.ep_idx)) {
			RXM_DBG("process_connreq sibling FAILED idx=%u -> reject",
				cm_entry->data.connect.ep_idx);
			goto reject;
		}
		fi_freeinfo(cm_entry->info);
		return;
	}

	memcpy(&peer_addr, cm_entry->info->dest_addr,
	       cm_entry->info->dest_addrlen);
	ofi_addr_set_port(&peer_addr.sa, cm_entry->data.connect.port);

	av = container_of(ep->util_ep.av, struct rxm_av, util_av);
	peer = util_get_peer(av, &peer_addr, 0);
	if (!peer) {
		RXM_WARN_ERR(FI_LOG_EP_CTRL, "util_get_peer", -FI_ENOMEM);
		goto reject;
	}

	conn = rxm_add_conn(ep, peer);
	if (!conn)
		goto remove;

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL, "connreq for %p\n", conn);
	switch (conn->states[0]) {
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
	ret = rxm_open_conn(conn, cm_entry->info);
	if (ret)
		goto free;

	rxm_set_peer_flow_ctrl(conn, cm_entry->data.connect.flow_ctrl);

	ret = rxm_accept_connreq(conn, 0, cm_entry);
	if (ret)
		goto close;

	conn->states[0] = RXM_CM_ACCEPTING;
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

void rxm_process_shutdown(struct rxm_conn *conn, uint8_t idx)
{
	assert(ofi_genlock_held(&conn->ep->util_ep.lock));

	FI_INFO(&rxm_prov, FI_LOG_EP_CTRL,
		"shutdown conn %p ep %u (state %d)\n",
		conn, idx, conn->states[idx]);

	if (idx > 0 && conn->states[idx] != RXM_CM_CONNECTED) {
		/* Sibling ep died while still handshaking — case A:
		 * truncate, keep conn alive. */
		rxm_drop_sibling_ep(conn, idx);
		return;
	}

	/* Ep 0, or a CONNECTED sibling — case B: tear down the whole
	 * conn. A peer dropping a CONNECTED sibling mid-flight typically
	 * signals a real problem worth resetting. */
	switch (conn->states[0]) {
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

	{
		struct rxm_conn *conn = entry.fid->context;
		int idx = rxm_get_ep_idx(conn, entry.fid);

		RXM_DBG("handle_error conn=%p idx=%d err=%d (%s)",
			(void*)conn, idx, entry.err, fi_strerror(entry.err));

		if (entry.err == ECONNREFUSED)
			rxm_process_reject(conn, (uint8_t) idx, &entry);
		else
			rxm_process_shutdown(conn, (uint8_t) idx);
	}
}

static void
rxm_handle_event(struct rxm_ep *ep, uint32_t event,
		 struct rxm_eq_cm_entry *cm_entry, size_t len)
{
	assert(ofi_genlock_held(&ep->util_ep.lock));
	RXM_DBG("handle_event event=%u (CONNREQ=%u CONNECTED=%u SHUTDOWN=%u NOTIFY=%u)",
		event, FI_CONNREQ, FI_CONNECTED, FI_SHUTDOWN, FI_NOTIFY);
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
		struct rxm_conn *conn = cm_entry->fid->context;
		int idx = rxm_get_ep_idx(conn, cm_entry->fid);

		RXM_DBG("handle_event SHUTDOWN conn=%p idx=%d", (void*)conn, idx);
		rxm_process_shutdown(conn, (uint8_t) idx);
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
	static __thread unsigned long progress_calls;
	static __thread unsigned long progress_calls_connecting;

	assert(ofi_genlock_held(&ep->util_ep.lock));
	if ((++progress_calls % 100000) == 1)
		RXM_DBG("conn_progress tick #%lu connecting_cnt=%d",
			progress_calls, ep->connecting_cnt);
	if (ep->connecting_cnt > 0 &&
	    (++progress_calls_connecting % 1000) == 1)
		RXM_DBG("conn_progress tick_connecting #%lu connecting_cnt=%d",
			progress_calls_connecting, ep->connecting_cnt);
	do {
		ret = fi_eq_read(ep->msg_eq, &event, &cm_entry,
				 sizeof(cm_entry), 0);
		if (ret > 0) {
			RXM_DBG("conn_progress fi_eq_read got event=%u connecting_cnt=%d",
				event, ep->connecting_cnt);
			rxm_handle_event(ep, event, &cm_entry, ret);
		} else if (ret == -FI_EAVAIL) {
			RXM_DBG("conn_progress fi_eq_read EAVAIL");
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
