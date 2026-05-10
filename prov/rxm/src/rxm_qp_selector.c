#include <stdlib.h>

#include "rxm.h"
#include "rxm_qp_selector.h"

static struct rxm_selector_result
rxm_single_qp_select(struct rxm_conn *conn,
		     const struct rxm_selector_ctx *ctx)
{
	OFI_UNUSED(conn);
	OFI_UNUSED(ctx);
	return (struct rxm_selector_result){ .idx = 0, .wants_spread = false };
}

const struct rxm_qp_selector rxm_selector_single_qp = {
	.select = rxm_single_qp_select,
	.destroy = NULL,
};

static uint8_t rxm_rr_next(struct rxm_rr_selector *rr, struct rxm_conn *conn)
{
	/* Range over currently-connected slots only; the fraction
	 * [1, connected) skips slot 0 so spread ops land on secondaries
	 * when at least one is up, and collapse to 0 otherwise. */
	if (conn->connected_msg_eps <= 1)
		return 0;
	return 1 + (rr->rr_counter++ % (conn->connected_msg_eps - 1));
}

static uint8_t rxm_clamp_slot(struct rxm_conn *conn, uint8_t idx)
{
	/* A SAR pin may name a slot that is no longer usable (failed_slot
	 * moved the cap below it). Demote to 0 so the remaining segments
	 * stay in-order on the primary. */
	return idx < conn->connected_msg_eps ? idx : 0;
}

static struct rxm_selector_result
rxm_rr_select(struct rxm_conn *conn, const struct rxm_selector_ctx *ctx)
{
	struct rxm_rr_selector *rr =
		container_of(conn->selector, struct rxm_rr_selector, base);
	void *slot;
	uint8_t idx;

	switch (ctx->op) {
	case RXM_OP_RMA:
	case RXM_OP_RNDV_RMA:
		return (struct rxm_selector_result){
			.idx = rxm_rr_next(rr, conn), .wants_spread = true };

	case RXM_OP_SAR_MIDDLE:
		slot = ofi_idm_lookup(&rr->sar_pins, (int) ctx->msg_id);
		if (slot)
			return (struct rxm_selector_result){
				.idx = rxm_clamp_slot(conn,
					(uint8_t)((uintptr_t) slot - 1)),
				.wants_spread = true };
		idx = rxm_rr_next(rr, conn);
		/* On map-grow OOM, fall back to qp 0 for the rest of this
		 * SAR message. Subsequent segments will also miss the
		 * lookup and land on 0, preserving in-order delivery. */
		if (ofi_idm_set(&rr->sar_pins, (int) ctx->msg_id,
				(void *)(uintptr_t)(idx + 1)) < 0)
			return (struct rxm_selector_result){
				.idx = 0, .wants_spread = true };
		return (struct rxm_selector_result){
			.idx = idx, .wants_spread = true };

	case RXM_OP_SAR_LAST:
		slot = ofi_idm_lookup(&rr->sar_pins, (int) ctx->msg_id);
		if (slot) {
			ofi_idm_clear(&rr->sar_pins, (int) ctx->msg_id);
			return (struct rxm_selector_result){
				.idx = rxm_clamp_slot(conn,
					(uint8_t)((uintptr_t) slot - 1)),
				.wants_spread = true };
		}
		return (struct rxm_selector_result){
			.idx = rxm_rr_next(rr, conn), .wants_spread = true };

	default:
		return (struct rxm_selector_result){
			.idx = 0, .wants_spread = false };
	}
}

static void rxm_rr_destroy(struct rxm_qp_selector *sel)
{
	struct rxm_rr_selector *rr =
		container_of(sel, struct rxm_rr_selector, base);

	/* Stored values are (idx + 1) cast to void*, not heap pointers. */
	ofi_idm_reset(&rr->sar_pins, NULL);
	free(rr);
}

struct rxm_qp_selector *rxm_rr_selector_alloc(void)
{
	struct rxm_rr_selector *rr = calloc(1, sizeof(*rr));

	if (!rr)
		return NULL;

	rr->base.select = rxm_rr_select;
	rr->base.destroy = rxm_rr_destroy;
	return &rr->base;
}
