#include <stdlib.h>

#include "rxm.h"
#include "rxm_ep_selector.h"

static uint8_t rxm_single_ep_select(struct rxm_conn *conn,
				    const struct rxm_pkt *pkt)
{
	OFI_UNUSED(conn);
	OFI_UNUSED(pkt);
	return 0;
}

const struct rxm_ep_selector rxm_selector_single_ep = {
	.select = rxm_single_ep_select,
	.destroy = NULL,
};

static uint8_t rxm_rr_next(struct rxm_rr_selector *rr, struct rxm_conn *conn)
{
	uint8_t idx;

	if (OFI_UNLIKELY(conn->num_msg_eps <= 1))
		return 0;

	/* Self-heal if num_msg_eps shrank (e.g. an ep connection failed)
	 * and left rr_next pointing past the current end. */
	if (OFI_UNLIKELY(rr->rr_next >= conn->num_msg_eps))
		rr->rr_next = 1;

	idx = rr->rr_next;

	/* Always advance the RR pointer, regardless of the slot's state, so
	 * the cursor keeps rotating during warm-up instead of stalling on a
	 * not-yet-connected slot. Over the first sends this walks every slot
	 * and fires a connect on each IDLE one, bringing all QPs up in
	 * parallel. */
	if (OFI_UNLIKELY(++rr->rr_next >= conn->num_msg_eps))
		rr->rr_next = 1;

	if (OFI_LIKELY(conn->states[idx] == RXM_CM_CONNECTED))
		return idx;

	/* Slot not connected yet: kick off its connect (if idle) and fall
	 * back to the always-connected primary (slot 0) for this send. A
	 * failed rxm_send_connect just leaves the slot IDLE; the cursor will
	 * wrap back and retry it. */
	if (OFI_UNLIKELY(conn->states[idx] == RXM_CM_IDLE))
		(void) rxm_send_connect(conn, idx);

	return 0;
}

static uint8_t rxm_rr_select(struct rxm_conn *conn, const struct rxm_pkt *pkt)
{
	struct rxm_rr_selector *rr =
		container_of(conn->selector, struct rxm_rr_selector, base);
	enum rxm_sar_seg_type seg_type;
	uint64_t msg_id;
	void *slot;
	uint8_t idx;

	if (!pkt)
                // RMA / rndv-RMA .
		return rxm_rr_next(rr, conn);

	return 0;
}

static void rxm_rr_destroy(struct rxm_ep_selector *sel)
{
	struct rxm_rr_selector *rr =
		container_of(sel, struct rxm_rr_selector, base);

	/* Stored values are (idx + 1) cast to void*, not heap pointers. */
	ofi_idm_reset(&rr->sar_pins, NULL);
	free(rr);
}

struct rxm_ep_selector *rxm_rr_selector_alloc(void)
{
	struct rxm_rr_selector *rr = calloc(1, sizeof(*rr));

	if (!rr)
		return NULL;

	rr->base.select = rxm_rr_select;
	rr->base.destroy = rxm_rr_destroy;
	rr->rr_next = 1;
	return &rr->base;
}
