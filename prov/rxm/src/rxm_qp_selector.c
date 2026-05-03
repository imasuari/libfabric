#include "rxm.h"
#include "rxm_qp_selector.h"

static uint8_t rxm_single_qp_select(struct rxm_conn *conn,
				    const struct rxm_selector_ctx *ctx)
{
	OFI_UNUSED(conn);
	OFI_UNUSED(ctx);
	return 0;
}

const struct rxm_qp_selector rxm_selector_single_qp = {
	.select = rxm_single_qp_select,
};
