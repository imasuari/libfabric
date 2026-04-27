#include "rxm.h"
#include "rxm_qp_selector.h"

int rxm_selector_noop_init(struct rxm_conn *conn, void **state)
{
	(void) conn;
	(void) state;
	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL,
	       "selector init not implemented (stateless selector)\n");
	return 0;
}

void rxm_selector_noop_fini(struct rxm_conn *conn, void *state)
{
	(void) conn;
	(void) state;
	FI_DBG(&rxm_prov, FI_LOG_EP_CTRL,
	       "selector fini not implemented (stateless selector)\n");
}

static uint8_t rxm_single_qp_select(struct rxm_conn *conn, void *state,
				    const struct rxm_selector_ctx *ctx)
{
	(void) conn;
	(void) state;
	(void) ctx;
	return 0;
}

const struct rxm_qp_selector rxm_selector_single_qp = {
	.name = "single_qp",
	.init = rxm_selector_noop_init,
	.fini = rxm_selector_noop_fini,
	.select = rxm_single_qp_select,
};
