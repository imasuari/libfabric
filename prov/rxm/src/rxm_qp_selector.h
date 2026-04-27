#ifndef RXM_QP_SELECTOR_H
#define RXM_QP_SELECTOR_H

#include <stddef.h>
#include <stdint.h>

struct rxm_conn;

enum rxm_op_type {
	RXM_OP_EAGER,
	RXM_OP_SAR_FIRST,
	RXM_OP_SAR_CONT,
	RXM_OP_RNDV_CTRL,
	RXM_OP_RNDV_RMA,
	RXM_OP_RMA,
	RXM_OP_ATOMIC,
};

struct rxm_selector_ctx {
	enum rxm_op_type op;
	uint64_t msg_id;
};

struct rxm_qp_selector {
	const char *name;
	int (*init)(struct rxm_conn *conn, void **state);
	void (*fini)(struct rxm_conn *conn, void *state);
	uint8_t (*select)(struct rxm_conn *conn, void *state,
			  const struct rxm_selector_ctx *ctx);
};

/* Shared placeholders for selectors that carry no per-conn state.
 * Both log a debug message and are safe to wire as .init / .fini so
 * call sites never need a NULL check.
 */
int  rxm_selector_noop_init(struct rxm_conn *conn, void **state);
void rxm_selector_noop_fini(struct rxm_conn *conn, void *state);

extern const struct rxm_qp_selector rxm_selector_single_qp;

#endif /* RXM_QP_SELECTOR_H */
