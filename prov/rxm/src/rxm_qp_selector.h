#ifndef RXM_QP_SELECTOR_H
#define RXM_QP_SELECTOR_H

#include <stddef.h>
#include <stdint.h>

#include <ofi_indexer.h>

struct rxm_conn;

enum rxm_op_type {
	RXM_OP_EAGER,
	RXM_OP_SAR_FIRST,
	RXM_OP_SAR_MIDDLE,
	RXM_OP_SAR_LAST,
	RXM_OP_RNDV_CTRL,
	RXM_OP_RNDV_RMA,
	RXM_OP_RMA,
	RXM_OP_ATOMIC,
};

static inline const char *rxm_op_type_str(enum rxm_op_type op)
{
	switch (op) {
	case RXM_OP_EAGER:      return "EAGER";
	case RXM_OP_SAR_FIRST:  return "SAR_FIRST";
	case RXM_OP_SAR_MIDDLE: return "SAR_MIDDLE";
	case RXM_OP_SAR_LAST:   return "SAR_LAST";
	case RXM_OP_RNDV_CTRL:  return "RNDV_CTRL";
	case RXM_OP_RNDV_RMA:   return "RNDV_RMA";
	case RXM_OP_RMA:        return "RMA";
	case RXM_OP_ATOMIC:     return "ATOMIC";
	default:                return "UNKNOWN";
	}
}

struct rxm_selector_ctx {
	enum rxm_op_type op;
	uint64_t msg_id;
};

struct rxm_qp_selector {
	uint8_t (*select)(struct rxm_conn *conn,
			  const struct rxm_selector_ctx *ctx);
	void (*destroy)(struct rxm_qp_selector *sel);
};

struct rxm_rr_selector {
	struct rxm_qp_selector base;
	uint32_t rr_counter;
	/* msg_id -> (qp_idx + 1) encoded as void*; entry present means
	 * the SAR message is pinned to that qp. Absent (NULL) means no
	 * pin yet. +1 encoding distinguishes "pinned to qp 0" from
	 * "not present". */
	struct index_map sar_pins;
};

extern const struct rxm_qp_selector rxm_selector_single_qp;

struct rxm_qp_selector *rxm_rr_selector_alloc(void);

#endif /* RXM_QP_SELECTOR_H */
