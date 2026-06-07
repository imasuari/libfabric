# rxm multi-QP connection management — design

## Background

The current branch (`mqp02-cm` vs. `multiQP-main`) introduces the data-plane
plumbing for routing a single rxm `conn` over multiple underlying msg-provider
endpoints (QPs):

- `struct rxm_conn` carries `msg_eps[]` (length `num_msg_eps`) and a pluggable
  `rxm_ep_selector`.
- `rxm_conn_msg_ep(conn, pkt)` (rxm.h:794) replaces every direct `conn->msg_ep`
  dereference on the TX path.
- The round-robin selector (`rxm_ep_selector.c`) spreads non-SAR packets across
  slots and pins each SAR message (middle/last segments) to one slot to keep
  in-order delivery.
- `FI_OFI_RXM_NUM_MSG_EPS` (`rxm_init.c`) sets the slot count.
- A verbs-only clamp lives in `rxm_alloc_conn` (rxm_conn.c:428).

What is **missing**: the CM layer still opens exactly one msg endpoint per
`rxm_conn` and copies that pointer into every slot of `msg_eps[]`
(`rxm_open_conn`, rxm_conn.c:223–229). The selector picks index `i`, but every
index resolves to the same underlying QP. There is no spray.

This document describes the changes needed to open, connect, and tear down N
real msg-provider endpoints per `rxm_conn`, so the existing selector actually
distributes traffic across distinct QPs.

## Goals

1. Each `rxm_conn` owns N distinct `fid_ep`s, all connected to the same remote
   peer's `rxm_conn`, with `msg_eps[i]` aligned on both sides (sender's slot
   `i` ↔ receiver's slot `i`).
2. CM events (CONNECTED, SHUTDOWN, REJECT, error) are routed back to the
   correct `(conn, slot)` pair.
3. A `conn` only transitions to `RXM_CM_CONNECTED` once **all** slots have
   completed their handshake.
4. Teardown closes every slot and frees per-slot resources.
5. Backward compatible with `num_msg_eps == 1` peers — the wire protocol does
   not break for legacy rxm.
6. Correctness of in-order semantics for SAR is preserved (already guaranteed
   by the selector's pinning).

## Non-goals

- Multi-NIC / multi-rail device selection. We open N QPs over the *same*
  msg-provider domain. Steering across NICs is a future step.
- Per-slot flow control negotiation. Flow control is negotiated once and
  applied uniformly to all slots.
- Dynamic resize of `num_msg_eps` after connect. Slot count is fixed at conn
  alloc time.

## Wire protocol

Today's `union rxm_cm_data._connect` (rxm.h:82) layout:

```
version : u8
endianness : u8
ctrl_version : u8
op_version : u8
port : u16
flow_ctrl : u8
padding : u8           <-- repurposed
eager_limit : u32
rx_size : u32
client_conn_id : u64
```

We repurpose the `padding` byte as `ep_idx`: the slot the initiator is
attempting to fill. Legacy peers send 0, which matches the existing single-ep
behavior, so no version bump is required.

`_accept` does not need to carry the slot — the acceptor echoes the slot
implicitly by replying on the matching connection request, and the initiator
already knows which `fi_connect` is being completed via the per-slot endpoint
context (see below).

## Resolving slot index at CM dispatch

`fi_endpoint(domain, info, &ep, context)` stores `context` in `ep->fid.context`,
and CM events arrive with `cm_entry->fid->context`. We keep `context` as
`rxm_conn *` exactly like today — no new ctx struct, no parallel array.

To know **which slot** fired, scan `msg_eps[]` by pointer identity:

```c
static inline int rxm_slot_of(struct rxm_conn *conn, struct fid *fid)
{
    uint8_t i;

    for (i = 0; i < conn->num_msg_eps; i++)
        if (conn->msg_eps[i] && &conn->msg_eps[i]->fid == fid)
            return i;
    return -1;
}
```

`rxm_conn`'s scalar `state` member is widened into a parallel array
indexed alongside `msg_eps[]`:

```c
enum rxm_cm_state *states;   /* parallel array, length num_msg_eps */
```

`states[]` reuses the existing `enum rxm_cm_state` — per-slot
lifecycle is identical to the conn-level one (`IDLE` → `CONNECTING` /
`ACCEPTING` → `CONNECTED`). `states[0]` *is* the conn-level CM state
(it replaces the old `conn->state` member); the selector reads
`states[idx]` without caring whether `idx == 0`.

No aggregate `connected_cnt` is needed in the lazy model —
`states[0]` tracks slot 0 / data-path readiness, and per-slot progress
for the lazy slots lives in `states[i]`.

There is **no `FAILED` slot state.** Lazy-open failure is handled by
truncation (see [Failure semantics](#failure-semantics)): a slot that
fails its `fi_endpoint`/`fi_connect` causes `conn->num_msg_eps` to be
clamped down to the failed index, dropping that slot and all higher
slots from rotation. Lazy opens are issued in strict ascending order
(rr stalls on slot K until it reaches `CONNECTED` before ever picking
slot K+1), so slots above the failed index are guaranteed to still be
`IDLE` and unopened — discarding them is correct.

CM dispatch sites get `conn` from `cm_entry->fid->context` (unchanged) and
derive the slot index via `rxm_slot_of(conn, cm_entry->fid)`:

- `rxm_process_connect` (rxm_conn.c:537)
- `rxm_handle_error` → `rxm_process_reject` / `rxm_process_shutdown`
  (rxm_conn.c:847, 849)

`num_msg_eps` is small (typical 2–8, capped at `uint8_t`), and CM events
are off the data path, so the linear scan is irrelevant cost-wise — and
strictly cheaper than the alternative cache miss into a parallel ctx
array.

## State machine

`states[0]` carries the meaning of the old `conn->state` — it tracks
the **primary** QP, the one that gates whether `rxm_get_conn` returns
success and lets data-plane sends proceed. Additional slots are opened
lazily and have their own per-slot lifecycle, but they do **not** delay
the conn becoming usable.

| state                | meaning (slot 0)                              |
|----------------------|-----------------------------------------------|
| `RXM_CM_IDLE`        | no slot has been opened                       |
| `RXM_CM_CONNECTING`  | slot 0 is mid-handshake (initiator)           |
| `RXM_CM_ACCEPTING`   | slot 0 is mid-handshake (acceptor)            |
| `RXM_CM_CONNECTED`   | slot 0 is connected; data-plane usable        |

Per-slot state for slots > 0 uses the same enum and the same array:

```c
struct rxm_conn {
    ...
    enum rxm_cm_state *states;  /* length num_msg_eps;
                                 * states[0] = conn-level CM state */
    ...
};
```

The four existing values (`RXM_CM_IDLE`, `RXM_CM_CONNECTING`,
`RXM_CM_ACCEPTING`, `RXM_CM_CONNECTED`) describe the per-slot lifecycle
exactly. No `FAILED` value is added — lazy-open failure shrinks
`num_msg_eps` instead of marking individual slots dead. See
[Failure semantics](#failure-semantics).

### Lazy opening

The whole point of this revision: slot 0 is opened during the initial
`rxm_send_connect`/`rxm_process_connreq` exactly as today. Slots 1..N−1 are
**not** opened until the selector hands traffic to them.

The selector owns slot availability. `rxm_conn_msg_ep` (rxm.h:794) is
trivial — it just trusts the selector:

```c
static inline struct fid_ep *
rxm_conn_msg_ep(struct rxm_conn *conn, const struct rxm_pkt *pkt)
{
    return conn->msg_eps[conn->selector->select(conn, pkt)];
}
```

The selector's contract: **never return a slot whose `states[idx]` is
not `RXM_CM_CONNECTED`.** If its preferred pick isn't ready, it falls
back to slot 0 (which is guaranteed `CONNECTED` whenever the conn is in
`RXM_CM_CONNECTED`) and, on the way out, may kick a lazy open for the
slot it skipped so future picks can land there.

This collapses three previously separate concerns — slot-state checking,
SAR pin correctness, and rr-counter accounting — into one place.

### Why fall back to slot 0

Slot 0 is guaranteed open whenever the conn is in `RXM_CM_CONNECTED`. Sends
that arrive while a sibling slot is mid-handshake go through slot 0. This:

- Keeps the data path lock-free of CM state machinery.
- Avoids deferring user-visible sends to wait for QP creation.
- Matches today's semantics (everything goes through slot 0) until traffic
  actually warrants spreading.

The trade-off: under steady-state, traffic ramps up onto sibling slots only
after their first selector-pick → CONNECTED round trip. For long-lived
conns this is a one-time warm-up cost; for short conns there's effectively
no spray, which is fine — short conns don't need it.

### Selector enforces slot availability

The rr selector (`rxm_rr_next`, rxm_ep_selector.c:19) consults
`conn->states[]` and only advances `rr_counter` when it lands on a
`CONNECTED` slot:

```c
static uint8_t rxm_rr_next(struct rxm_rr_selector *rr, struct rxm_conn *conn)
{
    uint8_t idx;

    if (conn->num_msg_eps <= 1)
        return 0;

    idx = 1 + (rr->rr_counter % (conn->num_msg_eps - 1));

    if (conn->states[idx] == RXM_CM_CONNECTED) {
        rr->rr_counter++;
        return idx;
    }

    /* Slot not ready: kick a lazy open if it has never been opened,
     * then fall back to slot 0. rr_counter is NOT advanced — the next
     * call retries the same slot, giving the lazy open a natural
     * chance to land. The kick path is rxm_open_msg_ep(conn, idx, info)
     * + rxm_init_connect_data + fi_connect, all inline (no wrapper
     * function — rxm_open_msg_ep is the reused per-ep helper that
     * also opens slot 0 in rxm_open_conn).
     */
    if (conn->states[idx] == RXM_CM_IDLE)
        rxm_lazy_connect_slot(conn, idx);

    return 0;
}
```

This single change handles three things at once:

1. **rr_counter is never wasted on a slot that didn't carry traffic.**
   Counter advances only when the pick is honored.
2. **SAR pin correctness is automatic.** When the rr-chosen slot is not
   `CONNECTED`, `rxm_rr_next` returns 0; the SAR-MIDDLE pin records 0;
   every later MIDDLE/LAST segment for that `msg_id` looks up the pin and
   gets 0. Mid-SAR transitions of slot N from `CONNECTING` → `CONNECTED`
   cannot split a sequence across QPs because the pin already says 0.
3. **Lazy open trigger lives where the slot was actually picked.** The
   selector knows it just considered slot N; it's the natural place to
   issue the lazy open.

`rxm_lazy_connect_slot(conn, idx)` is a small wrapper inside
`rxm_ep_selector.c` (or `rxm_conn.c` exposed as static-inline) that
calls `rxm_open_msg_ep(conn, idx, conn->ep->msg_info)` + builds
`cm_data` with `ep_idx = idx` + `fi_connect`. On non-zero return it
clamps `conn->num_msg_eps = idx`. The selector's modulo is naturally
self-correcting: on the next pick `rr->rr_counter % (conn->num_msg_eps
- 1)` no longer produces the failed index, and slots that were never
opened (the still-`IDLE` ones above `idx`) drop out of rotation
without any per-slot skip logic.

`rxm_conn_msg_ep` reduces to one line: `return conn->msg_eps[selector->select(conn, pkt)]`.

#### Future: looser SAR policy

Only the SAR FIRST segment has a strict ordering constraint (it must
arrive before any FIRST of another SAR on the same conn, which is why
FIRST is hard-pinned to slot 0). MIDDLEs may well reassemble correctly
when sprayed across QPs, and a future phase explicitly wants to use
multiple QPs for one SAR message.

When implementing the selector change above, leave a `TODO` comment near
the SAR MIDDLE/LAST cases noting that an alternative is to **not** record
the pin when falling back — letting future segments re-consult the rr
state and pick up newly-`CONNECTED` slots mid-message. We pick the
conservative pin-to-0-on-fallback path now until the cross-QP reassembly
assumption is verified.

### Locking

The lazy-open path calls `fi_endpoint` + `fi_connect` (via
`rxm_open_msg_ep` and the inline connect), which require
`conn->ep->util_ep.lock`. The TX path already holds it (`rxm_get_conn`
asserts the lock; all ep ops hold it). So no new locking — but document
the assumption.

## API changes

### rxm.h

```c
struct rxm_conn {
    ...
    struct fid_ep    **msg_eps;       /* existing */
    enum rxm_cm_state *states;    /* length num_msg_eps;
                                   * states[0] replaces the old scalar
                                   * `state` member. */
    uint8_t            num_msg_eps;
    ...
};

union rxm_cm_data {
    struct _connect {
        ...
        uint8_t flow_ctrl;
        uint8_t ep_idx;     /* was: padding */
        ...
    } connect;
    ...
};
```

### rxm_conn.c

Extract a per-ep helper from `rxm_open_conn`:

```c
static int rxm_open_msg_ep(struct rxm_conn *conn, uint8_t idx,
                           struct fi_info *info);
```

Responsibilities (everything that runs once per `fid_ep`):

- `fi_endpoint(domain, info, &ep, conn)`  /* context stays as conn */
- `fi_ep_bind(ep, msg_eq, 0)`
- `fi_ep_bind(ep, msg_srx, 0)` if SRX
- `rxm_bind_comp(ep, ...)` (CQ + counters)
- `fi_enable(ep)`
- if no SRX: `rxm_prepost_recv(ep, ep)` for this slot
- store into `conn->msg_eps[idx]`

`rxm_open_conn` is the slot-0 orchestrator (still called from
`rxm_send_connect` / `rxm_process_connreq` exactly as today):

- allocate `conn->msg_eps[]` (length `num_msg_eps`)
- call `rxm_open_msg_ep(conn, 0, info)`
- on success, sample `flow_ctrl_ops->available(msg_eps[0])` into
  `conn->flow_ctrl` (slot 0 only — flow_ctrl is conn-wide)
- slots > 0 are opened lazily — the selector's lazy-open path calls
  `rxm_open_msg_ep(conn, idx, info)` + `fi_connect` (no separate
  "kick" function; the inline reuse keeps the call sites symmetric:
  slot 0 = open + fi_connect from rxm_send_connect; slot i = open +
  fi_connect from the rr-selector lazy path)

`states[]` is allocated/freed in `rxm_alloc_conn` / `rxm_free_conn`
(see Allocation below).

### Initiator path — slot 0 (eager, unchanged behavior)

`rxm_send_connect` (rxm_conn.c:274) is essentially as today:

- open slot 0 via `rxm_open_msg_ep(conn, 0, info)`
- `rxm_init_connect_data(conn, &cm_data)` → set `cm_data.connect.ep_idx = 0`
- `fi_connect(msg_eps[0], ...)`
- `state = RXM_CM_CONNECTING`, `states[0] = RXM_CM_CONNECTING`

`rxm_process_connect` (rxm_conn.c:537), now slot-aware:

```c
conn = cm_entry->fid->context;
i    = rxm_slot_of(conn, cm_entry->fid);
assert(i >= 0);

conn->states[i] = RXM_CM_CONNECTED;

if (i == 0) {
    /* primary slot up — conn is now usable */
    if (conn->states[0] == RXM_CM_CONNECTING) {
        conn->remote_index = ...;     /* existing logic */
        rxm_set_peer_flow_ctrl(conn, ...);
    }
    if (conn->flow_ctrl && conn->peer_flow_ctrl)
        domain->flow_ctrl_ops->enable(conn->msg_eps[0], ...);
    conn->ep->connecting_cnt--;
    /* states[0] is set to RXM_CM_CONNECTED above (the unconditional
     * assignment), which is also the conn-level "data path usable" flip. */
} else {
    /* sibling slot up — apply flow_ctrl uniformly, no state change */
    if (conn->flow_ctrl && conn->peer_flow_ctrl)
        domain->flow_ctrl_ops->enable(conn->msg_eps[i], ...);
}
```

`states[0]` flips to `CONNECTED` as soon as slot 0 is up — siblings come
online opportunistically and don't gate user sends.

### Initiator path — slots > 0 (lazy)

The first time `rxm_rr_next` picks slot `idx` while it is `RXM_CM_IDLE`,
the selector inlines an open + connect for that slot. The pattern
mirrors `rxm_send_connect` for slot 0 — the only differences are the
slot index, the `ep_idx` byte in the cm_data, and the failure handling
(truncate, not conn-teardown). Concretely:

```c
/* In rxm_rr_next, on the IDLE branch */
{
    union rxm_cm_data cm_data;
    struct fi_info   *info = conn->ep->msg_info;
    int               ret;

    assert(conn->states[0] == RXM_CM_CONNECTED);   /* slot 0 already up */
    assert(ofi_genlock_held(&conn->ep->util_ep.lock));

    ret = rxm_open_msg_ep(conn, idx, info);
    if (ret) goto truncate;

    ret = rxm_init_connect_data(conn, &cm_data);
    if (ret) goto close_slot;

    cm_data.connect.ep_idx = idx;
    ret = fi_connect(conn->msg_eps[idx], info->dest_addr,
                     &cm_data, sizeof(cm_data));
    if (ret) goto close_slot;

    conn->states[idx] = RXM_CM_CONNECTING;
    /* fall through to "return 0" — selector still returns slot 0 for
     * this pick; sibling becomes usable once CONNECTED arrives. */
    goto fallback;

close_slot:
    fi_close(&conn->msg_eps[idx]->fid);
    conn->msg_eps[idx] = NULL;
truncate:
    /* Drop slot idx and everything above. Lazy opens are strictly
     * in-order (rr stalls on slot K until CONNECTED before picking
     * K+1), so slots > idx are still IDLE and unopened. The selector's
     * modulo on the new num_msg_eps stops landing on idx.
     */
    conn->num_msg_eps = idx;
fallback:
    return 0;   /* always fall back to slot 0 for this pick */
}
```

In practice this body should live in a small static helper inside
`rxm_ep_selector.c` (or `rxm_conn.c` exposed as `static inline`)
called from the IDLE branch of `rxm_rr_next` — keeps the selector
function small. The point is it's not a separate exported function
like the earlier `rxm_kick_lazy_connect`; it's a direct reuse of
`rxm_open_msg_ep` + `fi_connect`, the same primitives slot 0 uses.

A failed lazy open does **not** tear down the conn — slot 0 is still good.
The conn keeps running with however many slots already reached
`CONNECTED`, and the data path keeps spraying across those (or just
using slot 0 if `idx == 1`).

### Acceptor path

`rxm_process_connreq` (rxm_conn.c:683):

```c
slot = cm_data->connect.ep_idx;     /* 0 for legacy peers */

if (slot == 0) {
    /* existing flow, untouched: find/alloc conn, simultaneous-connect
     * coin flip, open slot 0, fi_accept on msg_eps[0],
     * states[0] = RXM_CM_ACCEPTING */
} else {
    /* sibling lazy-open from peer */
    conn = ofi_idm_lookup(&ep->conn_idx_map, peer->index);
    if (!conn || conn->states[0] != RXM_CM_CONNECTED ||
        slot >= conn->num_msg_eps ||
        conn->states[slot] != RXM_CM_IDLE)
        goto reject;

    if (rxm_open_msg_ep(conn, slot, cm_entry->info))
        goto reject;

    if (rxm_accept_connreq_slot(conn, cm_entry, slot))
        goto close_slot;

    conn->states[slot] = RXM_CM_ACCEPTING;
}
```

Slot 0's existing flow stays intact. Sibling slots:

- arrive **after** the primary conn is in `RXM_CM_CONNECTED` (peer must have
  slot 0 connected before its TX path can lazy-open slot `i`)
- skip the simultaneous-connect coin flip — the conn already exists; both
  sides have agreed on which `rxm_conn` represents this peer
- bump per-conn `accepting_cnt` is unnecessary; `states[slot]` carries
  the per-slot lifecycle

`rxm_process_connect` on the acceptor for slot `i > 0` flips
`states[i] = CONNECTED` and applies flow control. `states[0]` is
already `CONNECTED`, no transition.

### Cleanup

`rxm_close_conn` (rxm_conn.c:55):

```c
if (conn->msg_eps) {
    for (i = 0; i < conn->num_msg_eps; i++) {
        if (conn->msg_eps[i])
            fi_close(&conn->msg_eps[i]->fid);
    }
}
free(conn->msg_eps);
free(conn->states);
conn->msg_eps    = NULL;
conn->states = NULL;
```

The error path in `rxm_send_connect` (rxm_conn.c:309–313) gets the same
loop-and-free.

The liveness probe `rx_buf->conn->msg_eps[0]` at rxm.h:925 still works: slot 0
is always open while the conn is alive; the lazy slots are auxiliaries.

### Allocation

`rxm_alloc_conn` (rxm_conn.c:398) already determines `num_msg_eps` and selects
the selector. Extend it to also allocate two parallel arrays of length
`num_msg_eps`:

- `msg_eps[]`     — all NULL initially
- `states[i]` — all `RXM_CM_IDLE` initially

`rxm_open_conn` no longer allocates `msg_eps`.

### Flow control

`rxm_process_connect` enables flow control per slot, when each slot reaches
`CONNECTED` (initiator and acceptor both):

```c
if (conn->flow_ctrl && conn->peer_flow_ctrl)
    domain->flow_ctrl_ops->enable(conn->msg_eps[i],
                                  ep->msg_info->rx_attr->size / 2);
```

This sidesteps "wait for all slots before enabling" and matches the lazy
opening order. `flow_ctrl` itself is decided at slot 0 (initiator samples
`available()` on slot 0; acceptor mirrors via `_connect.flow_ctrl` flag),
then applied uniformly on whichever sibling slots actually open.

## SAR ordering

Already correct. The round-robin selector pins all middle/last SAR segments of
a `msg_id` to the slot chosen for the first middle segment
(`rxm_ep_selector.c:46-66`). Reconnect destroys the selector along with the
conn, so stale pins cannot leak across reconnects.

## Failure semantics

Slot 0 failure tears down the entire conn, as today — `rxm_process_reject` /
`rxm_process_shutdown` call `rxm_close_conn` + `rxm_free_conn`, and the
per-slot loop in `rxm_close_conn` cleans up any sibling slots that have been
opened.

Sibling slot (`i > 0`) failure splits into two cases by *when* it
happens. There is no `FAILED` slot state — the two cases are handled
without one.

### Case A — lazy-open failure (slot still `IDLE`)

The selector's lazy open (`rxm_open_msg_ep` + `fi_connect`) fails
before the slot ever reaches `CONNECTED`. Causes are narrow because
slot 0 succeeded against the same peer/domain/info: peer's
`num_msg_eps` is smaller than ours (deterministic permanent),
QP/resource cap reached on either side (possibly transient),
provider OOM (transient), RDMA-CM handshake timeout (rare — already
retried internally).

**Resolution: truncate.** The selector clamps
`conn->num_msg_eps = idx` on failure. Slots above `idx` are still
`IDLE` (lazy opens are strictly in-order: rr stalls on slot K until
`CONNECTED` before picking K+1), so dropping them is correct — they
were never opened. The selector's modulo on the new `num_msg_eps`
naturally avoids `idx` and everything above. The conn keeps running
with `idx` slots; if `idx == 1` only slot 0 is used, exactly matching
the single-ep config.

This handles the dominant real-world failure mode (peer config
mismatch) optimally: the very first slot above the peer's cap fails →
we truncate to the peer's cap → no further wasted attempts.

We deliberately do not retry. A retry without backoff storms the
resource we're already short on; a retry *with* backoff requires a
per-slot timer and a budget cap, which is just `FAILED` plus
machinery. The cost of not retrying is one fewer QP on a conn that's
still fully functional via the slots that did open.

### Case B — failure after `CONNECTED`

The slot reached `CONNECTED` and later receives `FI_SHUTDOWN` or an
async error. Truncation is unsafe here because slots above `idx` may
also be `CONNECTED` and carrying traffic, so we cannot drop them.

**Resolution: treat as conn-fatal.** The whole conn tears down (close
all slots, free conn, peer reconnects on next `rxm_get_conn`). This
matches slot-0 behavior. Rare in practice: a peer dropping one QP
mid-flight usually signals a real problem worth resetting.

If we ever want a non-fatal "skip this slot" path for case B, that's
when a `FAILED` slot state earns its keep — an additive change. For
now case B keeps the conn-level reconnect path simple and uniform.

### Dispatch

`rxm_process_reject` and `rxm_process_shutdown` derive the slot via
`rxm_slot_of(conn, cm_entry->fid)` and dispatch on it:

- `idx == 0`: existing behavior (close & free conn).
- `idx > 0` and `states[idx] != RXM_CM_CONNECTED`: case A — close
  the slot, truncate `num_msg_eps = idx`, leave conn alive. (Most
  case-A failures are caught synchronously inside the selector's
  lazy-open path; an async reject during a lazy-open hits this branch.)
- `idx > 0` and `states[idx] == RXM_CM_CONNECTED`: case B —
  close & free conn.

Sibling-slot reject on the acceptor follows the same rule.

## Step-by-step implementation order

1. Replace scalar `conn->state` with `enum rxm_cm_state *states` array
   (length `num_msg_eps`); allocate in `rxm_alloc_conn`. `states[0]`
   carries the old conn-level CM state.
2. Extract `rxm_open_msg_ep(conn, idx, info)` — the per-ep helper —
   from `rxm_open_conn`. `rxm_open_conn` becomes the slot-0
   orchestrator (alloc msg_eps[], open slot 0, sample flow_ctrl). No
   call-site change yet.
3. Add `rxm_slot_of(conn, fid)` and route CM dispatch
   (`rxm_process_connect`, `rxm_process_reject`, `rxm_process_shutdown`)
   through it to derive `idx`. `fi_endpoint` context stays as `conn`.
   Still single slot — verify no regression.
4. Add `ep_idx` to `cm_data.connect` (repurpose `padding`), send `0`, ignore
   on receive. Verify legacy interop and single-ep regression.
5. Make `rxm_rr_next` consult `conn->states[]`: return the rr-chosen
   slot only when `CONNECTED`, otherwise inline an open + connect
   (`rxm_open_msg_ep` + `fi_connect`) when `IDLE`, and return 0
   without advancing `rr_counter`. `rxm_conn_msg_ep` stays a one-liner
   that trusts the selector. SAR pin correctness falls out automatically.
6. Acceptor: handle `ep_idx > 0` in `rxm_process_connreq` — call
   `rxm_open_msg_ep(conn, slot, info)`, `fi_accept`, set
   `states[slot] = RXM_CM_ACCEPTING`.
7. Per-slot CM completion: `rxm_process_connect` flips `states[i] =
   RXM_CM_CONNECTED` and applies flow control to that slot.
8. Per-slot failure handling: lazy-open failure (slot `IDLE`) truncates
   `num_msg_eps`; CM failure on a `CONNECTED` sibling slot tears down
   the conn (case B). Slot 0 keeps existing behavior.
9. `rxm_close_conn` per-slot close loop and `states` free.

## Commit split

Each commit must compile cleanly, pass all existing rxm tests with
`FI_OFI_RXM_NUM_MSG_EPS=1` (the default), and stand on its own — no
partially-wired feature spanning two commits.

### Commit 1 — selector: slot-state awareness, no CM change

Adds the selector-side machinery that the lazy CM commit will rely on,
without changing CM behavior. `rxm_open_conn` still aliases all
`msg_eps[]` slots to slot 0's ep, so spray is inert and every
`states[i]` flips to `CONNECTED` simultaneously when slot 0 connects.

Scope:

- Replace the scalar `enum rxm_cm_state state` member with
  `enum rxm_cm_state *states` (length `num_msg_eps`, all
  `RXM_CM_IDLE`). `states[0]` *is* the conn-level CM state — every
  prior `conn->state` access becomes `conn->states[0]`. Allocate in
  `rxm_alloc_conn`; free in `rxm_free_conn`. Re-init `states[0] =
  RXM_CM_IDLE` in `rxm_close_conn` (replaces today's
  `conn->state = RXM_CM_IDLE`). Sibling slots stay `RXM_CM_IDLE`
  throughout commit 1 — they're never set otherwise — so no per-slot
  reset loop yet.
- `rxm_send_connect` writes `states[0] = RXM_CM_CONNECTING`;
  `rxm_process_connreq` writes `states[0] = RXM_CM_ACCEPTING`;
  `rxm_process_connect` writes `states[0] = RXM_CM_CONNECTED`. No
  separate scalar to keep in sync.
- Extract `rxm_open_msg_ep(conn, idx, info)` from `rxm_open_conn` (the
  per-ep helper: fi_endpoint, binds, fi_enable, prepost). `rxm_open_conn`
  becomes the slot-0 orchestrator that allocates `msg_eps[]`, calls
  `rxm_open_msg_ep(conn, 0, info)`, samples `flow_ctrl_ops->available()`
  on slot 0, and (commit-1 only) aliases sibling slots to slot 0's ep so
  the data-path keeps working with `num_msg_eps > 1`.
- Update `rxm_rr_next` to consult `states[]`: return the rr-chosen
  slot only when `RXM_CM_CONNECTED`, otherwise return 0. The `IDLE`
  branch has no kick yet — commit 1's aliasing means every
  `states[i]` is `CONNECTED` whenever slot 0 is, so the branch is
  unreachable.
- `rxm_conn_msg_ep` stays the one-liner that trusts the selector.

Why this is self-contained: with `num_msg_eps == 1` the rr selector
short-circuits (`conn->num_msg_eps <= 1` returns 0). With
`num_msg_eps > 1` the aliased slots all share slot 0's ep, so the
data path still works (no real spray, but no breakage either).
Single-slot tests behave identically. The
`rxm_open_msg_ep`/`rxm_open_conn` split is a pure refactor visible
nowhere outside `rxm_conn.c`.

No `FAILED` slot state is introduced — commit 2 handles lazy-open
failure by truncating `num_msg_eps`, and connected-slot failure by
tearing down the conn. Both reuse existing machinery, so the enum
stays at four states.

### Commit 2 — CM: lazy multi-slot open

Turns on the actual multi-QP plumbing. After this commit
`FI_OFI_RXM_NUM_MSG_EPS > 1` produces real spray.

Scope:

- Wire-protocol: repurpose `_connect.padding` as `ep_idx`. Initiator sets
  it; acceptor reads it (legacy peers send 0 → existing flow unchanged).
- Drop the commit-1 aliasing in `rxm_open_conn` — siblings start NULL
  and only become real eps via lazy open.
- Add `rxm_slot_of(conn, fid)` helper. Update `rxm_process_connect`,
  `rxm_process_reject`, `rxm_process_shutdown` to derive `idx` via it
  and dispatch on per-slot vs. slot-0 semantics.
- Wire up the lazy-open path in `rxm_rr_next`'s IDLE branch: call
  `rxm_open_msg_ep(conn, idx, info)` + `rxm_init_connect_data` +
  `fi_connect` (with `cm_data.connect.ep_idx = idx`); set
  `states[idx] = RXM_CM_CONNECTING`. On failure, **truncate**
  `conn->num_msg_eps = idx` (no conn teardown, no FAILED state).
- Acceptor: handle `ep_idx > 0` in `rxm_process_connreq` — find existing
  conn, call `rxm_open_msg_ep(conn, slot, info)`, `fi_accept`, set
  `states[slot] = RXM_CM_ACCEPTING`.
- Per-slot completion: `rxm_process_connect` for `idx > 0` flips
  `states[idx] = RXM_CM_CONNECTED` and applies flow control to that
  slot; `states[0]` stays `CONNECTED`, no transition.
- Per-slot failure: reject/shutdown for `idx > 0` while the slot is
  still `IDLE`/`CONNECTING`/`ACCEPTING` truncates `num_msg_eps = idx`
  (case A); reject/shutdown after `CONNECTED` tears down the conn
  (case B). Slot 0 keeps existing close-and-free behavior.
- `rxm_close_conn` walks `msg_eps[]` and closes every non-NULL slot.

Why this is self-contained: single-slot regression is preserved because
the new code paths only fire when the rr selector is active (which
requires `num_msg_eps > 1`) and when the peer sends `ep_idx > 0` (legacy
peers never do). The wire change is binary-compatible.
