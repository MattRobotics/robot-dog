#ifndef MATDOG_NETWORK_HTTP_MAILBOX_H
#define MATDOG_NETWORK_HTTP_MAILBOX_H

// Pure decision core for HttpTransport's single-slot request/response
// mailbox correlation — I7/I8 hardening, 2026-09-25, per the operator's
// finding: a timed-out dispatch() must never let its late-arriving
// response satisfy a LATER, unrelated dispatch().
//
// No FreeRTOS, no semaphore, no I/O — the actual blocking/signalling still
// happens with real semaphores in HttpTransport (that scheduling cannot be
// host-tested: there is no FreeRTOS scheduler on the host, see
// HttpTransport.h). What CAN be host-tested, and is
// (scripts/tests/test_http_mailbox.cpp), is the single question this class
// answers: "does a response now being computed still belong to a request
// somebody is waiting for?"
//
// THE PROTOCOL THIS CLASS RECORDS (enforced by HttpTransport, not here)
// -------------------------------------------------------------------------
//   httpd task                         Controller thread
//   -----------                        -----------------
//   Take(slot_free_)     <---- only one dispatch may hold the slot at once,
//                              so at most one request/response pair ever
//                              exists — no generation counter is needed.
//   beginDispatch()
//   write pending_request_
//   Give(request_ready_)
//   Take(response_ready_, bounded)
//     |
//     |                               Take(request_ready_)
//     |                               compute pending_response_
//     |                               if (awaitingResponse())
//     |                                 Give(response_ready_)
//     |                               delivered()
//     |                               Give(slot_free_)
//     v
//   timed out -> abandon()
//   OR
//   got it -> read pending_response_
//
// `slot_free_` is what makes a generation counter unnecessary: a NEW
// dispatch() cannot even write into the shared request/response structs
// until the Controller thread has fully finished with the PREVIOUS one
// (Give(slot_free_) is the Controller thread's last act on a request,
// always, whether or not anyone was still waiting for the answer) — so the
// data-race HttpTransport's single-slot design would otherwise have after
// a timeout (a second dispatch() overwriting pending_request_ while the
// Controller thread might still be reading it) cannot happen either.
//
// What THIS class alone prevents is the other half of the bug: even with
// no data race, the Controller thread's Give(response_ready_) for an
// ABANDONED request must not leave that semaphore signalled for a future,
// unrelated dispatch()'s Take(response_ready_) to wrongly consume. awaiting
// response_ is that guard, checked by the Controller thread immediately
// before every Give(response_ready_).

namespace matdog {
namespace network {

class HttpMailbox {
 public:
  // Called by the httpd-task side once it holds the shared slot exclusively
  // (i.e., immediately after taking slot_free_), before writing a request
  // and signalling request_ready_.
  void beginDispatch() { awaiting_response_ = true; }

  // Called by the httpd-task side when its bounded wait for a response
  // times out: "I am no longer listening for an answer to that request."
  void abandon() { awaiting_response_ = false; }

  // Called by the Controller-thread side once it has computed a response,
  // to decide whether to signal response_ready_. True: a dispatch() is
  // still genuinely waiting. False: it was abandoned — the result must be
  // dropped silently, never signalled.
  bool awaitingResponse() const { return awaiting_response_; }

  // Called by the Controller-thread side exactly once per processed
  // request, after awaitingResponse() has been read and acted on — resets
  // state so the NEXT dispatch()'s beginDispatch() starts clean regardless
  // of how this one ended.
  void delivered() { awaiting_response_ = false; }

 private:
  bool awaiting_response_ = false;
};

}  // namespace network
}  // namespace matdog

#endif  // MATDOG_NETWORK_HTTP_MAILBOX_H
