#ifndef MATDOG_NETWORK_HTTP_MAILBOX_H
#define MATDOG_NETWORK_HTTP_MAILBOX_H
#include <atomic>
namespace matdog {
namespace network {
// Atomic delivery/abandon decision. The successful reader owns slot release;
// Controller releases an abandoned slot. Request IDs additionally reject late
// semaphore wakes. Response bytes are written before claimDelivery (release).
class HttpMailbox {
public:
  void beginDispatch() { state_.store(State::WAITING); }
  bool abandon() {
    State expected = State::WAITING;
    return state_.compare_exchange_strong(expected, State::ABANDONED);
  }
  bool awaitingResponse() const { return state_.load() == State::WAITING; }
  bool claimDelivery() {
    State expected = State::WAITING;
    return state_.compare_exchange_strong(expected, State::DELIVERED);
  }
  void delivered() { claimDelivery(); }

private:
  enum class State { EMPTY, WAITING, ABANDONED, DELIVERED };
  std::atomic<State> state_{State::EMPTY};
};
} // namespace network
} // namespace matdog
#endif
