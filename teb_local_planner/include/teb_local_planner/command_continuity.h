#ifndef TEB_LOCAL_PLANNER_COMMAND_CONTINUITY_H_
#define TEB_LOCAL_PLANNER_COMMAND_CONTINUITY_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace teb_local_planner {

// Own the accepted planar reference, separately from measured robot velocity.
// Emergency zero is immediate. The elapsed-time budget never accumulates over
// pauses, and proposing a command does not commit it before feasibility checks.
class CommandContinuity {
 public:
  struct Command { double surge = 0.0; double yaw = 0.0; };

  bool configure(const std::string& scope, double period, double horizon) {
    if (scope.empty()) { scope_.clear(); reset(); return true; }
    if (!std::isfinite(period) || period <= 0.0 ||
        !std::isfinite(horizon) || horizon < period) {
      scope_.clear(); reset(); return false;
    }
    if (scope != scope_ || period != period_ || horizon != horizon_) reset();
    scope_ = scope;
    period_ = period;
    horizon_ = horizon;
    return true;
  }

  bool enabled() const { return !scope_.empty(); }
  double horizon() const { return horizon_; }
  void reset() { available_ = false; stamp_ = 0; previous_ = Command(); }

  bool select(const Command& requested, std::uint64_t now,
              double acceleration, double angular_acceleration,
              double forward, double reverse, double angular,
              Command& selected) const {
    selected = Command();
    if (!finite(requested) || !positive(acceleration) ||
        !positive(angular_acceleration) || !nonnegative(forward) ||
        !nonnegative(reverse) || !nonnegative(angular) || now == 0) return false;
    if (!enabled()) { selected = requested; return true; }
    if (requested.surge == 0.0 && requested.yaw == 0.0) return true;
    if (!available_ || now <= stamp_) return true;
    const double elapsed = static_cast<double>(now - stamp_) / 1e9;
    if (!std::isfinite(elapsed) || elapsed > horizon_) return true;
    const double dt = std::min(elapsed, period_);
    const double dx = acceleration * dt;
    const double dw = angular_acceleration * dt;
    if (!std::isfinite(dx) || !std::isfinite(dw)) return false;
    const double low_x = std::max(-reverse, previous_.surge - dx);
    const double high_x = std::min(forward, previous_.surge + dx);
    const double low_w = std::max(-angular, previous_.yaw - dw);
    const double high_w = std::min(angular, previous_.yaw + dw);
    // A newly tighter speed cap must never be overruled by continuity history.
    if (low_x > high_x || low_w > high_w) return false;
    selected.surge = std::max(low_x, std::min(high_x, requested.surge));
    selected.yaw = std::max(low_w, std::min(high_w, requested.yaw));
    return finite(selected);
  }

  void accept(const Command& command, std::uint64_t stamp) {
    if (!enabled() || !finite(command) || stamp == 0) { reset(); return; }
    previous_ = command;
    stamp_ = stamp;
    available_ = true;
  }

  // The route owner calls this once after its final identity check. Neither
  // optimization nor geometric checking is permitted to renew old history.
  void commit(Command& selected, std::uint64_t now, std::uint64_t proposed) {
    if (now == 0 || proposed == 0 || now < proposed ||
        static_cast<double>(now - proposed) / 1e9 > horizon_ ||
        (available_ && (now <= stamp_ ||
         static_cast<double>(now - stamp_) / 1e9 > horizon_))) selected = Command();
    accept(selected, now);
  }


 private:
  static bool positive(double value) { return std::isfinite(value) && value > 0.0; }
  static bool nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
  static bool finite(const Command& value) {
    return std::isfinite(value.surge) && std::isfinite(value.yaw);
  }
  std::string scope_;
  double period_ = 0.0;
  double horizon_ = 0.0;
  bool available_ = false;
  std::uint64_t stamp_ = 0;
  Command previous_;
};

}  // namespace teb_local_planner
#endif
