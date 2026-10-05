#ifndef TEB_TERMINAL_OBSERVATION_H_
#define TEB_TERMINAL_OBSERVATION_H_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace teb_local_planner {

// A value snapshot of the native arrival predicate, never a second controller.
// TEB replaces it as a whole; readers retain independent copies.
struct TerminalObservation {
  struct Epoch {
    std::uint64_t event_sequence = 0, plan_sequence = 0;
    std::uint64_t compute_sequence = 0, time_ns = 0;
  };
  struct Inputs {
    std::string pose_frame, goal_frame, velocity_frame;
    std::uint64_t pose_stamp_ns = 0, goal_stamp_ns = 0;
    std::array<double, 3> pose{{NAN, NAN, NAN}};
    std::array<double, 3> goal{{NAN, NAN, NAN}};
    std::array<double, 3> velocity{{NAN, NAN, NAN}};
    double position_error = NAN, yaw_error = NAN;
    double position_tolerance = NAN, yaw_tolerance = NAN;
    bool yaw_optional = false;
    bool route_phase_pending = false;
    double translational_stopped_limit = NAN, rotational_stopped_limit = NAN;
    bool complete_global_plan = false, free_goal_velocity = false;
    // The via-point container is not read when its clause is bypassed.
    std::int64_t via_count = -1;
  };

  Epoch epoch;
  Inputs inputs;
  bool evaluated = false;
  std::string reason = "uninitialized";
  bool position_ok = false, yaw_ok = false, via_ok = false;
  bool route_phase_ok = false;
  bool stopped = false, reached = false;

  static TerminalObservation reset(const Epoch& epoch, const char* reason) {
    TerminalObservation result;
    result.epoch = epoch;
    result.reason = reason;
    return result;
  }

  static TerminalObservation evaluate(const Epoch& epoch, const Inputs& inputs,
                                      bool native_stopped) {
    auto result = reset(epoch, "evaluated");
    result.inputs = inputs;
    result.evaluated = true;
    result.position_ok = std::fabs(inputs.position_error) < inputs.position_tolerance;
    result.yaw_ok = inputs.yaw_optional ||
                    std::fabs(inputs.yaw_error) <= inputs.yaw_tolerance;
    result.via_ok = !inputs.complete_global_plan || inputs.via_count == 0;
    result.route_phase_ok = !inputs.route_phase_pending;
    result.stopped = native_stopped;
    result.reached = result.position_ok && result.yaw_ok && result.via_ok &&
                     result.route_phase_ok &&
                     (result.stopped || inputs.free_goal_velocity);
    if (!result.route_phase_ok) result.reason = "route_phase_pending";
    else if (!result.position_ok) result.reason = "position_tolerance_not_met";
    else if (!result.yaw_ok) result.reason = "heading_tolerance_not_met";
    else if (!result.via_ok) result.reason = "via_points_pending";
    else if (!result.stopped && !inputs.free_goal_velocity)
      result.reason = "motion_not_stopped";
    else result.reason = "arrival_conditions_met";
    return result;
  }
};

}  // namespace teb_local_planner
#endif
