#ifndef TEB_LOCAL_PLANNER_EDGE_STATION_ENVELOPE_H_
#define TEB_LOCAL_PLANNER_EDGE_STATION_ENVELOPE_H_
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/station_envelope.h>
namespace teb_local_planner {
class EdgeStationEnvelope : public BaseTebUnaryEdge<1, double, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void setEnvelope(const StationEnvelope& envelope) { envelope_ = envelope; }
  void computeError() override {
    const auto* pose = static_cast<const VertexPose*>(_vertices[0]);
    const double distance = std::hypot(pose->estimate().x() - envelope_.center_x,
                                      pose->estimate().y() - envelope_.center_y);
    // Hold the requested station center throughout the turn. Hard envelope
    // containment is checked separately from this tracking objective.
    _error[0] = distance;
  }
 private:
  StationEnvelope envelope_;
};
}  // namespace teb_local_planner
#endif
