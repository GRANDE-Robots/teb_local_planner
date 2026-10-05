#include <nlopt.h>
#include <Eigen/Eigenvalues>
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <tf2/time.h>
#include <tf2_ros/buffer_interface.h>

#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/g2o_types/edge_station_envelope.h"

#include <map>
// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_shortest_path.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>

#include <memory>
#include <limits>

namespace teb_local_planner
{

namespace {
geometry_msgs::msg::Quaternion yawQuaternion(double yaw) {
  tf2::Quaternion q; q.setRPY(0, 0, yaw); return tf2::toMsg(q);
}
}

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(nullptr), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         initialized_(false), optimized_(false)
{
}

TebOptimalPlanner::TebOptimalPlanner(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  initialize(node, cfg, obstacles, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_)
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  node_ = node;
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();

  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;

  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
  initialized_ = true;

  setVisualization(visual);
}

void TebOptimalPlanner::setVisualization(const TebVisualizationPtr& visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;

  visualization_->publishLocalPlanAndPoses(teb_);

  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *cfg_->robot_model);

  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);

}

/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
  factory->registerType("VERTEX_TIMEDIFF", std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());
  factory->registerType("EDGE_TIME_OPTIMAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
  factory->registerType("EDGE_SHORTEST_PATH", std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
  factory->registerType("EDGE_VELOCITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
  factory->registerType("EDGE_ACCELERATION", std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
  factory->registerType("EDGE_ACCELERATION_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
  factory->registerType("EDGE_ACCELERATION_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
  factory->registerType("EDGE_KINEMATICS_CARLIKE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());
  factory->registerType("EDGE_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
  factory->registerType("EDGE_INFLATED_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
  factory->registerType("EDGE_STATION_ENVELOPE", std::make_shared<g2o::HyperGraphElementCreator<EdgeStationEnvelope>>());
  factory->registerType("EDGE_VIA_POINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
  factory->registerType("EDGE_PREFER_ROTDIR", std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
std::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static std::once_flag flag;
  std::call_once(flag, [this](){this->registerG2OTypes();});

  // allocating the optimizer
  std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
  auto linearSolver = std::make_unique<TEBLinearSolver>(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  auto blockSolver = std::make_unique<TEBBlockSolver>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading(); // required for >Eigen 3.1

  return optimizer;
}


bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false)
    return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;
  const PoseSE2 requested_goal = teb_.BackPose();

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  for(int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize &&
        !(cfg_->robot.max_vel_y == 0 && cfg_->robot.min_turning_radius == 0))
    {
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    }

    success = buildGraph(weight_multiplier);
    if (!success)
    {
        clearGraph();
        return false;
    }
    success = (cfg_->robot.max_vel_y == 0 && cfg_->robot.min_turning_radius == 0)
        ? optimizeControlTrajectory(iterations_innerloop, requested_goal)
        : optimizeGraph(iterations_innerloop, false);
    if (!success)
    {
        clearGraph();
        return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);

    clearGraph();

    if (cfg_->robot.max_vel_y == 0 && cfg_->robot.min_turning_radius == 0 &&
        std::chrono::steady_clock::now() >= control_deadline_) break;
    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::msg::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::msg::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
{
  TEB_ASSERT_MSG(initialized_, "Call initialize() first.");
  control_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(cfg_->trajectory.dt_ref));
  const bool differential = cfg_->robot.max_vel_y == 0 && cfg_->robot.min_turning_radius == 0;
  PoseSE2 start(initial_plan.front().pose);
  PoseSE2 goal(initial_plan.back().pose);
  if (differential)
    goal.theta() = start.theta() + g2o::normalize_theta(goal.theta() - start.theta());
  else {
    goal.theta() = start.theta();
    for (std::size_t i = 1; i < initial_plan.size(); ++i)
      goal.theta() += g2o::normalize_theta(PoseSE2(initial_plan[i].pose).theta() -
                                         PoseSE2(initial_plan[i-1].pose).theta());
  }
  // Keep the chosen angular branch while replanning the same goal.  Near the
  // antipodal heading, independently wrapping every update can alternate
  // between +pi and -pi and reverse the command before either turn completes.
  const bool warm_start = teb_.isInit() &&
      (goal.position()-teb_.BackPose().position()).norm() <
          cfg_->trajectory.force_reinit_new_goal_dist &&
      std::abs(g2o::normalize_theta(goal.theta()-teb_.BackPose().theta())) <
          cfg_->trajectory.force_reinit_new_goal_angular;
  if (warm_start) {
    if (differential)
      start.theta() = teb_.Pose(0).theta() +
          g2o::normalize_theta(start.theta()-teb_.Pose(0).theta());
    goal.theta() = teb_.BackPose().theta() +
        g2o::normalize_theta(goal.theta()-teb_.BackPose().theta());
  }
  // The message represents start yaw modulo 2pi. Express the retained terminal
  // branch in that same starting representation when constructing a cold seed.
  const boost::optional<double> seed_terminal_yaw = differential && warm_start
      ? boost::optional<double>(goal.theta()-start.theta()+
          PoseSE2(initial_plan.front().pose).theta()) : boost::none;
  const auto subdivide = [&]() {
    // Subdivide without changing geometry so bounded controls retain the endpoint.
    const double max_interval = M_PI/(2*std::max(1e-6,cfg_->robot.max_vel_theta));
    for (int i = 0; i < teb_.sizeTimeDiffs(); ++i)
    {
      const PoseSE2 from=teb_.Pose(i), to=teb_.Pose(i+1);
      const double turn=g2o::normalize_theta(to.theta()-from.theta());
      const double distance=from.longitudinalDistanceTo(to,true);
      const double duration=std::max(std::abs(distance)/std::max(1e-6,distance<0?cfg_->robot.max_vel_x_backwards:cfg_->robot.max_vel_x),
                                     std::abs(turn)/std::max(1e-6,cfg_->robot.max_vel_theta));
      if (duration > max_interval || std::abs(turn) > M_PI/2)
      {
        const double quarter=turn*.25;
        const double sinc=std::abs(quarter)<1e-8?1-quarter*quarter/6:std::sin(quarter)/quarter;
        const PoseSE2 midpoint(from.x()+distance*.5*sinc*std::cos(from.theta()+quarter),
                              from.y()+distance*.5*sinc*std::sin(from.theta()+quarter),from.theta()+turn*.5);
        const double half_dt=.5*teb_.TimeDiff(i);
        teb_.TimeDiff(i)=half_dt;
        teb_.insertPose(i+1,midpoint);
        teb_.insertTimeDiff(i+1,half_dt);
        --i;
      }
    }
    for (int i = 0; i < teb_.sizePoses(); ++i)
      teb_.setPoseVertexFixed(i, i == 0);
  };
  const auto initialize = [&]() {
    teb_.clearTimedElasticBand();
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
        cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples,
        cfg_->trajectory.allow_init_with_backwards_motion, differential, seed_terminal_yaw);
    if (differential) {
      // The executable seed owns the winding: translated path headings are
      // tangents, whereas explicit in-place turns remain in the seed. Summing
      // the raw guide headings can require a full revolution absent from the
      // actual motion and make an otherwise feasible seed fail endpoint equality.
      goal.theta() = teb_.Pose(0).theta();
      for (int i = 1; i < teb_.sizePoses(); ++i)
        goal.theta() += g2o::normalize_theta(teb_.Pose(i).theta() - teb_.Pose(i-1).theta());
    }
    teb_.BackPose().theta() = goal.theta();
    if (!differential) return;
    subdivide();
  };
  if (warm_start) {
    // The shifted band is only a guess. The control rollout must recover an
    // executable trajectory and the requested endpoint before it can be used.
    teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples,
        differential ? cfg_->robot.max_vel_x : 0, differential ? cfg_->robot.max_vel_theta : 0);
    if (differential) subdivide();
  } else
    initialize();
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)

  if (optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations))
    return true;
  if (!warm_start) return false;
  initialize();
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


//bool TebOptimalPlanner::plan(const tf2::Transform& start, const tf2::Transform& goal, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
//{
//  PoseSE2 start_(start);
//  PoseSE2 goal_(goal);
//  return plan(start_, goal_, start_vel);
//}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
{
  std::vector<geometry_msgs::msg::PoseStamped> initial_plan(2);
  initial_plan.front().pose.position.x = start.x();
  initial_plan.front().pose.position.y = start.y();
  initial_plan.front().pose.orientation = yawQuaternion(start.theta());
  initial_plan.back().pose.position.x = goal.x();
  initial_plan.back().pose.position.y = goal.y();
  initial_plan.back().pose.orientation = yawQuaternion(goal.theta());
  return plan(initial_plan, start_vel, free_goal_vel);
}


bool TebOptimalPlanner::buildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();

  AddEdgesViaPoints();
  AddEdgesStationEnvelope();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  AddEdgesShortestPath();

  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  AddEdgesPreferRotDir();

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0)
    AddEdgesVelocityObstacleRatio();

  return true;
}


namespace {
// One rollout is the shared source of poses for all existing TEB residuals.
struct ControlTrajectoryResidual {
  TimedElasticBand& band;
  const TebConfig& cfg;
  PoseSE2 start, goal;
  std::vector<g2o::OptimizableGraph::Edge*> edges;
  std::vector<Eigen::MatrixXd> roots;
  int dimension = 3;
  double best_cost=HUGE_VAL;
  std::vector<double> best_controls;
  // Endpoint equality alone cannot distinguish a direct heading correction
  // from an unnecessary full turn followed by the opposite full turn. Keep
  // optimization in the seed trajectory's angular winding class.
  double maximum_absolute_turn=HUGE_VAL;
  nlopt_opt optimizer = nullptr;
  std::chrono::steady_clock::time_point deadline;
  bool timedOut() const {
    if (std::chrono::steady_clock::now() < deadline) return false;
    if (optimizer) nlopt_force_stop(optimizer);
    return true;
  }

  ControlTrajectoryResidual(TimedElasticBand& b, const TebConfig& c,
                            const g2o::SparseOptimizer& graph, const PoseSE2& target)
      : band(b), cfg(c), start(b.Pose(0)), goal(target) {
    for (auto* item : graph.edges()) {
      auto* edge = static_cast<g2o::OptimizableGraph::Edge*>(item);
      const int n = edge->dimension();
      Eigen::Map<const Eigen::MatrixXd> information(edge->informationData(), n, n);
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(information);
      roots.push_back(eigen.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal()
                      * eigen.eigenvectors().transpose());
      edges.push_back(edge);
      dimension += n;
    }
  }

  bool operator()(double const* const* parameters, double* residuals) const {
    const double* u = parameters[0];
    band.Pose(0) = start;
    for (int i = 0; i < band.sizeTimeDiffs(); ++i) {
      const double v = u[3*i], w = u[3*i+1], dt = u[3*i+2];
      if (!std::isfinite(v+w+dt) || dt <= 0) return false;
      const double half = w * dt * .5;
      const double sinc = std::abs(half) < 1e-8 ? 1-half*half/6 : std::sin(half)/half;
      const auto& previous = band.Pose(i);
      band.Pose(i+1) = PoseSE2(previous.x()+v*dt*sinc*std::cos(previous.theta()+half),
                              previous.y()+v*dt*sinc*std::sin(previous.theta()+half),
                              previous.theta()+2*half);
      band.TimeDiff(i) = dt;
    }
    int offset = 0;
    for (std::size_t i = 0; i < edges.size(); ++i) {
      edges[i]->computeError();
      const int n = edges[i]->dimension();
      Eigen::Map<Eigen::VectorXd> output(residuals+offset,n);
      output = roots[i] * Eigen::Map<const Eigen::VectorXd>(edges[i]->errorData(),n);
      if (!output.allFinite()) return false;
      offset += n;
    }
    const auto error = band.BackPose().position()-goal.position();
    residuals[offset] = error.x();
    residuals[offset+1] = error.y();
    residuals[offset+2] = band.BackPose().theta()-goal.theta();
    return true;
  }
  Eigen::MatrixXd rolloutJacobian(const double* u) const {
    const int count=band.sizeTimeDiffs();
    Eigen::MatrixXd jac=Eigen::MatrixXd::Zero(3*(count+1),3*count);
    for(int i=0;i<count;++i) {
      const double v=u[3*i],w=u[3*i+1],dt=u[3*i+2],h=w*dt*.5;
      const double sinc=std::abs(h)<1e-8?1-h*h/6:std::sin(h)/h;
      const double dsinc=std::abs(h)<1e-8?-h/3:(h*std::cos(h)-std::sin(h))/(h*h);
      const double angle=band.Pose(i).theta()+h,c=std::cos(angle),s=std::sin(angle);
      const double dx=v*dt*sinc*c,dy=v*dt*sinc*s;
      jac.block(3*(i+1),0,3,3*count)=jac.block(3*i,0,3,3*count);
      jac.row(3*(i+1))-=dy*jac.row(3*i+2);
      jac.row(3*(i+1)+1)+=dx*jac.row(3*i+2);
      jac(3*(i+1),3*i)+=dt*sinc*c;
      jac(3*(i+1)+1,3*i)+=dt*sinc*s;
      jac(3*(i+1),3*i+1)+=v*dt*dt*.5*(dsinc*c-sinc*s);
      jac(3*(i+1)+1,3*i+1)+=v*dt*dt*.5*(dsinc*s+sinc*c);
      jac(3*(i+1)+2,3*i+1)+=dt;
      jac(3*(i+1),3*i+2)+=v*(sinc*c+dt*w*.5*(dsinc*c-sinc*s));
      jac(3*(i+1)+1,3*i+2)+=v*(sinc*s+dt*w*.5*(dsinc*s+sinc*c));
      jac(3*(i+1)+2,3*i+2)+=w;
    }
    return jac;
  }

  double absoluteTurn(const double* controls) const {
    double total=0;
    for(int i=0;i<band.sizeTimeDiffs();++i)
      total+=std::abs(controls[3*i+1]*controls[3*i+2]);
    return total;
  }

  bool preservesWindingClass(const double* controls) const {
    return absoluteTurn(controls)<=maximum_absolute_turn+1e-3;
  }

  static double incidentCost(const g2o::HyperGraph::Vertex& vertex) {
    double cost=0;
    for(auto* item:vertex.edges()) {
      auto* edge=static_cast<g2o::OptimizableGraph::Edge*>(item);
      edge->computeError();cost+=.5*edge->chi2();
    }
    return cost;
  }

  static double objective(unsigned n, const double* values, double* gradient, void* data) {
    auto& self=*static_cast<ControlTrajectoryResidual*>(data);
    std::vector<double> errors(self.dimension);
    if(!self(&values,errors.data()))return HUGE_VAL;
    double cost=0;
    for(int i=0;i<self.dimension-3;++i)cost+=.5*errors[i]*errors[i];
    bool feasible=true;
    for(int i=self.dimension-3;i<self.dimension;++i)feasible=feasible&&std::abs(errors[i])<=1e-6;
    if(feasible&&self.preservesWindingClass(values)&&cost<self.best_cost) {
      self.best_cost=cost;
      self.best_controls.assign(values,values+n);
    }
    if(gradient) {
      std::fill(gradient,gradient+n,0.0);
      Eigen::VectorXd pose_gradient=Eigen::VectorXd::Zero(3*self.band.sizePoses());
      for(int i=1;i<self.band.sizePoses();++i) {
        if(self.timedOut())return cost;
        auto& pose=self.band.Pose(i);
        double* coordinates[]={&pose.x(),&pose.y(),&pose.theta()};
        for(int j=0;j<3;++j) {
          const double value=*coordinates[j],h=1e-6*std::max(1.0,std::abs(value));
          *coordinates[j]=value+h;double plus=incidentCost(*self.band.PoseVertex(i));
          *coordinates[j]=value-h;double minus=incidentCost(*self.band.PoseVertex(i));
          *coordinates[j]=value;pose_gradient[3*i+j]=(plus-minus)/(2*h);
        }
      }
      Eigen::Map<Eigen::VectorXd> output(gradient,n);
      output=self.rolloutJacobian(values).transpose()*pose_gradient;
      for(int i=0;i<self.band.sizeTimeDiffs();++i) {
        double& dt=self.band.TimeDiff(i);const double value=dt,h=1e-6*std::max(1.0,value);
        dt=value+h;double plus=incidentCost(*self.band.TimeDiffVertex(i));
        dt=value-h;double minus=incidentCost(*self.band.TimeDiffVertex(i));
        dt=value;output[3*i+2]+=(plus-minus)/(2*h);
      }
    }
    return cost;
  }

  static void endpoint(unsigned m,double* result,unsigned n,const double* values,double* gradient,void* data) {
    auto& self=*static_cast<ControlTrajectoryResidual*>(data);
    // Endpoint derivatives use the analytic motion Jacobian, not graph probes.
    std::vector<double> residual(self.dimension);
    if(!self(&values,residual.data())) {for(unsigned i=0;i<m;++i)result[i]=HUGE_VAL;return;}
    for(unsigned i=0;i<m;++i)result[i]=residual[self.dimension-3+i];
    if(gradient) {
      Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> out(gradient,m,n);
      out=self.rolloutJacobian(values).bottomRows(3);
    }
  }

};
}

bool TebOptimalPlanner::optimizeControlTrajectory(int iterations, const PoseSE2& goal)
{
  const int count = teb_.sizeTimeDiffs();
  if (count < 1) return false;
  std::vector<double> controls(3*count);
  const double max_dt = M_PI / (2*std::max(1e-6,cfg_->robot.max_vel_theta));
  for (int i=0;i<count;++i) {
    double v,vy,w;
    extractVelocity(teb_.Pose(i),teb_.Pose(i+1),teb_.TimeDiff(i),v,vy,w);
    const double scale=std::max(1.0,std::max(std::abs(v)/std::max(1e-6,v<0?cfg_->robot.max_vel_x_backwards:cfg_->robot.max_vel_x),
                                        std::abs(w)/std::max(1e-6,cfg_->robot.max_vel_theta)));
    const double dt=std::max(1e-3,std::min(max_dt,teb_.TimeDiff(i)*scale));
    controls[3*i]=v*teb_.TimeDiff(i)/dt;
    controls[3*i+1]=w*teb_.TimeDiff(i)/dt;
    controls[3*i+2]=dt;
  }
  ControlTrajectoryResidual residual(teb_,*cfg_,*optimizer_,goal);
  const std::vector<double> seed=controls;
  residual.maximum_absolute_turn=residual.absoluteTurn(seed.data());
  std::vector<double> lower(controls.size()),upper(controls.size());
  for(int i=0;i<count;++i) {
    lower[3*i]=-cfg_->robot.max_vel_x_backwards;upper[3*i]=cfg_->robot.max_vel_x;
    lower[3*i+1]=-cfg_->robot.max_vel_theta;upper[3*i+1]=cfg_->robot.max_vel_theta;
    lower[3*i+2]=1e-3;upper[3*i+2]=max_dt;
  }
  nlopt_opt optimizer=nlopt_create(NLOPT_LD_SLSQP,controls.size());
  if(!optimizer)return false;
  residual.optimizer=optimizer;
  residual.deadline=control_deadline_;
  nlopt_set_lower_bounds(optimizer,lower.data());
  nlopt_set_upper_bounds(optimizer,upper.data());
  nlopt_set_min_objective(optimizer,ControlTrajectoryResidual::objective,&residual);
  const double tolerance[3]={1e-6,1e-6,1e-6};
  nlopt_add_equality_mconstraint(optimizer,3,ControlTrajectoryResidual::endpoint,&residual,tolerance);
  // Preferred obstacle spacing stays in TEB's objective. Collision feasibility
  // is checked against the costmap footprint along the resulting motion.
  nlopt_set_maxeval(optimizer,std::max(1,iterations)*count*10);
  const double remaining=std::chrono::duration<double>(control_deadline_-std::chrono::steady_clock::now()).count();
  nlopt_set_maxtime(optimizer,std::max(1e-6,remaining));
  double cost=0;
  const nlopt_result solve_result=remaining>0 ? nlopt_optimize(optimizer,controls.data(),&cost) : NLOPT_MAXTIME_REACHED;
  RCLCPP_DEBUG(node_->get_logger(), "Control solve status=%d evaluations=%d objective=%g best_feasible=%g",int(solve_result),nlopt_get_numevals(optimizer),cost,residual.best_cost);
  nlopt_destroy(optimizer);
  auto restore=[&](const std::vector<double>& candidate) {
    for(std::size_t i=0;i<candidate.size();++i)
      if(!std::isfinite(candidate[i]) || candidate[i]<lower[i]-1e-9 || candidate[i]>upper[i]+1e-9)return false;
    std::vector<double> errors(residual.dimension);
    const double* values=candidate.data();
    if(!residual.preservesWindingClass(values))return false;
    if(!residual(&values,errors.data()))return false;
    for(int i=residual.dimension-3;i<residual.dimension;++i)
      if(std::abs(errors[i])>1e-6)return false;
    return true;
  };
  // A time-limited solve may stop outside the equality manifold. Retain the
  // feasible seed rather than publishing a trajectory to a different endpoint.
  if(!residual.best_controls.empty() && restore(residual.best_controls))return true;
  if(restore(controls))return true;
  RCLCPP_DEBUG(node_->get_logger(), "Control trajectory solver status=%d; retaining feasible initial trajectory",int(solve_result));
  return restore(seed);
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;
  }

  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;
  }

  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
    RCLCPP_ERROR(node_->get_logger(), "optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();

  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for(auto& v : vertices)
      v.second->edges().clear();

    optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  RCLCPP_DEBUG_EXPRESSION(node_->get_logger(), cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}


void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!


  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  auto create_edge = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle) {
    if (inflated)
    {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };

  // iterate all teb points, skipping the last and, if the EdgeVelocityObstacleRatio edges should not be created, the first one too
  const int first_vertex = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
  for (int i = first_vertex; i < teb_.sizePoses() - 1; ++i)
  {
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      ObstaclePtr left_obstacle;
      ObstaclePtr right_obstacle;

      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

      // iterate obstacles
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
          continue;

          // calculate distance to robot model
          double dist = cfg_->robot_model->calculateDistance(teb_.Pose(i), obst.get());

          // force considering obstacle if really close to the current pose
        if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
          {
              iter_obstacle->push_back(obst);
              continue;
          }
          // cut-off distance
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
            continue;

          // determine side (left or right) and assign obstacle if closer than the previous one
          if (cross2d(pose_orient, obst->getCentroid() - teb_.Pose(i).position()) > 0) // left
          {
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst;
              }
          }
          else
          {
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst;
              }
          }
      }

      if (left_obstacle)
        iter_obstacle->push_back(left_obstacle);
      if (right_obstacle)
        iter_obstacle->push_back(right_obstacle);

      // continue here to ignore obstacles for the first pose, but use them later to create the EdgeVelocityObstacleRatio edges
      if (i == 0)
      {
        ++iter_obstacle;
        continue;
      }

      // create obstacle edges
      for (const ObstaclePtr obst : *iter_obstacle)
        create_edge(i, obst.get());
      ++iter_obstacle;
  }
}


void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue;

    int index;

    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));


    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue;

    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    }

  }
}


void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;

  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;

  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {

    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points

    // check if point conicides with goal or is located behind it
    if ( index > n-2 )
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        RCLCPP_DEBUG(node_->get_logger(), "TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);

    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);
  }
}

void TebOptimalPlanner::AddEdgesStationEnvelope()
{
  if (!station_envelope_.enabled) return;
  Eigen::Matrix<double,1,1> information;
  information.fill(10000.0);
  for (int i = 0; i < teb_.sizePoses(); ++i) {
    if (teb_.PoseVertex(i)->fixed()) continue;
    auto* edge = new EdgeStationEnvelope;
    edge->setVertex(0, teb_.PoseVertex(i));
    edge->setInformation(information);
    edge->setEnvelope(station_envelope_);
    optimizer_->addEdge(edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }

  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0)
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();

  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}



void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  double forward_drive_weight = cfg_->optim.weight_kinematics_forward_drive;
  if (forward_drive_weight > 1.0 && teb_.sizePoses() > 1)
  {
    const Eigen::Vector2d goal_delta =
        teb_.BackPose().position() - teb_.Pose(0).position();
    if (goal_delta.norm() > 1e-6)
    {
      const Eigen::Vector2d heading(
          std::cos(teb_.Pose(0).theta()), std::sin(teb_.Pose(0).theta()));
      const double alignment = std::max(
          -1.0, std::min(1.0, goal_delta.dot(heading) / goal_delta.norm()));
      const double forward_blend = 0.5 * (1.0 + std::tanh(8.0 * alignment));
      forward_drive_weight =
          1.0 + (forward_drive_weight - 1.0) * forward_blend;
    }
  }
  information_kinematics(1, 1) = forward_drive_weight;

  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));
    kinematics_edge->setVertex(2,teb_.TimeDiffVertex(i));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;

  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}


void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    RCLCPP_WARN(node_->get_logger(), "TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);

  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));
    rotdir_edge->setInformation(information_rotdir);

    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();

    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio()
{
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1,1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0,1) = information(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.sizePoses() - 1; ++index)
  {
    for (const ObstaclePtr obstacle : (*iter_obstacle++))
    {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0,teb_.PoseVertex(index));
      edge->setVertex(1,teb_.PoseVertex(index + 1));
      edge->setVertex(2,teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(*cfg_, cfg_->robot_model.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

bool TebOptimalPlanner::hasDiverged() const
{
  // Early returns if divergence detection is not active
  if (!cfg_->recovery.divergence_detection_enable)
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
    return false;

  // Grab the statistics of the final iteration
  const auto last_iter_stats = stats_vector.back();

  return last_iter_stats.chi2 > cfg_->recovery.divergence_detection_max_chi_squared;
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag)
    clearGraph();
}


void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();

  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    vx = pose1.longitudinalDistanceTo(pose2, cfg_->useExactArcLength()) / dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;
  }

  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("teb_local_planner"), "TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  const bool differential = cfg_->robot.max_vel_y == 0 && cfg_->robot.min_turning_radius == 0;
  const int max_look_ahead_poses =
      std::max(1, teb_.sizePoses() - 1 - cfg_->trajectory.prevent_look_ahead_poses_near_goal);
  const double look_ahead_time = cfg_->trajectory.dt_ref * std::max(1, look_ahead_poses);
  if (differential)
  {
    vx = vy = omega = 0;
    if (!std::isfinite(look_ahead_time) || look_ahead_time <= 0)
      return false;
    double duration = 0, surge_integral = 0, yaw_integral = 0;
    int surge_sign = 0, yaw_sign = 0;
    // Numerical zero, consistent with the control-trajectory bound tolerance.
    const auto sign = [](double value) { return (value > 1e-9) - (value < -1e-9); };
    for (int i = 0; i < max_look_ahead_poses && duration < look_ahead_time; ++i)
    {
      const double primitive_dt = teb_.TimeDiff(i);
      if (!std::isfinite(primitive_dt) || primitive_dt <= 0)
        return false;
      double surge, sway, yaw;
      extractVelocity(teb_.Pose(i), teb_.Pose(i + 1), primitive_dt, surge, sway, yaw);
      if (!std::isfinite(surge) || !std::isfinite(yaw))
        return false;
      const int next_surge_sign = sign(surge), next_yaw_sign = sign(yaw);
      // Preserve reversals and a rotation following translation (including the
      // terminal turn). Explicit route turns already bound the transformed plan.
      if ((surge_sign && next_surge_sign != surge_sign) ||
          (yaw_sign && next_yaw_sign && next_yaw_sign != yaw_sign))
        break;
      if (next_surge_sign) surge_sign = next_surge_sign;
      if (next_yaw_sign) yaw_sign = next_yaw_sign;
      const double used_dt = std::min(primitive_dt, look_ahead_time - duration);
      surge_integral += surge * used_dt;
      yaw_integral += yaw * used_dt;
      duration += used_dt;
    }
    if (duration <= 0)
      return false;
    // Preview controls, not endpoint displacement: a short alignment pivot can
    // flow into translation. The ROS owner still checks the shaped command arc.
    vx = surge_integral / duration;
    omega = yaw_integral / duration;
    return true;
  }
  look_ahead_poses = 1;
  double dt = 0.0;
  for(int counter = 0; counter < max_look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    look_ahead_poses = counter + 1;
    if(dt >= look_ahead_time)
    {
        break;
    }
  }
  if (dt<=0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("teb_local_planner"), "TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::msg::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;

  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }

  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<teb_msgs::msg::TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();

  trajectory.resize(n);

  if (n == 0)
    return;

  double curr_time = 0;

  // start
  teb_msgs::msg::TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start = durationFromSec(curr_time);

  curr_time += teb_.TimeDiff(0);

  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    teb_msgs::msg::TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);
    point.time_from_start = durationFromSec(curr_time);

    curr_time += teb_.TimeDiff(i);
  }

  // goal
  teb_msgs::msg::TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start = durationFromSec(curr_time);
}


bool TebOptimalPlanner::isTrajectoryFeasible(SweptFootprint& collision, int look_ahead_idx,
                                             double feasibility_check_lookahead_distance)
{
  if (teb().sizePoses() == 0 || !collision.begin(teb().Pose(0).x(), teb().Pose(0).y(), teb().Pose(0).theta()))
    return false;
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;
  if (feasibility_check_lookahead_distance > 0)
    for (int i = 1; i < teb().sizePoses(); ++i)
      if ((teb().Pose(i).position() - teb().Pose(0).position()).norm() > feasibility_check_lookahead_distance) {
        look_ahead_idx = i;
        break;
      }
  const bool differential = cfg_->robot.max_vel_y == 0 && cfg_->robot.min_turning_radius == 0;
  for (int i = 0; i < look_ahead_idx; ++i) {
    const double dt = teb().TimeDiff(i);
    if (!std::isfinite(dt) || dt <= 0) return false;
    const auto& from = teb().Pose(i);
    const auto& to = teb().Pose(i + 1);
    const double omega = g2o::normalize_theta(to.theta() - from.theta()) / dt;
    const double vx = differential ? from.longitudinalDistanceTo(to, true) / dt : (to.x() - from.x()) / dt;
    const double vy = differential ? 0.0 : (to.y() - from.y()) / dt;
    if (!collision.advance(vx, vy, omega, dt, !differential)) {
      if (visualization_)
        visualization_->publishInfeasibleRobotPose(to, *cfg_->robot_model);
      return false;
    }
  }
  return true;
}



} // namespace teb_local_planner
