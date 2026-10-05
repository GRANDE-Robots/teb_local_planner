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

#include "teb_local_planner/teb_local_planner_ros.h"

//#include <tf_conversions/tf_eigen.h>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <string>

// pluginlib macros
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


namespace teb_local_planner
{


namespace {
int firstIntermediateRotationEnd(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  for (std::size_t index = 1; index + 1 < plan.size(); ++index)
  {
    const auto& a = plan[index - 1].pose;
    const auto& b = plan[index].pose;
    if (std::hypot(b.position.x - a.position.x, b.position.y - a.position.y) <= 1e-6 &&
        std::abs(g2o::normalize_theta(tf2::getYaw(b.orientation) - tf2::getYaw(a.orientation))) > 1e-6)
      return static_cast<int>(index);
  }
  return -1;
}
geometry_msgs::msg::Quaternion yawQuaternion(double yaw) {
  tf2::Quaternion q; q.setRPY(0,0,yaw); return tf2::toMsg(q);
}
}

TebLocalPlannerROS::TebLocalPlannerROS()
    : costmap_ros_(nullptr), tf_(nullptr), cfg_(new TebConfig()), intra_proc_node_(nullptr),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           custom_via_points_active_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::initialize(Nav2LifecycleNode::SharedPtr node)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {
    // The converter owns a private node. Forward the plugin's parameter block
    // from the controller YAML and keep the private node on the same clock.
    std::vector<rclcpp::Parameter> converter_parameter_overrides;
    const auto plugin_parameter_prefix = name_ + ".costmap_converter.";
    const auto& resolved_parameter_overrides =
      node->get_node_parameters_interface()->get_parameter_overrides();
    for (const auto& [parameter_name, value] : resolved_parameter_overrides) {
      if (parameter_name.compare(0, plugin_parameter_prefix.size(), plugin_parameter_prefix) == 0) {
        converter_parameter_overrides.emplace_back(
          parameter_name.substr(plugin_parameter_prefix.size()), value);
      }
    }
    bool use_sim_time = false;
    node->get_parameter_or("use_sim_time", use_sim_time, false);
    converter_parameter_overrides.emplace_back("use_sim_time", use_sim_time);
    rclcpp::NodeOptions converter_node_options;
    converter_node_options.parameter_overrides(converter_parameter_overrides);
    intra_proc_node_.reset(
      new rclcpp::Node("costmap_converter", node->get_namespace(), converter_node_options));
    cfg_->declareParameters(node, name_);

    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_->loadRosParamFromNodeHandle(node, name_);

    // reserve some memory for obstacles
    obstacles_.reserve(500);

    // create the planner instance
    if (cfg_->hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(node, *cfg_.get(), &obstacles_, visualization_, &via_points_));
      RCLCPP_INFO(logger_, "Parallel planning in distinctive topologies disabled.");
    }

    // init other variables
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.


    cfg_->map_frame = costmap_ros_->getGlobalFrameID(); // TODO

    //Initialize a costmap to polygon converter
    if (!cfg_->obstacles.costmap_converter_plugin.empty())
    {
      if (!cfg_->obstacles.costmap_converter_spin_thread) {
        throw nav2_core::ControllerException(
          "costmap_converter_spin_thread=false is unsupported because the converter node has no external executor");
      }
      try
      {
        costmap_converter_ = costmap_converter_loader_.createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_->obstacles.costmap_converter_plugin);
        RCLCPP_INFO(logger_, "library path : %s", costmap_converter_loader_.getClassLibraryPath(cfg_->obstacles.costmap_converter_plugin).c_str());
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");

        costmap_converter_->setOdomTopic(cfg_->odom_topic);
        costmap_converter_->setGlobalFrame(costmap_ros_->getGlobalFrameID());
        costmap_converter_->initialize(intra_proc_node_);
        costmap_converter_->setCostmap2D(costmap_);
        const auto rate = std::make_shared<rclcpp::Rate>((double)cfg_->obstacles.costmap_converter_rate);
        costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
        RCLCPP_INFO(logger_, "Costmap conversion plugin %s loaded.", cfg_->obstacles.costmap_converter_plugin.c_str());
      }
      catch(pluginlib::PluginlibException& ex)
      {
        RCLCPP_INFO(logger_,
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else {
      RCLCPP_INFO(logger_, "No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
    }


    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    std::tie(robot_inscribed_radius_, robot_circumscribed_radius_) = nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_);

    // Add callback for dynamic parameters
    dyn_params_handler = node->add_on_set_parameters_callback(
      std::bind(&TebConfig::dynamicParametersCallback, std::ref(cfg_), std::placeholders::_1));

    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_->robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_->obstacles.min_obstacle_dist);

    // setup callback for custom obstacles
    custom_obst_sub_ = node->rclcpp_lifecycle::LifecycleNode::create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
                "obstacles",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customObstacleCB, this, std::placeholders::_1));

    // setup callback for custom via-points
    via_points_sub_ = node->rclcpp_lifecycle::LifecycleNode::create_subscription<nav_msgs::msg::Path>(
                "via_points",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&TebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));

    // initialize failure detector
    //rclcpp::Node::SharedPtr nh_move_base("~");
    double controller_frequency = 5;
    node->get_parameter("controller_frequency", controller_frequency);
    if (!std::isfinite(controller_frequency) || controller_frequency <= 0.0)
      throw nav2_core::ControllerException("controller_frequency must be positive");
    command_period_sec_ = 1.0 / controller_frequency;
    failure_detector_.setBufferLength(std::round(cfg_->recovery.oscillation_filter_duration*controller_frequency));

    // set initialized flag
    initialized_ = true;

    // This should be called since to prevent different time sources exception
    time_last_infeasible_plan_ = clock_->now();
    time_last_oscillation_ = clock_->now();
    RCLCPP_DEBUG(logger_, "teb_local_planner plugin initialized.");
  }
  else
  {
    RCLCPP_INFO(logger_, "teb_local_planner has already been initialized, doing nothing.");
  }
}

void TebLocalPlannerROS::configure(
    const Nav2ParentNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  nh_ = parent;

  auto node = nh_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  name_ = name;

  initialize(node);
  visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
  visualization_->on_configure();
  planner_->setVisualization(visualization_);

  return;
}

void TebLocalPlannerROS::setPlan(const nav_msgs::msg::Path& orig_global_plan)
{
  std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
  // check if plugin is initialized
  if(!initialized_)
  {
    RCLCPP_ERROR(logger_, "teb_local_planner has not been initialized, please call initialize() before using this planner");
    throw nav2_core::ControllerException("TEB is not configured");
  }

  // store the global plan
  global_plan_.clear();
  for (const auto& pose : orig_global_plan.poses) {
    // Remove duplicate route samples before replacing the first with live pose.
    // Otherwise the duplicate becomes an artificial return-to-start target.
    if (!global_plan_.empty() &&
        std::hypot(pose.pose.position.x-global_plan_.back().pose.position.x,
                   pose.pose.position.y-global_plan_.back().pose.position.y)<=1e-6 &&
        std::abs(g2o::normalize_theta(tf2::getYaw(pose.pose.orientation)-
            tf2::getYaw(global_plan_.back().pose.orientation)))<=1e-6)
      global_plan_.back()=pose;
    else global_plan_.push_back(pose);
  }

  if(global_plan_.size()>2 && cfg_->robot.max_vel_y==0 && cfg_->robot.min_turning_radius==0) {
    // Geometric samples are a guide, not mandatory steering manoeuvres.
    // Bound simplification by half a map cell and retain explicit rotations.
    const double tolerance=.5*costmap_ros_->getCostmap()->getResolution();
    std::vector<bool> keep(global_plan_.size(),false), rotation(global_plan_.size(),false);
    keep.front()=keep.back()=true;
    for(std::size_t i=1;i<global_plan_.size();++i)
      if((PoseSE2(global_plan_[i].pose).position()-PoseSE2(global_plan_[i-1].pose).position()).norm()<=1e-6)
        rotation[i-1]=rotation[i]=keep[i-1]=keep[i]=true;
    std::vector<std::pair<std::size_t,std::size_t>> pending;
    std::size_t last=0;
    for(std::size_t i=1;i<keep.size();++i)if(keep[i]){pending.emplace_back(last,i);last=i;}
    while(!pending.empty()) {
      const auto span=pending.back();pending.pop_back();
      const auto a=PoseSE2(global_plan_[span.first].pose).position();
      const auto b=PoseSE2(global_plan_[span.second].pose).position();
      double largest=tolerance;std::size_t split=span.first;
      for(std::size_t i=span.first+1;i<span.second;++i) {
        const double error=distance_point_to_segment_2d(PoseSE2(global_plan_[i].pose).position(),a,b);
        if(error>largest){largest=error;split=i;}
      }
      if(split!=span.first){keep[split]=true;pending.emplace_back(span.first,split);pending.emplace_back(split,span.second);}
    }
    std::vector<geometry_msgs::msg::PoseStamped> reduced;
    for(std::size_t i=0;i<keep.size();++i)if(keep[i]) {
      auto point=global_plan_[i];
      if(i>0 && i+1<keep.size() && !rotation[i]) {
        std::size_t before=i-1,after=i+1;
        while(!keep[before])--before;
        while(!keep[after])++after;
        const auto incoming=(PoseSE2(point.pose).position()-PoseSE2(global_plan_[before].pose).position()).normalized();
        const auto outgoing=(PoseSE2(global_plan_[after].pose).position()-PoseSE2(point.pose).position()).normalized();
        Eigen::Vector2d tangent=incoming+outgoing;
        if(tangent.dot(PoseSE2(point.pose).orientationUnitVec())<0)tangent=-tangent;
        if(tangent.norm()>1e-9)point.pose.orientation=yawQuaternion(std::atan2(tangent.y(),tangent.x()));
      }
      reduced.push_back(point);
    }
    global_plan_.swap(reduced);
  }

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

  // reset goal_reached_ flag
  goal_reached_ = false;
  resetTerminalObservation(true);

  return;
}


geometry_msgs::msg::TwistStamped TebLocalPlannerROS::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
{
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
#if TEB_NAV2_PATH_HANDLER
    double path_length_tolerance = 0.0;
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance, path_length_tolerance)) {
#else
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
#endif
      std::lock_guard<std::mutex> lock(cfg_->configMutex());
      cfg_->goal_tolerance.xy_goal_tolerance = pose_tolerance.position.x;
    }
  }
  geometry_msgs::msg::TwistStamped output;
  std::string reason;
  if (!computeCommand(pose, velocity, output, reason))
    throw nav2_core::NoValidControl(reason);
  return output;
}

#if TEB_NAV2_PATH_HANDLER
geometry_msgs::msg::TwistStamped TebLocalPlannerROS::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker, const nav_msgs::msg::Path& transformed_global_plan,
    const geometry_msgs::msg::PoseStamped& global_goal)
{
  if (goal_checker) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
#if TEB_NAV2_PATH_HANDLER
    double path_length_tolerance = 0.0;
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance, path_length_tolerance)) {
#else
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
#endif
      std::lock_guard<std::mutex> lock(cfg_->configMutex());
      cfg_->goal_tolerance.xy_goal_tolerance = pose_tolerance.position.x;
    }
  }
  geometry_msgs::msg::TwistStamped output;
  std::string reason;
  if (!computeCommand(pose, velocity, output, reason, &transformed_global_plan, &global_goal))
    throw nav2_core::NoValidControl(reason);
  return output;
}
#endif

bool TebLocalPlannerROS::computeCommand(const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity, geometry_msgs::msg::TwistStamped& cmd_vel,
    std::string& message, const nav_msgs::msg::Path* native_plan,
    const geometry_msgs::msg::PoseStamped* native_goal)
{
  std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
  cmd_vel.twist = geometry_msgs::msg::Twist();
  // check if plugin initialized
  if(!initialized_)
  {
    RCLCPP_ERROR(logger_, "teb_local_planner has not been initialized, please call initialize() before using this planner");
    message = "teb_local_planner has not been initialized";
    return false;
  }

  cmd_vel.twist = geometry_msgs::msg::Twist();
  unshaped_command_ = geometry_msgs::msg::Twist();
  command_time_ns_ = static_cast<std::uint64_t>(clock_->now().nanoseconds());
  command_proposal_ns_ = command_time_ns_;
  bool command_accepted = false;
  struct FinishCommand {
    CommandContinuity& continuity;
    geometry_msgs::msg::Twist& output;
    geometry_msgs::msg::Twist& last;
    std::uint64_t& now;
    bool& accepted;
    rclcpp::Clock::SharedPtr clock;
    ~FinishCommand() {
      const std::uint64_t finished = static_cast<std::uint64_t>(clock->now().nanoseconds());
      if (finished < now) accepted = false;
      now = finished;
      if (!accepted) { output = geometry_msgs::msg::Twist(); continuity.reset(); }
      // Scoped commands remain proposals until the wrapper's final identity
      // check. It calls commitCommand once with the actually accepted output.
      last = output;
    }
  } finish_command{command_continuity_, cmd_vel.twist, last_cmd_, command_time_ns_, command_accepted, clock_};

  // Restore the nominal robot limits on every return. The optimizer and final
  // saturation see the same scoped caps; configuration is never left altered.
  struct RestoreRobot {
    TebConfig& cfg;
    TebConfig::Robot saved;
    ~RestoreRobot() { cfg.robot = saved; }
  } restore_robot{*cfg_, cfg_->robot};
  auto* station_planner = dynamic_cast<TebOptimalPlanner*>(planner_.get());
  if (command_continuity_.enabled()) {
    if (cfg_->robot.max_vel_y != 0.0 || cfg_->robot.cmd_angle_instead_rotvel ||
        cfg_->robot.min_turning_radius != 0.0) {
      message = "command_continuity_unsupported";
      return false;
    }
    // The optimizer and command selection share any explicit scoped reverse cap.
    cfg_->robot.max_vel_x_backwards = std::min(cfg_->robot.max_vel_x_backwards, command_reverse_limit_);
  }
  if (station_envelope_.enabled) {
    if (!station_planner || cfg_->robot.max_vel_y != 0.0 ||
        cfg_->robot.cmd_angle_instead_rotvel || cfg_->robot.min_turning_radius != 0.0 ||
        !station_envelope_.valid() || station_envelope_.frame != cfg_->map_frame) {
      cmd_vel.twist = geometry_msgs::msg::Twist();
      message = "station_envelope_unsupported_or_invalid";
      return false;
    }
    cfg_->robot.max_vel_x = std::min(cfg_->robot.max_vel_x, station_envelope_.surge_limit);
    cfg_->robot.max_vel_x_backwards = std::min(cfg_->robot.max_vel_x_backwards, station_envelope_.surge_limit);
    cfg_->robot.use_proportional_saturation = true;
  }

  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;
  resetTerminalObservation(false);

  // Get robot pose
  geometry_msgs::msg::PoseStamped robot_pose = pose;
  if (robot_pose.header.frame_id != cfg_->map_frame ||
      !std::isfinite(robot_pose.pose.position.x) ||
      !std::isfinite(robot_pose.pose.position.y) ||
      !std::isfinite(tf2::getYaw(robot_pose.pose.orientation))) {
    message = "robot_pose_unavailable_or_nonfinite";
    {
      std::lock_guard<std::mutex> lock(terminal_observation_mutex_);
      terminal_observation_.reason = message;
    }
    return false;
  }
  robot_pose_ = PoseSE2(robot_pose.pose);

  // Nav2 supplies the measured body-frame velocity used by this compute cycle.
  if (!std::isfinite(velocity.linear.x + velocity.linear.y + velocity.angular.z)) {
    message = "robot_velocity_nonfinite";
    return false;
  }
  robot_vel_ = velocity;
  if (global_plan_.empty()) {
    message = "global_plan_empty";
    return false;
  }

  // Retain the existing ordered-phase progress on the raw reference. The SDK
  // local path remains the input to optimization on its native controller API.
  pruneGlobalPlan(robot_pose, global_plan_, cfg_->trajectory.global_plan_prune_distance);

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  int goal_idx = 0;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;
  if (native_plan)
  {
    const auto finite_pose = [](const geometry_msgs::msg::PoseStamped& value) {
      const auto& p = value.pose.position;
      const auto& q = value.pose.orientation;
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
          std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
          std::isfinite(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) &&
          std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) <= 1e-6;
    };
    if (!native_goal || native_plan->poses.empty() ||
        native_plan->header.frame_id != cfg_->map_frame ||
        native_goal->header.frame_id != cfg_->map_frame || !finite_pose(*native_goal)) {
      message = "native_path_or_final_goal_unavailable";
      return false;
    }
    for (const auto& point : native_plan->poses)
      if (point.header.frame_id != cfg_->map_frame || !finite_pose(point)) {
        message = "native_path_frame_or_pose_invalid";
        return false;
      }
    // Lyrical's path handler owns the supplied local path. Retain the full raw
    // reference only for ordered phases and the separate final-goal predicate.
    try {
      tf_plan_to_global = tf_->lookupTransform(cfg_->map_frame,
          global_plan_.front().header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException&) {
      message = "native_reference_transform_unavailable";
      return false;
    }
    geometry_msgs::msg::PoseStamped expected_goal;
    tf2::doTransform(global_plan_.back(), expected_goal, tf_plan_to_global);
    if (std::hypot(expected_goal.pose.position.x-native_goal->pose.position.x,
                   expected_goal.pose.position.y-native_goal->pose.position.y) > 1e-6 ||
        std::abs(g2o::normalize_theta(tf2::getYaw(expected_goal.pose.orientation)-
                 tf2::getYaw(native_goal->pose.orientation))) > 1e-6) {
      message = "native_final_goal_does_not_match_reference";
      return false;
    }
    // Apply TEB's maintained costmap and lookahead bounds to the supplied SDK
    // path. Path-handler defaults must not silently expand this horizon.
    if (!transformGlobalPlan(native_plan->poses, robot_pose, *costmap_,
            cfg_->map_frame, cfg_->trajectory.max_global_plan_lookahead_dist,
            transformed_plan, nullptr, nullptr)) {
      message = "native_local_path_transform_failed";
      return false;
    }
    double closest = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < global_plan_.size(); ++index) {
      geometry_msgs::msg::PoseStamped point;
      tf2::doTransform(global_plan_[index], point, tf_plan_to_global);
      const double distance = std::hypot(point.pose.position.x-transformed_plan.back().pose.position.x,
                                         point.pose.position.y-transformed_plan.back().pose.position.y);
      if (distance < closest) { closest = distance; goal_idx = static_cast<int>(index); }
    }
    const int phase = command_continuity_.enabled() && !station_envelope_.enabled ?
        firstIntermediateRotationEnd(global_plan_) : -1;
    if (phase >= 0 && goal_idx >= phase) {
      geometry_msgs::msg::PoseStamped endpoint;
      tf2::doTransform(global_plan_[phase], endpoint, tf_plan_to_global);
      auto found = std::find_if(transformed_plan.begin(), transformed_plan.end(),
          [&endpoint](const auto& point) {
            return std::hypot(point.pose.position.x-endpoint.pose.position.x,
                               point.pose.position.y-endpoint.pose.position.y) <= 1e-6 &&
                std::abs(g2o::normalize_theta(tf2::getYaw(point.pose.orientation)-
                    tf2::getYaw(endpoint.pose.orientation))) <= 1e-6;
          });
      if (found == transformed_plan.end()) {
        message = "native_path_skips_ordered_rotation";
        return false;
      }
      transformed_plan.erase(std::next(found), transformed_plan.end());
      goal_idx = phase;
    }
  }
  else if (!transformGlobalPlan( global_plan_, robot_pose, *costmap_, cfg_->map_frame, cfg_->trajectory.max_global_plan_lookahead_dist,
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    RCLCPP_WARN(logger_, "Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    {
      std::lock_guard<std::mutex> lock(terminal_observation_mutex_);
      terminal_observation_.reason = "global_plan_transform_failed";
    }
    return false;
  }

  const int pending_rotation = command_continuity_.enabled() && !station_envelope_.enabled ?
      firstIntermediateRotationEnd(global_plan_) : -1;
  const bool phase_endpoint_selected = pending_rotation >= 0 && goal_idx == pending_rotation;

  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);

  nav_msgs::msg::Odometry base_odom;
  base_odom.child_frame_id = costmap_ros_->getBaseFrameID();
  base_odom.twist.twist = velocity;

  // check if global goal is reached
  geometry_msgs::msg::PoseStamped global_goal;
  if (native_goal) global_goal = *native_goal;
  else tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();
  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  TerminalObservation::Inputs terminal_inputs;
  terminal_inputs.pose_frame = robot_pose.header.frame_id;
  terminal_inputs.goal_frame = global_goal.header.frame_id;
  terminal_inputs.velocity_frame = base_odom.child_frame_id;
  terminal_inputs.pose_stamp_ns = static_cast<std::uint64_t>(rclcpp::Time(robot_pose.header.stamp).nanoseconds());
  terminal_inputs.goal_stamp_ns = static_cast<std::uint64_t>(rclcpp::Time(global_goal.header.stamp).nanoseconds());
  terminal_inputs.pose = {{robot_pose_.x(), robot_pose_.y(), robot_pose_.theta()}};
  terminal_inputs.goal = {{global_goal.pose.position.x, global_goal.pose.position.y,
                           tf2::getYaw(global_goal.pose.orientation)}};
  terminal_inputs.velocity = {{base_odom.twist.twist.linear.x,
                               base_odom.twist.twist.linear.y,
                               base_odom.twist.twist.angular.z}};
  terminal_inputs.position_error = std::sqrt(dx * dx + dy * dy);
  terminal_inputs.yaw_error = delta_orient;
  terminal_inputs.position_tolerance = cfg_->goal_tolerance.xy_goal_tolerance;
  terminal_inputs.yaw_tolerance =
      command_continuity_.enabled() && command_terminal_yaw_tolerance_ >= 0.0
          ? command_terminal_yaw_tolerance_
          : cfg_->goal_tolerance.yaw_goal_tolerance;
  terminal_inputs.yaw_optional = command_continuity_.enabled() &&
                                 command_terminal_yaw_optional_;
  terminal_inputs.route_phase_pending = pending_rotation >= 0;
  terminal_inputs.translational_stopped_limit = cfg_->goal_tolerance.trans_stopped_vel;
  terminal_inputs.rotational_stopped_limit = cfg_->goal_tolerance.theta_stopped_vel;
  // MARINER owns ordered route progress. Preserve explicit intermediate turns,
  // but always snapshot the same-cycle terminal state and expose phase status as
  // one of the evaluated conditions instead of dropping the observation.
  terminal_inputs.complete_global_plan =
      cfg_->goal_tolerance.complete_global_plan && !command_continuity_.enabled();
  terminal_inputs.free_goal_velocity = cfg_->goal_tolerance.free_goal_vel;
  if (terminal_inputs.complete_global_plan) {
    std::lock_guard<std::mutex> lock(via_point_mutex_);
    terminal_inputs.via_count = via_points_.size();
  }
  TerminalObservation::Epoch terminal_epoch;
  {
    std::lock_guard<std::mutex> lock(terminal_observation_mutex_);
    terminal_epoch = terminal_observation_.epoch;
  }
  terminal_epoch.time_ns = static_cast<std::uint64_t>(clock_->now().nanoseconds());
  const auto evaluated_terminal_observation = TerminalObservation::evaluate(
      terminal_epoch, terminal_inputs, (std::hypot(velocity.linear.x, velocity.linear.y) <= cfg_->goal_tolerance.trans_stopped_vel &&
       std::abs(velocity.angular.z) <= cfg_->goal_tolerance.theta_stopped_vel));
  bool observation_committed = false;
  {
    std::lock_guard<std::mutex> lock(terminal_observation_mutex_);
    // A newer compute cycle or plan reset owns the slot; an older calculation
    // must not overwrite its terminal snapshot.
    if (terminal_observation_.epoch.plan_sequence == terminal_epoch.plan_sequence &&
        terminal_observation_.epoch.compute_sequence == terminal_epoch.compute_sequence) {
      terminal_observation_ = evaluated_terminal_observation;
      observation_committed = true;
    }
  }
  if (observation_committed && evaluated_terminal_observation.reached) {
    goal_reached_ = true;
    if (!persistent_goal_) {
      command_accepted = true;
      return true;
    }
  }

  // Keep a selected phase endpoint fixed while retaining other recovery behavior.
  configureBackupModes(transformed_plan, goal_idx, phase_endpoint_selected);


  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    RCLCPP_WARN(logger_, "Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return false;
  }

  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  // Overwrite goal orientation if needed
  if (cfg_->trajectory.global_plan_overwrite_orientation && !phase_endpoint_selected)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::msg::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start

  // clear currently existing obstacles
  obstacles_.clear();

  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();

  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();


  // Do not allow config changes during the following optimization step


  // Now perform the actual planning
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_->goal_tolerance.free_goal_vel); // straight line init
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    RCLCPP_WARN(logger_, "teb_local_planner was not able to obtain a local plan for the current setting.");

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner was not able to obtain a local plan";
    return false;
  }

  // Check for divergence
  if (planner_->hasDiverged())
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // Reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");
    message = "teb_local_planner trajectory has diverged";

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    return false;
  }

  // Check feasibility (but within the first few states only)
  if(cfg_->robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    std::tie(robot_inscribed_radius_, robot_circumscribed_radius_) = nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_);
  }

  // A short contact-recovery command can deviate from the certified global
  // route, so unknown space remains occupied here regardless of costmap
  // display or rolling-window parameters.
  SweptFootprint collision(*costmap_, costmap_ros_->getUnpaddedRobotFootprint(), footprint_spec_, false);
  const bool feasible = planner_->isTrajectoryFeasible(collision, cfg_->trajectory.feasibility_check_no_poses, cfg_->trajectory.feasibility_check_lookahead_distance);
  const std::string trajectory_reason =
      collision.reason().empty() ? "invalid_plan_geometry" : collision.reason();
  // Near a known obstacle, the complete optimized band may contain a later
  // padded-hull intersection even though its first command safely separates.
  // For the surge/yaw model, validate that short command exactly and replan
  // after executing only it. Physical-hull and unknown-space intersections
  // remain unrecoverable here.
  const bool contact_model_supported =
      cfg_->robot.max_vel_y == 0.0 && cfg_->robot.min_turning_radius == 0.0 &&
      !cfg_->robot.cmd_angle_instead_rotvel;
  // The optimized band can start at a slightly stale pose.  Recovery must be
  // based on the pose whose command will actually be executed, otherwise a
  // clear band start can hide a current padding contact and make every safe
  // separating command unreachable.
  const bool measured_start_valid =
      collision.begin(robot_pose_.x(), robot_pose_.y(), robot_pose_.theta());
  const bool recoverable_padded_sweep =
      trajectory_reason == "padded_sweep_intersection";
  const bool contact_recovery =
      !feasible && measured_start_valid && contact_model_supported &&
      (collision.initialOverlap() || recoverable_padded_sweep);
  if (!feasible && !contact_recovery)
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    planner_->clearPlanner();
    RCLCPP_WARN(logger_, "TebLocalPlannerROS: trajectory is not feasible (%s). Resetting planner...",
             trajectory_reason.c_str());

    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    message = "trajectory_" + trajectory_reason;
    return false;
  }
  if (contact_recovery)
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "TebLocalPlannerROS: using receding-horizon contact recovery (%s).",
        trajectory_reason.c_str());

  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_->trajectory.control_look_ahead_poses))

  {
    planner_->clearPlanner();
    RCLCPP_WARN(logger_, "TebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner velocity command invalid";
    return false;
  }

  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   cfg_->robot.max_vel_x, cfg_->robot.max_vel_y, cfg_->robot.max_vel_trans, cfg_->robot.max_vel_theta,
                   cfg_->robot.max_vel_x_backwards);

  unshaped_command_ = cmd_vel.twist;
  command_time_ns_ = static_cast<std::uint64_t>(clock_->now().nanoseconds());
  command_proposal_ns_ = command_time_ns_;
  const double forward_limit =
      std::min(cfg_->robot.max_vel_x, cfg_->robot.max_vel_trans);
  const double reverse_limit =
      std::min(cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_trans);
  const auto shape_command = [&](const geometry_msgs::msg::Twist& requested,
                                 geometry_msgs::msg::Twist& selected) {
    selected = requested;
    if (requested.linear.y != 0.0 ||
        !std::isfinite(requested.linear.x + requested.angular.z)) return false;
    if (!command_continuity_.enabled()) return true;
    CommandContinuity::Command input, output;
    input.surge = requested.linear.x;
    input.yaw = requested.angular.z;
    if (!command_continuity_.select(input, command_time_ns_, cfg_->robot.acc_lim_x,
            cfg_->robot.acc_lim_theta, forward_limit, reverse_limit,
            cfg_->robot.max_vel_theta, output)) return false;
    selected.linear.x = output.surge;
    selected.angular.z = output.yaw;
    return true;
  };

  geometry_msgs::msg::Twist shaped;
  if (!shape_command(cmd_vel.twist, shaped)) {
    message = "command_continuity_invalid";
    return false;
  }
  cmd_vel.twist = shaped;

  // Collision checking is independent of optional command smoothing. If the
  // optimized first command cannot leave an existing contact, project it onto
  // a tiny, actuator-valid candidate set. The exact swept-hull check remains
  // the authority; this changes no obstacle or footprint.
  bool contact_recovery_used = contact_recovery;
  std::string selected_command_reason;
  if (contact_model_supported) {
    bool command_feasible = isCommandArcFeasible(cmd_vel.twist, collision);
    if (!command_feasible)
      selected_command_reason = collision.reason().empty()
          ? std::string("invalid_motion") : collision.reason();
    const bool command_contact_recovery =
        !command_feasible && collision.initialOverlap();
    contact_recovery_used = contact_recovery_used || command_contact_recovery;
    if (!command_feasible && contact_recovery_used) {
      std::vector<geometry_msgs::msg::Twist> candidates;
      const auto add_candidate = [&](double surge, double yaw) {
        if (!std::isfinite(surge) || !std::isfinite(yaw) ||
            (std::abs(surge) <= 1e-9 && std::abs(yaw) <= 1e-9)) return;
        geometry_msgs::msg::Twist candidate;
        candidate.linear.x = surge;
        candidate.angular.z = yaw;
        const bool duplicate = std::any_of(
            candidates.begin(), candidates.end(),
            [&](const geometry_msgs::msg::Twist& value) {
              return std::abs(value.linear.x - candidate.linear.x) <= 1e-9 &&
                     std::abs(value.angular.z - candidate.angular.z) <= 1e-9;
            });
        if (!duplicate) candidates.push_back(candidate);
      };

      const auto& route_goal = transformed_plan.back().pose.position;
      const double dx = route_goal.x - robot_pose_.x();
      const double dy = route_goal.y - robot_pose_.y();
      const double heading = robot_pose_.theta();
      const double route_forward = std::cos(heading) * dx + std::sin(heading) * dy;
      const double route_lateral = -std::sin(heading) * dx + std::cos(heading) * dy;
      const double forward_recovery = forward_limit;
      const double reverse_recovery = -reverse_limit;
      const double preferred_recovery =
          route_forward < 0.0 ? reverse_recovery : forward_recovery;
      const double alternate_recovery =
          route_forward < 0.0 ? forward_recovery : reverse_recovery;
      double turn_hint = unshaped_command_.angular.z;
      if (std::abs(turn_hint) <= 1e-9) turn_hint = route_lateral;
      if (std::abs(turn_hint) <= 1e-9) turn_hint = 1.0;
      const double preferred_turn =
          std::copysign(cfg_->robot.max_vel_theta, turn_hint);

      // Project the optimizer's request onto a small underactuated
      // surge/yaw set. Curved and in-place alternatives let the hull separate
      // from a wall or corner when neither straight direction is executable.
      add_candidate(unshaped_command_.linear.x, unshaped_command_.angular.z);
      for (double surge : {preferred_recovery, 0.5 * preferred_recovery,
                           alternate_recovery, 0.5 * alternate_recovery}) {
        add_candidate(surge, unshaped_command_.angular.z);
        add_candidate(surge, 0.0);
        add_candidate(surge, preferred_turn);
        add_candidate(surge, -preferred_turn);
      }
      add_candidate(0.0, preferred_turn);
      add_candidate(0.0, -preferred_turn);

      for (const auto& candidate : candidates) {
        geometry_msgs::msg::Twist projected;
        if (!shape_command(candidate, projected)) continue;
        // A zero first sample is the continuity controller's normal bootstrap;
        // accepting it preserves history so the next cycle can accelerate.
        if (isCommandArcFeasible(projected, collision)) {
          cmd_vel.twist = projected;
          command_feasible = true;
          break;
        }
        if (!collision.reason().empty())
          selected_command_reason = collision.reason();
      }
    }
    if (!command_feasible) {
      message = "selected_command_" + selected_command_reason;
      return false;
    }
  }
  if (contact_recovery_used)
    message = "contact_recovery_" +
        (trajectory_reason.empty() ? selected_command_reason : trajectory_reason);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_->robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
                                                                cfg_->robot.wheelbase, 0.95*cfg_->robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      RCLCPP_WARN(logger_, "TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = clock_->now();
      message = "teb_local_planner steering angle is not finite";
      return false;
    }
  }

  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;

  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;

  // Now visualize everything
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  command_accepted = true;
  return true;
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{
  // Add costmap obstacles if desired
  if (cfg_->obstacles.include_costmap_obstacles)
  {
    std::lock_guard<std::recursive_mutex> lock(*costmap_->getMutex());

    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));

          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist  )
            continue;

          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;

  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter_msgs::msg::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::msg::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}


void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  std::lock_guard<std::mutex> l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try
    {
      geometry_msgs::msg::TransformStamped obstacle_to_map = tf_->lookupTransform(
                  cfg_->map_frame, tf2::timeFromSec(0),
                  custom_obstacle_msg_.header.frame_id, tf2::timeFromSec(0),
                  custom_obstacle_msg_.header.frame_id, tf2::durationFromSec(0.5));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
      //tf2::fromMsg(obstacle_to_map.transform, obstacle_to_map_eig);
    }
    catch (tf2::TransformException ex)
    {
      RCLCPP_ERROR(logger_, "%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        RCLCPP_INFO(logger_, "Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();

  if (min_separation<=0)
    return;

  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;

    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }

}

//Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
//{
//  Eigen::Vector2d vel;
//  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
//  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
//  return vel;
//}


bool TebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, std::vector<geometry_msgs::msg::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;

  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    //geometry_msgs::msg::TransformStamped global_to_plan_transform = tf_->lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, tf2::timeFromSec(0));
    geometry_msgs::msg::PoseStamped robot = tf_->transform(
              global_pose,
              global_plan.front().header.frame_id);

    //robot.setData( global_to_plan_transform * global_pose );

    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;

    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::msg::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::msg::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;

    if (command_continuity_.enabled() && !station_envelope_.enabled)
    {
      const int phase = firstIntermediateRotationEnd(global_plan);
      if (phase >= 0) {
        const auto& target=global_plan[phase].pose;
        const bool completed=std::hypot(robot.pose.position.x-target.position.x,
            robot.pose.position.y-target.position.y)<=cfg_->goal_tolerance.xy_goal_tolerance &&
            std::abs(g2o::normalize_theta(tf2::getYaw(robot.pose.orientation)-
                tf2::getYaw(target.orientation)))<=cfg_->goal_tolerance.yaw_goal_tolerance;
        if (completed) erase_end=global_plan.begin()+phase;
        else erase_end=std::min(erase_end,global_plan.begin()+phase-1);
      }
    }
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(logger_, "Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}


bool TebLocalPlannerROS::transformGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                  const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::msg::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h



  transformed_plan.clear();

  try
  {
    if (global_plan.empty())
    {
      RCLCPP_ERROR(logger_, "Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    const auto& plan_pose = global_plan.front();
    // get plan_to_global_transform from plan frame to global_frame
    auto plan_to_global_transform = tf_->lookupTransform(global_frame,
        rclcpp::Time(0, 0, clock_->get_clock_type()), plan_pose.header.frame_id,
        rclcpp::Time(plan_pose.header.stamp, clock_->get_clock_type()),
        plan_pose.header.frame_id, rclcpp::Duration::from_seconds(cfg_->robot.transform_tolerance));

    //let's get the pose of the robot in the frame of the plan
    auto robot_pose = tf_->transform(global_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap


    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // squared-distance threshold; take the first local minimum, not a later
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }

    const int phase = command_continuity_.enabled() && !station_envelope_.enabled ? firstIntermediateRotationEnd(global_plan) : -1;
    if (phase >= 0) i = std::min(i, phase - 1);
    const int horizon_end = phase >= 0 ? phase : static_cast<int>(global_plan.size()) - 1;
    if (phase >= 0)
    {
      const double dx = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      const double dy = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = dx * dx + dy * dy;
    }
    geometry_msgs::msg::PoseStamped newer_pose;

    double plan_length = 0; // check cumulative Euclidean distance along the plan

    //now we'll transform until points are outside of our distance threshold
    while(i <= horizon_end && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::msg::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan[horizon_end], newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = horizon_end;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }

    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf2::LookupException& ex)
  {
    RCLCPP_ERROR(logger_, "No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf2::ConnectivityException& ex)
  {
    RCLCPP_ERROR(logger_, "Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf2::ExtrapolationException& ex)
  {
    RCLCPP_ERROR(logger_, "Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      RCLCPP_ERROR(logger_, "Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}




double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal,
                    int current_goal_idx, const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();

  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
//      tf::Quaternion global_orientation;
//      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
//      return  tf2::getYaw(tf_plan_to_global.getRotation() *  global_orientation );

        tf2::Quaternion global_orientation, tf_plan_to_global_orientation;
        tf2::fromMsg(global_plan.back().pose.orientation, global_orientation);
        tf2::fromMsg(tf_plan_to_global.transform.rotation, tf_plan_to_global_orientation);

        return tf2::getYaw(tf_plan_to_global_orientation * global_orientation);
    }
  }

  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before

  std::vector<double> candidates;
  geometry_msgs::msg::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::msg::PoseStamped tf_pose_kp1;

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    const geometry_msgs::msg::PoseStamped& pose = global_plan.at(i+1);
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
              tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );

    if (i<range_end-1)
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}


void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_trans, double max_vel_theta,
              double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;

  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);

  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);

  // Limit backwards velocity
  if (max_vel_x_backwards < 0)
  {
    RCLCPP_WARN_ONCE(logger_, "TebLocalPlannerROS(): max_vel_x_backwards must be nonnegative; clamping reverse commands to zero.");
    max_vel_x_backwards = 0;
  }
  if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_->robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }

  double vel_linear = std::hypot(vx, vy);
  if (vel_linear > max_vel_trans)
  {
    double max_vel_trans_ratio = max_vel_trans / vel_linear;
    vx *= max_vel_trans_ratio;
    vy *= max_vel_trans_ratio;
    if (station_envelope_.enabled) omega *= max_vel_trans_ratio;
  }
}


double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;

  double radius = v/omega;

  if (fabs(radius) < min_turning_radius)
    radius = double(sign(radius)) * min_turning_radius;

  return std::atan(wheelbase / radius);
}


void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    RCLCPP_WARN_EXPRESSION(
                logger_, opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}



void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int& goal_idx, bool preserve_goal)
{
    rclcpp::Time current_time = clock_->now();

    // reduced horizon backup mode
    if (!preserve_goal && cfg_->recovery.shrink_horizon_backup &&
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).seconds() < cfg_->recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        RCLCPP_INFO_EXPRESSION(
                    logger_,
                    no_infeasible_plans_==1,
                    "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_->recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;

        if (no_infeasible_plans_ > 9)
        {
            RCLCPP_INFO_EXPRESSION(
                        logger_,
                        no_infeasible_plans_==10,
                        "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }

        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-)
    }


    // detect and resolve oscillations
    if (cfg_->recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_->robot.max_vel_x : cfg_->robot.max_vel_x_backwards;
        if (cfg_->robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_->robot.min_turning_radius),  cfg_->robot.max_vel_theta );
        else
            max_vel_theta = cfg_->robot.max_vel_theta;

        failure_detector_.update(last_cmd_, cfg_->robot.max_vel_x, cfg_->robot.max_vel_x_backwards, max_vel_theta,
                               cfg_->recovery.oscillation_v_eps, cfg_->recovery.oscillation_omega_eps);

        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (clock_->now()-time_last_oscillation_).seconds() < cfg_->recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently

        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                RCLCPP_INFO(logger_, "TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = clock_->now();
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            RCLCPP_INFO(logger_, "TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}


void TebLocalPlannerROS::customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
{
  std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
  if (!initialized_) return;
  std::lock_guard<std::mutex> l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
}

void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
{
  std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());
  if (!initialized_) return;
  RCLCPP_INFO_ONCE(logger_, "Via-points received. This message is printed once.");
  if (cfg_->trajectory.global_plan_viapoint_sep > 0)
  {
    RCLCPP_INFO(logger_, "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
             "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  std::lock_guard<std::mutex> l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::msg::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}

void TebLocalPlannerROS::activate() {
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  if (visualization_) visualization_->on_activate();

  return;
}
void TebLocalPlannerROS::deactivate() {
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  command_continuity_.reset();
  last_cmd_ = geometry_msgs::msg::Twist();
  if (visualization_) visualization_->on_deactivate();

  return;
}
void TebLocalPlannerROS::cleanup() {
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  initialized_ = false;
  dyn_params_handler.reset();
  custom_obst_sub_.reset();
  via_points_sub_.reset();
  if (costmap_converter_) costmap_converter_->stopWorker();
  costmap_converter_.reset();
  if (visualization_) visualization_->on_cleanup();
  planner_.reset();
  visualization_.reset();
  intra_proc_node_.reset();
  global_plan_.clear();
  obstacles_.clear();
  via_points_.clear();
  custom_via_points_active_ = false;
  {
    std::lock_guard<std::mutex> obstacle_lock(custom_obst_mutex_);
    custom_obstacle_msg_ = costmap_converter_msgs::msg::ObstacleArrayMsg();
  }
  command_continuity_ = CommandContinuity();
  station_envelope_ = StationEnvelope();
  persistent_goal_ = goal_reached_ = false;
  command_terminal_yaw_tolerance_ = -1.0;
  command_terminal_yaw_optional_ = false;
  command_period_sec_ = command_reverse_limit_ = 0.0;
  cfg_->setSpeedLimit(0.0, false);
  command_time_ns_ = command_proposal_ns_ = 0;
  unshaped_command_ = last_cmd_ = robot_vel_ = geometry_msgs::msg::Twist();
  {
    std::lock_guard<std::mutex> terminal_lock(terminal_observation_mutex_);
    terminal_observation_ = TerminalObservation();
  }
  no_infeasible_plans_ = 0;
  failure_detector_.clear();
  last_preferred_rotdir_ = RotType::none;
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  nh_.reset();
}

void TebLocalPlannerROS::resetTerminalObservation(bool new_plan)
{
  std::lock_guard<std::mutex> lock(terminal_observation_mutex_);
  auto epoch = terminal_observation_.epoch;
  ++epoch.event_sequence;
  if (new_plan) ++epoch.plan_sequence;
  else ++epoch.compute_sequence;
  epoch.time_ns = static_cast<std::uint64_t>(clock_->now().nanoseconds());
  terminal_observation_ = TerminalObservation::reset(
      epoch, new_plan ? "plan_reset" : "compute_reset");
}

bool TebLocalPlannerROS::setStationEnvelope(const StationEnvelope& envelope)
{
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  if (!initialized_) return false;
  auto* optimal = dynamic_cast<TebOptimalPlanner*>(planner_.get());
  if (!envelope.valid() || (envelope.enabled &&
      (!optimal || envelope.frame != cfg_->map_frame || cfg_->robot.max_vel_y != 0.0 ||
       cfg_->robot.cmd_angle_instead_rotvel || cfg_->robot.min_turning_radius != 0.0)))
    return false;
  if (optimal && !optimal->setStationEnvelope(envelope)) return false;
  station_envelope_ = envelope;
  return true;
}

bool TebLocalPlannerROS::setCommandScope(const std::string& request_id,
    const std::string& route_frame, double reverse_limit, double nominal_horizon,
    double terminal_yaw_tolerance, bool terminal_yaw_optional)
{
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  if (request_id.empty()) {
    command_continuity_.configure("", 0.0, 0.0);
    command_terminal_yaw_tolerance_ = -1.0;
    command_terminal_yaw_optional_ = false;
    return true;
  }
  if (!initialized_ || route_frame.empty() || cfg_->robot.max_vel_y != 0.0 ||
      cfg_->robot.cmd_angle_instead_rotvel || cfg_->robot.min_turning_radius != 0.0 ||
      !std::isfinite(reverse_limit) || reverse_limit < 0.0 ||
      !std::isfinite(terminal_yaw_tolerance) || terminal_yaw_tolerance < 0.0 ||
      terminal_yaw_tolerance > 3.14159265358979323846 ||
      !command_continuity_.configure(request_id + "\n" + route_frame,
                                     command_period_sec_, nominal_horizon)) {
    command_continuity_.configure("", 0.0, 0.0);
    return false;
  }
  command_reverse_limit_ = reverse_limit;
  command_terminal_yaw_tolerance_ = terminal_yaw_tolerance;
  command_terminal_yaw_optional_ = terminal_yaw_optional;
  return true;
}

void TebLocalPlannerROS::discardCommand()
{
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  command_continuity_.reset();
  last_cmd_ = geometry_msgs::msg::Twist();
}

void TebLocalPlannerROS::commitCommand(geometry_msgs::msg::Twist& command)
{
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  if (!command_continuity_.enabled()) return;
  command_time_ns_ = static_cast<std::uint64_t>(clock_->now().nanoseconds());
  CommandContinuity::Command accepted;
  accepted.surge = command.linear.x;
  accepted.yaw = command.angular.z;
  command_continuity_.commit(accepted, command_time_ns_, command_proposal_ns_);
  command.linear.x = accepted.surge;
  command.angular.z = accepted.yaw;
  last_cmd_ = command;
}

bool TebLocalPlannerROS::isCommandArcFeasible(const geometry_msgs::msg::Twist& command, SweptFootprint& collision) const
{
  if (command.linear.y != 0.0 || !collision.begin(robot_pose_.x(), robot_pose_.y(), robot_pose_.theta()))
    return false;
  return collision.advance(command.linear.x, 0.0, command.angular.z,
      command_continuity_.enabled() ? command_continuity_.horizon() : command_period_sec_);
}

bool TebLocalPlannerROS::isGoalReached()
{
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  if (!initialized_) return false;
  if (goal_reached_)
  {
    if (persistent_goal_) return true;
    RCLCPP_INFO(logger_, "GOAL Reached!");
    command_continuity_.reset();
    last_cmd_ = geometry_msgs::msg::Twist();
    planner_->clearPlanner();
    return true;
  }
  return false;
}
void TebLocalPlannerROS::setSpeedLimit(const double& speed_limit, const bool& percentage) {
  std::lock_guard<std::mutex> lock(cfg_->configMutex());
  if (!std::isfinite(speed_limit) || speed_limit < 0.0)
    throw nav2_core::ControllerException("invalid speed limit");
  cfg_->setSpeedLimit(speed_limit, percentage);
}

} // end namespace teb_local_planner

// register this planner as a nav2_core::Controller plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)
