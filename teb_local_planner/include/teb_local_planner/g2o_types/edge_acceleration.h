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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef EDGE_ACCELERATION_H_
#define EDGE_ACCELERATION_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include <geometry_msgs/msg/twist.hpp>



namespace teb_local_planner
{

/**
 * @class EdgeAcceleration
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot } ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationStart
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() and EdgeAccelerationGoal() for defining boundary values!
 */
class EdgeAcceleration : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAcceleration()
  {
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
    const double vel1 = pose1->pose().longitudinalDistanceTo(
        pose2->pose(), cfg_->useExactArcLength()) / dt1->dt();
    const double vel2 = pose2->pose().longitudinalDistanceTo(
        pose3->pose(), cfg_->useExactArcLength()) / dt2->dt();

    const double acc_lin  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );


    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);

    // ANGULAR ACCELERATION
    const double omega1 = angle_diff1 / dt1->dt();
    const double omega2 = angle_diff2 / dt2->dt();
    const double acc_rot  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );

    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);


    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    TEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() rotational: _error[1]=%f\n",_error[1]);
  }






public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeAccelerationStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses. \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationGoal() for defining boundary values at the end of the trajectory!
 */
class EdgeAccelerationStart : public BaseTebMultiEdge<2, const geometry_msgs::msg::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationStart()
  {
    _measurement = NULL;
    this->resize(3);
  }


  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION
    const double angle_diff = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double vel2 = pose1->pose().longitudinalDistanceTo(
        pose2->pose(), cfg_->useExactArcLength()) / dt->dt();
    const double vel1 = _measurement->linear.x;

    const double acc_lin  = (vel2 - vel1) / dt->dt();

    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);

    // ANGULAR ACCELERATION
    const double omega1 = _measurement->angular.z;
    const double omega2 = angle_diff / dt->dt();
    const double acc_rot  = (omega2 - omega1) / dt->dt();

    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
    TEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() rotational: _error[1]=%f\n",_error[1]);
  }

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * @param vel_start twist message containing the translational and rotational velocity
   */
  void setInitialVelocity(const geometry_msgs::msg::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




/**
 * @class EdgeAccelerationGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeAccelerationGoal : public BaseTebMultiEdge<2, const geometry_msgs::msg::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationGoal()
  {
    _measurement = NULL;
    this->resize(3);
  }


  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION

    const double angle_diff = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta());
    const double vel1 = pose_pre_goal->pose().longitudinalDistanceTo(
        pose_goal->pose(), cfg_->useExactArcLength()) / dt->dt();
    const double vel2 = _measurement->linear.x;

    const double acc_lin  = (vel2 - vel1) / dt->dt();

    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);

    // ANGULAR ACCELERATION
    const double omega1 = angle_diff / dt->dt();
    const double omega2 = _measurement->angular.z;
    const double acc_rot  = (omega2 - omega1) / dt->dt();

    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
    TEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() rotational: _error[1]=%f\n",_error[1]);
  }

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * @param vel_goal twist message containing the translational and rotational velocity
   */
  void setGoalVelocity(const geometry_msgs::msg::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




/**
 * @class EdgeAccelerationHolonomic
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot } ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x position parts of all three poses \n
 * \e ay is calculated using the difference quotient (twice) and the y position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration (x-dir),
 * the second one the strafing acceleration and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomicStart
 * @see EdgeAccelerationHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicStart() and EdgeAccelerationHolonomicGoal() for defining boundary values!
 */
class EdgeAccelerationHolonomic : public BaseTebMultiEdge<3, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic()
  {
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    Eigen::Vector2d diff2 = pose3->position() - pose2->position();

    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta());
    double cos_theta2 = std::cos(pose2->theta());
    double sin_theta2 = std::sin(pose2->theta());

    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
    double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
    // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
    double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
    double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

    double vel1_x = p1_dx / dt1->dt();
    double vel1_y = p1_dy / dt1->dt();
    double vel2_x = p2_dx / dt2->dt();
    double vel2_y = p2_dy / dt2->dt();

    double dt12 = dt1->dt() + dt2->dt();

    double acc_x  = (vel2_x - vel1_x)*2 / dt12;
    double acc_y  = (vel2_y - vel1_y)*2 / dt12;

    _error[0] = penaltyBoundToInterval(acc_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);

    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
    double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
    double acc_rot  = (omega2 - omega1)*2 / dt12;

    _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);


    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    TEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() strafing: _error[1]=%f\n",_error[1]);
    TEB_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAcceleration::computeError() rotational: _error[2]=%f\n",_error[2]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeAccelerationHolonomicStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses. \n
 * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses. \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
 * the second one the strafing acceleration and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic
 * @see EdgeAccelerationHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicGoal() for defining boundary values at the end of the trajectory!
 */
class EdgeAccelerationHolonomicStart : public BaseTebMultiEdge<3, const geometry_msgs::msg::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomicStart()
  {
    this->resize(3);
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff = pose2->position() - pose1->position();

    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta());

    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
    double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

    double vel1_x = _measurement->linear.x;
    double vel1_y = _measurement->linear.y;
    double vel2_x = p1_dx / dt->dt();
    double vel2_y = p1_dy / dt->dt();

    double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
    double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

    _error[0] = penaltyBoundToInterval(acc_lin_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_lin_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);

    // ANGULAR ACCELERATION
    double omega1 = _measurement->angular.z;
    double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
    double acc_rot  = (omega2 - omega1) / dt->dt();

    _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
    TEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() strafing: _error[1]=%f\n",_error[1]);
    TEB_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationStart::computeError() rotational: _error[2]=%f\n",_error[2]);
  }

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * @param vel_start twist message containing the translational and rotational velocity
   */
  void setInitialVelocity(const geometry_msgs::msg::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



/**
 * @class EdgeAccelerationHolonomicGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses \n
 * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
 * the second one is the strafing velocity and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic
 * @see EdgeAccelerationHolonomicStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicStart() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeAccelerationHolonomicGoal : public BaseTebMultiEdge<3, const geometry_msgs::msg::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomicGoal()
  {
    _measurement = NULL;
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION

    Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

    double cos_theta1 = std::cos(pose_pre_goal->theta());
    double sin_theta1 = std::sin(pose_pre_goal->theta());

    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
    double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

    double vel1_x = p1_dx / dt->dt();
    double vel1_y = p1_dy / dt->dt();
    double vel2_x = _measurement->linear.x;
    double vel2_y = _measurement->linear.y;

    double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
    double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

    _error[0] = penaltyBoundToInterval(acc_lin_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_lin_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);

    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
    double omega2 = _measurement->angular.z;
    double acc_rot  = (omega2 - omega1) / dt->dt();

    _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
    TEB_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() strafing: _error[1]=%f\n",_error[1]);
    TEB_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationGoal::computeError() rotational: _error[2]=%f\n",_error[2]);
  }


  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * @param vel_goal twist message containing the translational and rotational velocity
   */
  void setGoalVelocity(const geometry_msgs::msg::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




}; // end namespace

#endif /* EDGE_ACCELERATION_H_ */
