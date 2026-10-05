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

#ifndef EDGE_VELOCITY_H
#define EDGE_VELOCITY_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/misc.h>


#include <iostream>

namespace teb_local_planner
{


/**
 * @class EdgeVelocity
 * @brief Edge defining the cost function for limiting the translational and rotational velocity.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v,omega]^T ) \cdot weight \f$. \n
 * \e v is calculated using the difference quotient and the position parts of both poses. \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational velocity and
 * the second one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocity : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocity()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    const double vel = conf1->pose().longitudinalDistanceTo(
        conf2->pose(), cfg_->useExactArcLength()) / deltaT->estimate();

    const double omega = angle_diff / deltaT->estimate();

    _error[0] = penaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }




public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};






/**
 * @class EdgeVelocityHolonomic
 * @brief Edge defining the cost function for limiting the translational and rotational velocity according to x,y and theta.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [vx,vy,omega]^T ) \cdot weight \f$. \n
 * \e vx denotes the translational velocity w.r.t. x-axis (computed using finite differneces). \n
 * \e vy denotes the translational velocity w.r.t. y-axis (computed using finite differneces). \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational velocity w.r.t. x-axis,
 * the second one w.r.t. the y-axis and the third one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocityHolonomic : public BaseTebMultiEdge<3, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocityHolonomic()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocityHolonomic()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

    double cos_theta1 = std::cos(conf1->theta());
    double sin_theta1 = std::sin(conf1->theta());

    // transform conf2 into current robot frame conf1 (inverse 2d rotation matrix)
    double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double r_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();

    double vx = r_dx / deltaT->estimate();
    double vy = r_dy / deltaT->estimate();
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) / deltaT->estimate();


    double max_vel_trans_remaining_y;
    double max_vel_trans_remaining_x;
    max_vel_trans_remaining_y = std::sqrt(std::max(0.0, cfg_->robot.max_vel_trans * cfg_->robot.max_vel_trans - vx * vx));
    max_vel_trans_remaining_x = std::sqrt(std::max(0.0, cfg_->robot.max_vel_trans * cfg_->robot.max_vel_trans - vy * vy));

    double max_vel_y = std::min(max_vel_trans_remaining_y, cfg_->robot.max_vel_y);
    double max_vel_x = std::min(max_vel_trans_remaining_x, cfg_->robot.max_vel_x);
    double max_vel_x_backwards = std::min(max_vel_trans_remaining_x, cfg_->robot.max_vel_x_backwards);

    // we do not apply the penalty epsilon for linear velocities on
    // holonomic robots, since either vx or yy could be close to zero
    _error[0] = penaltyBoundToInterval(vx, -max_vel_x_backwards, max_vel_x, 0.0);
    _error[1] = penaltyBoundToInterval(vy, max_vel_y, 0.0);
    _error[2] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]) && std::isfinite(_error[2]),
                   "EdgeVelocityHolonomic::computeError() _error[0]=%f _error[1]=%f _error[2]=%f\n",_error[0],_error[1],_error[2]);
  }


public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


} // end namespace

#endif
