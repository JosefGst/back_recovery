/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#include <back_recovery/back_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(back_recovery::BackRecovery, nav_core::RecoveryBehavior)

namespace back_recovery
{
    BackRecovery::BackRecovery() : local_costmap_(NULL), initialized_(false), world_model_(NULL)
    {
    }

    void BackRecovery::initialize(std::string name, tf2_ros::Buffer *,
                                  costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap)
    {
        if (!initialized_)
        {
            local_costmap_ = local_costmap;

            // get some parameters from the parameter server
            ros::NodeHandle private_nh("~/" + name);
            ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

            // we'll simulate every degree by default
            private_nh.param("sim_granularity", sim_granularity_, 0.017);
            private_nh.param("frequency", frequency_, 20.0);
            private_nh.param("back_dist", back_dist_, 0.50);

            acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
            max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
            min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
            blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);
            blp_nh.param("acc_lim_x", acc_lim_x_, 2.5);
            blp_nh.param("max_vel_x", max_vel_x_, 0.4);
            blp_nh.param("min_vel_x", min_vel_x_, -0.3);

            world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

            initialized_ = true;
        }
        else
        {
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    BackRecovery::~BackRecovery()
    {
        delete world_model_;
    }

    void BackRecovery::runBehavior()
    {
        if (!initialized_)
        {
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        if (local_costmap_ == NULL)
        {
            ROS_ERROR("The costmap passed to the BackRecovery object cannot be NULL. Doing nothing.");
            return;
        }
        ROS_WARN("Back recovery behavior started.");

        ros::Rate r(frequency_);
        ros::NodeHandle n;
        ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        geometry_msgs::PoseStamped global_pose;
        local_costmap_->getRobotPose(global_pose);

        double current_angle = tf2::getYaw(global_pose.pose.orientation);
        // double start_angle = current_angle;

        bool got_back = false;

        while (n.ok() && (!got_back || std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > tolerance_))
        {
            // TODO: calculate distance between goal and current position
            //     // Update Current Angle
            //     local_costmap_->getRobotPose(global_pose);
            //     current_angle = tf2::getYaw(global_pose.pose.orientation);

            //     // compute the distance left to back
            //     double dist_left;
            //     if (!got_180)
            //     {
            //         // If we haven't hit 180 yet, we need to back a half circle plus the distance to the 180 point
            //         double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
            //         dist_left = M_PI + distance_to_180;

            //         if (distance_to_180 < tolerance_)
            //         {
            //             got_180 = true;
            //         }
            //     }
            //     else
            //     {
            //         // If we have hit the 180, we just have the distance back to the start
            //         dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
            //     }

            // calculate planned position
            double x = global_pose.pose.position.x, y = global_pose.pose.position.y;
            x = x + back_dist_ * cos(current_angle);
            y = y + back_dist_ * sin(current_angle);
            ROS_ERROR("current_angle: %.2f", current_angle);
            ROS_ERROR("x: %.2f", x);
            ROS_ERROR("y: %.2f", y);

            // check if that velocity is legal by forward simulating
            double sim_angle = 0.0;

            // make sure that the point is legal, if it isn't... we'll abort
            double footprint_cost = world_model_->footprintCost(x, y, current_angle, local_costmap_->getRobotFootprint(), 0.0, 0.0);
            if (footprint_cost < 0.0)
            {
                ROS_ERROR("Back recovery can't back in place because there is a potential collision. Cost: %.2f",
                          footprint_cost);
                return;
            }

            // compute the velocity that will let us stop by the time we reach the goal
            double vel = sqrt(2 * acc_lim_x_ * back_dist_);

            // make sure that this velocity falls within the specified limits
            vel = std::min(std::max(vel, min_vel_x_), max_vel_x_);
            ROS_ERROR("vel: %.2f", vel);

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = -vel;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;

            vel_pub.publish(cmd_vel);

            r.sleep();
        }
    }
}; // namespace back_recovery