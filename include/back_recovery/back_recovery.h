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
#ifndef BACK_RECOVERY_BACK_RECOVERY_H
#define BACK_RECOVERY_BACK_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>

namespace back_recovery
{
    /**
     * @class backRecovery
     * @brief A recovery behavior that backs the robot to attempt to clear out space
     */
    class BackRecovery : public nav_core::RecoveryBehavior
    {
    public:
        /**
         * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
         */
        BackRecovery();

        /**
         * @brief  Initialization function for the BackRecovery recovery behavior
         * @param name Namespace used in initialization
         * @param tf (unused)
         * @param global_costmap (unused)
         * @param local_costmap A pointer to the local_costmap used by the navigation stack
         */
        void initialize(std::string name, tf2_ros::Buffer *,
                        costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap);

        /**
         * @brief  Run the BackRecovery recovery behavior.
         */
        void runBehavior();

        /**
         * @brief  Destructor for the Back recovery behavior
         */
        ~BackRecovery();

    private:
        costmap_2d::Costmap2DROS *local_costmap_;
        bool initialized_;
        double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_, back_dist_, acc_lim_x_, max_vel_x_, min_vel_x_;
        base_local_planner::CostmapModel *world_model_;
    };
};     // namespace back_recovery
#endif // BACK_RECOVERY_BACK_RECOVERY_H
