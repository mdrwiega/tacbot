/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * @file   gazebo_ros_tacbot.h
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @brief  The Gazebo plugin for Tacbot robot with the control system in ROS.
 */

#pragma once

#include <string>
#include <array>

#include <ros/node_handle.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>

namespace gazebo {

constexpr uint WHEELS_COUNT = 4;

enum class Wheel {BL = 0, BR = 1, FL = 2, FR = 3};

/// Converts Wheel enum to string.
constexpr auto toString(const Wheel wheel) noexcept {
    switch (wheel) {
    case Wheel::BL: return "BL";
    case Wheel::BR: return "BR";
    case Wheel::FL: return "FL";
    case Wheel::FR: return "FR";
    }
}

/**
 * @brief The GazeboRosTacbot class
 */
class GazeboRosTacbot  : public ModelPlugin
{
public:
    GazeboRosTacbot() = default;
    ~GazeboRosTacbot();

private:
    /// Called when plugin is loaded
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;
    /// Setup ROS publishers and subscribers
    void setupRosPubAndSub();
    /// Called by the world update start event
    void onUpdate();
    /// Callback for resetting the odometry data
    void resetOdomCB(const std_msgs::EmptyConstPtr & msg) noexcept;
    /// Callback for incoming velocity commands
    void cmdVelocitiesCb(const geometry_msgs::TwistConstPtr & msg) noexcept;
    /// Parse sdf model to set joints parameters
    bool parseSdfAndSetupWheelJoints();
    /// Parse sdf model to set other parameters
    bool parseOtherSdfParameters();
    /// Publishes joints states
    void publishJointState();
    /// Updates and publishes odometery
    void updateAndPublishOdometry(common::Time && step_time);
    /// Sets velocities joints based on received commands
    void setJointsVelocities();

private:
    //-----------------------------------------------------------------------------------------------
    physics::ModelPtr model_;                       ///< Pointer to the Gazebo model
    sdf::ElementPtr sdf_;                           ///< Pointer the the SDF element of the plugin.
    event::ConnectionPtr update_connection_;        ///< Pointer to the update event connection
    std::array<physics::JointPtr, WHEELS_COUNT> joints_;          ///< Pointers to Gazebo's joints
    std::array<double, WHEELS_COUNT> wheel_speed_cmd_{{0,0,0,0}}; ///< Desired speeds of wheels

    ros::NodeHandle ros_node_;                      ///< A node use for ROS transport
    sensor_msgs::JointState joint_state_;           ///< ROS message for joint sates
    std::string tf_prefix_;                         ///< TF Prefix
    common::Time prev_update_time_;                 ///< Simulation time on previous update
    std::array<double,3> odom_pose_{{0,0,0}};       ///< Odometry position

    // Publishers and subscribers
    //-----------------------------------------------------------------------------------------------
    ros::Publisher  pub_odom_;                      ///< Odometry data publisher
    ros::Publisher  joint_state_pub_;               ///< Joint states publisher
    tf2_ros::TransformBroadcaster tf_broadcaster_;  ///< Broadcaster for tf odom publishing
    ros::Subscriber sub_cmd_vel_;                   ///< Velocity commands subscriber
    ros::Subscriber odom_reset_sub_;                ///< Subscriber for reseting the odometry data

    // Parameters provided by robot urdf description (gazebo plugin parameters)
    //-----------------------------------------------------------------------------------------------
    double back_wheel_separation_{1.0};         ///< Back wheel separation in meters
    double front_wheel_separation_{1.0};        ///< Front wheel separation in meters
    double wheel_diameter_{1.0};                ///< Wheel diameter in meters
    double torque_{1.0};                        ///< Max. torque applied to the wheels
    bool publish_tf_{true};                     ///< Fkag determines if publish tf transform for odom
};

}
