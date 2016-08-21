/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Michal Drwiega (drwiega.michal@gmail.com)
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
 * @file   gazebo_tacbot_imu.h
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   2016
 * @brief  The Gazebo plugin for Tacbot IMU.
 */

#pragma once

#include <string>
#include <memory>

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>

namespace gazebo {

/**
 * @brief The GazeboTacbotIMU class
 */
class GazeboTacbotIMU  : public ModelPlugin
{
public:
    GazeboTacbotIMU() : ros_node_(std::make_unique<ros::NodeHandle>("gazebo_tacbot_imu")) { }
    ~GazeboTacbotIMU();

private:
    /// Called when plugin is loaded
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;
    /// Setup ROS publishers and subscribers
    void setupRosPubAndSub();
    /// Called by the world update start event
    void onUpdate();
    /// Parses SDF for IMU sensor parameters
    bool parseSdfForIMUParameters();
    /// Publishes IMU sensor data
    void publishIMU();

private:
    //---------------------------------------------------------------------------------------------
    physics::ModelPtr model_;                   ///< Pointer to the model
    sdf::ElementPtr sdf_;                       ///< Pointer the the SDF element of the plugin.
    event::ConnectionPtr update_connection_;    ///< Pointer to the update event connection
    sensors::ImuSensorPtr imu_;                 ///< Pointer to IMU sensor model

    std::unique_ptr<ros::NodeHandle> ros_node_; ///< A node use for ROS transport

    ros::Publisher  pub_imu_;                   ///< IMU data publisher

    std::string frame_name_{"imu_link"};        ///< Name of IMU sensor frame
    std::string topic_name_{"imu_raw"};         ///< IMU publisher topic
};

}
