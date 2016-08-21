/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Michal Drwiega (drwiega.michal@gmail.com)
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
 * @file   gazebo_tacbot_imu.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   2016
 * @brief  The Gazebo plugin for Tacbot IMU.
 */

#include <sensor_msgs/Imu.h>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/ImuSensor.hh>

#include <gazebo_tacbot_imu.h>

namespace gazebo {

//=================================================================================================
GazeboTacbotIMU::~GazeboTacbotIMU()
{
    pub_imu_.shutdown();
}

//=================================================================================================
void GazeboTacbotIMU::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    sdf_ = sdf;
    model_ = parent;

    if (!model_)
    {
        ROS_ERROR_STREAM("Invalid model pointer! [" << model_->GetName() << "]");
        return;
    }
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("Unable to load Gazebo plugin");
        return;
    }

    // Name of the parent model used as node name
    std::string model_name = sdf->GetParent()->Get<std::string>("name");
    gzdbg << "Plugin model name: " << model_name << "\n";

    if(!parseSdfForIMUParameters()) return;

    setupRosPubAndSub();
    update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboTacbotIMU::onUpdate, this));

    ROS_INFO_STREAM("GazeboRosTacbot plugin ready to go! [" << model_->GetName() << "]");
}

//=================================================================================================
void GazeboTacbotIMU::setupRosPubAndSub()
{
    const std::string imu_topic = "/" + model_->GetName() + "/" + topic_name_;
    pub_imu_ = ros_node_->advertise<sensor_msgs::Imu>(imu_topic, 1);
    ROS_INFO_STREAM(ros_node_->getNamespace() << ": Advertise IMU[" << imu_topic << "].");
}

//=================================================================================================
bool GazeboTacbotIMU::parseSdfForIMUParameters()
{
    for (auto & param : { "imu_name", "topic_name", "frame_name"})
    {
        if (!sdf_->HasElement(param))
        {
            ROS_ERROR_STREAM("Couldn't find the " << param << " parameter in the model description!");
            return false;
        }
    }

    auto imu_name = sdf_->GetElement("imu_name")->Get<std::string>();
    topic_name_ = sdf_->GetElement("topic_name")->Get<std::string>();
    frame_name_ = sdf_->GetElement("frame_name")->Get<std::string>();

    imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(
                sensors::get_sensor(imu_name));
    if (!imu_)
    {
        ROS_ERROR_STREAM("Couldn't find the IMU in the model! [" << model_->GetName() <<"]");
        return false;
    }
    imu_->SetActive(true);
    return true;
}

//=================================================================================================
void GazeboTacbotIMU::onUpdate()
{
    ros::spinOnce(); // Process ROS callbacks

    publishIMU();
}

//=================================================================================================
void GazeboTacbotIMU::publishIMU()
{
    sensor_msgs::Imu msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = model_->GetName() + "_" + frame_name_;

    math::Quaternion quat = imu_->Orientation();
    msg.orientation.x = quat.x;
    msg.orientation.y = quat.y;
    msg.orientation.z = quat.z;
    msg.orientation.w = quat.w;
    msg.orientation_covariance[0] = 1e6;
    msg.orientation_covariance[4] = 1e6;
    msg.orientation_covariance[8] = 0.05;

    math::Vector3 ang_vel = imu_->AngularVelocity();
    msg.angular_velocity.x = ang_vel.x;
    msg.angular_velocity.y = ang_vel.y;
    msg.angular_velocity.z = ang_vel.z;
    msg.angular_velocity_covariance[0] = 1e6;
    msg.angular_velocity_covariance[4] = 1e6;
    msg.angular_velocity_covariance[8] = 0.05;

    math::Vector3 lin_acc = imu_->LinearAcceleration();
    msg.linear_acceleration.x = lin_acc.x;
    msg.linear_acceleration.y = lin_acc.y;
    msg.linear_acceleration.z = lin_acc.z;

    pub_imu_.publish(msg);
}

//=================================================================================================

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboTacbotIMU)

}


