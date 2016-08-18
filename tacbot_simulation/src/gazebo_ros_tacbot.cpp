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
 * @file   gazebo_ros_tacbot.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   10.2015
 * @brief  tacbot simulation
 */

#include <cmath>
#include <functional>

#include <gazebo_ros_tacbot.h>

#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <gazebo/common/PID.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/math/gzmath.hh>


namespace gazebo {

//=================================================================================================
GazeboRosTacbot::~GazeboRosTacbot()
{
    sub_cmd_vel_.shutdown();
    pub_odom_.shutdown();
    joint_state_pub_.shutdown();
    odom_reset_sub_.shutdown();
}

//=================================================================================================
void GazeboRosTacbot::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    //    ros_node_.reset(new ros::NodeHandle("gazebo_ros_tacbot"));

    world_ = parent->GetWorld();
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

    if(!parseSdfAndSetupWheelJoints() || !parseOtherSdfParameters()) return;

    setupRosPubAndSub();
    update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboRosTacbot::onUpdate, this));

    prev_update_time_ = world_->GetSimTime();
    ROS_INFO_STREAM("GazeboRosTacbot plugin ready to go! [" << model_->GetName() << "]");
}

//=================================================================================================
void GazeboRosTacbot::setupRosPubAndSub()
{
    std::string joint_states_topic = "/" + model_->GetName() + "/joint_states";
    joint_state_pub_ = ros_node_->advertise<sensor_msgs::JointState>(joint_states_topic, 1);
    ROS_INFO_STREAM(ros_node_->getNamespace() << ": Advertise joint_states[" << joint_states_topic << "].");

    std::string odom_topic = "/" + model_->GetName() + "/odom_raw";
    pub_odom_ = ros_node_->advertise<nav_msgs::Odometry>(odom_topic, 1);
    ROS_INFO_STREAM(ros_node_->getNamespace() << ": Advertise Odometry[" << odom_topic << "].");

    std::string odom_reset_topic = "/" + model_->GetName() + "/reset_odometry";
    odom_reset_sub_ = ros_node_->subscribe(odom_reset_topic, 10, &GazeboRosTacbot::resetOdomCB, this);
    ROS_INFO_STREAM(ros_node_->getNamespace() << ": Subscribe to " << odom_reset_topic << ".");

    std::string cmd_vel_topic = "/" + model_->GetName() + "/cmd_vel";
    sub_cmd_vel_ = ros_node_->subscribe(cmd_vel_topic, 10, &GazeboRosTacbot::cmdVelocitiesCb, this);
    ROS_INFO_STREAM(ros_node_->getNamespace() << ": Subscribe to " << cmd_vel_topic << ".");
}

//=================================================================================================
void GazeboRosTacbot::onUpdate()
{
    ros::spinOnce(); // Process ROS callbacks

    auto time_now = world_->GetSimTime();

    publishJointState();
    setJointsVelocities();
    updateAndPublishOdometry(time_now - prev_update_time_);

    prev_update_time_ = time_now;
}

//=================================================================================================
void GazeboRosTacbot::resetOdomCB(const std_msgs::EmptyConstPtr & msg)  noexcept
{
    odom_pose_ = {0.0, 0.0, 0.0};
}

//==============================================================================
void GazeboRosTacbot::cmdVelocitiesCb(const geometry_msgs::TwistConstPtr &msg) noexcept
{
    wheel_speed_cmd_[static_cast<int>(Wheel::BL)] = msg->linear.x - msg->angular.z * back_wheel_separation_ / 2;
    wheel_speed_cmd_[static_cast<int>(Wheel::BR)] = msg->linear.x + msg->angular.z * back_wheel_separation_ / 2;
    wheel_speed_cmd_[static_cast<int>(Wheel::FL)] = msg->linear.x - msg->angular.z * front_wheel_separation_ / 2;
    wheel_speed_cmd_[static_cast<int>(Wheel::FR)] = msg->linear.x + msg->angular.z * front_wheel_separation_ / 2;
}

//=================================================================================================
bool GazeboRosTacbot::parseSdfAndSetupWheelJoints()
{
    std::array<double,3> pid_k;
    int index = 0;

    for (const auto & param : { "reg_kp", "reg_ki", "reg_kd" })
    {
        if (!sdf_->HasElement(param))
        {
            ROS_ERROR_STREAM("Couldn't find left wheel joint in the model description!");
            return false;
        }
        pid_k[index++] = sdf_->GetElement(param)->Get<double>();
    }

    std::array<std::string, WHEELS_COUNT> joint_names;
    std::vector<std::string> params = { "wheel_BL_joint_name", "wheel_BR_joint_name",
                                        "wheel_FL_joint_name", "wheel_FR_joint_name" };

    for (int i = 0; i < WHEELS_COUNT; ++i)
    {
        if (!sdf_->HasElement(params[i]))
        {
            ROS_ERROR_STREAM("Couldn't find left wheel joint in the model description!");
            return false;
        }
        joint_names[i] = sdf_->GetElement(params[i])->Get<std::string>();
        joints_[i] = model_->GetJoint(joint_names[i]);
        model_->GetJointController()->SetVelocityPID(joints_[i]->GetScopedName(),
                                                     common::PID(pid_k[0], pid_k[1], pid_k[2]));
    }

    if (!joints_[0] || !joints_[1] || !joints_[2] || !joints_[3])
    {
        ROS_ERROR_STREAM("Couldn't find specified wheel joints in the model! [" << model_->GetName() <<"]");
        return false;
    }

    joint_state_.header.frame_id = "Joint States";
    for (auto & name : joint_names)
    {
        joint_state_.name.push_back(name);
        joint_state_.position.push_back(0);
        joint_state_.velocity.push_back(0);
        joint_state_.effort.push_back(0);
    }
    return true;
}

//=================================================================================================
bool GazeboRosTacbot::parseOtherSdfParameters()
{
    // Check if parameters were defined in sdf file
    for (auto & param : { "back_wheel_separation", "front_wheel_separation", "wheel_diameter",
         "max_torque","publish_tf" })
    {
        if (!sdf_->HasElement(param))
        {
            ROS_ERROR_STREAM("Couldn't find the " << param << " parameter in the model description!");
            return false;
        }
    }

    back_wheel_separation_ = sdf_->GetElement("back_wheel_separation")->Get<double>();
    front_wheel_separation_ = sdf_->GetElement("front_wheel_separation")->Get<double>();
    wheel_diameter_ = sdf_->GetElement("wheel_diameter")->Get<double>();
    torque_ = sdf_->GetElement("max_torque")->Get<double>();
    publish_tf_ = sdf_->GetElement("publish_tf")->Get<bool>();

    return true;
}

//=================================================================================================
void GazeboRosTacbot::publishJointState()
{
    std::string baselink_frame = model_->GetName() + "_base_link";
    joint_state_.header.stamp = ros::Time::now();
    joint_state_.header.frame_id = baselink_frame;

    for (int i = 0; i < WHEELS_COUNT; ++i)
    {
        joint_state_.position[i] = joints_[i]->GetAngle(0).Radian();
        joint_state_.velocity[i] = joints_[i]->GetVelocity(0);
    }
    joint_state_pub_.publish(joint_state_);
}

//=================================================================================================
void GazeboRosTacbot::updateAndPublishOdometry(common::Time && step_time)
{
    std::string odom_frame =  model_->GetName() + "_odom_raw";
    std::string base_frame = model_->GetName() + "_base_link";

    nav_msgs::Odometry odom_msg; // ROS message for odometry data

    odom_msg.header.stamp = joint_state_.header.stamp;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_frame;

    const auto left = (joints_[static_cast<int>(Wheel::BL)]->GetVelocity(0)
            + joints_[static_cast<int>(Wheel::FL)]->GetVelocity(0)) / 2;
    const auto right = (joints_[static_cast<int>(Wheel::BR)]->GetVelocity(0)
            + joints_[static_cast<int>(Wheel::FR)]->GetVelocity(0)) / 2;

    auto dl = step_time.Double() * (wheel_diameter_ / 2) * left;
    auto dr = step_time.Double() * (wheel_diameter_ / 2) * right;

    if (std::isnan(dl))
    {
        ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo plugin: NaN in dl. Step time: "
                                 << step_time.Double() << ", velocity: " << left);
        dl = 0.0;
    }
    if (std::isnan(dr))
    {
        ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo plugin: NaN in dr. Step time: "
                                 << step_time.Double() << ", velocity: " << right);
        dr = 0.0;
    }
    const auto d_lin = (dl + dr) / 2;
    const auto d_ang = (dr - dl) / back_wheel_separation_;
    const auto direction = odom_pose_[2] + d_ang / 2;

    odom_pose_[0] += d_lin * cos( direction );
    odom_pose_[1] += d_lin * sin( direction );
    odom_pose_[2] += d_ang;

    std::array<double,3> odom_vel_ = { d_lin / step_time.Double(),
                                       d_ang / step_time.Double(), 0.0 };

    odom_msg.pose.pose.position.x = odom_pose_[0];
    odom_msg.pose.pose.position.y = odom_pose_[1];
    odom_msg.pose.pose.position.z = 0;

    tf::Quaternion qt;
    qt.setEuler(0,0,odom_pose_[2]);
    odom_msg.pose.pose.orientation.x = qt.getX();
    odom_msg.pose.pose.orientation.y = qt.getY();
    odom_msg.pose.pose.orientation.z = qt.getZ();
    odom_msg.pose.pose.orientation.w = qt.getW();

    odom_msg.pose.covariance[0]  = 0.1;
    odom_msg.pose.covariance[7]  = 0.1;
    odom_msg.pose.covariance[35] = 0.05;
    odom_msg.pose.covariance[14] = 1e6;
    odom_msg.pose.covariance[21] = 1e6;
    odom_msg.pose.covariance[28] = 1e6;

    odom_msg.twist.twist.linear.x = odom_vel_[0];
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;
    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = odom_vel_[2];

    pub_odom_.publish(odom_msg);

    if (publish_tf_)
    {
        geometry_msgs::TransformStamped odom_tf; // TF transform for the odom frame
        odom_tf.header = odom_msg.header;
        odom_tf.child_frame_id = odom_msg.child_frame_id;
        odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
        tf_broadcaster_.sendTransform(odom_tf);
    }
}

//=================================================================================================
void GazeboRosTacbot::setJointsVelocities()
{
    for(int i = 0; i < WHEELS_COUNT; ++i)
    {
        model_->GetJointController()->SetVelocityTarget( joints_[i]->GetScopedName(),
                                                         wheel_speed_cmd_[i] / (wheel_diameter_ / 2.0));
    }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTacbot);

}


