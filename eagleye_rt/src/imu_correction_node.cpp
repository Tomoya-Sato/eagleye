// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * correction_imu.cpp
 * Author MapIV Sekino
 */

#include "ros/ros.h"
#include "navigation/navigation.hpp"
#include "navigation/imu_correction.hpp"

// AVOS: angular_velocity_offset_stop
namespace ealgeye_navigation
{
class IMUCorrectionNode
{
public:
  IMUCorrectionNode(ros::NodeHandle& nh) : nh_(nh)
  {
    std::string yaml_file;
    nh_.getParam("yaml_file", yaml_file);
    std::cout << "yaml_file: " << yaml_file << std::endl;

    IMUCorrectionParameter param;
    param.load(yaml_file);
    corrector_.setParameter(param);

    imu_transformed_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_tf_converted", 1000);
    imu_unbiased_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_corrected", 1000);
    yawrate_offset_sub_ = nh_.subscribe("yawrate_offset_2nd", 1000, &IMUCorrector::yawrateOffsetCallback, this, ros::TransportHints().tcpNoDelay());
    AVOS_sub_ = nh_.subscribe("angular_velocity_offset_stop", 1000, &IMUCorrector::AVOSCallback, this, ros::TransportHints().tcpNoDelay());
    acc_x_offset_sub_ = nh_.subscribe("acc_x_offset", 1000, &IMUCorrector::accXOffsetCallback, this, ros::TransportHints().tcpNoDelay());
    acc_x_scale_factor_sub_ = nh_.subscribe("acc_x_scale_factor", 1000, &IMUCorrector::accXScaleFactorCallback, this, ros::TransportHints().tcpNoDelay());
    imu_sub_ = nh_.subscribe("imu/data_raw", 1000, &IMUCorrector::imuCallback, this, ros::TransportHints().tcpNoDelay());
  }
  void run()
  {
    ros::spin();
  }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Subscriber yawrate_offset_sub_;
  ros::Subscriber AVOS_sub_;
  ros::Subscriber acc_x_offset_sub_;
  ros::Subscriber acc_x_scale_factor_sub_;
  ros::Subscriber imu_sub_;

  // Corrector
  IMUCorrection corrector_;

  // Callbacks
  void yawrateOffsetCallback(const eagleye_msgs::YawrateOffset::ConstPtr& msg)
  {
    // Do nothing?
  }

  void AVOSCallback(const eagleye_msgs::AngularVelocityOffset::ConstPtr& msg)
  {
    Eigen::Vector3d angular_velocity_bias(msg->angular_velocity_offset.x, msg->angular_velocity_offset.y, msg->angular_velocity_offset.z);
    corrector_.angularBiasCallback(angular_velocity_bias);
  }

  void accXOffsetCallback(const eagleye_msgs::AccXOffset::ConstPtr& msg)
  {
    double acc_x_bias = msg->acc_x_offset;
    corrector_.accXBiasCallback(acc_x_bias);
  }

  void accXScaleFactorCallback(const eagleye_msgs::AccXScaleFactor::ConstPtr& msg)
  {
    double acc_x_scale_factor = msg->acc_x_scale_factor;
    corrector_.accXScaleFactorCallback(acc_x_scale_factor);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    IMUData raw_data;
    raw_data.linear_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    raw_data.angular_velocity = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    raw_data.is_transformed = false;
    raw_data.is_unbiased = false;

    IMUData transformed_data = corrector_.transformIMUData(raw_data);
    IMUData unbiased_data = corrector_.unbiasIMUData(transformed_data);

    sensor_msgs::Imu transformed_msg;
    transformed_msg = *msg;
    transformed_msg.header.frame_id = "base_link";
    transformed_msg.linear_acceleration.x = transformed_data.linear_acc.x;
    transformed_msg.linear_acceleration.y = transformed_data.linear_acc.y;
    transformed_msg.linear_acceleration.z = transformed_data.linear_acc.z;
    transformed_msg.angular_velocity.x = transformed_data.angular_velocity.x;
    transformed_msg.angular_velocity.y = transformed_data.angular_velocity.y;
    transformed_msg.angular_velocity.z = transformed_data.angular_velocity.z;

    sensor_msgs::Imu unbiased_msg;
    unbiased_msg = transformed_msg;
    unbiased_msg.linear_acceleration.x = transformed_data.linear_acc.x;
    unbiased_msg.linear_acceleration.y = transformed_data.linear_acc.y;
    unbiased_msg.linear_acceleration.z = transformed_data.linear_acc.z;
    unbiased_msg.angular_velocity.x = transformed_data.angular_velocity.x;
    unbiased_msg.angular_velocity.y = transformed_data.angular_velocity.y;
    unbiased_msg.angular_velocity.z = transformed_data.angular_velocity.z;

    imu_transformed_pub_.publish(transformed_msg);
    imu_unbiased_pub_.publish(unbiased_msg);
  }
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "correction_imu");
  ros::NodeHandle nh;

  eagleye_navigation::IMUCorrectionNode node(nh);
  node.run();

  return 0;
}
