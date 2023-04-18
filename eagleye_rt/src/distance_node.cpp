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
 * distance.cpp
 * Author MapIV Sekino
 */

#include <ros/ros.h>
#include <navigation/distance.hpp>
#include <navigation/navigation.hpp>

class DistanceEstimatorNode
{
public:
  DistanceEstimatorNode(ros::NodeHandle& nh) : nh_(nh)
  {
    bool use_canless_mode = false;
    nh_.getParam("use_canless_mode", use_canless_mode);

    estimator_.setCanlessMode(use_canless_mode);

    distance_pub_ = nh_.advertise<eagleye_msgs::Distance>("distance", 1000);
    velocity_status_sub_ = nh_.subscribe("velocity_status", 1000, &DistanceEstimatorNode::velocityStatusCallback, this, ros::TransportHints().tcpNoDelay());
    twist_sub_ = nh_.subscribe("velocity", 1000, &DistanceEstimatorNode::twistCallback, this, ros::TransportHints().tcpNoDelay());
  }
  void run()
  {
    ros::spin();
  }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher distance_pub_;
  ros::Subscriber velocity_status_sub_;
  ros::Subscriber twist_sub_;

  // Estimator
  DistanceEstimator estimator_;

  // Callback
  void velocityStatusCallback(const eagleye_msgs::StatusStamped::ConstPtr& msg)
  {
    estimator_.velocityStatusCallback(msg->status.enabled_status);
  }

  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    // Trigger estimation
    double stamp = msg->header.stamp.toSec();
    Eigen::Vector3d velocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    DistanceStatus distance_status = estimator_.velocityCallback(stamp, velocity);

    // Convert to ROS message
    if (distance_status.is_estimation_started)
    {
      eagleye_msgs::Distance distance_msg;
      distance_msg.header = msg->header;
      distance_msg.header.frame_id = "base_link";

      distance_msg.distance = distance_status.estimated_distance;
      distance_msg.status.enabled_status = true;

      distance_pub_.publish(distance_msg);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distance");

  ros::NodeHandle nh;

  DistanceEstimatorNode node(nh);
  node.run();

  return 0;
}
