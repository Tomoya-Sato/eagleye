// Copyright (c) 2023, MAP IV, Inc.
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
 * imu_correction.hpp
 * Author MAP IV Tomoya-Sato
 */

#ifndef IMU_CORRECTION_HPP
#define IMU_CORRECTION_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

#include "navigation/pose.hpp"

namespace eagleye_navigation
{
struct IMUCorrectorParameter
{
  Eigen::Matrix4f imu_to_baselink_matrix;
  
  void load(const std::string& yaml_file)
  {
    try
    {
      YAML::Node conf = YAML::LoadFile(yaml_file);

      Pose pose;
      pose.x = conf["baselink2imu"]["x"].as<double>();
      pose.y = conf["baselink2imu"]["y"].as<double>();
      pose.z = conf["baselink2imu"]["z"].as<double>();
      pose.roll = conf["baselink2imu"]["roll"].as<double>();
      pose.pitch = conf["baselink2imu"]["pitch"].as<double>();
      pose.yaw = conf["baselink2imu"]["yaw"].as<double>();

      imu_to_baselink_matrix = pose2Matrix(pose);
    }
    catch (YAML::Exception& e)
    {
      std::cerr << "\033[1;31mIMUCorrectorParameter YAML Error: " << e.msg << "\033[m" << std::endl;
      exit(3);
    }
  }
};

struct IMUData
{
  // Data
  Eigen::Vector3d linear_acc;
  Eigen::Vector3d angular_velocity;

  // Status
  bool is_transformed;
  bool is_unbiased;
};

class IMUCorrector
{
public:
  IMUCorrector();
  void setParameter(const IMUCorrectorParameter& param);
  void linearBiasCallback(const Eigen::Vector3d& bias);
  void accXBiasCallback(const double& bias);
  void angularBiasCallback(const Eigen::Vector3d& bias);
  void yawrateBiasCallback(const double& bias);
  void linearScaleFactorCallback(const Eigen::Vector3d& scale_factor);
  void accXScaleFactorCallback(const double& scale_factor);

  IMUData transformIMUData(const IMUData& data);
  IMUData unbiasIMUData(const IMUData& data);

private:
  // Param
  IMUCorrectorParameter param_;

  // Scale factor
  Eigen::Vector3d linear_acc_scale_factor_;
  Eigen::Vector3d angular_velocity_scale_factor_;

  // Bias
  Eigen::Vector3d linear_acc_bias_;
  Eigen::Vector3d angular_velocity_bias_;
};
}

#endif
