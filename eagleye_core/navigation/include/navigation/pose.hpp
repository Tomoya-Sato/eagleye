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
 * pose.hpp
 * Author MAP IV Tomoya-Sato
 */

#include <Eigen/Core>
#include <Eigen/Dense>

namespace eagleye_navigation
{
struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Pose(float x, float y, float z, float roll, float pitch, float yaw)
    : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw)
  {
  }
  Pose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0)
  {
  }

  Pose operator=(const Pose& p)
  {
    this->x = p.x;
    this->y = p.y;
    this->z = p.z;
    this->roll = p.roll;
    this->pitch = p.pitch;
    this->yaw = p.yaw;
    return *this;
  }

  Pose operator-(const Pose& p) const
  {
    Pose diff;
    diff.x = this->x - p.x;
    diff.y = this->y - p.y;
    diff.z = this->z - p.z;
    diff.roll = this->roll - p.roll;
    diff.pitch = this->pitch - p.pitch;
    diff.yaw = this->yaw - p.yaw;
    return diff;
  }

  Pose operator+(const Pose& p) const
  {
    Pose diff;
    diff.x = this->x + p.x;
    diff.y = this->y + p.y;
    diff.z = this->z + p.z;
    diff.roll = this->roll + p.roll;
    diff.pitch = this->pitch + p.pitch;
    diff.yaw = this->yaw + p.yaw;
    return diff;
  }

  Pose operator+=(const Pose& p)
  {
    this->x += p.x;
    this->y += p.y;
    this->z += p.z;
    this->roll += p.roll;
    this->pitch += p.pitch;
    this->yaw += p.yaw;
    return *this;
  }

  Pose operator*(double d) const
  {
    Pose diff;
    diff.x = this->x * d;
    diff.y = this->y * d;
    diff.z = this->z * d;
    diff.roll = this->roll * d;
    diff.pitch = this->pitch * d;
    diff.yaw = this->yaw * d;
    return diff;
  }

  Pose operator/(double d) const
  {
    Pose diff;
    diff.x = this->x / d;
    diff.y = this->y / d;
    diff.z = this->z / d;
    diff.roll = this->roll / d;
    diff.pitch = this->pitch / d;
    diff.yaw = this->yaw / d;
    return diff;
  }

  Pose operator/=(double d)
  {
    this->x /= d;
    this->y /= d;
    this->z /= d;
    this->roll /= d;
    this->pitch /= d;
    this->yaw /= d;
    return *this;
  }

  bool operator==(const Pose& p)
  {
    bool is_equal = true;
    is_equal = is_equal && (this->x == p.x);
    is_equal = is_equal && (this->y == p.y);
    is_equal = is_equal && (this->z == p.z);
    is_equal = is_equal && (this->roll == p.roll);
    is_equal = is_equal && (this->pitch == p.pitch);
    is_equal = is_equal && (this->yaw == p.yaw);
    return is_equal;
  }

  Eigen::Vector3d xyz() const
  {
    return Eigen::Vector3d(x, y, z);
  }

  Eigen::Vector3d rpy() const
  {
    return Eigen::Vector3d(roll, pitch, yaw);
  }
};

static Eigen::Matrix4d pose2Matrix(const Pose& pose)
{
  Eigen::AngleAxisf init_rotation_x(pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(pose.x, pose.y, pose.z);
  Eigen::Matrix4f matrix = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
  return matrix;
}

Pose pose_conversions::matrix2Pose(Eigen::Matrix4f matrix)
{
  Pose pose;
  tf::Matrix3x3 mat;
  mat.setValue(static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)), static_cast<double>(matrix(0, 2)),
               static_cast<double>(matrix(1, 0)), static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
               static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)), static_cast<double>(matrix(2, 2)));
  pose.x = matrix(0, 3);
  pose.y = matrix(1, 3);
  pose.z = matrix(2, 3);
  mat.getRPY(pose.roll, pose.pitch, pose.yaw, 1);
  return pose;
}
}
