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
// * Neither the name of the MAP IV, Inc. nor the names of its contributors
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

#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

struct DistanceStatus
{
  bool is_estimation_started;
  double estimated_distance;
};

class DistanceEstimator
{
public:
  DistanceEstimator();
  DistanceStatus velocityCallback(const double& stamp, const Eigen::Vector3d& velocity);
  void velocityStatusCallback(const bool& is_velocity_valid) { is_velocity_valid_ = is_velocity_valid; }
  void setCanlessMode(const bool& is_canless_mode) { is_canless_mode_ = is_canless_mode; }

private:
  // Flags
  bool is_estimation_started_;
  bool is_velocity_valid_;
  bool is_canless_mode_;

  // Distance
  double previous_time_;
  double accumulated_distance_;
};

#endif
