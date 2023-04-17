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

#include "navigation/distance.hpp"

DistanceEstimator::DistanceEstimator()
{
  // Initialization
  is_estimation_started_ = false;
  is_velocity_valid_ = false;
  is_canless_mode_ = false;
  previous_time_ = -1.0;
  accumulated_distance_ = 0.0;
}

DistanceStatus DistanceEstimator::velocityCallback(const double& stamp, const Eigen::Vector3d& velocity)
{
  // Accumulate
  if (is_estimation_started_)
  {
    accumulated_distance_ += velocity[0] * (stamp - previous_time_);
  }

  DistanceStatus status;
  status.is_estimation_started = is_estimation_started_;
  status.estimated_distance = accumulated_distance_;

  // Update internal status
  previous_time_ = stamp;
  is_estimation_started_ = true;

  return status;
}
