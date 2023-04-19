#include "navigation/imu_correction.hpp"

namespace eagleye_navigation
{
IMUCorrector::IMUCorrector()
{
  linear_acc_scale_factor_ = Eigen::Vector3d(1.0, 1.0, 1.0);
  angular_velocity_scale_factor_ = Eigen::Vector3d(1.0, 1.0, 1.0);

  linear_acc_bias_ = Eigen::Vector3d::Zero();
  angular_velocity_bias_ = Eigen::Vector3d::Zero();
}

void IMUCorrector::setParameter(const IMUCorrectorParameter& param)
{
  param_ = param;
}

void IMUCorrector::linearBiasCallback(const Eigen::Vector3d& bias)
{
  linear_acc_bias_ = bias;
}

void IMUCorrector::accXBiasCallback(const double& bias)
{
  linear_acc_bias_[0] = bias;
}

void IMUCorrector::angularBiasCallback(const Eigen::Vector3d& bias)
{
  angular_velocity_bias_ = bias;
}

void IMUCorrector::yawrateBiasCallback(const double& bias)
{
  angular_velocity_bias_[2] = bias;
}

void IMUCorrector::linearScaleFactorCallback(const Eigen::Vector3d& scale_factor)
{
  linear_acc_scale_factor_ = scale_factor;
}

void IMUCorrector::accXScaleFactorCallback(const double& scale_factor)
{
  linear_acc_scale_factor_[0] = scale_factor;
}

IMUData IMUCorrector::transformIMUData(const IMUData& data)
{
  Eigen::Matrix3d rotation_matrix = param_.imu_to_baselink_matrix.toLeftCorner(3, 3);

  IMUData transformed_data;
  transformed_data.linear_acc = rotation_matrix(data.linear_acc);
  transformed_data.angular_velocity = rotation_matrix(data.angular_velocity);

  transformed_data.is_transformed = true;
  transformed_data.is_unbiased = data.is_unbiased;

  return transformed_data;
}

IMUData IMUCorrector::unbiasIMUData(const IMUData& data)
{
  IMUData unbiased_data;
  unbiased_data.linear_acc = data.linear_acc * linear_acc_scale_factor_ - linear_acc_bias_;
  unbiased_data.angular_velocity = data.angular_velocity * angular_velocity_scale_factor_ - angular_velocity_bias_;
  unbiased_data.is_transformed = data.is_transformed;
  unbiased_data.is_unbiased = true;

  return unbiased_data;
}
