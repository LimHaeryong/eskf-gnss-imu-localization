#include <cmath>

#include <unsupported/Eigen/MatrixFunctions>

#include "eskf_gnss_imu_localization/eskf.hpp"

ErrorStateKalmanFilter::ErrorStateKalmanFilter()
    : x(Eigen::Matrix<double, 19, 1>::Zero())
    , error_x(Eigen::Matrix<double, 18, 1>::Zero())
    , P(Mat18d::Identity())
    , F_x(Mat18d::Identity())
    , F_i(Eigen::Matrix<double, 18, 12>::Zero())
    , Q_i(Eigen::Matrix<double, 12, 12>::Identity())
    , velocity_noise_variance(0.01 * 0.01)
    , orientation_noise_variance(0.01 * 0.01)
    , H_x(Eigen::Matrix<double, 6, 19>::Zero())
    , x_error_x(Eigen::Matrix<double, 19, 18>::Zero())
    , V(Eigen::Matrix<double, 6, 6>::Identity())
    , G(Mat18d::Identity())
{
    x(6) = 1.0;
    x(18) = 9.8;
    F_i.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();
    H_x.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    x_error_x.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    x_error_x.block<9, 9>(10, 9) = Eigen::Matrix<double, 9, 9>::Identity();
    x_error_x.block<4, 3>(6, 6) = computeQ_error_theta(x.block<4, 1>(6, 0));

    // TODO
    // Q_i (0, 0)~(3, 3) and (3, 3)~(6, 6) is experimental value : velocity covariance, angular covariance
    // V (3, 3)~(6,6) is experimental value : GNSS velocity covariance
}

void ErrorStateKalmanFilter::predictWithImu(std::shared_ptr<ImuMeasurement> imuMeasurement)
{   
    // compute variables
    std::chrono::duration<double> duration = imuMeasurement->timestamp - prevTime;
    prevTime = imuMeasurement->timestamp;
    double dt = duration.count();
    if(dt > 0.01 * 10)
    {
        dt = 0.01;
    }
    double dt2 = dt * dt;

    Eigen::Matrix3d log_dR = vector3dToSkewSymmetric((imuMeasurement->angularVelocity - x.block<3, 1>(13, 0)) * dt);
    Eigen::Matrix3d dR = log_dR.exp();
    Eigen::Matrix3d R = quaternionToRotationMatrix(x.block<4, 1>(6, 0)) * dR;

    F_x.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F_x.block<3, 3>(3, 6) = -1.0 * R * vector3dToSkewSymmetric(imuMeasurement->acceleration - x.block<3, 1>(10, 0)) * dt;
    F_x.block<3, 3>(3, 9) = -1.0 * R * dt;
    F_x.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    F_x.block<3, 3>(6, 6) = dR.transpose();
    F_x.block<3, 3>(6, 12) = -1.0 * Eigen::Matrix3d::Identity() * dt;

    Q_i.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * velocity_noise_variance * dt2;
    Q_i.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * orientation_noise_variance * dt2;
    Q_i.block<3, 3>(6, 6) = imuMeasurement->accelerationCovariance * dt;
    Q_i.block<3, 3>(9, 9) = imuMeasurement->angularVelocityCovariance * dt;

    // update 
    // no need to predict error_x
    P = F_x * P * F_x.transpose() + F_i * Q_i * F_i.transpose();
}

void ErrorStateKalmanFilter::updateWithGnss(std::shared_ptr<GnssMeasurement> gnssMeasurement)
{
    // compute variables
    V.block<3, 3>(0, 0) = gnssMeasurement->positionCovariance;

    x_error_x.block<4, 3>(6, 6) = computeQ_error_theta(x.block<4, 1>(6, 0));

    H = H_x * x_error_x;
    auto H_t = H.transpose();

    K = P * H_t * (H * P * H_t + V).inverse();

    z.block<3, 1>(0, 0) = llaToEnu(gnssMeasurement->position);
    z.block<3, 1>(3, 0) = gnssMeasurement->linearVelocity;

    // update
    error_x = K * (z - H_x * x);
    auto i_minus_kh = Mat18d::Identity() - K * H;
    P = i_minus_kh * P * i_minus_kh.transpose() + K * V * K.transpose();

    injectErrorToNominal();
    resetErrorState();
}

void ErrorStateKalmanFilter::injectErrorToNominal()
{
    x.block<6, 1>(0, 0) += error_x.block<6, 1>(0, 0);
    auto error_quat = rotationVectorToQuaternion(error_x.block<3, 1>(6, 0));
    x.block<4, 1>(6, 0) = quaternionToLeftProductMatrix(x.block<4, 1>(6, 0)) * error_quat;
    x.block<9, 1>(10, 0) += error_x.block<9, 1>(9, 0);
}

void ErrorStateKalmanFilter::resetErrorState()
{
    G.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() + 0.5 * vector3dToSkewSymmetric(error_x.block<3, 1>(6, 0));
    P = G * P * G.transpose();
    error_x.setZero();
}

Eigen::Matrix<double, 4, 3> ErrorStateKalmanFilter::computeQ_error_theta(const Eigen::Matrix<double, 4, 1>& quaternion)
{
    Eigen::Matrix<double, 4, 3> error_q_error_theta = Eigen::Matrix<double, 4, 3>::Zero();
    error_q_error_theta.block<3, 3>(1, 0) = Eigen::Matrix3d::Identity() * 0.5;
    auto Q_error_theta = quaternionToLeftProductMatrix(quaternion) * error_q_error_theta;
    return Q_error_theta;
}

Eigen::Matrix4d ErrorStateKalmanFilter::quaternionToLeftProductMatrix(const Eigen::Matrix<double, 4, 1>& quaternion)
{
    Eigen::Vector3d q_v = quaternion.block<3, 1>(1, 0);
    double q_w = quaternion(0);
    Eigen::Matrix4d LeftProductMatrix = q_w * Eigen::Matrix4d::Identity();
    LeftProductMatrix.block<1, 3>(3, 0) += q_v.transpose();
    LeftProductMatrix.block<3, 1>(0, 3) -= q_v;
    LeftProductMatrix.block<3, 3>(1, 1) += vector3dToSkewSymmetric(q_v);
    return LeftProductMatrix;
}

Eigen::Matrix4d ErrorStateKalmanFilter::quaternionToRightProductMatrix(const Eigen::Matrix<double, 4, 1>& quaternion)
{
    Eigen::Vector3d q_v = quaternion.block<3, 1>(1, 0);
    double q_w = quaternion(0, 0);
    Eigen::Matrix4d RightProductMatrix = q_w * Eigen::Matrix4d::Identity();
    RightProductMatrix.block<1, 3>(3, 0) += q_v.transpose();
    RightProductMatrix.block<3, 1>(0, 3) -= q_v;
    RightProductMatrix.block<3, 3>(1, 1) -= vector3dToSkewSymmetric(q_v);
    return RightProductMatrix;
}

Eigen::Matrix3d ErrorStateKalmanFilter::vector3dToSkewSymmetric(const Eigen::Vector3d& vec3d)
{
    Eigen::Matrix3d skewSymmetric;
    skewSymmetric << 0.0, -vec3d.z(), vec3d.y(), 
        vec3d.z(), 0.0, -vec3d.x(), 
        -vec3d.y(), vec3d.x(), 0.0;
    return skewSymmetric;
}

Eigen::Matrix3d ErrorStateKalmanFilter::quaternionToRotationMatrix(const Eigen::Matrix<double, 4, 1>& quaternion)
{
    Eigen::Quaterniond q;
    q.w() = quaternion(0);
    q.vec() = quaternion.block<3, 1>(1, 0);
    return q.toRotationMatrix();
}

Eigen::Vector4d ErrorStateKalmanFilter::rotationVectorToQuaternion(const Eigen::Vector3d& rotationVector)
{
    double phi = rotationVector.norm();
    Eigen::Vector3d u = rotationVector.normalized();
    Eigen::Vector4d quaternion;
    quaternion(0) = std::cos(phi / 2.0);
    quaternion.block<3, 1>(1, 0) = u * std::sin(phi / 2.0);
    return quaternion;
}

Eigen::Vector3d ErrorStateKalmanFilter::getPosition() const 
{
    return x.block<3, 1>(0, 0); 
}

Eigen::Vector3d ErrorStateKalmanFilter::llaToEnu(const Eigen::Vector3d& llaPosition)
{
    if(!mLocalCartesinInitialized)
    {
        mLocalCartesian.Reset(llaPosition(0), llaPosition(1), llaPosition(2));
        mLocalCartesinInitialized = true;
    }
    Eigen::Vector3d enuPosition;
    mLocalCartesian.Forward(llaPosition(0), llaPosition(1), llaPosition(2), enuPosition(0), enuPosition(1), enuPosition(2));
    return enuPosition;
}