#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "inputData.h"
#include "readParam.h"

#include <Eigen/Dense>
#include <queue>


/** @brief Desired position, velocity, acceleration, attitude of drone */
struct Desired_PVAQ_t
{
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    Eigen::Quaterniond q;
    double yaw, yaw_rate;

    
	Desired_PVAQ_t() {} // default constructor

    Desired_PVAQ_t(Odom_Data_t &odom) 
        : p(odom.p),
          v(Eigen::Vector3d::Zero()),
          a(Eigen::Vector3d::Zero()),
          q(odom.q),
          yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
          yaw_rate(0) {}
};


/** @brief The control output required for the drone to achieve the desired position, velocity, acceleration, attitude */
struct Controller_t
{
	// Orientation of the body frame with respect to the world frame
    Eigen::Quaterniond q;

    // Body rates in body frame
    Eigen::Vector3d bodyrates; // [rad/s]

    // Collective mass normalized thrust
    double thrust;
};


class LinearController
{
public:
    LinearController() {}
    LinearController(const Parameter_t &param) : param_(param) { resetThrustMapping(); }
    bool estimateThrustModel(const Eigen::Vector3d &, const Parameter_t &);
    void calculateControl(const Desired_PVAQ_t &, const Odom_Data_t &, const Imu_Data_t &, Controller_t &);
    void resetThrustMapping();
    
private:
    Parameter_t param_; // fpv parameters

    /* Thrust queue of past time */
    std::queue<std::pair<ros::Time, double>> timed_thrusts_;

    /* Thrust-accel mapping params */
    const double rho2_ = 0.998; // do not change
    double thrust2acc_, P_; // the estimate and covariance of mapping from thrust to acceleration

};


#endif