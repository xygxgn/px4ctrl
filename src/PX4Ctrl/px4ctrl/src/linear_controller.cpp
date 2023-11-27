/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#include "linear_controller.h"


void LinearController::resetThrustMapping()
{
    thrust2acc_ = param_.gravity / param_.thrust_mapping.hover_percentage;
    P_ = 1e6;
}


/** @brief Estimate thrust2acc_(mapping from thrust to acceleration) 
 * @param est_acc Acceleration provided by IMU accelerometer
 * @param param fpv parameters
*/
bool LinearController::estimateThrustModel(const Eigen::Vector3d &est_acc, const Parameter_t &param)
{
    ros::Time now_time = ros::Time::now();
    while (timed_thrusts_.size() >= 1)
    {
        /* Choose data before 35~45ms ago */
        std::pair<ros::Time, double> timed_thrust_ = timed_thrusts_.front();
        double time_passed = (now_time - timed_thrust_.first).toSec();
        if (time_passed > 0.045)
        {
            timed_thrusts_.pop();
            continue;
        }
        if (time_passed < 0.035)
        {
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thrust = timed_thrust_.second;
        timed_thrusts_.pop();

        /********************************************/
        /* Model: est_acc(2) = thrust2acc_ * thrust */
        /********************************************/
        double gamma = 1 / (rho2_ + thrust * P_ * thrust);
        double K = gamma * P_ * thrust; // filtering gain
        thrust2acc_ = thrust2acc_ + K * (est_acc(2) - thrust * thrust2acc_); // posteriori estimate
        P_ = (1 - K * thrust) * P_ / rho2_; // posteriori covariance

        return true;
    }
    return false;
}


/** @brief Compute u.thrust and u.q, controller gains and other parameters are in param_
 * @param des Desired PVAQ
 * @param odom Odometry data of drone
 * @param imu Imu data of drone
 * @param u Output vector of controller to reach desired PVAQ
 */
void LinearController::calculateControl(const Desired_PVAQ_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, Controller_t &u)
{
    /* 1.Compute thrust */
    // compute disired acceleration
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp, Kv;
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
    des_acc += Eigen::Vector3d(0.0, 0.0, param_.gravity); // counteract gravity

    /* compute throttle percentage (desired collective thrust) */
    u.thrust = des_acc(2) / thrust2acc_;
    ROS_DEBUG("convergence thrust: %f", u.thrust);


    /* 2.Compute attitude */
    double roll, pitch, yaw;
    double yaw_odom = uav_utils::get_yaw_from_quaternion(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    
    roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gravity;
    pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gravity;
    yaw = uav_utils::normalize_angle(des.yaw + uav_utils::get_yaw_from_quaternion(imu.q) - yaw_odom);

    Eigen::Quaterniond q = uav_utils::ypr_to_quaternion(Eigen::Vector3d(yaw, pitch, roll));

    // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
    //     * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    //     * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // u.q = imu.q * odom.q.inverse() * q; // from odometry frame to imu frame
    
    u.q = q; // using odometry frame
    

    /* 3.Used for thrust to acceleration mapping estimation */
    timed_thrusts_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrusts_.size() > 100)
    {
        timed_thrusts_.pop();
    }

}
