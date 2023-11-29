/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#include "linear_controller.h"


LinearController::LinearController() 
{
    set_parameter();
    resetThrustMapping();
}


void LinearController::resetThrustMapping()
{
    thrust2acc = gravity / hover_percentage;
    P = 1e6;
}


void LinearController::set_parameter()
{
    hover_percentage = Parameter_t::thrust_mapping.hover_percentage;
    gravity = Parameter_t::gravity;
    gain = Parameter_t::gain;
}


/** @brief Estimate thrust2acc_(mapping from thrust to acceleration) 
 * @param est_acc Acceleration provided by IMU accelerometer
*/
bool LinearController::estimateThrustModel(const Eigen::Vector3d &est_acc)
{
    ros::Time now_time = ros::Time::now();
    while (timed_thrusts.size() >= 1)
    {
        /* Choose data before 35~45ms ago */
        std::pair<ros::Time, double> timed_thrust = timed_thrusts.front();
        double time_passed = (now_time - timed_thrust.first).toSec();
        if (time_passed > 0.045)
        {
            timed_thrusts.pop();
            continue;
        }
        if (time_passed < 0.035)
        {
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thrust = timed_thrust.second;
        timed_thrusts.pop();

        /********************************************/
        /* Model: est_acc(2) = thrust2acc * thrust */
        /********************************************/
        double gamma = 1 / (rho2 + thrust * P * thrust);
        double K = gamma * P * thrust; // filtering gain
        thrust2acc = thrust2acc + K * (est_acc(2) - thrust * thrust2acc); // posteriori estimate
        P = (1 - K * thrust) * P / rho2; // posteriori covariance

        return true;
    }
    return false;
}


/** @brief Compute u.thrust and u.q, controller gains and other parameters are in param
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
    Kp << gain.Kp0, gain.Kp1, gain.Kp2;
    Kv << gain.Kv0, gain.Kv1, gain.Kv2;
    des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
    des_acc += Eigen::Vector3d(0.0, 0.0, gravity); // counteract gravity

    /* compute throttle percentage (desired collective thrust) */
    u.thrust = des_acc(2) / thrust2acc;
    ROS_DEBUG("convergence thrust: %f", u.thrust);


    /* 2.Compute attitude */
    double roll, pitch, yaw;
    double yaw_odom = uav_utils::get_yaw_from_quaternion(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    
    roll = (des_acc(0) * sin - des_acc(1) * cos) / gravity;
    pitch = (des_acc(0) * cos + des_acc(1) * sin) / gravity;
    yaw = uav_utils::normalize_angle(des.yaw + uav_utils::get_yaw_from_quaternion(imu.q) - yaw_odom);

    Eigen::Quaterniond q = uav_utils::ypr_to_quaternion(Eigen::Vector3d(yaw, pitch, roll));

    // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
    //     * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    //     * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // u.q = imu.q * odom.q.inverse() * q; // from odometry frame to imu frame
    
    u.q = q; // using odometry frame
    

    /* 3.Used for thrust to acceleration mapping estimation */
    timed_thrusts.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
    while (timed_thrusts.size() > 100)
    {
        timed_thrusts.pop();
    }

}
