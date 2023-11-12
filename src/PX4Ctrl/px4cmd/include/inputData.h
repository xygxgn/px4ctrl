#ifndef __INPUTDATA_H
#define __INPUTDATA_H


#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "uav_utils/utils.h"

#include "fcuData.h"


/** @brief FCU data */
class FCU_Data_t
{
public:
    State_Data_t state_data;
    RC_Data_t rc_data;
    Battery_Data_t battery_data;
    Imu_Data_t imu_data;


    FCU_Data_t(const Parameter_t &param) : param_(param), 
        state_data(State_Data_t(param)), rc_data(RC_Data_t(param)), 
        battery_data(Battery_Data_t(param)), imu_data(Imu_Data_t(param)) {}

private:
    Parameter_t param_;
};


/** @brief Odometry data */
class Odom_Data_t : public Base_Data_t
{
public:
    nav_msgs::Odometry odom_msg;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaterniond q; // orientation
    Eigen::Vector3d w; // angular velocity
    Eigen::Vector3d p; // position
    Eigen::Vector3d v; // linear velocity

    
    Odom_Data_t(const Parameter_t &);
    void feed(nav_msgs::OdometryConstPtr);

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < param_.msg_timeout.odom; }

private:
    Parameter_t param_;
};


/** @brief Command data */
class Command_Data_t : public Base_Data_t
{
public:
    quadrotor_msgs::PositionCommand msg;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;


    Command_Data_t(const Parameter_t &);

    void feed(quadrotor_msgs::PositionCommandConstPtr);

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < param_.msg_timeout.cmd; }

private:
    const Parameter_t param_;
};


#endif