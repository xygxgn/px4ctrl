#ifndef __INPUT_H
#define __INPUT_H

#include <Eigen/Dense>

#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include "uav_utils/utils.h"
#include "parameters.h"


class Base_Data_t
{
public:
    ros::Time rcv_stamp; // The timestamp of the received data
    virtual inline bool is_received(const ros::Time &) const = 0;
};


/*****************************/
/** flight control unit data */
/*****************************/

/** @brief State data */
class State_Data_t : public Base_Data_t
{
public:
    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;


    State_Data_t(const Parameter_t &);
    
    void feed(const mavros_msgs::StateConstPtr);

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < param_.msg_timeout.state; }

private:
    Parameter_t param_;
};


/** @brief RC data from FCU */
class RC_Data_t : public Base_Data_t
{
public:
    mavros_msgs::RCIn rc_msg;

    bool have_init_last_rc;
    double mode, gear, reboot;
    double last_mode, last_gear, last_reboot;
    double channels[4]; // channels[0-3] --> [roll, pitch, throttle, yaw]

    static constexpr double DEAD_ZONE = 0.25; // used to decrease motion amplitude of the drone
    static constexpr double ALTITUDE_MODE_THRESHOLD = 0.25; // toggle to altitude mode
    static constexpr double POSITION_MODE_THRESHOLD = 0.75; // toggle to position mode
    static constexpr double HOVER_OFFBOARD_THRESHOLD = 0.25; // toggle to hover mode
    static constexpr double COMMAND_OFFBOARD_THRESHOLD = 0.75; // toggle to command mode
    static constexpr double REBOOT_THRESHOLD = 0.5; // reboot

    bool is_hover;
    bool is_command;
    bool enter_hover;
    bool enter_command;
    bool exit_hover;
    bool exit_command;

    bool toggle_reboot;


    RC_Data_t(const Parameter_t &);

    void feed(mavros_msgs::RCInConstPtr);
    
    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < param_.msg_timeout.rc; }

private:
    Parameter_t param_;
};


/** @brief Battery data from FCU */
class Battery_Data_t : public Base_Data_t
{
public:
    sensor_msgs::BatteryState battery_msg;


    Battery_Data_t(const Parameter_t &);

    void feed(sensor_msgs::BatteryStateConstPtr);

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < param_.msg_timeout.battery; }

private:
    Parameter_t param_;
};


/** @brief Imu data from FCU */
class Imu_Data_t : public Base_Data_t
{
public:
    sensor_msgs::Imu imu_msg;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaterniond q; // orientation
    Eigen::Vector3d w; // angular velocity
    Eigen::Vector3d a; // linear acceleration


    Imu_Data_t(const Parameter_t &);

    void feed(sensor_msgs::ImuConstPtr);

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < param_.msg_timeout.imu; }

private:
    const Parameter_t param_;
};


/** @brief All FCU data */
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



/******************************/
/** Odometry and Command data */
/******************************/

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













