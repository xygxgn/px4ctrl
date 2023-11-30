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
    virtual void set_parameter() = 0;
    virtual inline bool is_received(const ros::Time &) const = 0;

    ros::Time rcv_stamp; // The timestamp of the received data
    double msg_timeout;
};


/*****************************/
/** flight control unit data */
/*****************************/

/** @brief State data */
class State_Data_t : public Base_Data_t
{
public:
    State_Data_t();

    virtual void set_parameter() override {
        msg_timeout = Parameter_t::msg_timeout.state; }

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < msg_timeout; }

    void feed(const mavros_msgs::StateConstPtr);


    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;

private:
};


/** @brief RC data from FCU */
class RC_Data_t : public Base_Data_t
{
public:
    RC_Data_t();

    virtual void set_parameter() override {
        msg_timeout = Parameter_t::msg_timeout.rc; }

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < msg_timeout; }

    void feed(mavros_msgs::RCInConstPtr);


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

private:
    
};


/** @brief Battery data from FCU */
class Battery_Data_t : public Base_Data_t
{
public:
    Battery_Data_t();

    virtual void set_parameter() override {
        msg_timeout = Parameter_t::msg_timeout.battery; }

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < msg_timeout; }

    void feed(sensor_msgs::BatteryStateConstPtr);


    sensor_msgs::BatteryState battery_msg;

private:
};


/** @brief Imu data from FCU */
class Imu_Data_t : public Base_Data_t
{
public:
    Imu_Data_t();

    virtual void set_parameter() override {
        msg_timeout = Parameter_t::msg_timeout.imu; }

    void feed(sensor_msgs::ImuConstPtr);

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < msg_timeout; }


    sensor_msgs::Imu imu_msg;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaterniond q; // orientation
    Eigen::Vector3d w; // angular velocity
    Eigen::Vector3d a; // linear acceleration

private:
};


/** @brief All FCU data */
class FCU_Data_t
{
public:
    FCU_Data_t() { };
    
    State_Data_t state_data;
    RC_Data_t rc_data;
    Battery_Data_t battery_data;
    Imu_Data_t imu_data;

private:
};



/******************************/
/** Odometry and Command data */
/******************************/

/** @brief Odometry data */
class Odom_Data_t : public Base_Data_t
{
public:
    Odom_Data_t();

    virtual void set_parameter() override {
        msg_timeout = Parameter_t::msg_timeout.odom; }

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < msg_timeout; }

    void feed(nav_msgs::OdometryConstPtr);


    nav_msgs::Odometry odom_msg;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaterniond q; // orientation
    Eigen::Vector3d w; // angular velocity
    Eigen::Vector3d p; // position
    Eigen::Vector3d v; // linear velocity

private:
};


/** @brief Command data */
class Command_Data_t : public Base_Data_t
{
public:
    Command_Data_t();

    virtual void set_parameter() override {
        msg_timeout = Parameter_t::msg_timeout.cmd; }

    virtual inline bool is_received(const ros::Time &now_time) const override { 
        return (now_time - rcv_stamp).toSec() < msg_timeout; }

    void feed(quadrotor_msgs::PositionCommandConstPtr);


    quadrotor_msgs::PositionCommand msg;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d j;
    double yaw;
    double yaw_rate;

private:
};

#endif














