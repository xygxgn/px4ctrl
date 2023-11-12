/*****************************/
/* Flight control units data */
/*****************************/
#include <Eigen/Dense>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>

#include "readParam.h"


class Base_Data_t
{
public:
    ros::Time rcv_stamp; // The timestamp of the received data
    virtual inline bool is_received(const ros::Time &) const = 0;
};

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



