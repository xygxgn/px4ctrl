/**
 * @file input.cpp
 * @brief This file defines the structure of the data from the flight control units (FCU), odometry and setpoint command. 
 */
#include "input.h"


/***********************************/
/* Flight Control Units (FCU) data */
/***********************************/

/** @brief State data */
State_Data_t::State_Data_t()
{
    rcv_stamp = ros::Time(0);
    set_parameter();

    current_state.mode = "MUNUAL";
    state_before_offboard = current_state;
}


void State_Data_t::feed(const mavros_msgs::StateConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    current_state = *pMsg;
}


/** @brief Remote Control data */
RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);
    set_parameter();

    last_mode = -1.0;
    last_gear = -1.0;

    is_hover = false;
    is_command = false;
    enter_hover = false;
    enter_command = false;

    for (int i = 0; i < 4; i++)
    {
        channels[i] = 0.0;
    }
}


void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    rc_msg = *pMsg;


    for (int i = 0; i != 4; i++)
    {
        channels[i] = ((double)rc_msg.channels[i] - 1500.0) / 500.0; // channels[i] \in [-1,1]
        if (channels[i] > DEAD_ZONE)
            channels[i] = (channels[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (channels[i] < -DEAD_ZONE)
            channels[i] = (channels[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            channels[i] = 0.0;
    }

    mode = ((double)rc_msg.channels[4] - 1000.0) / 1000.0; // toggle mode (set it as SB)
    gear = ((double)rc_msg.channels[5] - 1000.0) / 1000.0; // toggle gear (set it as SC)
    reboot = ((double)rc_msg.channels[7] - 1000.0) / 1000.0; // reboot (set it as SD)

    
    if (!have_init_last_rc)
    {
        have_init_last_rc = true;
        last_mode = mode;
        last_gear = gear;
        last_reboot = reboot;
    }

    if (gear > HOVER_OFFBOARD_THRESHOLD && gear < COMMAND_OFFBOARD_THRESHOLD) // SC is on middle
    {
        is_hover = true;
        is_command = false;
        if (!(last_gear > HOVER_OFFBOARD_THRESHOLD)) // bottom --> middle
        {
            enter_hover = true;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }
        else if (!(last_gear < COMMAND_OFFBOARD_THRESHOLD)) // top --> middle
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = true;
        }
        else // hold middle
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }
    }
    else if (!(gear < COMMAND_OFFBOARD_THRESHOLD)) // SC is on top
    {
        is_hover = true;
        is_command = true;
        if (last_gear > HOVER_OFFBOARD_THRESHOLD && last_gear < COMMAND_OFFBOARD_THRESHOLD) // middle --> top
        {
            enter_hover = false;
            enter_command = true;
            exit_hover = false;
            exit_command = false;
        }
        else if (!(last_gear > HOVER_OFFBOARD_THRESHOLD)) // bottom --> top
        {
            enter_hover = false;
            enter_command = true;
            exit_hover = false;
            exit_command = false;
        }
        else // hold top
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }
    }
    else // SC is on bottom
    {
        is_hover = false;
        is_command = false;
        if (last_gear > HOVER_OFFBOARD_THRESHOLD && last_gear < COMMAND_OFFBOARD_THRESHOLD)
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = true;
            exit_command = false;
        }
        else if (!(last_gear < COMMAND_OFFBOARD_THRESHOLD)) // top --> bottom
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = true;
            exit_command = true;
        }
        else
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }

    }

    last_mode = mode;
    last_gear = gear;
    last_reboot = reboot;
}


/** @brief Battery data */
Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
    set_parameter();
}


void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    battery_msg = *pMsg;
}


/** @brief Inertial Measurement Unit data */
Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
    set_parameter();
}


void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    imu_msg = *pMsg;

    q.x() = imu_msg.orientation.x;
    q.y() = imu_msg.orientation.y;
    q.z() = imu_msg.orientation.z;
    q.w() = imu_msg.orientation.w;

    w(0) = imu_msg.angular_velocity.x;
    w(1) = imu_msg.angular_velocity.y;
    w(2) = imu_msg.angular_velocity.z;

    a(0) = imu_msg.linear_acceleration.x;
    a(1) = imu_msg.linear_acceleration.y;
    a(2) = imu_msg.linear_acceleration.z;
}


/*****************************/
/* Odometry and Command data */
/*****************************/

/** @brief Odom data */
Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    set_parameter();
    q.setIdentity();
}

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    odom_msg = *pMsg;

    uav_utils::extract_odometry(pMsg, p, v, q, w);
}


/** @brief Command data */
Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
    set_parameter();
}


void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    msg = *pMsg;
    
    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}
