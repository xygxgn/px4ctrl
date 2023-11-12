/**
 * @file inputData.cpp
 * @brief This file defines the structure of the data from the flight control units (FCU), odometry and setpoint command. 
 */
#include "inputData.h"

/*****************/
/* Odometry data */
/*****************/
/** @brief Odom_Data constructor */
Odom_Data_t::Odom_Data_t(const Parameter_t &param) : param_(param) 
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
}

/** @brief Odom_Data callback */
void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    odom_msg = *pMsg;

    uav_utils::extract_odometry(pMsg, p, v, q, w);
}


/****************/
/* Command data */
/****************/

/** @brief Command_Data constructor */
Command_Data_t::Command_Data_t(const Parameter_t &param) : param_(param) 
{
    rcv_stamp = ros::Time(0);
}

/** @brief Command_Data callback */
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
