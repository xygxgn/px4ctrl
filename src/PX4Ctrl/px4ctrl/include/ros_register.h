#ifndef __ROS_REGISTER_H
#define __ROS_REGISTER_H

#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>

#include "PX4FSM.h"


ros::Subscriber state_sub;
ros::Subscriber rc_sub;
ros::Subscriber bat_sub;
ros::Subscriber imu_sub;
ros::Subscriber odom_sub;
ros::Subscriber cmd_sub;

ros::Publisher set_attitude_pub; // The publisher used to advertise the setpoint commands

ros::ServiceClient mode_srv; // The service used to set the mode of FCU
ros::ServiceClient arming_srv; // The service used to arming the drone
ros::ServiceClient reboot_srv; // The service used to reboot FCU


class ROSRegister
{
public:
    ROSRegister(ros::NodeHandle &, PX4FSM &);
};

ROSRegister::ROSRegister(ros::NodeHandle &nh, PX4FSM &fsm)
{
    /* Subscriber */
    state_sub = 
        nh.subscribe<mavros_msgs::State>("/mavros/state", 
                                         10, 
                                         boost::bind(&State_Data_t::feed, &fsm.fcu_data.state_data, _1));

    rc_sub = 
        nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 
                                        10,
                                        boost::bind(&RC_Data_t::feed, &fsm.fcu_data.rc_data, _1));

    bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.fcu_data.battery_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    imu_sub = 
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                         100,
                                         boost::bind(&Imu_Data_t::feed, &fsm.fcu_data.imu_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    odom_sub = 
        nh.subscribe<nav_msgs::Odometry>("odom", 
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    cmd_sub = 
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                         100,
                                         boost::bind(&Command_Data_t::feed, &fsm.command_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    /* Publisher */
    set_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    /* Service */
    mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    reboot_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
}

#endif