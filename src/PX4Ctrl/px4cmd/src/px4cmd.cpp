#include <ros/ros.h>
#include "fsm.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/PositionCommand.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4cmd");
    ros::NodeHandle nh("~");
    Parameter_t param(nh);
    FSM fsm(param);


    /* Subscriber */
    ros::Subscriber state_sub = 
        nh.subscribe<mavros_msgs::State>("/mavros/state", 
                                         10, 
                                         boost::bind(&State_Data_t::feed, &fsm.fcu_data.state_data, _1));

    ros::Subscriber rc_sub = 
        nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 
                                        10,
                                        boost::bind(&RC_Data_t::feed, &fsm.fcu_data.rc_data, _1));

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.fcu_data.battery_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub = 
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                         100,
                                         boost::bind(&Imu_Data_t::feed, &fsm.fcu_data.imu_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber odom_sub = 
        nh.subscribe<nav_msgs::Odometry>("odom", 
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub = 
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                         100,
                                         boost::bind(&Command_Data_t::feed, &fsm.command_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    /* Publisher */
    fsm.fcu_set_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);

    /* Service */
    fsm.fcu_mode_srv = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.fcu_arming_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.fcu_reboot_srv = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");


    ros::Duration(0.5).sleep();

    /* RC check */
    ROS_INFO("PX4CTRL] Waiting for RC");
    while (ros::ok())
    {
        ros::spinOnce();
        if (fsm.fcu_data.rc_data.is_received(ros::Time::now()))
        {
            ROS_INFO("[PX4CTRL] RC received.");
            break;
        }
        ros::Duration(0.1).sleep();
    }

    /* Process */
    ros::Rate rate(param.ctrl_freq_max);
    while (ros::ok())
    {
        fsm.process();
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
