#ifndef __PARAMETERS_H
#define __PARAMETERS_H

#include <ros/ros.h>


class Parameter_t
{
public:
    static double mass;
    static double gravity;
    static double ctrl_freq_max;
    static double max_manual_vel;

    static struct MsgTimeout
    {
        double state;
        double rc;
        double imu;
        double battery;
        double odom;
        double cmd;
    } msg_timeout;

	static struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	} rc_reverse;

	static struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	} thrust_mapping;

    static struct Gain
    {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
    } gain;
    
    
    Parameter_t() {}

    static void config_from_ros_handle(const ros::NodeHandle &);

private:
    template <typename TName, typename TVal>
    static void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
    {
        if (!nh.getParam(name, val))
        {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    }

};

#endif