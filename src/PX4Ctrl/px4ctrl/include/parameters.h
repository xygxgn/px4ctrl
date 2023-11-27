#ifndef __PARAMETERS_H
#define __PARAMETERS_H

#include <ros/ros.h>


class Parameter_t
{
public:
    double mass;
    double gravity;
    double ctrl_freq_max;
    double max_manual_vel;

    struct MsgTimeout
    {
        double state;
        double rc;
        double imu;
        double battery;
        double odom;
        double cmd;
    } msg_timeout;

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	} rc_reverse;

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	} thrust_mapping;

    struct Gain
    {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
    } gain;
    
    
    Parameter_t() {}
    Parameter_t(const ros::NodeHandle &nh) { config_from_ros_handle(nh); }

private:
    void config_from_ros_handle(const ros::NodeHandle &);

    template <typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
    {
        if (!nh.getParam(name, val))
        {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    }

};


#endif