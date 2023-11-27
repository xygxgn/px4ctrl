#include "parameters.h"


void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
    read_essential_param(nh, "mass", mass);
    read_essential_param(nh, "gravity", gravity);
    read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
    read_essential_param(nh, "max_manual_vel", max_manual_vel);
    

    read_essential_param(nh, "msg_timeout/state", msg_timeout.state);
    read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
    read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
    read_essential_param(nh, "msg_timeout/battery", msg_timeout.battery);
    read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
    read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);


	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);


	read_essential_param(nh, "thrust_model/K1", thrust_mapping.K1);
	read_essential_param(nh, "thrust_model/K2", thrust_mapping.K2);
	read_essential_param(nh, "thrust_model/K3", thrust_mapping.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thrust_mapping.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thrust_mapping.hover_percentage);
	

	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);
}