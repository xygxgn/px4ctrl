#ifndef __PX4FSM_H
#define __PX4FSM_H

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include "linear_controller.h"


extern ros::Publisher set_attitude_pub;

extern ros::ServiceClient mode_srv;
extern ros::ServiceClient arming_srv;
extern ros::ServiceClient reboot_srv;


class PX4FSM
{
public:
    FCU_Data_t fcu_data;
    Odom_Data_t odom_data;
    Command_Data_t command_data;

    LinearController controller;

	Eigen::Vector4d hover_pose; // [x,y,z,yaw]'
	ros::Time last_set_hover_pose_time;


    /* Constructor */
    PX4FSM();

    void set_parameter();

    void process();

private:
    double max_manual_vel;
    Parameter_t::RCReverse rc_reverse;

    /* State of drone */
    enum State_t
    {
        MANUAL = 1,
        HOVER,
        COMMAND
    } state_;


    /* Set hover mode */
    void set_hover_with_rc();
    void set_hover_with_odom();

    /* Get desired PVAQ for hover */
    Desired_PVAQ_t get_hover_des();
    Desired_PVAQ_t get_command_des();

    // Service/Publisher
    bool toggle_offboard(bool on_off);
	void publish_attitude_ctrl(const Controller_t &, const ros::Time &);
    
};


#endif