#ifndef __FSM_H
#define __FSM_H

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include "readParam.h"
#include "inputData.h"
#include "controller.h"


class FSM
{
public:
    FCU_Data_t fcu_data;
    Odom_Data_t odom_data;
    Command_Data_t command_data;


    ros::Publisher fcu_set_attitude_pub; // The publisher used to advertise the setpoint commands
    
	ros::ServiceClient fcu_mode_srv; // The service used to set the mode of FCU
	ros::ServiceClient fcu_arming_srv; // The service used to arming the drone
	ros::ServiceClient fcu_reboot_srv; // The service used to reboot FCU

    LinearController controller;

	Eigen::Vector4d hover_pose; // [x,y,z,yaw]'
	ros::Time last_set_hover_pose_time;


    /* Constructor */
    FSM(const Parameter_t &);

    void process();

private:
    Parameter_t param_;

    /* FSM clock */
    struct FSM_CLOCK
    {
        ros::Time command;
    } fsm_clock;

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