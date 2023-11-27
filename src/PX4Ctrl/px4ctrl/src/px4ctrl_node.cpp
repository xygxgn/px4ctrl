#include "ros_register.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    Parameter_t param(nh);

    /* Finite state machine */
    PX4FSM fsm(param);

    /* Register ROS Master */
    ROSRegister ros_register(nh, fsm);


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

    /* machine begins */
    ros::Rate rate(param.ctrl_freq_max);
    while (ros::ok())
    {
        fsm.process();
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
