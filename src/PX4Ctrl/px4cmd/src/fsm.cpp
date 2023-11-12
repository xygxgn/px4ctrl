/**
 * @file fsm.cpp
 * @brief This file defines the structure finite state machine (FSM). 
 */
#include "fsm.h"


FSM::FSM(const Parameter_t &param) : param_(param), 
    fcu_data(FCU_Data_t(param)), 
    odom_data(Odom_Data_t(param)), 
    command_data(Command_Data_t(param)),
    controller(LinearController(param))
{
    state_ = MANUAL;
    hover_pose.setZero();
    fsm_clock.command = ros::Time(0);
}


void FSM::process()
{
    ros::Time now_time = ros::Time::now();

    Desired_PVAQ_t des(odom_data); // desired PVAQ
    Controller_t u; // Linear control output

    /************************************/
    /* Step1: finite state machine runs */
    /************************************/
    switch (state_)
    {
    case MANUAL:
    {
        /* MANUAL */
        if (fcu_data.state_data.current_state.mode == "OFFBOARD")
        {
            toggle_offboard(false); // try to exit OFFBOARD mode
        }
        /* HOVER */
        if (fcu_data.rc_data.enter_hover && !fcu_data.rc_data.exit_command) // try to jump to HOVER mode
        {
            if (!odom_data.is_received(now_time))
            {
                ROS_ERROR("[px4ctrl] Reject HOVER(L2). No odom!");
            }
            else if (odom_data.v.norm() > 3.0)
            {
                ROS_ERROR("[px4ctrl] Reject HOVER(L2). \
                    Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
            }
            else if (command_data.is_received(now_time))
            {
                ROS_ERROR("[px4ctrl] Reject COMMAND(L3). \
                    Sending commands before toggling into COMMAND is not allowed. Stop sending commands now!");
            }
            else /* enter HOVER */
            {
                state_ = HOVER;

                controller.resetThrustMapping();
                set_hover_with_odom();

                toggle_offboard(true); // try to enter OFFBOARD mode
                
                for (int i = 0; i != 10 && ros::ok(); i++) // wait for 0.1 seconds to allow mode change by FCU
                {
                    ros::Duration(0.01).sleep();
                    ros::spinOnce();
                }

                ROS_INFO("\033[32m[px4ctrl] MANUAL(L1) --> HOVER(L2).\033[32m");
                ROS_WARN("[px4ctrl] Not allow setpoint command.");
            }
        }
        /* MANUAL */
        // do notion

        break;
    }
    case HOVER:
    {
        /* MANUAL */
        if (!fcu_data.rc_data.is_hover || !odom_data.is_received(now_time))
        {
            state_ = MANUAL;

            toggle_offboard(false); // try to exit OFFBOARD mode

            ROS_WARN("[px4ctrl] HOVER(L2) --> MANUAL(L1).");
        }
        /* COMMAND */
        else if (fcu_data.rc_data.is_command && command_data.is_received(now_time))
        {
            if (fcu_data.state_data.current_state.mode == "OFFBOARD")
            {
                state_ = COMMAND;

                des = get_command_des();

                ROS_INFO("\033[32m[px4ctrl] HOVER(L2) --> COMMAND(L3).\033[32m");
            }
        }
        /* HOVER */
        else
        {
            if (fcu_data.rc_data.enter_command)
            {
                ROS_INFO("\033[32m[px4ctrl] Allow setpoint command.\033[32m");
            }

            if (fcu_data.rc_data.exit_command)
            {
                ROS_WARN("[px4ctrl] Not allow setpoint command.");
            }
            
            if (command_data.is_received(now_time) && (now_time - fsm_clock.command).toSec() > 0.5)
            {
                fsm_clock.command = now_time;
                
                ROS_WARN("[px4ctrl] Reject COMMAND(L3). \
                    Sending commands before toggling into COMMAND, which is not recommanded.");
            }

            set_hover_with_rc(); // Adjusting the hovering pose by RC
            des = get_hover_des(); // Calculate the desired PVAQ required to hover
        } 

        break;
    }
    case COMMAND:
    {
        /* MANUAL */
        if (!(fcu_data.state_data.current_state.mode == "OFFBOARD"))
        {
            state_ = MANUAL;
            ROS_ERROR("[px4ctrl] Keep \"OFFBOARD\" mode to maintain HOVER!");
            ROS_WARN("[px4ctrl] COMMAND(L3) --> MANUAL(L1).");
        }
        /* MANUAL */
        if (!fcu_data.rc_data.is_hover || !odom_data.is_received(now_time)) 
        {
            state_ = MANUAL;

            toggle_offboard(false); // try to exit OFFBOARD mode

            ROS_WARN("[px4ctrl] COMMAND(L3) --> MANUAL(L1).");
        }
        /* HOVER not allow command */
        else if (!fcu_data.rc_data.is_command)
        {
            state_ = HOVER;

            set_hover_with_odom();
            des = get_hover_des();

            ROS_WARN("[px4ctrl] COMMAND(L3) --> HOVER(L2).");
            ROS_WARN("[px4ctrl] Not allow setpoint command.");
        }
        /* HOVER allow command */
        else if (!command_data.is_received(now_time))
        {
            state_ = HOVER;

            set_hover_with_odom();
            des = get_hover_des();

            ROS_WARN("[px4ctrl] COMMAND(L3) --> HOVER(L2).");
            ROS_INFO("\033[32m[px4ctrl] Waiting for setpoint commands.\033[32m");
        }
        /* COMMAND */
        else
        {
            des = get_command_des();
        }

        break;
    }
    default:
        break;
    }


    /********************************/
    /* Step2: estimate thrust model */
    /********************************/
	if (state_ == HOVER || state_ == COMMAND)
	{
		controller.estimateThrustModel(fcu_data.imu_data.a, param_);
	}

    controller.calculateControl(des, odom_data, fcu_data.imu_data, u);


    /***********************************/
    /* Step3: publish setpoint control */
    /***********************************/
    publish_attitude_ctrl(u, now_time);


    /************************/
    /* Step4: reset RC data */
    /************************/
    fcu_data.rc_data.enter_hover = false;
    fcu_data.rc_data.enter_command = false;
    fcu_data.rc_data.exit_hover = false;
    fcu_data.rc_data.exit_command = false;
}


/** @brief Calculate the desired PVAQ required to hover
 * @note MUST align odometry frame with FCU frame!!!
 * @return Desired PVAQ
 */
Desired_PVAQ_t FSM::get_command_des()
{
	Desired_PVAQ_t des;
	des.p = command_data.p;
	des.v = command_data.v;
	des.a = command_data.a;
	des.yaw = command_data.yaw;
	des.yaw_rate = command_data.yaw_rate;

	return des;
}


/** @brief Calculate the desired PVAQ required to hover
 * @note MUST align odometry frame with FCU frame!!!
 * @return Desired PVAQ
 */
Desired_PVAQ_t FSM::get_hover_des()
{
	Desired_PVAQ_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;
    // ROS_INFO("\nx = %.12f\ny = %.12f\nz = %.12f\nyaw:%f", hover_pose(0), hover_pose(1),hover_pose(2),hover_pose(3));

	return des;
}


/** @brief Set hover pose with the aid of odometry.
 */
void FSM::set_hover_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = uav_utils::get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}


/** @brief Set hover pose with the aid of RC (Reach a similar result to POSITION mode) */
void FSM::set_hover_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	hover_pose(0) += fcu_data.rc_data.channels[1] * param_.max_manual_vel * delta_t * (param_.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += fcu_data.rc_data.channels[0] * param_.max_manual_vel * delta_t * (param_.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += fcu_data.rc_data.channels[2] * param_.max_manual_vel * delta_t * (param_.rc_reverse.throttle ? -1 : 1);
	hover_pose(3) += fcu_data.rc_data.channels[3] * param_.max_manual_vel * delta_t * (param_.rc_reverse.yaw ? 1 : -1);

	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;
}


/** @brief toggle OFFBOARD or not, the setpoint publishing rate must be faster than 2Hz
 * @param on_off enter/exit OFFBOARD mode if true/false
 */
bool FSM::toggle_offboard(bool on_off)
{
    mavros_msgs::SetMode offb_set_mode;

    if (on_off)
    {
        if (fcu_data.state_data.current_state.mode == "OFFBOARD")
        {
            ROS_INFO("\"OFFBOARD\" enabled!"); // already \"OFFBOARD\" mode!
            return true;
        }

        fcu_data.state_data.state_before_offboard = fcu_data.state_data.current_state;
        if (fcu_data.state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
        {
            fcu_data.state_data.state_before_offboard.mode = "MANUAL";
        }

        offb_set_mode.request.custom_mode = "OFFBOARD";
        if (!(fcu_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
        {
            ROS_ERROR("Enter \"OFFBOARD\" rejected by PX4!");
            return false;
        }
        ROS_INFO("\"OFFBOARD\" enabled");
    }
    else
    {
        if (!(fcu_data.state_data.current_state.mode == "OFFBOARD"))
        {
            ROS_INFO("\"OFFBOARD\" disabled!"); // already \"OFFBOARD\" mode!
            return true;
        }

        offb_set_mode.request.custom_mode = fcu_data.state_data.state_before_offboard.mode;
        if (fcu_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("\"OFFBOARD\" disabled!"); // enter the state before OFFBOARD
            return true;
        }

        offb_set_mode.request.custom_mode = "MANUAL";
        if (fcu_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {

            ROS_WARN("Exit \"OFFBOARD\" but enter \"MANUAL\" unfortunately!");
            return true;
        }

        ROS_ERROR("Exit \"OFFBOARD\" rejected by PX4!");
        return false;
    }
    // ROS_INFO("Toggle \"OFFBOARD\" successfully!");
    return true;
}


/** @brief publish attitude control messages to /mavros/setpoint_raw/attitude */
void FSM::publish_attitude_ctrl(const Controller_t &u, const ros::Time &now_time)
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = now_time;

    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    
    msg.orientation.x = u.q.x();
    msg.orientation.y = u.q.y();
    msg.orientation.z = u.q.z();
    msg.orientation.w = u.q.w();

    msg.thrust = u.thrust;


    fcu_set_attitude_pub.publish(msg);
}
