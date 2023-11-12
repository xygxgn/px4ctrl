#include "fcuData.h"


/**************/
/* State_Data */
/**************/
/** @brief State_Data constructor */
State_Data_t::State_Data_t(const Parameter_t &param) : param_(param)
{
    rcv_stamp = ros::Time(0);

    current_state.mode = "MUNUAL";
    state_before_offboard = current_state;
}

/** @brief State_Data callback */
void State_Data_t::feed(const mavros_msgs::StateConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    current_state = *pMsg;
}


/***********/
/* RC_Data */
/***********/
/** @brief RC_Data constructor */
RC_Data_t::RC_Data_t(const Parameter_t &param) : param_(param) 
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    is_hover = false;
    is_command = false;
    enter_hover = false;
    enter_command = false;

    for (int i = 0; i < 4; i++)
    {
        channels[i] = 0.0;
    }
}

/** @brief RC_Data callback */
void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    rc_msg = *pMsg;


    for (int i = 0; i != 4; i++)
    {
        channels[i] = ((double)rc_msg.channels[i] - 1500.0) / 500.0; // channels[i] \in [-1,1]
        if (channels[i] > DEAD_ZONE)
            channels[i] = (channels[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (channels[i] < -DEAD_ZONE)
            channels[i] = (channels[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            channels[i] = 0.0;
    }

    mode = ((double)rc_msg.channels[4] - 1000.0) / 1000.0; // toggle mode (set it as SB)
    gear = ((double)rc_msg.channels[5] - 1000.0) / 1000.0; // toggle gear (set it as SC)
    reboot = ((double)rc_msg.channels[7] - 1000.0) / 1000.0; // reboot (set it as SD)

    
    if (!have_init_last_rc)
    {
        have_init_last_rc = true;
        last_mode = mode;
        last_gear = gear;
        last_reboot = reboot;
    }

    if (gear > HOVER_OFFBOARD_THRESHOLD && gear < COMMAND_OFFBOARD_THRESHOLD) // SC is on middle
    {
        is_hover = true;
        is_command = false;
        if (!(last_gear > HOVER_OFFBOARD_THRESHOLD)) // bottom --> middle
        {
            enter_hover = true;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }
        else if (!(last_gear < COMMAND_OFFBOARD_THRESHOLD)) // top --> middle
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = true;
        }
        else // hold middle
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }
    }
    else if (!(gear < COMMAND_OFFBOARD_THRESHOLD)) // SC is on top
    {
        is_hover = true;
        is_command = true;
        if (last_gear > HOVER_OFFBOARD_THRESHOLD && last_gear < COMMAND_OFFBOARD_THRESHOLD) // middle --> top
        {
            enter_hover = false;
            enter_command = true;
            exit_hover = false;
            exit_command = false;
        }
        else if (!(last_gear > HOVER_OFFBOARD_THRESHOLD)) // bottom --> top
        {
            enter_hover = false;
            enter_command = true;
            exit_hover = false;
            exit_command = false;
        }
        else // hold top
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }
    }
    else // SC is on bottom
    {
        is_hover = false;
        is_command = false;
        if (last_gear > HOVER_OFFBOARD_THRESHOLD && last_gear < COMMAND_OFFBOARD_THRESHOLD)
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = true;
            exit_command = false;
        }
        else if (!(last_gear < COMMAND_OFFBOARD_THRESHOLD)) // top --> bottom
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = true;
            exit_command = true;
        }
        else
        {
            enter_hover = false;
            enter_command = false;
            exit_hover = false;
            exit_command = false;
        }

    }

    last_mode = mode;
    last_gear = gear;
    last_reboot = reboot;
}


/****************/
/* Battery_Data */
/****************/
/** @brief Battery_Data constructor */
Battery_Data_t::Battery_Data_t(const Parameter_t &param) : param_(param) 
{
    rcv_stamp = ros::Time(0);
}

/** @brief Battery_Data callback */
void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    battery_msg = *pMsg;
}


/************/
/* Imu_Data */
/************/
/** @brief Imu_Data constructor */
Imu_Data_t::Imu_Data_t(const Parameter_t &param) : param_(param) 
{
    rcv_stamp = ros::Time(0);
}

/** @brief Imu_Data callback */
void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    imu_msg = *pMsg;

    q.x() = imu_msg.orientation.x;
    q.y() = imu_msg.orientation.y;
    q.z() = imu_msg.orientation.z;
    q.w() = imu_msg.orientation.w;

    w(0) = imu_msg.angular_velocity.x;
    w(1) = imu_msg.angular_velocity.y;
    w(2) = imu_msg.angular_velocity.z;

    a(0) = imu_msg.linear_acceleration.x;
    a(1) = imu_msg.linear_acceleration.y;
    a(2) = imu_msg.linear_acceleration.z;
}
