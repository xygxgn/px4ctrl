#ifndef __TF_BROADCASTER
#define __TF_BROADCASTER

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "uav_utils/utils.h"


#include <Eigen/Eigen>


ros::Publisher vision_pose_pub;
ros::Publisher vision_odom_pub;
ros::Publisher planner_cmd_pub;


class TF_Broadcaster
{
public:
    TF_Broadcaster(ros::NodeHandle &);

    virtual void tf_world_to_map() = 0;
    virtual void tf_odom_to_baselink(const std::string) = 0;
    virtual void vision_callback(nav_msgs::OdometryConstPtr) = 0;
    virtual void command_callback(quadrotor_msgs::PositionCommandConstPtr) = 0;

protected:
    tf2_ros::StaticTransformBroadcaster broadcaster_;
    geometry_msgs::TransformStamped ts_;
};



TF_Broadcaster::TF_Broadcaster(ros::NodeHandle &nh)
{
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vision_pose", 1000);
    vision_odom_pub = nh.advertise<nav_msgs::Odometry>("vision_odom", 1000);
    planner_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("planner_cmd",100);
}

#endif