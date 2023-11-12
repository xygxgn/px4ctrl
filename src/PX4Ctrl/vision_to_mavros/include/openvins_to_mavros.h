#ifndef __OPENVINS_TO_MAVROS
#define __OPENVINS_TO_MAVROS

#include"tf_broadcaster.h"


namespace openvins_to_mavros {

class TF_EDN_to_ENU : public TF_Broadcaster
{
public:
    TF_EDN_to_ENU(ros::NodeHandle &nh) : TF_Broadcaster(nh) {}

    virtual void tf_world_to_map() override;
    virtual void tf_odom_to_baselink(const std::string) override;

    virtual void vision_callback(nav_msgs::OdometryConstPtr) override;
    virtual void command_callback(quadrotor_msgs::PositionCommandConstPtr) override;
};


class TF_ENU_to_ENU : public TF_Broadcaster
{
public:
    TF_ENU_to_ENU(ros::NodeHandle &nh) : TF_Broadcaster(nh) {}

    virtual void tf_world_to_map() override;
    virtual void tf_odom_to_baselink(const std::string) override;
    
    virtual void vision_callback(nav_msgs::OdometryConstPtr) override;
    virtual void command_callback(quadrotor_msgs::PositionCommandConstPtr) override;
};



/**********************************************************/
/* Transform East-Down-North frame to East-North-Up frame */
/**********************************************************/

/** @brief transform frame from "world" to "map"  */
void TF_EDN_to_ENU::tf_world_to_map()
{
    std::string source_frame_id = "world";
    std::string target_frame_id = "map";

    ts_.header.seq = 100;
    ts_.header.stamp = ros::Time::now();
    ts_.header.frame_id = source_frame_id;
    ts_.child_frame_id = target_frame_id;

    ts_.transform.translation.x = 0.0;
    ts_.transform.translation.y = 0.0;
    ts_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // 绕 source_frame 定轴转动
    ts_.transform.rotation.x = q.getX();
    ts_.transform.rotation.y = q.getY();
    ts_.transform.rotation.z = q.getZ();
    ts_.transform.rotation.w = q.getW();

    broadcaster_.sendTransform(ts_);
}


/** @brief transform frame from "odom" to "base_link"  */
void TF_EDN_to_ENU::tf_odom_to_baselink(const std::string source_frame_id)
{
    std::string target_frame_id = "base_link";

    ts_.header.seq = 100;
    ts_.header.stamp = ros::Time::now();
    ts_.header.frame_id = source_frame_id;
    ts_.child_frame_id = target_frame_id;

    ts_.transform.translation.x = 0.0;
    ts_.transform.translation.y = 0.0;
    ts_.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(M_PI/2, -M_PI/2, 0); // 绕 source_frame 坐标系定轴转动
    ts_.transform.rotation.x = q.getX();
    ts_.transform.rotation.y = q.getY();
    ts_.transform.rotation.z = q.getZ();
    ts_.transform.rotation.w = q.getW();

    broadcaster_.sendTransform(ts_);
}


/** @brief publish base_link pose in "map" frame to mavros */
void TF_EDN_to_ENU::vision_callback(nav_msgs::OdometryConstPtr pMsg)
{
    Eigen::Vector3d p_mav;
    Eigen::Vector3d v_mav;
    Eigen::Quaterniond q_mav;
    if(pMsg->header.frame_id == "world")
    {
        p_mav = Eigen::Vector3d(pMsg->pose.pose.position.x, 
                                pMsg->pose.pose.position.y, 
                                pMsg->pose.pose.position.z);

        v_mav = Eigen::Vector3d(pMsg->twist.twist.linear.z, 
                                -pMsg->twist.twist.linear.x, 
                                -pMsg->twist.twist.linear.y);

        q_mav = Eigen::Quaterniond(pMsg->pose.pose.orientation.w, 
                                   pMsg->pose.pose.orientation.x, 
                                   pMsg->pose.pose.orientation.y, 
                                   pMsg->pose.pose.orientation.z);
        
        Eigen::AngleAxisd roll(M_PI/2, Eigen::Vector3d::UnitX()); // 绕 x 轴旋转 pi / 2
        Eigen::AngleAxisd pitch(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(M_PI/2, Eigen::Vector3d::UnitZ());

        q_mav = q_mav * Eigen::Quaterniond(roll * pitch * yaw);


        geometry_msgs::PoseStamped vision_pose;

        vision_pose.header.frame_id = "map";
        vision_pose.header.stamp = ros::Time::now();

        vision_pose.pose.position.x = p_mav[0];
        vision_pose.pose.position.y = p_mav[1];
        vision_pose.pose.position.z = p_mav[2];

        vision_pose.pose.orientation.x = q_mav.x();
        vision_pose.pose.orientation.y = q_mav.y();
        vision_pose.pose.orientation.z = q_mav.z();
        vision_pose.pose.orientation.w = q_mav.w();

        ROS_INFO("\nposition:\n   x: %.18f\n   y: %.18f\n   z: %.18f"\
                "\norientation:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f"\
                "\nvelocity:\n   x: %.18f\n   y: %.18f\n   z: %.18f"\
                "\nangle:\n   yaw:%.18f",
                    p_mav[0], p_mav[1], p_mav[2], q_mav.x(), q_mav.y(), q_mav.z(), q_mav.w(),
                    v_mav[0], v_mav[1], v_mav[2], uav_utils::get_yaw_from_quaternion(q_mav) * 180.0 / M_PI);
                    
        vision_pose_pub.publish(vision_pose);



        nav_msgs::Odometry vision_odom = *pMsg;

        vision_odom.header.frame_id = "map";
        vision_odom.child_frame_id = "map";

        vision_odom.pose.pose.position.x = p_mav[0];
        vision_odom.pose.pose.position.y = p_mav[1];
        vision_odom.pose.pose.position.z = p_mav[2];

        vision_odom.pose.pose.orientation.x = q_mav.x();
        vision_odom.pose.pose.orientation.y = q_mav.y();
        vision_odom.pose.pose.orientation.z = q_mav.z();
        vision_odom.pose.pose.orientation.w = q_mav.w();

        vision_odom.twist.twist.linear.x = v_mav[0];
        vision_odom.twist.twist.linear.y = v_mav[1];
        vision_odom.twist.twist.linear.z = v_mav[2];

        vision_odom_pub.publish(vision_odom);
    }
}


void TF_EDN_to_ENU::command_callback(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    quadrotor_msgs::PositionCommand cmd;

    cmd.position.x = pMsg->position.x;
    cmd.position.y = pMsg->position.y;
    cmd.position.z = pMsg->position.z;

    cmd.velocity.x = pMsg->velocity.z;
    cmd.velocity.y = pMsg->velocity.x;
    cmd.velocity.z = -pMsg->velocity.y;

    cmd.acceleration.x = pMsg->acceleration.z;
    cmd.acceleration.y = -pMsg->acceleration.x;
    cmd.acceleration.z = -pMsg->acceleration.y;

    cmd.yaw = uav_utils::normalize_angle(pMsg->yaw + M_PI/2);
    cmd.yaw_dot = pMsg->yaw_dot;

    planner_cmd_pub.publish(cmd);
}



/**********************************************************/
/* Transform East-North-Up frame to East-North-Up frame */
/**********************************************************/

/** @brief transform frame from "world" to "map"  */
void TF_ENU_to_ENU::tf_world_to_map()
{
    std::string source_frame_id = "world";
    std::string target_frame_id = "map";

    ts_.header.seq = 100;
    ts_.header.stamp = ros::Time::now();
    ts_.header.frame_id = source_frame_id;
    ts_.child_frame_id = target_frame_id;

    ts_.transform.translation.x = 0.0;
    ts_.transform.translation.y = 0.0;
    ts_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI); // 绕 source_frame 定轴转动
    ts_.transform.rotation.x = q.getX();
    ts_.transform.rotation.y = q.getY();
    ts_.transform.rotation.z = q.getZ();
    ts_.transform.rotation.w = q.getW();

    broadcaster_.sendTransform(ts_);
}


/** @brief transform frame from "body" to "base_link"  */
void TF_ENU_to_ENU::tf_odom_to_baselink(const std::string source_frame_id)
{
    std::string target_frame_id = "base_link";

    ts_.header.seq = 100;
    ts_.header.stamp = ros::Time::now();
    ts_.header.frame_id = source_frame_id;
    ts_.child_frame_id = target_frame_id;

    ts_.transform.translation.x = 0.0;
    ts_.transform.translation.y = 0.0;
    ts_.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // 绕 source_frame 坐标系定轴转动
    ts_.transform.rotation.x = q.getX();
    ts_.transform.rotation.y = q.getY();
    ts_.transform.rotation.z = q.getZ();
    ts_.transform.rotation.w = q.getW();

    broadcaster_.sendTransform(ts_);
}


/** @brief publish base_link pose in "map" frame to mavros */
void TF_ENU_to_ENU::vision_callback(nav_msgs::OdometryConstPtr pMsg)
{
    Eigen::Vector3d p_mav;
    Eigen::Vector3d v_mav;
    Eigen::Quaterniond q_mav;
    if(pMsg->header.frame_id == "world")
    {
        p_mav = Eigen::Vector3d(-pMsg->pose.pose.position.x,
                                -pMsg->pose.pose.position.y, 
                                pMsg->pose.pose.position.z);

        v_mav = Eigen::Vector3d(pMsg->twist.twist.linear.x, 
                                pMsg->twist.twist.linear.y, 
                                pMsg->twist.twist.linear.z);

        q_mav = Eigen::Quaterniond(pMsg->pose.pose.orientation.w, 
                                   pMsg->pose.pose.orientation.x, 
                                   pMsg->pose.pose.orientation.y, 
                                   pMsg->pose.pose.orientation.z);

        Eigen::AngleAxisd roll(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(M_PI, Eigen::Vector3d::UnitZ());

        q_mav = q_mav * Eigen::Quaterniond(roll * pitch * yaw);


        geometry_msgs::PoseStamped vision_pose;

        vision_pose.header.frame_id = "map";
        vision_pose.header.stamp = ros::Time::now();

        vision_pose.pose.position.x = p_mav[0];
        vision_pose.pose.position.y = p_mav[1];
        vision_pose.pose.position.z = p_mav[2];

        vision_pose.pose.orientation.x = q_mav.x();
        vision_pose.pose.orientation.y = q_mav.y();
        vision_pose.pose.orientation.z = q_mav.z();
        vision_pose.pose.orientation.w = q_mav.w();

        ROS_INFO("\nposition:\n   x: %.18f\n   y: %.18f\n   z: %.18f"\
                "\norientation:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f"\
                "\nvelocity:\n   x: %.18f\n   y: %.18f\n   z: %.18f"\
                "\nangle:\n   yaw:%.18f",
                    p_mav[0], p_mav[1], p_mav[2], q_mav.x(), q_mav.y(), q_mav.z(), q_mav.w(),
                    v_mav[0], v_mav[1], v_mav[2], uav_utils::get_yaw_from_quaternion(q_mav) * 180.0 / M_PI);
                    
        vision_pose_pub.publish(vision_pose);



        nav_msgs::Odometry vision_odom = *pMsg;

        vision_odom.header.frame_id = "map";
        vision_odom.child_frame_id = "map";

        vision_odom.pose.pose.position.x = p_mav[0];
        vision_odom.pose.pose.position.y = p_mav[1];
        vision_odom.pose.pose.position.z = p_mav[2];

        vision_odom.pose.pose.orientation.x = q_mav.x();
        vision_odom.pose.pose.orientation.y = q_mav.y();
        vision_odom.pose.pose.orientation.z = q_mav.z();
        vision_odom.pose.pose.orientation.w = q_mav.w();

        vision_odom.twist.twist.linear.x = v_mav[0];
        vision_odom.twist.twist.linear.y = v_mav[1];
        vision_odom.twist.twist.linear.z = v_mav[2];

        vision_odom_pub.publish(vision_odom);
    }
}


void TF_ENU_to_ENU::command_callback(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    quadrotor_msgs::PositionCommand cmd;

    cmd.position.x = -pMsg->position.x;
    cmd.position.y = -pMsg->position.y;
    cmd.position.z = pMsg->position.z;

    cmd.velocity.x = pMsg->velocity.x;
    cmd.velocity.y = pMsg->velocity.y;
    cmd.velocity.z = pMsg->velocity.z;

    cmd.acceleration.x = -pMsg->acceleration.x;
    cmd.acceleration.y = -pMsg->acceleration.y;
    cmd.acceleration.z = pMsg->acceleration.z;

    cmd.yaw = uav_utils::normalize_angle(pMsg->yaw + M_PI);
    cmd.yaw_dot = pMsg->yaw_dot;

    planner_cmd_pub.publish(cmd);
}

}

#endif