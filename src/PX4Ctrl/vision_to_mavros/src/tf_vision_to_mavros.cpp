// #include "tf_broadcaster.h"
#include "vins_to_mavros.h"
#include "msckf_vio_to_mavros.h"
#include "openvins_to_mavros.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_vision_to_mavros");
    ros::NodeHandle nh("~");

    if(argc != 3)
    {
        printf("please intput: rosrun tf_vision_to_mavros tf_vision_to_mavros_node [arg1] [arg2] \n"
               "for example: rosrun vins vins_node "
               "vins EDN\n");
        return 1;
    }

    std::string odom_algorithm = argv[1]; // vins or msckf_vio or openvins
    std::string odom_frame = argv[2]; // East-North-Up(ENU) or East-Down-North(EDN)
    
    // "body" for vins, "odom" for msckf_vio, "imu" for openvins
    std::string source_frame_id; 
    
    // subscriber
    ros::Subscriber odometry_sub;
    ros::Subscriber command_sub;

    // tf broadcaster
    std::shared_ptr<TF_Broadcaster> ptr_tf_broadcaster;


    if (odom_frame == "EDN")
    {
        if (odom_algorithm == "vins")
        {
            source_frame_id = "body";

            ptr_tf_broadcaster = std::shared_ptr<TF_Broadcaster>(new vins_to_mavros::TF_EDN_to_ENU(nh));
            
            odometry_sub = 
                nh.subscribe<nav_msgs::Odometry>("odom", 
                                                100, 
                                                boost::bind(&TF_Broadcaster::vision_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
            command_sub = 
                nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                100,
                                                boost::bind(&TF_Broadcaster::command_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
        }
        else if (odom_algorithm == "msckfvio")
        {
            source_frame_id = "odom";

            ptr_tf_broadcaster = std::shared_ptr<TF_Broadcaster>(new msckfvio_to_mavros::TF_EDN_to_ENU(nh));

            odometry_sub = 
                nh.subscribe<nav_msgs::Odometry>("odom", 
                                                100, 
                                                boost::bind(&TF_Broadcaster::vision_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
            command_sub = 
                nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                100,
                                                boost::bind(&TF_Broadcaster::command_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
        }
        else if (odom_algorithm == "openvins")
        {
            source_frame_id = "imu";

            ptr_tf_broadcaster = std::shared_ptr<TF_Broadcaster>(new openvins_to_mavros::TF_EDN_to_ENU(nh));
            odometry_sub = 
                nh.subscribe<nav_msgs::Odometry>("odom", 
                                                100, 
                                                boost::bind(&TF_Broadcaster::vision_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
            command_sub = 
                nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                100,
                                                boost::bind(&TF_Broadcaster::command_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());        
        }
        else
        {
            ROS_ERROR("please choose \"EDN\" or \"ENU\" as argument 2!");
        }
    }
    else if (odom_frame == "ENU")
    {
        if (odom_algorithm == "vins")
        {
            source_frame_id = "body";

            ptr_tf_broadcaster = std::shared_ptr<TF_Broadcaster>(new vins_to_mavros::TF_ENU_to_ENU(nh));
            odometry_sub = 
                nh.subscribe<nav_msgs::Odometry>("odom", 
                                                100, 
                                                boost::bind(&TF_Broadcaster::vision_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
            command_sub = 
                nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                100,
                                                boost::bind(&TF_Broadcaster::command_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
        }
        else if (odom_algorithm == "msckfvio")
        {
            source_frame_id = "odom";

            ptr_tf_broadcaster = std::shared_ptr<TF_Broadcaster>(new msckfvio_to_mavros::TF_ENU_to_ENU(nh));
            odometry_sub = 
                nh.subscribe<nav_msgs::Odometry>("odom", 
                                                100, 
                                                boost::bind(&TF_Broadcaster::vision_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
            command_sub = 
                nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                100,
                                                boost::bind(&TF_Broadcaster::command_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());        
        }
        else if (odom_algorithm == "openvins")
        {
            source_frame_id = "imu";

            ptr_tf_broadcaster = std::shared_ptr<TF_Broadcaster>(new openvins_to_mavros::TF_ENU_to_ENU(nh));
            odometry_sub = 
                nh.subscribe<nav_msgs::Odometry>("odom", 
                                                100, 
                                                boost::bind(&TF_Broadcaster::vision_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());
            command_sub = 
                nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                100,
                                                boost::bind(&TF_Broadcaster::command_callback, &(*ptr_tf_broadcaster), _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());        
        }
        else
        {
            ROS_ERROR("please choose \"vins\", \"msckfvio\" or \"openvins\" as argument 1!");
        }
    }
    else
    {
        ROS_ERROR("please choose \"EDN\" or \"ENU\" as argument 2!");
    }



    ros::Rate rate(100);
    while (ros::ok())
    {
        ptr_tf_broadcaster->tf_world_to_map();

        ptr_tf_broadcaster->tf_odom_to_baselink(source_frame_id);

        ros::spinOnce();
        rate.sleep();
    }


    
    return 0;
}
