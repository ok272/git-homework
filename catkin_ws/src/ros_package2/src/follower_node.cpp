#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath> 

geometry_msgs::Twist tb3_0_vel;  
bool has_obstacle = false;      
double min_obstacle_dist = 0.5; 

void tb30VelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    tb3_0_vel = *msg;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    has_obstacle = false;
    int total_points = scan->ranges.size();
    double angle_min = scan->angle_min;  
    double angle_max = scan->angle_max;  
    double angle_increment = scan->angle_increment;  


    int start_idx = ( -0.523 - angle_min ) / angle_increment;
    int end_idx = ( 0.523 - angle_min ) / angle_increment; 

    start_idx = std::max(0, std::min(start_idx, total_points - 1));
    end_idx = std::max(0, std::min(end_idx, total_points - 1));

    for (int i = start_idx; i < end_idx; ++i) {
        if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max) {
            if (scan->ranges[i] < min_obstacle_dist) {  
                has_obstacle = true;
                break;
            }
        }
    }
}

int main(int argc, char**argv)
{
    //编码，初始化，NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "follower_node"); 
    ros::NodeHandle nh;

    //创建订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);
    
    //创建发布对象 
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel",100);

    ros::Subscriber tb30_vel_sub = nh.subscribe("/tb3_0/cmd_vel", 10, tb30VelCallback);
    ros::Subscriber scan_sub = nh.subscribe("/tb3_1/scan", 10, scanCallback);

    double target_distance;
    nh.param<double>("/follower_node/target_distance", target_distance, 1.0);

    double start_delay = 3.0;
    ros::Time start_time = ros::Time::now();

    const double dist_deadzone = 0.3;   // 添加死区，避免碰撞
    const double angle_deadzone = 0.2; 
    const double angular_gain = 0.6;    
    const double linear_gain = 0.08;
    const double max_angular_vel = 2.5;
    double relative_angle; 

    ros::Rate rate(10);
    while (ros::ok())
    {
        if ((ros::Time::now() - start_time).toSec() < start_delay) {
            geometry_msgs::Twist stop_twist;
            pub.publish(stop_twist);
            ROS_INFO("启动延迟中（剩余%.1f秒）...", start_delay - (ros::Time::now() - start_time).toSec());
            rate.sleep();
            ros::spinOnce();
            continue;
        }

        //核心
        
        try
        {   
            //计算tb3_0与tb3_1的相对关系
            /*
            目标坐标系
            源坐标系
            tb3_0相对world与tb3_1相对world在时间上有差值，使用Time(0)查找误差最小时的的相对关系
            */
            geometry_msgs::TransformStamped tb3_0Totb3_1 = buffer.lookupTransform("tb3_1","tb3_0",ros::Time(0));
            ROS_INFO("tb3_0相对于tb3_1的信息:父级:%s, 子级:%s 偏移量(%.2f,%.2f,%.2f) ",
                    tb3_0Totb3_1.header.frame_id.c_str(),
                    tb3_0Totb3_1.child_frame_id.c_str(),
                    tb3_0Totb3_1.transform.translation.x,
                    tb3_0Totb3_1.transform.translation.y,
                    tb3_0Totb3_1.transform.translation.z
                    );
  
            //根据相对计算并组织速度消息
            double manhattan_dist = fabs(tb3_0Totb3_1.transform.translation.x) + fabs(tb3_0Totb3_1.transform.translation.y);
            geometry_msgs::Twist twist;
            
            relative_angle = atan2(tb3_0Totb3_1.transform.translation.y, tb3_0Totb3_1.transform.translation.x);  

            if (fabs(relative_angle) > angle_deadzone) {
                twist.angular.z = angular_gain * relative_angle;
            } else {
                twist.angular.z = 0;  
                }
            if (fabs(relative_angle) <= angle_deadzone) {  
                double dist_error = manhattan_dist - target_distance;
                if (fabs(dist_error) > dist_deadzone) {  
                    twist.linear.x = linear_gain * dist_error;
                } else {
                    twist.linear.x = 0;
                }
            } else {
                twist.linear.x = 0;  
            }

            //发布
            pub.publish(twist);
        }
        catch(const tf2::TransformException& e)
        {
            ROS_WARN("错误提示:%s",e.what());
            geometry_msgs::Twist stop_twist;  
            pub.publish(stop_twist);
        }     

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

