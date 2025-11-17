#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// 目标点坐标（对应作业中的A、B点）
const double TARGET_A_X = 0.55, TARGET_A_Y = -0.49;
const double TARGET_B_X = 0.55, TARGET_B_Y = -1.64;
// 移动状态：0-前往A点；1-前往B点；2-停止
int move_state = 0;

// 里程计回调函数：获取当前位置并切换状态
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // 到达A点（误差小于0.05）
    if (move_state == 0 && fabs(current_x - TARGET_A_X) < 0.05 && fabs(current_y - TARGET_A_Y) < 0.05) {
        move_state = 1;
        ROS_INFO("已到达A点，开始前往B点");
    }
    // 到达B点
    else if (move_state == 1 && fabs(current_x - TARGET_B_X) < 0.05 && fabs(current_y - TARGET_B_Y) < 0.05) {
        move_state = 2;
        ROS_INFO("已到达B点，停止移动");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot3_nav_node");
    ros::NodeHandle nh;

    // 订阅里程计话题（获取小车位置）
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    // 发布速度控制话题（控制小车移动）
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    geometry_msgs::Twist vel_cmd;
    ros::Rate loop_rate(10); // 10Hz频率发布控制指令

    while (ros::ok()) {
        // 根据状态设置速度
        switch (move_state) {
            case 0: // 前往A点：直线前进
                vel_cmd.linear.x = 0.2;
                vel_cmd.angular.z = 0.0;
                break;
            case 1: // 前往B点：沿y轴负方向移动
                vel_cmd.linear.x = 0.0;
                vel_cmd.angular.z = 0.0;
                vel_cmd.linear.y = -0.2;
                break;
            case 2: // 停止
                vel_cmd.linear.x = 0.0;
                vel_cmd.angular.z = 0.0;
                break;
        }

        cmd_vel_pub.publish(vel_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

