#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <limits> 

const double TARGET_A_X = 0.55, TARGET_A_Y = -0.49;
const double TARGET_B_X = 0.55, TARGET_B_Y = -1.64;

const double FRONT_ANGLE_RANGE = M_PI / 18;  
const double RADAR_STOP_DIST = 0.3; 
const double RADAR_DECEL_DIST = 0.8;       
const double SLOW_SPEED = 0.1;               
const double NORMAL_SPEED = 0.2;             


double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;
int move_state = 0;
bool laser_received = false;
sensor_msgs::LaserScan laser_data;
sensor_msgs::LaserScan::ConstPtr g_scan_msg;

ros::Publisher vel_pub;
ros::Subscriber odom_sub;
ros::Subscriber laser_sub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg); 

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_controller");
    ros::NodeHandle nh;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    odom_sub = nh.subscribe("/odom", 50, odomCallback);
    laser_sub = nh.subscribe("/scan", 50, laserCallback);

    ros::Rate rate(20); 
    geometry_msgs::Twist vel_cmd; 
    
    ROS_INFO("开始运动。");

    while (ros::ok()) {
        switch (move_state){
            case 0:{
                vel_cmd.linear.x = 0.3;
                vel_cmd.angular.z = 0.0;
                if (fabs(current_x - TARGET_A_X) < 0.05 && fabs(current_y - TARGET_A_Y) < 0.05) {
                    move_state = 1;
                    ROS_INFO("已到达A点。");
                }
                break;
            }

            case 1:{
                vel_cmd.linear.x = 0.0;
                double target_yaw = -M_PI/2;
                double angle_diff = target_yaw - current_yaw;
                angle_diff = fmod(angle_diff + M_PI, 2*M_PI) - M_PI;

                if (fabs(angle_diff) > 0.05) {
                        vel_cmd.angular.z = (angle_diff > 0) ? 0.3 : -0.3;
                } else {
                    vel_cmd.angular.z = 0.0; 
                    move_state = 2; 
                    ROS_INFO("转向完成,前往B点");
                    }
                break;
            } 

            case 2:{
                vel_cmd.linear.x = 0.2;
                vel_cmd.angular.z = 0.0;

                if (fabs(current_x - TARGET_B_X) < 0.05 && fabs(current_y - TARGET_B_Y) < 0.05){
                    move_state = 3;
                    ROS_INFO("已到达B点");
                }
                break;
            }

            case 3:{
                bool obstacle_ahead = false;
                double front_dist = std::numeric_limits<double>::max();  // 初始化正前方最近距离

                if (laser_received) {
                    int scan_size = laser_data.ranges.size();  // 声明scan_size
                    for (int i = 0; i < scan_size; ++i) {
                        double radar_angle = laser_data.angle_min + i * laser_data.angle_increment;  // 修正为.运算符
                        if (fabs(radar_angle) <= FRONT_ANGLE_RANGE) {
                            double d = laser_data.ranges[i];  // 修正为.运算符
                            if (!std::isnan(d) && !std::isinf(d) && d >= laser_data.range_min && d <= laser_data.range_max) {  // 修正为.运算符
                                front_dist = std::min(front_dist, d); 
                            }
                        }
                    }
                    // 处理无有效数据的情况
                    if (front_dist == std::numeric_limits<double>::max()) {
                        front_dist = laser_data.range_max;  // 修正为.运算符
                    }
                }

                // 根据前方距离设置速度
                if (front_dist <= RADAR_STOP_DIST) {
                    vel_cmd.linear.x = 0.0;
                    ROS_INFO("距离障碍物%.2fm，停车！", front_dist);
                    move_state = 4;
                } else if (front_dist <= RADAR_DECEL_DIST) {
                    vel_cmd.linear.x = SLOW_SPEED;
                    ROS_INFO("减速行驶...");
                } else {
                    vel_cmd.linear.x = NORMAL_SPEED;
                    ROS_INFO("正常行驶");
                }
                vel_cmd.angular.z = 0.0;
                break;
            }

            case 4:
                vel_cmd.linear.x = 0.0;
                vel_cmd.angular.z = 0.0;
                ROS_INFO("已停车");
                break;
        }
        vel_pub.publish(vel_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    current_yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    laser_data = *msg;
    laser_received = true;
}
