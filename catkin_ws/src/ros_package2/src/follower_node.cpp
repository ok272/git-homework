#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>  
#include <deque>  // 用于存储路径点队列
#include <cmath> 
#include <algorithm>
 
bool has_obstacle = false;      
const double min_obstacle_dist = 0.3; 
std::deque<geometry_msgs::PoseStamped> tb3_0_path;

// 曼哈顿距离计算函数
double manhattanDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return fabs(p1.x - p2.x) + fabs(p1.y - p2.y);
}

// 从四元数获取偏航角函数
double getYawFromQuaternion(const geometry_msgs::Quaternion& quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

//路径回调函数
void pathCallback(const nav_msgs::Path::ConstPtr& path) {
    tb3_0_path.clear();  // 清空旧路径
    size_t start_idx = std::max(0, (int)(path->poses.size() - 100));
    for (size_t i = start_idx; i < path->poses.size(); ++i) {
        tb3_0_path.push_back(path->poses[i]);
    }
}

//检测障碍物
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    has_obstacle = false;
    const int total_points = scan->ranges.size();
    const double angle_min = scan->angle_min;   
    const double angle_increment = scan->angle_increment;  

    const int start_idx = std::max(0, (int)((-1.047 - angle_min) / angle_increment));
    const int end_idx = std::min(total_points - 1, (int)((1.047 - angle_min) / angle_increment));

    for (int i = start_idx; i <= end_idx; ++i) {
        if (scan->ranges[i] > scan->range_min && scan->ranges[i] < min_obstacle_dist) {
            has_obstacle = true;
            break;
        }
    }
}

// 获取tb3_1在odom坐标系下的位置
bool getTb3_1Pose(tf2_ros::Buffer& buffer, geometry_msgs::Point& pose, double& yaw) {
    try {
        auto transform = buffer.lookupTransform("odom", "tb3_1/base_footprint", ros::Time(0));
        pose.x = transform.transform.translation.x;
        pose.y = transform.transform.translation.y;
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
        return true;
    } catch (tf2::TransformException& e) {
        ROS_WARN("获取tb3_1位置失败: %s", e.what());
        return false;
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
    tf2_ros::TransformListener tf_listener(buffer);
    
    //创建发布对象 
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel",100);

    //创建订阅对象
    ros::Subscriber path_sub = nh.subscribe("/tb3_0/path", 20, pathCallback);
    ros::Subscriber scan_sub = nh.subscribe("/tb3_1/scan", 20, scanCallback);

    double target_distance;
    nh.param<double>("/follower_node/target_distance", target_distance, 0.5);

    double start_delay = 2.0;
    ros::Time start_time = ros::Time::now();

    const double dist_deadzone = 0.15;   // 添加死区，避免碰撞
    const double angle_deadzone = 0.15; 
    const double angular_gain = 0.8;    
    const double linear_gain = 0.4;
    const double max_angular_vel = 0.5;
    const double max_linear_vel = 0.3;

    ros::Rate rate(15);
    while (ros::ok())
    {
        if ((ros::Time::now() - start_time).toSec() < start_delay) {
            pub.publish(geometry_msgs::Twist());
            ROS_INFO("启动延迟中（剩余%.1f秒）...", start_delay - (ros::Time::now() - start_time).toSec());
            rate.sleep();
            ros::spinOnce();
            continue;
        }

        if (has_obstacle) {
            ROS_WARN("检测到障碍物，开始后退！");
            geometry_msgs::Twist twist;
            twist.linear.x = -0.1;  
            twist.angular.z = 0;    
            pub.publish(twist);     
            rate.sleep();
            continue;
        }

        // 等待路径数据
        if (tb3_0_path.size() < 3) {  
            ROS_INFO_THROTTLE(0.5, "等待tb3_0的路径数据（当前%d个点，需要≥3个）...", (int)tb3_0_path.size());
            rate.sleep();
            ros::spinOnce();
            continue;
        }

        // 获取tb3_1当前位置
        geometry_msgs::Point tb3_1_pos;
        double tb3_1_yaw;
        if (!getTb3_1Pose(buffer, tb3_1_pos,tb3_1_yaw)) {
            rate.sleep();
            ros::spinOnce();
            continue;
        }

        // 核心：选择目标点（tb3_0路径中，与最后一个点（当前位置）相距target_distance的历史点）
        geometry_msgs::PoseStamped target_pose = tb3_0_path.front();  // 默认第一个点
        double cumulative_dist = 0.0;
        bool target_found = false;

        // 从路径倒数第二个点开始向前累加曼哈顿距离
        for (int i = tb3_0_path.size() - 2; i >= 0; --i) {
            // 累加相邻路径点的曼哈顿距离
            cumulative_dist += manhattanDistance(
                tb3_0_path[i].pose.position,
                tb3_0_path[i+1].pose.position
            );
            if (cumulative_dist >= target_distance) {
                target_pose = tb3_0_path[i];
                target_found = true;
                break;
            }
        }

        if (!target_found) {
            target_pose = tb3_0_path[0];
        }

        // 计算tb3_1到目标点的相对位置
        double dx = target_pose.pose.position.x - tb3_1_pos.x;
        double dy = target_pose.pose.position.y - tb3_1_pos.y;
        
        double current_dist = manhattanDistance(tb3_1_pos, target_pose.pose.position);

        // 从目标点的四元数提取yaw角
        double target_yaw = getYawFromQuaternion(target_pose.pose.orientation);

        // 计算tb3_1需要对准的方向
        double angle_error = target_yaw - tb3_1_yaw;
        angle_error = fmod(angle_error + M_PI, 2 * M_PI) - M_PI;

        // 速度控制
        geometry_msgs::Twist twist;

        // 角度控制
        if (fabs(angle_error) > angle_deadzone) {
            twist.angular.z = angular_gain * angle_error;
            twist.angular.z = std::max(-max_angular_vel, std::min(max_angular_vel, twist.angular.z));
        } else {
            twist.angular.z = 0;
        }

        // 距离控制
        if (fabs(angle_error) <= angle_deadzone) {
            const double dist_error = current_dist - dist_deadzone;
            twist.linear.x = std::max(0.0, std::min(max_linear_vel, linear_gain * dist_error));
        } else {
            twist.linear.x = std::max(0.0, std::min(0.1, linear_gain * (current_dist - dist_deadzone)));  // 角度没对准也给点速度，避免不动
        }

        pub.publish(twist);
        
        ROS_INFO_THROTTLE(0.3, 
            "路径点数量: %d | 目标点: (%.2f, %.2f) | 距离目标: %.2f | 角度误差: %.2f | "
            "线速度: %.2f | 角速度: %.2f | 目标找到: %s",
            (int)tb3_0_path.size(),
            target_pose.pose.position.x, target_pose.pose.position.y,
            current_dist, angle_error,
            twist.linear.x, twist.angular.z,
            target_found ? "是" : "否（用兜底点）"
        );

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}