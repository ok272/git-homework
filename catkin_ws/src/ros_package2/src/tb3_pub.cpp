#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>  
#include <string>

//声明变量来接受传递的参数
std::string tb3_name;
nav_msgs::Path robot_path;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    static tf2_ros::TransformBroadcaster pub;
    geometry_msgs::TransformStamped ts;

    ts.header.frame_id = "odom";
    ts.header.stamp = odom->header.stamp;  
    ts.child_frame_id = tb3_name + "/base_footprint";

    //坐标系偏移量设置
    ts.transform.translation.x = odom->pose.pose.position.x;
    ts.transform.translation.y = odom->pose.pose.position.y;
    ts.transform.translation.z = odom->pose.pose.position.z;

    ts.transform.rotation = odom->pose.pose.orientation;
    
    pub.sendTransform(ts);

    if (tb3_name == "tb3_0") {  // 只让tb3_0发布路径
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "odom";
        pose_stamped.header.stamp = odom->header.stamp;
        pose_stamped.pose = odom->pose.pose;  // 直接使用里程计位姿

        // 限制路径点数量（避免内存溢出）
        if (robot_path.poses.size() > 1000) {
            robot_path.poses.erase(robot_path.poses.begin());
        }
        robot_path.poses.push_back(pose_stamped);
        robot_path.header = pose_stamped.header;  // 同步头部信息

        // 发布路径（话题名：/tb3_0/path）
        static ros::Publisher path_pub = ros::NodeHandle().advertise<nav_msgs::Path>("/tb3_0/path", 10);
        path_pub.publish(robot_path);
    }
}                                        
                                      

int main (int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_tf_pub");
    ros::NodeHandle nh;
    /*
        解析launch文件通过args传入的参数
    */
    if (argc != 2)
    {
        ROS_ERROR("请传入一个参数。");
        return 1;
    }else{
        tb3_name = argv[1];
    }
    
    ros::Subscriber sub = nh.subscribe(tb3_name +"/odom",100,odomCallback);
    ros::spin();
    return 0;
}
