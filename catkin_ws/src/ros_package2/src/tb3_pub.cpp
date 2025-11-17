#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2/LinearMath/Matrix3x3.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//声明变量来接受传递的参数
std::string tb3_name;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    static tf2_ros::TransformBroadcaster pub;
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "world";
    ts.header.stamp = ros::Time::now();
    ts.child_frame_id = tb3_name;
    //坐标系偏移量设置
    ts.transform.translation.x = odom->pose.pose.position.x;
    ts.transform.translation.y = odom->pose.pose.position.y;
    ts.transform.translation.z = odom->pose.pose.position.z;
    //坐标系四元数
    tf2::Quaternion q;//a.存储里程计的四元数
    tf2::fromMsg(odom->pose.pose.orientation, q);//b.将里程计消息中的四元数转成tf2类型
    //c.将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //d.用欧拉角重新生成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(roll, pitch, yaw);

    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();

    pub.sendTransform(ts);
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
