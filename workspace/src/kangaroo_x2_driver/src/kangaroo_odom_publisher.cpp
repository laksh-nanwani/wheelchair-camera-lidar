#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

#define DEG_2_RAD 3.14159265359/180.0

class OdomPublisher
{
    public:
    OdomPublisher();
    ~OdomPublisher();
    void joint_state_callback(const sensor_msgs::JointState& msg);
    
    // Subscriber & Publisher
    ros::Subscriber joint_state_sub; // Subscribes to topic '/joint_state'
    ros::Publisher odometry_pub; // Publishes to topic '/odom'

    // Variables
    nav_msgs::Odometry odometry; // To be published
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double last_dist = 0.0;

    // TF stuff
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
};

OdomPublisher::OdomPublisher(){}
OdomPublisher::~OdomPublisher(){}

void OdomPublisher::joint_state_callback(const sensor_msgs::JointState& msg)
{
    // double del_theta = (msg.position[1])*(DEG_2_RAD) - theta;
    double del_dist = msg.position[0] - last_dist;

    x = x + del_dist * cos(theta);
    y = y + del_dist * sin(theta);
    theta = msg.position[1] * (DEG_2_RAD); // 'theta' is in radians
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    last_dist = msg.position[0];

    // Setting header
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base_link";

    // Setting pose
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = odom_quat;

    // Setting covariances
    // odometry.pose.covariance[0] = 0.0001;
    // odometry.pose.covariance[7] = 0.0001;
    // odometry.pose.covariance[14] = 0.000001;
    // odometry.pose.covariance[21] = 0.000001;
    // odometry.pose.covariance[28] = 0.000001;
    // odometry.pose.covariance[35] = 0.001;

    // TF stuff
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    // Publish
    odom_broadcaster.sendTransform(odom_trans);
    odometry_pub.publish(odometry);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_publisher");
    ROS_INFO("Kangaroo Odom Publisher Started");
    ros::NodeHandle nh;
    OdomPublisher odom_publisher_obj;
    
    odom_publisher_obj.joint_state_sub = nh.subscribe("/joint_state", 10, &OdomPublisher::joint_state_callback, &odom_publisher_obj);
    ROS_INFO("Subscribed to /joint_state");
    odom_publisher_obj.odometry_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
    ROS_INFO("Publishing Odom");
    //ros::Rate rate(20);
    ros::spin();
}
