#include <wheelchair_navigation/odom_publisher.h>

OdomPublisher::OdomPublisher() : x(0.0), y(0.0)
{

}

OdomPublisher::~OdomPublisher() 
{

}

void OdomPublisher::twistCallback(const geometry_msgs::Twist& twist)
{
    m_gTwist = twist;
}
void OdomPublisher::positionCallback(const geometry_msgs::Point& position)
{
    ROS_INFO("arduino data :: drive : %f :: turn : %f", m_gPose.x, m_gPose.z);
    m_gPose = position;
    double distance = (m_gPose.x - m_gPrevPose.x);
    double theta = m_gPose.z * angle_error_factor * radian;
    x = x + distance*cos(theta);
    y = y + distance*sin(theta);
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    std::cout << x << " " << y <<" " << theta << std::endl;
    //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x/1000.0;
    odom_trans.transform.translation.y = y/1000.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //set the position
    odom.pose.pose.position.x = x/1000.0;
    odom.pose.pose.position.y = y/1000.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // setting covariances
    // larger value idicates more uncertainity
    odom.pose.covariance[0] = 0.0001;  // x
    odom.pose.covariance[7] = 0.0001;  // y
    odom.pose.covariance[14] = 0.000001;  // z
    odom.pose.covariance[21] = 0.000001;  // roll
    odom.pose.covariance[28] = 0.000001; // pitch
    odom.pose.covariance[35] = 0.001; // yaw earlier it was 0.01

    //set the velocity
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y =  0.0;
    odom.twist.twist.angular.z =  0.0;

    m_gPrevPose.x = m_gPose.x;
    //publish the message
    odom_broadcaster.sendTransform(odom_trans);
    m_Odom_pub_.publish(odom);
}

int main(int argc, char** argv)
{
        ros::init(argc, argv, "odom_publisher");
        ros::NodeHandle n;
        OdomPublisher m_obj;
        ros::Rate rate(20);
        m_obj.m_Position_sub_ = n.subscribe("position", 10, &OdomPublisher::positionCallback, &m_obj);
        m_obj.m_Velocity_sub_ = n.subscribe("wheelchair_diff/cmd_vel", 10, &OdomPublisher::twistCallback, &m_obj);
        m_obj.m_Odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 1000);
        while (ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
