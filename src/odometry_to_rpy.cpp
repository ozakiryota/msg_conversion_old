#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

class OdometryToRPY{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscriber*/
		ros::Subscriber sub_odom;
		/*publisher*/
		ros::Publisher pub_rpy;
		/*objects*/
	public:
		OdometryToRPY();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Print(nav_msgs::Odometry odom, std_msgs::Float64MultiArray rpy);
};

OdometryToRPY::OdometryToRPY()
{
	sub_odom = nh.subscribe("/odom", 1, &OdometryToRPY::CallbackOdom, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy", 1);
}

void OdometryToRPY::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	tf::Quaternion q_orientation;
	quaternionMsgToTF(msg->pose.pose.orientation, q_orientation);
	std_msgs::Float64MultiArray rpy_pub;	//[deg]
	rpy_pub.data.resize(3);
	tf::Matrix3x3(q_orientation).getRPY(rpy_pub.data[0], rpy_pub.data[1], rpy_pub.data[2]);
	for(int i=0;i<3;i++)	rpy_pub.data[i] = rpy_pub.data[i]/M_PI*180.0;
	pub_rpy.publish(rpy_pub);
	Print(*msg, rpy_pub);
}

void OdometryToRPY::Print(nav_msgs::Odometry odom, std_msgs::Float64MultiArray rpy)
{
	std::cout << "----- " << odom.child_frame_id << " -----" << std::endl;
	std::cout 
		<< "(x, y, z) = "
		<< odom.pose.pose.position.x << "," 
		<< odom.pose.pose.position.y << "," 
		<< odom.pose.pose.position.z << std::endl;
	std::cout 
		<< "(r, p, y) = "
		<< rpy.data[0] << "," 
		<< rpy.data[1] << "," 
		<< rpy.data[2] << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_to_rpy");

	OdometryToRPY odometry_to_rpy;

	ros::spin();
}
