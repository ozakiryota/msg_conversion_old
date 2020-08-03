#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>

class OdometryToRPY{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_rpy;
		/*msg*/
		geometry_msgs::Vector3Stamped _rpy;
	public:
		OdometryToRPY();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void conversion(nav_msgs::Odometry odom);
		void print(nav_msgs::Odometry odom);
		void publication(void);
};

OdometryToRPY::OdometryToRPY()
{
	_sub_odom = _nh.subscribe("/odom", 1, &OdometryToRPY::callbackOdom, this);
	_pub_rpy = _nh.advertise<geometry_msgs::Vector3Stamped>("/rpy", 1);
}

void OdometryToRPY::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	conversion(*msg);
	print(*msg);
	publication();
}

void OdometryToRPY::conversion(nav_msgs::Odometry odom)
{
	_rpy.header = odom.header;
	tf::Quaternion q;
	quaternionMsgToTF(odom.pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(_rpy.vector.x, _rpy.vector.y, _rpy.vector.z);
}

void OdometryToRPY::print(nav_msgs::Odometry odom)
{
	std::cout << "----- " << odom.child_frame_id << " -----" << std::endl;
	std::cout 
		<< "(x, y, z)[m] = "
		<< odom.pose.pose.position.x << ", " 
		<< odom.pose.pose.position.y << ", " 
		<< odom.pose.pose.position.z << std::endl;
	double d = sqrt(
		odom.pose.pose.position.x*odom.pose.pose.position.x
		+ odom.pose.pose.position.y*odom.pose.pose.position.y
		+ odom.pose.pose.position.z*odom.pose.pose.position.z
	);
	std::cout << "Euclidian distance = " << d << std::endl;
	std::cout 
		<< "(r, p, y)[deg] = "
		<< _rpy.vector.x/M_PI*180.0 << ", " 
		<< _rpy.vector.y/M_PI*180.0 << ", " 
		<< _rpy.vector.z/M_PI*180.0 << std::endl;
}

void OdometryToRPY::publication(void)
{
	_pub_rpy.publish(_rpy);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_to_rpy");

	OdometryToRPY odometry_to_rpy;

	ros::spin();
}
