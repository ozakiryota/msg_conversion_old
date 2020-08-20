#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>

class OdometryToQuaternionStamped{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		// ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_pose;
	public:
		OdometryToQuaternionStamped();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
};

OdometryToQuaternionStamped::OdometryToQuaternionStamped()
//	: _nhPrivate("~")
{
	/*subscriber*/
	_sub_odom = _nh.subscribe("/odom", 1, &OdometryToQuaternionStamped::CallbackOdom, this);
	/*publisher*/
	_pub_pose = _nh.advertise<geometry_msgs::QuaternionStamped>("/quat/from_odom", 1);
}

void OdometryToQuaternionStamped::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	geometry_msgs::QuaternionStamped quat;
	quat.header = msg->header;
	quat.quaternion = msg->pose.pose.orientation;
	_pub_pose.publish(quat);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_to_quaternionstamped");

	OdometryToQuaternionStamped odometry_to_quaternionstamped;

	ros::spin();
}
