#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

class OdometryLinearGlobalToLocal{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_odom;
		/*msg*/
		nav_msgs::Odometry _odom;
	public:
		OdometryLinearGlobalToLocal();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void conversion(void);
		void publication(void);
};

OdometryLinearGlobalToLocal::OdometryLinearGlobalToLocal()
{
	_sub_odom = _nh.subscribe("/odom", 1, &OdometryLinearGlobalToLocal::callbackOdom, this);
	_pub_odom = _nh.advertise<nav_msgs::Odometry>("/odom/local_linear", 1);
}

void OdometryLinearGlobalToLocal::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom = *msg;
	conversion();
	publication();
}

void OdometryLinearGlobalToLocal::conversion(void)
{
	/*transformation*/
	tf::Quaternion q_orientation;
	quaternionMsgToTF(_odom.pose.pose.orientation, q_orientation);
	tf::Quaternion q_twist_global(
		_odom.twist.twist.linear.x,
		_odom.twist.twist.linear.y,
		_odom.twist.twist.linear.z,
		0.0
	);
	tf::Quaternion q_twist_local = q_orientation.inverse()*q_twist_global*q_orientation;
	/*input*/
	_odom.twist.twist.linear.x = q_twist_local.x();
	_odom.twist.twist.linear.y = q_twist_local.y();
	_odom.twist.twist.linear.z = q_twist_local.z();
}

void OdometryLinearGlobalToLocal::publication(void)
{
	_pub_odom.publish(_odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_linear_global_to_local");

	OdometryLinearGlobalToLocal odometry_linear_global_to_local;

	ros::spin();
}
