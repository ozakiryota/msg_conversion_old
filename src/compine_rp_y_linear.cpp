#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class CompineRPYLinear{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_quat;
		ros::Subscriber _sub_imu;
		/*publisher*/
		ros::Publisher _pub_odom;
		/*odom*/
		nav_msgs::Odometry _odom;
		/*attitude*/
		tf::Quaternion _attitude{0.0, 0.0, 0.0, 1.0};
		/*parameter*/
		bool _linear_vel_is_available;
		std::string _frame_id;
		std::string _child_frame_id;
	public:
		CompineRPYLinear();
		void initializeOdom(nav_msgs::Odometry& odom);
		void callbackQuat(const geometry_msgs::QuaternionStampedConstPtr& msg);
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void publication(ros::Time stamp);
};

CompineRPYLinear::CompineRPYLinear()
	: _nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("linear_vel_is_available", _linear_vel_is_available, false);
	std::cout << "_linear_vel_is_available = " << (bool)_linear_vel_is_available << std::endl;
	_nhPrivate.param("frame_id", _frame_id, std::string("/odom"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("child_frame_id", _child_frame_id, std::string("/child"));
	std::cout << "_child_frame_id = " << _child_frame_id << std::endl;
	/*subscriber*/
	_sub_quat = _nh.subscribe("/quat", 1, &CompineRPYLinear::callbackQuat, this);
	_sub_imu = _nh.subscribe("/imu/data", 1, &CompineRPYLinear::callbackIMU, this);
	/*publisher*/
	_pub_odom = _nh.advertise<nav_msgs::Odometry>("/odom", 1);
	/*initialize*/
	initializeOdom(_odom);
}

void CompineRPYLinear::initializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = _frame_id;
	odom.child_frame_id = _child_frame_id;
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void CompineRPYLinear::callbackQuat(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
	double r, p, y, no_use;
	tf::Quaternion q;
	/*RP*/
	quaternionMsgToTF(msg->quaternion, q);
	tf::Matrix3x3(q).getRPY(r, p, no_use);
	/*Y*/
	quaternionMsgToTF(_odom.pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(no_use, no_use, y);
	/*combine*/
	q = tf::createQuaternionFromRPY(r, p, y);
	quaternionTFToMsg(q, _odom.pose.pose.orientation);

	publication(msg->header.stamp);
}

void CompineRPYLinear::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
}

void CompineRPYLinear::publication(ros::Time stamp)
{
	_odom.header.stamp = stamp;
	// nav_msgs::Odometry odom_pub;
	// odom_pub.header = msg->header;
	// odom_pub.child_frame_id = _child_frame_id.c_str();
	// odom_pub.pose.pose = msg->pose;
	_pub_odom.publish(_odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "compine_rp_y_linear");

	CompineRPYLinear compine_rp_y_linear;

	ros::spin();
}
