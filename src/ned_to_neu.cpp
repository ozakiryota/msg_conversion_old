#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

class NedToNeu{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		ros::Subscriber _sub_imu;
		/*publisher*/
		ros::Publisher _pub_odom;
		ros::Publisher _pub_imu;
		tf::TransformBroadcaster _tf_broadcaster;
		/*msg*/
		nav_msgs::Odometry _odom_msg;
		sensor_msgs::Imu _imu_msg;
		/*fram_id*/
		std::string _parent_frame_id;
		std::string _child_frame_id;
	public:
		NedToNeu();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void callbackImu(const sensor_msgs::ImuConstPtr& msg);
		void publicationOdom(void);
		void publicationImu(void);
};

NedToNeu::NedToNeu()
	: _nhPrivate("~")
{
	std::cout << "--- ned_to_neu ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("parent_frame_id", _parent_frame_id, std::string("/odom"));
	std::cout << "_parent_frame_id = " << _parent_frame_id << std::endl;
	_nhPrivate.param("child_frame_id", _child_frame_id, std::string("/odom/neu"));
	std::cout << "_child_frame_id = " << _child_frame_id << std::endl;
	/*subscriber*/
	_sub_odom = _nh.subscribe("/odom", 1, &NedToNeu::callbackOdom, this);
	_sub_imu = _nh.subscribe("/imu/data", 1, &NedToNeu::callbackImu, this);
	/*publisher*/
	_pub_odom = _nh.advertise<nav_msgs::Odometry>("/odom/neu", 1);
	_pub_imu = _nh.advertise<sensor_msgs::Imu>("/imu/data/neu", 1);
}

void NedToNeu::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	_odom_msg = *msg;

	_odom_msg.header.frame_id = _parent_frame_id;
	_odom_msg.child_frame_id = _child_frame_id;

	_odom_msg.pose.pose.position.y *= -1;
	_odom_msg.pose.pose.position.z *= -1;
	_odom_msg.pose.pose.orientation.y *= -1;
	_odom_msg.pose.pose.orientation.z *= -1;
	_odom_msg.twist.twist.linear.y *= -1;
	_odom_msg.twist.twist.linear.z *= -1;
	_odom_msg.twist.twist.angular.y *= -1;
	_odom_msg.twist.twist.angular.z *= -1;

	publicationOdom();
}

void NedToNeu::callbackImu(const sensor_msgs::ImuConstPtr& msg)
{
	_imu_msg = *msg;

	_imu_msg.header.frame_id = _child_frame_id;

	_imu_msg.orientation.y *= -1;
	_imu_msg.orientation.z *= -1;
	_imu_msg.angular_velocity.y *= -1;
	_imu_msg.angular_velocity.z *= -1;
	_imu_msg.linear_acceleration.y *= -1;
	_imu_msg.linear_acceleration.z *= -1;

	publicationImu();
}

void NedToNeu::publicationOdom(void)
{
	/*publish*/
	_pub_odom.publish(_odom_msg);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header = _odom_msg.header;
	transform.child_frame_id = _odom_msg.child_frame_id;
	transform.transform.translation.x = _odom_msg.pose.pose.position.x;
	transform.transform.translation.y = _odom_msg.pose.pose.position.y;
	transform.transform.translation.z = _odom_msg.pose.pose.position.z;
	transform.transform.rotation = _odom_msg.pose.pose.orientation;
	_tf_broadcaster.sendTransform(transform);
}

void NedToNeu::publicationImu(void)
{
	/*publish*/
	_pub_imu.publish(_imu_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ned_to_neu");

	NedToNeu ned_to_neu;

	ros::spin();
}
