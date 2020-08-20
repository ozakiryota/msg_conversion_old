#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class CombineOrientationLinear{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_quat;
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_odom;
		tf::TransformBroadcaster _tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry _odom;
		nav_msgs::Odometry _odom_cblast;
		/*flag*/
		bool _got_first_odom = false;
		/*parameter*/
		bool _linear_vel_is_available;
		std::string _frame_id;
		std::string _child_frame_id;
	public:
		CombineOrientationLinear();
		void initializeOdom(nav_msgs::Odometry& odom);
		void callbackOrientation(const geometry_msgs::QuaternionStampedConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void transformLinVel(nav_msgs::Odometry odom_cbnow, double dt);
		void publication(ros::Time stamp);
};

CombineOrientationLinear::CombineOrientationLinear()
	: _nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("linear_vel_is_available", _linear_vel_is_available, false);
	std::cout << "_linear_vel_is_available = " << (bool)_linear_vel_is_available << std::endl;
	_nhPrivate.param("frame_id", _frame_id, std::string("/odom"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("child_frame_id", _child_frame_id, std::string("/combined_odom"));
	std::cout << "_child_frame_id = " << _child_frame_id << std::endl;
	/*subscriber*/
	_sub_quat = _nh.subscribe("/orientation", 1, &CombineOrientationLinear::callbackOrientation, this);
	_sub_odom = _nh.subscribe("/odom", 1, &CombineOrientationLinear::callbackOdom, this);
	/*publisher*/
	_pub_odom = _nh.advertise<nav_msgs::Odometry>("/combined_odom", 1);
	/*initialize*/
	initializeOdom(_odom);
}

void CombineOrientationLinear::initializeOdom(nav_msgs::Odometry& odom)
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

void CombineOrientationLinear::callbackOrientation(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
	/*input*/
	_odom.pose.pose.orientation = msg->quaternion;
	/*publication*/
	publication(msg->header.stamp);
}

void CombineOrientationLinear::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	/*skip first callback*/
	if(!_got_first_odom){
		_odom_cblast = *msg;
		_got_first_odom = true;
		return;
	}
	/*get dt*/
	double dt;
	try{
		dt = (msg->header.stamp - _odom_cblast.header.stamp).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
		return;
	}
	/*prediction*/
	transformLinVel(*msg, dt);
	/*publication*/
	publication(msg->header.stamp);
	/*reset*/
	_odom_cblast = *msg;
}

void CombineOrientationLinear::transformLinVel(nav_msgs::Odometry odom_cbnow, double dt)
{
	tf::Quaternion q_ori;
	quaternionMsgToTF(_odom.pose.pose.orientation, q_ori);
	tf::Quaternion q_global_move;
	/*transform*/
	if(_linear_vel_is_available){
		tf::Quaternion q_local_move(
			odom_cbnow.twist.twist.linear.x*dt,
			odom_cbnow.twist.twist.linear.y*dt,
			odom_cbnow.twist.twist.linear.z*dt,
			0.0
		);
		q_global_move = q_ori*q_local_move*q_ori.inverse();
	}
	else{
		tf::Quaternion q_ori_cblast;
		quaternionMsgToTF(_odom_cblast.pose.pose.orientation, q_ori_cblast);

		tf::Quaternion q_global_move_cb(
			odom_cbnow.pose.pose.position.x - _odom_cblast.pose.pose.position.x,
			odom_cbnow.pose.pose.position.y - _odom_cblast.pose.pose.position.y,
			odom_cbnow.pose.pose.position.z - _odom_cblast.pose.pose.position.z,
			0.0
		);
		tf::Quaternion q_local_move = q_ori_cblast.inverse()*q_global_move_cb*q_ori_cblast;
		q_global_move = q_ori*q_local_move*q_ori.inverse();
	}
	/*input*/
	_odom.pose.pose.position.x += q_global_move.x();
	_odom.pose.pose.position.y += q_global_move.y();
	_odom.pose.pose.position.z += q_global_move.z();
}

void CombineOrientationLinear::publication(ros::Time stamp)
{
	/*publish*/
	_odom.header.stamp = stamp;
	_pub_odom.publish(_odom);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = stamp;
	transform.header.frame_id = _frame_id;
	transform.child_frame_id = _child_frame_id;
	transform.transform.translation.x = _odom.pose.pose.position.x;
	transform.transform.translation.y = _odom.pose.pose.position.y;
	transform.transform.translation.z = _odom.pose.pose.position.z;
	transform.transform.rotation = _odom.pose.pose.orientation;
	_tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "combine_orientation_linear");

	CombineOrientationLinear combine_orientation_linear;

	ros::spin();
}
