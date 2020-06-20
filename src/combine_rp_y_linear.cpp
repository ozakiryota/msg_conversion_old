#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class CombineRPYLinear{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_quat;
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_bias;
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_odom;
		tf::TransformBroadcaster _tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry _odom;
		nav_msgs::Odometry _odom_cblast;
		/*attitude*/
		tf::Quaternion _attitude{0.0, 0.0, 0.0, 1.0};
		/*time*/
		ros::Time _stamp_imu_last;
		/*bias*/
		sensor_msgs::Imu _bias;
		/*flag*/
		bool _got_first_imu = false;
		bool _got_first_odom = false;
		bool _got_bias = false;
		/*parameter*/
		bool _linear_vel_is_available;
		std::string _frame_id;
		std::string _child_frame_id;
	public:
		CombineRPYLinear();
		void initializeOdom(nav_msgs::Odometry& odom);
		void callbackQuat(const geometry_msgs::QuaternionStampedConstPtr& msg);
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackBias(const sensor_msgs::ImuConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void combineRPY(tf::Quaternion q_rp, tf::Quaternion q_y);
		void transformAngVel(sensor_msgs::Imu imu, double dt);
		void transformLinVel(nav_msgs::Odometry odom_cbnow, double dt);
		void publication(ros::Time stamp);
};

CombineRPYLinear::CombineRPYLinear()
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
	_sub_quat = _nh.subscribe("/quat", 1, &CombineRPYLinear::callbackQuat, this);
	_sub_imu = _nh.subscribe("/imu/data", 1, &CombineRPYLinear::callbackIMU, this);
	_sub_bias = _nh.subscribe("/imu/bias", 1, &CombineRPYLinear::callbackBias, this);
	_sub_odom = _nh.subscribe("/odom", 1, &CombineRPYLinear::callbackOdom, this);
	/*publisher*/
	_pub_odom = _nh.advertise<nav_msgs::Odometry>("/combined_odom", 1);
	/*initialize*/
	initializeOdom(_odom);
}

void CombineRPYLinear::initializeOdom(nav_msgs::Odometry& odom)
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

void CombineRPYLinear::callbackQuat(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
	tf::Quaternion q_rp, q_y;
	quaternionMsgToTF(msg->quaternion, q_rp);
	quaternionMsgToTF(_odom.pose.pose.orientation, q_y);
	/*combine*/
	combineRPY(q_rp, q_y);
	/*publication*/
	publication(msg->header.stamp);
}

void CombineRPYLinear::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	/*skip first callback*/
	if(!_got_first_imu){
		_stamp_imu_last = msg->header.stamp;
		_got_first_imu = true;
		return;
	}
	/*get dt*/
	double dt;
	try{
		dt = (msg->header.stamp - _stamp_imu_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
		return;
	}
	/*prediction*/
	transformAngVel(*msg, dt);
	/*publication*/
	publication(msg->header.stamp);
	/*reset*/
	_stamp_imu_last = msg->header.stamp;
}

void CombineRPYLinear::callbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!_got_bias){
		_bias = *msg;
		_got_bias = true;
	}
}

void CombineRPYLinear::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
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

void CombineRPYLinear::combineRPY(tf::Quaternion q_rp, tf::Quaternion q_y)
{
	double r1, p1, y1, r2, p2, y2;
	/*RP*/
	tf::Matrix3x3(q_rp).getRPY(r1, p1, y1);
	/*Y*/
	tf::Matrix3x3(q_y).getRPY(r2, p2, y2);
	/*combine*/
	tf::Quaternion q = tf::createQuaternionFromRPY(r1, p1, y2);
	quaternionTFToMsg(q, _odom.pose.pose.orientation);
}

void CombineRPYLinear::transformAngVel(sensor_msgs::Imu imu, double dt)
{
	/*relative rotation*/
	double dr = imu.angular_velocity.x*dt;
	double dp = imu.angular_velocity.y*dt;
	double dy = imu.angular_velocity.z*dt;
	if(_got_bias){
		dr -= _bias.angular_velocity.x*dt;
		dp -= _bias.angular_velocity.y*dt;
		dy -= _bias.angular_velocity.z*dt;
	}
	tf::Quaternion q_rel_rot = tf::createQuaternionFromRPY(dr, dp, dy);
	/*original*/
	tf::Quaternion q;
	quaternionMsgToTF(_odom.pose.pose.orientation, q);
	/*transform*/
	tf::Quaternion q_trans = q*q_rel_rot;
	q_trans.normalize();
	/*combine*/
	combineRPY(q, q_trans);
}

void CombineRPYLinear::transformLinVel(nav_msgs::Odometry odom_cbnow, double dt)
{
	tf::Quaternion q_ori;
	quaternionMsgToTF(_odom.pose.pose.orientation, q_ori);
	tf::Quaternion q_global_move;
	/*transform*/
	if(_linear_vel_is_available){
		tf::Quaternion q_local_move(
			odom_cbnow.twist.twist.linear.x*dt,
			0.0,
			0.0,
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

void CombineRPYLinear::publication(ros::Time stamp)
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
	ros::init(argc, argv, "combine_rp_y_linear");

	CombineRPYLinear combine_rp_y_linear;

	ros::spin();
}
