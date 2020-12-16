#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class OdometryResetOrigin{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_odom;
		/*publisher*/
		ros::Publisher _pub_odom;
		tf::TransformBroadcaster _tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry _ini_odom;
		nav_msgs::Odometry _reset_odom;
		/*flags*/
		bool _got_first_odom = false;
		/*parameter*/
		std::string _parent_frame_id;
		std::string _child_frame_id;
	public:
		OdometryResetOrigin();
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void conversion(nav_msgs::Odometry now_odom);
		void publication(void);
};

OdometryResetOrigin::OdometryResetOrigin()
	: _nhPrivate("~")
{
	std::cout << "--- odometry_reset_origin ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("parent_frame_id", _parent_frame_id, std::string("/parent_frame"));
	std::cout << "_parent_frame_id = " << _parent_frame_id << std::endl;
	_nhPrivate.param("child_frame_id", _child_frame_id, std::string("/child_frame"));
	std::cout << "_child_frame_id = " << _child_frame_id << std::endl;
	/*subscriber*/
	_sub_odom = _nh.subscribe("/odom", 1, &OdometryResetOrigin::callbackOdom, this);
	/*publisher*/
	_pub_odom = _nh.advertise<nav_msgs::Odometry>("/odom/reset_origin", 1);
	/*initialization*/
	_reset_odom.header.frame_id = _parent_frame_id;
	_reset_odom.child_frame_id = _child_frame_id;
}

void OdometryResetOrigin::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(!_got_first_odom){
		_ini_odom = *msg;
		_got_first_odom = true;
		return;
	}
	conversion(*msg);
	publication();
}

void OdometryResetOrigin::conversion(nav_msgs::Odometry now_odom)
{
	/*tf*/
	tf::Quaternion q_ini_orientation;
	quaternionMsgToTF(_ini_odom.pose.pose.orientation, q_ini_orientation);
	tf::Quaternion q_now_orientation;
	quaternionMsgToTF(now_odom.pose.pose.orientation, q_now_orientation);
	/*diff*/
	tf::Quaternion q_reset_abs_position = tf::Quaternion(
		now_odom.pose.pose.position.x - _ini_odom.pose.pose.position.x,
		now_odom.pose.pose.position.y - _ini_odom.pose.pose.position.y,
		now_odom.pose.pose.position.z - _ini_odom.pose.pose.position.z,
		0.0
	);
	/*rotation*/
	tf::Quaternion q_reset_rel_position = q_ini_orientation.inverse()*q_reset_abs_position*q_ini_orientation;
	tf::Quaternion q_reset_rel_orientation = q_ini_orientation.inverse()*q_now_orientation;
	/*input*/
	_reset_odom.header.stamp = now_odom.header.stamp;
	_reset_odom.pose.pose.position.x = q_reset_rel_position.x();
	_reset_odom.pose.pose.position.y = q_reset_rel_position.y();
	_reset_odom.pose.pose.position.z = q_reset_rel_position.z();
	quaternionTFToMsg(q_reset_rel_orientation, _reset_odom.pose.pose.orientation);
	_reset_odom.twist = now_odom.twist;
}

void OdometryResetOrigin::publication(void)
{
	/*msg*/
	_pub_odom.publish(_reset_odom);
	/*tf*/
    geometry_msgs::TransformStamped tf_transform;
	tf_transform.header = _reset_odom.header;
	tf_transform.child_frame_id = _reset_odom.child_frame_id;
	tf_transform.transform.translation.x = _reset_odom.pose.pose.position.x;
	tf_transform.transform.translation.y = _reset_odom.pose.pose.position.y;
	tf_transform.transform.translation.z = _reset_odom.pose.pose.position.z;
	tf_transform.transform.rotation = _reset_odom.pose.pose.orientation;
	_tf_broadcaster.sendTransform(tf_transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_reset_origin");

	OdometryResetOrigin odometry_reset_origin;

	ros::spin();
}
