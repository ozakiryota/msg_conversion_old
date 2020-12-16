#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class PoseStampedResetOrigin{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pose;
		/*publisher*/
		ros::Publisher _pub_pose;
		tf::TransformBroadcaster _tf_broadcaster;
		/*odom*/
		geometry_msgs::PoseStamped _ini_pose;
		geometry_msgs::PoseStamped _reset_pose;
		/*flags*/
		bool _got_first_pose = false;
		/*parameter*/
		std::string _parent_frame_id;
		std::string _child_frame_id;
	public:
		PoseStampedResetOrigin();
		void callbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void conversion(geometry_msgs::PoseStamped now_pose);
		void publication(void);
};

PoseStampedResetOrigin::PoseStampedResetOrigin()
	: _nhPrivate("~")
{
	std::cout << "--- posestamped_reset_origin ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("parent_frame_id", _parent_frame_id, std::string("/parent_frame"));
	std::cout << "_parent_frame_id = " << _parent_frame_id << std::endl;
	_nhPrivate.param("child_frame_id", _child_frame_id, std::string("/child_frame"));
	std::cout << "_child_frame_id = " << _child_frame_id << std::endl;
	/*subscriber*/
	_sub_pose = _nh.subscribe("/pose", 1, &PoseStampedResetOrigin::callbackPose, this);
	/*publisher*/
	_pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("/pose/reset_origin", 1);
	/*initialization*/
	_reset_pose.header.frame_id = _parent_frame_id;
}

void PoseStampedResetOrigin::callbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	if(!_got_first_pose){
		_ini_pose = *msg;
		_got_first_pose = true;
		return;
	}
	conversion(*msg);
	publication();
}

void PoseStampedResetOrigin::conversion(geometry_msgs::PoseStamped now_pose)
{
	/*tf*/
	tf::Quaternion q_ini_orientation;
	quaternionMsgToTF(_ini_pose.pose.orientation, q_ini_orientation);
	tf::Quaternion q_now_orientation;
	quaternionMsgToTF(now_pose.pose.orientation, q_now_orientation);
	/*diff*/
	tf::Quaternion q_reset_abs_position = tf::Quaternion(
		now_pose.pose.position.x - _ini_pose.pose.position.x,
		now_pose.pose.position.y - _ini_pose.pose.position.y,
		now_pose.pose.position.z - _ini_pose.pose.position.z,
		0.0
	);
	/*rotation*/
	tf::Quaternion q_reset_rel_position = q_ini_orientation.inverse()*q_reset_abs_position*q_ini_orientation;
	tf::Quaternion q_reset_rel_orientation = q_ini_orientation.inverse()*q_now_orientation;
	/*input*/
	_reset_pose.header.stamp = now_pose.header.stamp;
	_reset_pose.pose.position.x = q_reset_rel_position.x();
	_reset_pose.pose.position.y = q_reset_rel_position.y();
	_reset_pose.pose.position.z = q_reset_rel_position.z();
	quaternionTFToMsg(q_reset_rel_orientation, _reset_pose.pose.orientation);
}

void PoseStampedResetOrigin::publication(void)
{
	/*msg*/
	_pub_pose.publish(_reset_pose);
	/*tf*/
    geometry_msgs::TransformStamped tf_transform;
	tf_transform.header = _reset_pose.header;
	tf_transform.child_frame_id = _child_frame_id;
	tf_transform.transform.translation.x = _reset_pose.pose.position.x;
	tf_transform.transform.translation.y = _reset_pose.pose.position.y;
	tf_transform.transform.translation.z = _reset_pose.pose.position.z;
	tf_transform.transform.rotation = _reset_pose.pose.orientation;
	_tf_broadcaster.sendTransform(tf_transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "posestamped_reset_origin");

	PoseStampedResetOrigin posestamped_reset_origin;

	ros::spin();
}
