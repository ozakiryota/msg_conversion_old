#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class TfToPoseStamped{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*tf*/
		tf::TransformListener _tf_listener;
		/*publisher*/
		ros::Publisher _pub_pose;
		/*msg*/
		geometry_msgs::PoseStamped _pose_msg;
		/*parameter*/
		std::string _parent_frame_id;
		std::string _child_frame_id;
		int _loop_rate_hz;
	public:
		TfToPoseStamped(void);
		void loop(void);
		void listenTF(void);
		void inputToPoseStamped(tf::StampedTransform tf_transform);
		void publication(void);
};

TfToPoseStamped::TfToPoseStamped()
	: _nhPrivate("~")
{
	std::cout << "--- tf_to_posestamped ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("parent_frame_id", _parent_frame_id, std::string("/parent_frame"));
	std::cout << "_parent_frame_id = " << _parent_frame_id << std::endl;
	_nhPrivate.param("child_frame_id", _child_frame_id, std::string("/child_frame"));
	std::cout << "_child_frame_id = " << _child_frame_id << std::endl;
	_nhPrivate.param("loop_rate_hz", _loop_rate_hz, 100);
	std::cout << "_loop_rate_hz = " << _loop_rate_hz << std::endl;
	/*publisher*/
	_pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("/tf_to_posestamped", 1);
	/*initialization*/
	_pose_msg.header.frame_id = _parent_frame_id;
}

void TfToPoseStamped::loop(void)
{
	ros::Rate loop_rate(_loop_rate_hz);
	while(ros::ok()){
		listenTF();
		loop_rate.sleep();
	}
}

void TfToPoseStamped::listenTF(void)
{
	try{
		tf::StampedTransform tf_transform;
		_tf_listener.lookupTransform(_parent_frame_id, _child_frame_id, ros::Time(0), tf_transform);
		inputToPoseStamped(tf_transform);
		publication();
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}

void TfToPoseStamped::inputToPoseStamped(tf::StampedTransform tf_transform)
{
	/*stamp*/
	_pose_msg.header.stamp = tf_transform.stamp_;
	/*position*/
	_pose_msg.pose.position.x = tf_transform.getOrigin().x();
	_pose_msg.pose.position.y = tf_transform.getOrigin().y();
	_pose_msg.pose.position.z = tf_transform.getOrigin().z();
	/*orientation*/
	quaternionTFToMsg(tf_transform.getRotation(), _pose_msg.pose.orientation);
}

void TfToPoseStamped::publication(void)
{
	_pub_pose.publish(_pose_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_to_posestamped");

	TfToPoseStamped tf_to_posestamped;
	tf_to_posestamped.loop();
}
