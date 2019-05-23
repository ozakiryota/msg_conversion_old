#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class ResetOriginOdometry{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscriber*/
		ros::Subscriber sub_odom;
		/*publisher*/
		ros::Publisher pub_odom;
		tf::TransformBroadcaster tf_broadcaster;
		/*objects*/
		tf::Quaternion q_ini_position;
		tf::Quaternion q_ini_orientation;
		tf::Quaternion q_raw_position;
		tf::Quaternion q_raw_orientation;
		tf::Quaternion q_relative_position;
		tf::Quaternion q_relative_orientation;
		/*time*/
		ros::Time time_pub;
		/*flags*/
		bool inipose_is_available = false;
		/*fram_id*/
		std::string parent_frame_id_name;
		std::string child_frame_id_name;
	public:
		ResetOriginOdometry();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Publication(void);
};

ResetOriginOdometry::ResetOriginOdometry()
	: nhPrivate("~")
{
	sub_odom = nh.subscribe("/odom", 1, &ResetOriginOdometry::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom/reset_origin", 1);

	nhPrivate.param("parent_frame_id", parent_frame_id_name, std::string("/odom"));
	nhPrivate.param("child_frame_id", child_frame_id_name, std::string("/odom/reset_origin"));
}

void ResetOriginOdometry::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	time_pub = msg->header.stamp;
	q_raw_position = tf::Quaternion(
		msg->pose.pose.position.x,
		msg->pose.pose.position.y,
		msg->pose.pose.position.z,
		0.0
	);
	quaternionMsgToTF(msg->pose.pose.orientation, q_raw_orientation);
	if(!inipose_is_available){
		q_ini_position = q_raw_position;
		q_ini_orientation = q_raw_orientation;
		inipose_is_available = true;
	}
	else{
		q_relative_position = tf::Quaternion(
			q_raw_position.x() - q_ini_position.x(),
			q_raw_position.y() - q_ini_position.y(),
			q_raw_position.z() - q_ini_position.z(),
			0.0
		);
		q_relative_position = q_ini_orientation.inverse()*q_relative_position*q_ini_orientation;
		q_relative_orientation = q_ini_orientation.inverse()*q_raw_orientation;

		Publication();
	}
}

void ResetOriginOdometry::Publication(void)
{
	/*publish*/
	nav_msgs::Odometry odom_pub;
	odom_pub.header.frame_id = parent_frame_id_name;
	odom_pub.child_frame_id = child_frame_id_name;
	odom_pub.header.stamp = time_pub;
	odom_pub.pose.pose.position.x = q_relative_position.x();
	odom_pub.pose.pose.position.y = q_relative_position.y();
	odom_pub.pose.pose.position.z = q_relative_position.z();
	quaternionTFToMsg(q_relative_orientation, odom_pub.pose.pose.orientation);
	pub_odom.publish(odom_pub);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header = odom_pub.header;
	transform.child_frame_id = odom_pub.child_frame_id;
	transform.transform.translation.x = odom_pub.pose.pose.position.x;
	transform.transform.translation.y = odom_pub.pose.pose.position.y;
	transform.transform.translation.z = odom_pub.pose.pose.position.z;
	transform.transform.rotation = odom_pub.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "reset_origin_odometry");

	ResetOriginOdometry reset_origin_odometry;

	ros::spin();
}
