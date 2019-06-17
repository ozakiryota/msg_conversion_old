#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Odometry3dTo2d{
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
		nav_msgs::Odometry odom_pub;
		/*fram_id*/
		std::string parent_frame_id_name;
		std::string child_frame_id_name;
	public:
		Odometry3dTo2d();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Publication(void);
};

Odometry3dTo2d::Odometry3dTo2d()
	: nhPrivate("~")
{
	sub_odom = nh.subscribe("/odom", 1, &Odometry3dTo2d::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom/3dto2d", 1);

	nhPrivate.param("parent_frame_id", parent_frame_id_name, std::string("/odom"));
	nhPrivate.param("child_frame_id", child_frame_id_name, std::string("/odom/3dto2d"));
}

void Odometry3dTo2d::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom_pub = *msg;
	odom_pub.child_frame_id = child_frame_id_name;
	/*position*/
	odom_pub.pose.pose.position.z = 0.0;
	/*orientation*/
	tf::Quaternion q_orientation;
	quaternionMsgToTF(msg->pose.pose.orientation, q_orientation);
	double r, p, y;
	tf::Matrix3x3(q_orientation).getRPY(r, p, y);
	q_orientation = tf::createQuaternionFromRPY(0.0, 0.0, y);
	quaternionTFToMsg(q_orientation, odom_pub.pose.pose.orientation);

	Publication();
}

void Odometry3dTo2d::Publication(void)
{
	/*publish*/
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
	ros::init(argc, argv, "odometry_3d_to_2d");

	Odometry3dTo2d odometry_3d_to_2d;

	ros::spin();
}
