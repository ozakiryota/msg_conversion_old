#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdometryZXYToXYZ{
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
		OdometryZXYToXYZ();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Publication(void);
};

OdometryZXYToXYZ::OdometryZXYToXYZ()
	: nhPrivate("~")
{
	sub_odom = nh.subscribe("/odom", 1, &OdometryZXYToXYZ::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom/change_axis", 1);

	nhPrivate.param("parent_frame_id", parent_frame_id_name, std::string("/odom"));
	nhPrivate.param("child_frame_id", child_frame_id_name, std::string("/odom/change_axis"));
}

void OdometryZXYToXYZ::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom_pub.header.stamp = msg->header.stamp;
	odom_pub.header.frame_id = parent_frame_id_name;
	odom_pub.child_frame_id = child_frame_id_name;
	odom_pub.pose.pose.position.x = msg->pose.pose.position.z;
	odom_pub.pose.pose.position.y = msg->pose.pose.position.x;
	odom_pub.pose.pose.position.z = msg->pose.pose.position.y;
	odom_pub.pose.pose.orientation.x = msg->pose.pose.orientation.z;
	odom_pub.pose.pose.orientation.y = msg->pose.pose.orientation.x;
	odom_pub.pose.pose.orientation.z = msg->pose.pose.orientation.y;
	odom_pub.pose.pose.orientation.w = msg->pose.pose.orientation.w;

	Publication();
}

void OdometryZXYToXYZ::Publication(void)
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
	ros::init(argc, argv, "odom_creater_for_loamvelodyne");

	OdometryZXYToXYZ velodyne_odometry;

	ros::spin();
}
