#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class OdometryToPoseStamped{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscriber*/
		ros::Subscriber sub_odom;
		/*publisher*/
		ros::Publisher pub_pose;
	public:
		OdometryToPoseStamped();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
};

OdometryToPoseStamped::OdometryToPoseStamped()
	: nhPrivate("~")
{
	sub_odom = nh.subscribe("/odom", 1, &OdometryToPoseStamped::CallbackOdom, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose/from_odom", 1);
}

void OdometryToPoseStamped::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	geometry_msgs::PoseStamped pose_pub;
	pose_pub.header = msg->header;
	pose_pub.pose = msg->pose.pose;
	pub_pose.publish(pose_pub);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_to_posestamped");

	OdometryToPoseStamped odometry_to_posestamped;

	ros::spin();
}
