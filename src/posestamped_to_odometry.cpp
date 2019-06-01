#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class PoseStampedToOdometry{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscriber*/
		ros::Subscriber sub_pose;
		/*publisher*/
		ros::Publisher pub_odom;
		/*objects*/
	public:
		PoseStampedToOdometry();
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
};

PoseStampedToOdometry::PoseStampedToOdometry()
{
	sub_pose = nh.subscribe("/pose", 1, &PoseStampedToOdometry::CallbackPose, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom/from_posestamped", 1);
}

void PoseStampedToOdometry::CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	nav_msgs::Odometry odom_pub;
	odom_pub.header = msg->header;
	odom_pub.pose.pose = msg->pose;
	pub_odom.publish(odom_pub);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "posestamped_to_odometry");

	PoseStampedToOdometry posestamped_to_odometry;

	ros::spin();
}
