#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

class OdometryInputLinearVelocity{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub_odom;
		/*objects*/
		nav_msgs::Odometry odom;
		nav_msgs::Odometry odom_last;
		nav_msgs::Odometry odom_now;
		/*flags*/
		bool first_callback_odom = true;
	public:
		OdometryInputLinearVelocity();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void GetVelocity(void);
		void Publication(void);
};

OdometryInputLinearVelocity::OdometryInputLinearVelocity()
{
	sub_odom = nh.subscribe("/odom", 1, &OdometryInputLinearVelocity::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom/input_linearvelocity", 1);
}

void OdometryInputLinearVelocity::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom_now = *msg;
	odom = *msg;

	if(!first_callback_odom){
		GetVelocity();
		Publication();
	}

	odom_last = odom_now;
	first_callback_odom = false;
}

void OdometryInputLinearVelocity::GetVelocity(void)
{
	/*dt*/
	double dt = (odom_now.header.stamp - odom_last.header.stamp).toSec();
	/*dx*/
	tf::Quaternion q_pose_last;
	quaternionMsgToTF(odom_last.pose.pose.orientation, q_pose_last);

	tf::Quaternion q_global_move(
		odom_now.pose.pose.position.x - odom_last.pose.pose.position.x,
		odom_now.pose.pose.position.y - odom_last.pose.pose.position.y,
		odom_now.pose.pose.position.z - odom_last.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_pose_last.inverse()*q_global_move*q_pose_last;

	/*input*/
	odom.twist.twist.linear.x = q_local_move.x()/dt;
	odom.twist.twist.linear.y = q_local_move.y()/dt;
	odom.twist.twist.linear.z = q_local_move.z()/dt;
}

void OdometryInputLinearVelocity::Publication(void)
{
	/*publish*/
	pub_odom.publish(odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_input_linearvelocity");

	OdometryInputLinearVelocity odometry_input_linearvelocity;

	ros::spin();
}
