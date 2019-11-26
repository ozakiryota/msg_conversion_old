#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
/* #include <Eigen/Core> */
/* #include <Eigen/LU> */
#include <sensor_msgs/Imu.h>

class OdometryYawToQuaternion{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub;
		/*odom*/
		nav_msgs::Odometry odom;
		/*objects*/
		tf::Quaternion q_pose;
		/*time*/
		ros::Time time_now_odom;
		ros::Time time_last_odom;
		/*flags*/
		bool first_callback_odom = true;
        /*angle*/
        double yaw = 0.0;
		/*parameters*/
		double velocity_scalar_ratio;
	public:
		OdometryYawToQuaternion();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
};

OdometryYawToQuaternion::OdometryYawToQuaternion()
	:nhPrivate("~")
{
	sub_odom = nh.subscribe("/tinypower/odom", 1, &OdometryYawToQuaternion::CallbackOdom, this);
	pub = nh.advertise<nav_msgs::Odometry>("/tinypower/odom/republished", 1);
	InitializeOdom(odom);

	nhPrivate.param("velocity_scalar_ratio", velocity_scalar_ratio, 1.0);
	std::cout << "velocity_scalar_ratio = " << velocity_scalar_ratio << std::endl;
}

void OdometryYawToQuaternion::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/tinypower/odom/republished";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void OdometryYawToQuaternion::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	time_now_odom = ros::Time::now();
	double dt = (time_now_odom - time_last_odom).toSec();
	time_last_odom = time_now_odom;
	if(first_callback_odom)	dt = 0.0;

    yaw += msg->twist.twist.angular.z*dt;
    yaw = atan2(sin(yaw), cos(yaw));
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
    quaternionTFToMsg(q, odom.pose.pose.orientation);
	
	odom.pose.pose.position.x += msg->twist.twist.linear.x*dt * cos(yaw);
	odom.pose.pose.position.y += msg->twist.twist.linear.x*dt * sin(yaw);
	odom.pose.pose.position.z = 0.0;
	
    odom.header.stamp = msg->header.stamp;
	odom.twist = msg->twist;
	odom.twist.twist.linear.x *= velocity_scalar_ratio;
    pub.publish(odom);

    first_callback_odom = false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_yaw_to_quaternion");

	OdometryYawToQuaternion odometry_yaw_to_quaternion;

	ros::spin();
}
