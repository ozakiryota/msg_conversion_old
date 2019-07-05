#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>

class PoseStampedToRPY{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscriber*/
		ros::Subscriber sub_pose;
		/*publisher*/
		ros::Publisher pub_rpy;
		/*objects*/
	public:
		PoseStampedToRPY();
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void Print(geometry_msgs::PoseStamped pose, std_msgs::Float64MultiArray rpy);
};

PoseStampedToRPY::PoseStampedToRPY()
{
	sub_pose = nh.subscribe("/pose", 1, &PoseStampedToRPY::CallbackPose, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy", 1);
}

void PoseStampedToRPY::CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	tf::Quaternion q_orientation;
	quaternionMsgToTF(msg->pose.orientation, q_orientation);
	std_msgs::Float64MultiArray rpy_pub;	//[deg]
	rpy_pub.data.resize(3);
	tf::Matrix3x3(q_orientation).getRPY(rpy_pub.data[0], rpy_pub.data[1], rpy_pub.data[2]);
	for(int i=0;i<3;i++)	rpy_pub.data[i] = rpy_pub.data[i]/M_PI*180.0;
	pub_rpy.publish(rpy_pub);
	Print(*msg, rpy_pub);
}

void PoseStampedToRPY::Print(geometry_msgs::PoseStamped pose, std_msgs::Float64MultiArray rpy)
{
	std::cout 
		<< "(x, y, z) = "
		<< pose.pose.position.x << "," 
		<< pose.pose.position.y << "," 
		<< pose.pose.position.z << std::endl;
	double d = sqrt(pose.pose.position.x*pose.pose.position.x + pose.pose.position.y*pose.pose.position.y + pose.pose.position.z*pose.pose.position.z);
	std::cout << "Euclidian distance = " << d << std::endl;
	std::cout 
		<< "(r, p, y) = "
		<< rpy.data[0] << "," 
		<< rpy.data[1] << "," 
		<< rpy.data[2] << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "posestamped_to_rpy");

	PoseStampedToRPY posestamped_to_rpy;

	ros::spin();
}
