#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>

class PoseStampedToRPY{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		/*subscriber*/
		ros::Subscriber _sub_pose;
		/*publisher*/
		ros::Publisher _pub_rpy;
		/*msg*/
		geometry_msgs::Vector3Stamped _rpy;
	public:
		PoseStampedToRPY();
		void callbackPose(const geometry_msgs::PoseStampedConstPtr msg);
		void conversion(geometry_msgs::PoseStamped);
		void print(geometry_msgs::PoseStamped pose);
		void publication(void);
};

PoseStampedToRPY::PoseStampedToRPY()
{
	_sub_pose = _nh.subscribe("/pose", 1, &PoseStampedToRPY::callbackPose, this);
	_pub_rpy = _nh.advertise<geometry_msgs::Vector3Stamped>("/rpy", 1);
}

void PoseStampedToRPY::callbackPose(const geometry_msgs::PoseStampedConstPtr msg)
{
	conversion(*msg);
	print(*msg);
	publication();
}

void PoseStampedToRPY::conversion(geometry_msgs::PoseStamped pose)
{
	_rpy.header = pose.header;
	tf::Quaternion q;
	quaternionMsgToTF(pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(_rpy.vector.x, _rpy.vector.y, _rpy.vector.z);
}

void PoseStampedToRPY::print(geometry_msgs::PoseStamped pose)
{
	std::cout << "-----" << std::endl;
	std::cout 
		<< "(x, y, z)[m] = "
		<< pose.pose.position.x << ", " 
		<< pose.pose.position.y << ", " 
		<< pose.pose.position.z << std::endl;
	std::cout 
		<< "(r, p, y)[deg] = "
		<< _rpy.vector.x/M_PI*180.0 << ", " 
		<< _rpy.vector.y/M_PI*180.0 << ", " 
		<< _rpy.vector.z/M_PI*180.0 << std::endl;
}

void PoseStampedToRPY::publication(void)
{
	_pub_rpy.publish(_rpy);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "posestamped_to_rpy");

	PoseStampedToRPY posestamped_to_rpy;

	ros::spin();
}
