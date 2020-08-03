#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>

class QuatstampedToRPY{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		/*subscriber*/
		ros::Subscriber _sub_quat;
		/*publisher*/
		ros::Publisher _pub_rpy;
		/*msg*/
		geometry_msgs::Vector3Stamped _rpy;
	public:
		QuatstampedToRPY();
		void callbackQuat(const geometry_msgs::QuaternionStampedConstPtr& msg);
		void conversion(geometry_msgs::QuaternionStamped quat);
		void print(void);
		void publication(void);
};

QuatstampedToRPY::QuatstampedToRPY()
{
	_sub_quat = _nh.subscribe("/quat", 1, &QuatstampedToRPY::callbackQuat, this);
	_pub_rpy = _nh.advertise<geometry_msgs::Vector3Stamped>("/rpy", 1);
}

void QuatstampedToRPY::callbackQuat(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
	conversion(*msg);
	print();
	publication();
}

void QuatstampedToRPY::conversion(geometry_msgs::QuaternionStamped quat)
{
	_rpy.header = quat.header;
	tf::Quaternion q;
	quaternionMsgToTF(quat.quaternion, q);
	tf::Matrix3x3(q).getRPY(_rpy.vector.x, _rpy.vector.y, _rpy.vector.z);
}

void QuatstampedToRPY::print(void)
{
	std::cout << "----- " << std::endl;
	std::cout 
		<< "(r, p, y)[deg] = "
		<< _rpy.vector.x/M_PI*180.0 << ", " 
		<< _rpy.vector.y/M_PI*180.0 << ", " 
		<< _rpy.vector.z/M_PI*180.0 << std::endl;
}

void QuatstampedToRPY::publication(void)
{
	_pub_rpy.publish(_rpy);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "quatstamped_to_rpy");

	QuatstampedToRPY quatstamped_to_rpy;

	ros::spin();
}
