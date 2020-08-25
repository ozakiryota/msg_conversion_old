#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>

class ImuToRPY{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		/*subscriber*/
		ros::Subscriber _sub_imu;
		/*publisher*/
		ros::Publisher _pub_rpy;
		/*msg*/
		geometry_msgs::Vector3Stamped _rpy;
	public:
		ImuToRPY();
		void callbackIMU(const sensor_msgs::ImuConstPtr msg);
		void conversion(sensor_msgs::Imu imu);
		void print(void);
		void publication(void);
};

ImuToRPY::ImuToRPY()
{
	_sub_imu = _nh.subscribe("/imu/data", 1, &ImuToRPY::callbackIMU, this);
	_pub_rpy = _nh.advertise<geometry_msgs::Vector3Stamped>("/rpy", 1);
}

void ImuToRPY::callbackIMU(const sensor_msgs::ImuConstPtr msg)
{
	conversion(*msg);
	print();
	publication();
}

void ImuToRPY::conversion(sensor_msgs::Imu imu)
{
	_rpy.header = imu.header;
	tf::Quaternion q;
	quaternionMsgToTF(imu.orientation, q);
	tf::Matrix3x3(q).getRPY(_rpy.vector.x, _rpy.vector.y, _rpy.vector.z);
}

void ImuToRPY::print(void)
{
	std::cout << "-----" << std::endl;
	std::cout 
		<< "(r, p, y)[deg] = "
		<< _rpy.vector.x/M_PI*180.0 << ", " 
		<< _rpy.vector.y/M_PI*180.0 << ", " 
		<< _rpy.vector.z/M_PI*180.0 << std::endl;
}

void ImuToRPY::publication(void)
{
	_pub_rpy.publish(_rpy);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_to_rpy");

	ImuToRPY imu_to_rpy;

	ros::spin();
}
