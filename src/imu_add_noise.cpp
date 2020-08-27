#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuAddNoise{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_imu;
		/*publisher*/
		ros::Publisher _pub_imu;
		/*msg*/
		sensor_msgs::Imu _imu_with_noise;
		/*parameter*/
		double _angular_noise_range;
		double _linear_noise_range;
	public:
		ImuAddNoise();
		void callbackIMU(const sensor_msgs::ImuConstPtr msg);
		void addNoise(void);
		void publication(void);
};

ImuAddNoise::ImuAddNoise()
	: _nhPrivate("~")
{
	std::cout << "--- imu_add_noise ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("angular_noise_range", _angular_noise_range, 0.001);
	std::cout << "_angular_noise_range = " << _angular_noise_range << std::endl;
	_nhPrivate.param("linear_noise_range", _linear_noise_range, 0.001);
	std::cout << "_linear_noise_range = " << _linear_noise_range << std::endl;
	/*subscriber*/
	_sub_imu = _nh.subscribe("/imu/data", 1, &ImuAddNoise::callbackIMU, this);
	/*publisher*/
	_pub_imu = _nh.advertise<sensor_msgs::Imu>("/imu/data/with_noise", 1);
}

void ImuAddNoise::callbackIMU(const sensor_msgs::ImuConstPtr msg)
{
	_imu_with_noise = *msg;
	addNoise();
	publication();
}

void ImuAddNoise::addNoise(void)
{
	/*random tool*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<> nd_angular(-_angular_noise_range, _angular_noise_range);
	std::normal_distribution<> nd_linear(-_linear_noise_range, _linear_noise_range);
	/*add*/
	_imu_with_noise.angular_velocity.x += nd_angular(mt);
	_imu_with_noise.angular_velocity.y += nd_angular(mt);
	_imu_with_noise.angular_velocity.z += nd_angular(mt);
	_imu_with_noise.linear_acceleration.x += nd_linear(mt);
	_imu_with_noise.linear_acceleration.y += nd_linear(mt);
	_imu_with_noise.linear_acceleration.z += nd_linear(mt);
}

void ImuAddNoise::publication(void)
{
	_pub_imu.publish(_imu_with_noise);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_add_noise");

	ImuAddNoise imu_add_noise;

	ros::spin();
}
