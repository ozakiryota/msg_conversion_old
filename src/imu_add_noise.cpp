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
		double _angular_noise_std;
		double _linear_noise_std;
	public:
		ImuAddNoise();
		void callbackIMU(const sensor_msgs::ImuConstPtr msg);
		void addNoise(void);
		void print(sensor_msgs::Imu imu);
		void publication(void);
};

ImuAddNoise::ImuAddNoise()
	: _nhPrivate("~")
{
	std::cout << "--- imu_add_noise ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("angular_noise_std", _angular_noise_std, 0.1);
	std::cout << "_angular_noise_std = " << _angular_noise_std << std::endl;
	_nhPrivate.param("linear_noise_std", _linear_noise_std, 0.1);
	std::cout << "_linear_noise_std = " << _linear_noise_std << std::endl;
	/*subscriber*/
	_sub_imu = _nh.subscribe("/imu/data", 1, &ImuAddNoise::callbackIMU, this);
	/*publisher*/
	_pub_imu = _nh.advertise<sensor_msgs::Imu>("/imu/data/with_noise", 1);
}

void ImuAddNoise::callbackIMU(const sensor_msgs::ImuConstPtr msg)
{
	_imu_with_noise = *msg;
	addNoise();
	print(*msg);
	publication();
}

void ImuAddNoise::addNoise(void)
{
	/*random tool*/
	std::random_device rd;
	std::mt19937 mt(rd());
	std::normal_distribution<> nd_angular(0.0, _angular_noise_std);
	std::normal_distribution<> nd_linear(0.0, _linear_noise_std);
	/*add*/
	_imu_with_noise.angular_velocity.x += nd_angular(mt);
	_imu_with_noise.angular_velocity.y += nd_angular(mt);
	_imu_with_noise.angular_velocity.z += nd_angular(mt);
	_imu_with_noise.linear_acceleration.x += nd_linear(mt);
	_imu_with_noise.linear_acceleration.y += nd_linear(mt);
	_imu_with_noise.linear_acceleration.z += nd_linear(mt);
}

void ImuAddNoise::print(sensor_msgs::Imu imu)
{
	std::cout << "-----" << std::endl;
	std::cout << "Original:" << std::endl
		<< "angular_velocity: "
			<< imu.angular_velocity.x << ", "
			<< imu.angular_velocity.y << ", "
			<< imu.angular_velocity.z << std::endl
		<< "linear_acceleration: "
			<< imu.linear_acceleration.x << ", "
			<< imu.linear_acceleration.y << ", "
			<< imu.linear_acceleration.z << std::endl;
	std::cout << "With noise:" << std::endl
		<< "angular_velocity: "
			<< _imu_with_noise.angular_velocity.x << ", "
			<< _imu_with_noise.angular_velocity.y << ", "
			<< _imu_with_noise.angular_velocity.z << std::endl
		<< "linear_acceleration: "
			<< _imu_with_noise.linear_acceleration.x << ", "
			<< _imu_with_noise.linear_acceleration.y << ", "
			<< _imu_with_noise.linear_acceleration.z << std::endl;
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
