#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>

class GravityToRPY{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		/*subscriber*/
		ros::Subscriber _sub_vector;
		/*publisher*/
		ros::Publisher _pub_rpy;
		/*msg*/
		geometry_msgs::Vector3Stamped _rpy;
	public:
		GravityToRPY();
		void callbackVector(const geometry_msgs::Vector3StampedConstPtr& msg);
		void conversion(geometry_msgs::Vector3Stamped gravity);
		void print(void);
		void publication(void);
};

GravityToRPY::GravityToRPY()
{
	std::cout << "--- gravity_to_rpy ---" << std::endl;
	/*subscriber*/
	_sub_vector = _nh.subscribe("/gravity", 1, &GravityToRPY::callbackVector, this);
	/*publisher*/
	_pub_rpy = _nh.advertise<geometry_msgs::Vector3Stamped>("/rpy", 1);
}

void GravityToRPY::callbackVector(const geometry_msgs::Vector3StampedConstPtr& msg)
{
	conversion(*msg);
	// print();
	publication();
}

void GravityToRPY::conversion(geometry_msgs::Vector3Stamped gravity)
{
	/*inverse*/
	gravity.vector.x *= -1;
	gravity.vector.y *= -1;
	gravity.vector.z *= -1;
	/*rp*/
    double r = atan2(gravity.vector.y, gravity.vector.z);
	double p = atan2(-gravity.vector.x, sqrt(gravity.vector.y*gravity.vector.y + gravity.vector.z*gravity.vector.z));
	/*input*/
	_rpy.header = gravity.header;
	_rpy.vector.x = r;
	_rpy.vector.y = p;
	_rpy.vector.z = 0;
}

void GravityToRPY::print(void)
{
	std::cout << "----- " << std::endl;
	std::cout 
		<< "(r, p, y)[deg] = "
		<< _rpy.vector.x/M_PI*180.0 << ", " 
		<< _rpy.vector.y/M_PI*180.0 << ", " 
		<< _rpy.vector.z/M_PI*180.0 << std::endl;
}

void GravityToRPY::publication(void)
{
	_pub_rpy.publish(_rpy);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gravity_to_rpy");

	GravityToRPY gravity_to_rpy;

	ros::spin();
}
