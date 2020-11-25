#include <ros/ros.h>
// #include <tf/tf.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>

class VectorToArrow{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_vector;
		/*publisher*/
		ros::Publisher _pub_vismarker;
		/*visualization marker*/
		visualization_msgs::Marker _arrow;
		/*parameter*/
		bool _use_subscribed_frameid;
		std::string _frame_id;
		double _shaft_length;
		double _shaft_diameter;

	public:
		VectorToArrow();
		void initializeVisMarker(void);
		void callbackVector(const geometry_msgs::Vector3StampedConstPtr& msg);
		void inputVisMarker(geometry_msgs::Vector3Stamped vector_msg);
		void publication(void);
};

VectorToArrow::VectorToArrow()
	: _nhPrivate("~")
{
	std::cout << "--- vector_to_arrow ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("use_subscribed_frameid", _use_subscribed_frameid, true);
	std::cout << "_use_subscribed_frameid = " << (bool)_use_subscribed_frameid << std::endl;
	_nhPrivate.param("frame_id", _frame_id, std::string("/frame"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("shaft_length", _shaft_length, 5.0);
	std::cout << "_shaft_length = " << _shaft_length << std::endl;
	_nhPrivate.param("shaft_diameter", _shaft_diameter, 0.25);
	std::cout << "_shaft_diameter = " << _shaft_diameter << std::endl;
	/*subscriber*/
	_sub_vector = _nh.subscribe("/vector", 1, &VectorToArrow::callbackVector, this);
	/*subscriber*/
	_pub_vismarker = _nh.advertise<visualization_msgs::Marker>("/vis/arrow", 1);
	/*initialize*/
	initializeVisMarker();
}

void VectorToArrow::initializeVisMarker(void)
{
	if(!_use_subscribed_frameid)	_arrow.header.frame_id = _frame_id;
	_arrow.points.resize(2);
	_arrow.points[0].x = 0.0;
	_arrow.points[0].y = 0.0;
	_arrow.points[0].z = 0.0;
	_arrow.ns = "arrow";
	_arrow.id = 0;
	_arrow.type = visualization_msgs::Marker::ARROW;
	_arrow.action = visualization_msgs::Marker::ADD;
	_arrow.scale.x = _shaft_diameter;		//shaft diameter
	_arrow.scale.y = 1.5*_shaft_diameter;	//head diameter
	// _arrow.scale.z = 0.25*_shaft_length;	//head length
	_arrow.color.r = 1.0;
	_arrow.color.g = 0.0;
	_arrow.color.b = 1.0;
	_arrow.color.a = 1.0;
}

void VectorToArrow::callbackVector(const geometry_msgs::Vector3StampedConstPtr& msg)
{
	/*input*/
	inputVisMarker(*msg);
	/*publication*/
	publication();
}

void VectorToArrow::inputVisMarker(geometry_msgs::Vector3Stamped vector_msg)
{
	/*header*/
	if(_use_subscribed_frameid)	_arrow.header.frame_id = vector_msg.header.frame_id;
	_arrow.header.stamp = vector_msg.header.stamp;
	/*end point*/
	double norm = sqrt(
		vector_msg.vector.x*vector_msg.vector.x
		+ vector_msg.vector.y*vector_msg.vector.y
		+ vector_msg.vector.z*vector_msg.vector.z
	);
	geometry_msgs::Point end;
	end.x = vector_msg.vector.x/norm;
	end.y = vector_msg.vector.y/norm;
	end.z = vector_msg.vector.z/norm;
	/*scale*/
	_arrow.points[1].x = end.x*_shaft_length;
	_arrow.points[1].y = end.y*_shaft_length;
	_arrow.points[1].z = end.z*_shaft_length;
}

void VectorToArrow::publication(void)
{
	/*reset*/
	// visualization_msgs::Marker delete_markers;
	// delete_markers.action = visualization_msgs::Marker::DELETEALL;
	// _pub_vismarker.publish(delete_markers);
	/*_arrow*/
	_pub_vismarker.publish(_arrow);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vector_to_arrow");
	
	VectorToArrow vector_to_arrow;

	ros::spin();
}
