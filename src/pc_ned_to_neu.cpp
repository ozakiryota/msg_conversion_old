#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PCNEDToNEU{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		// ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_pc;
		/*publish*/
		ros::Publisher _pub_pc;
		/*pc*/
		sensor_msgs::PointCloud2 _pc_submsg;
		sensor_msgs::PointCloud2 _pc_pubmsg;
		/*list*/
		std::vector<std::string> _list_fields;

	public:
		PCNEDToNEU();
		void listUpPointType(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void convertNEDToNEU(void);
		template<typename CloudPtr> void flipAxises(CloudPtr pc);
		void publication(void);
};

PCNEDToNEU::PCNEDToNEU()
	// : _nhPrivate("~")
{
	std::cout << "--- pc_ned_to_neu ---" << std::endl;
	/*subscriber*/
	_sub_pc = _nh.subscribe("/cloud", 1, &PCNEDToNEU::callbackPC, this);
	/*publisher*/
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/neu", 1);
	/*initialize*/
	listUpPointType();
}

void PCNEDToNEU::listUpPointType(void)
{
	_list_fields = {
		"x y z",
		"x y z intensity",
		"x y z intensity ring"	//velodyne
		// "x y z strength",
		// "x y z normal_x normal_y normal_z curvature",
		// "x y z intensity normal_x normal_y normal_z curvature"
	};
}

void PCNEDToNEU::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	_pc_submsg = *msg;
	convertNEDToNEU();
	publication();
}

void PCNEDToNEU::convertNEDToNEU(void)
{
	std::string fields = pcl::getFieldsList(_pc_submsg);

	if(fields == _list_fields[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
		flipAxises(pc);
	}
	else if(fields == _list_fields[1] || fields == _list_fields[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
		flipAxises(pc);
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr>
void PCNEDToNEU::flipAxises(CloudPtr pc)
{
	pcl::fromROSMsg(_pc_submsg, *pc);

	for(size_t i=0; i<pc->points.size(); ++i){
		pc->points[i].y *= -1;
		pc->points[i].z *= -1;
	}

	pcl::toROSMsg(*pc, _pc_pubmsg);	
}

void PCNEDToNEU::publication(void)
{
	_pub_pc.publish(_pc_pubmsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_ned_to_neu");
	
	PCNEDToNEU pc_ned_to_neu;

	ros::spin();
}
