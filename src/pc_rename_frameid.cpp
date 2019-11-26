#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
/* #include <nav_msgs/Odometry.h> */
/* #include <pcl/point_cloud.h> */
/* #include <pcl/point_types.h> */
/* #include <pcl/visualization/cloud_viewer.h> */
/* #include <pcl_conversions/pcl_conversions.h> */
/* #include <tf/tf.h> */
/* #include <pcl/common/transforms.h> */

class PCStoreWithOdometry{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*publish*/
		ros::Publisher pub_pc;
		/*parameters*/
		std::string frame_name;

	public:
		PCStoreWithOdometry();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
};

PCStoreWithOdometry::PCStoreWithOdometry()
	: nhPrivate("~")
{
	sub_pc = nh.subscribe("/cloud", 1, &PCStoreWithOdometry::CallbackPC, this);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/cloud/renamed", 1);

	nhPrivate.param("frame_name", frame_name, std::string("/frame"));
	std::cout << "frame_name = " << frame_name << std::endl;
}

void PCStoreWithOdometry::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	sensor_msgs::PointCloud2 pc_out;
	pc_out = *msg;
	pc_out.header.frame_id = frame_name;
	pub_pc.publish(pc_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store_with_odometry");
	
	PCStoreWithOdometry pc_store_with_odometry;

	ros::spin();
}
