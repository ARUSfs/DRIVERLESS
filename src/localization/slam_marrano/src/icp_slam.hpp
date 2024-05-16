#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include "common_msgs/Map.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/impl/centroid.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "PointXYZColorScore.h"

using namespace std;

class ICP_handle {
	private:
		bool has_map = false;
		float prev_time = 0;
		int i = 0;
	public:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher trans_pub;
		ros::Publisher pcl_pub;
		pcl::PointCloud<PointXYZColorScore>::Ptr previous_map;
		pcl::PointCloud<PointXYZColorScore>::Ptr allp_clustered;
		pcl::PointCloud<PointXYZColorScore>::Ptr to_be_clustered;
		Eigen::Matrix4f position;

		float score = 1.0;

		tf2_ros::TransformBroadcaster br;
		int flag = 0;

	public:
		ICP_handle();
		void map_callback(common_msgs::Map);
		void send_position();

};
