#define PCL_NO_PRECOMPILE

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>


#include <pcl/common/impl/centroid.hpp>
#include "icp_slam.hpp"


using namespace std;

ICP_handle::ICP_handle(){
	previous_map = pcl::PointCloud<PointXYZColorScore>::Ptr(new pcl::PointCloud<PointXYZColorScore>);

	position = Eigen::Matrix4f::Identity(4, 4);
	prev_transformation = Eigen::Matrix4f::Identity(4, 4);

	sub = nh.subscribe<sensor_msgs::PointCloud2>("/perception_map", 1000, &ICP_handle::map_callback, this);

	map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/mapa_icp", 10);

	ros::Time hola = ros::Time::now();

}

void ICP_handle::map_callback(sensor_msgs::PointCloud2 map_msg) {
	callback_iteration++;

	pcl::PointCloud<PointXYZColorScore>::Ptr new_map(new pcl::PointCloud<PointXYZColorScore>);
	pcl::fromROSMsg(map_msg, *new_map);

	if(!has_map){
		*previous_map = *new_map;
		has_map = true;

		send_position();
		return;
	}

	pcl::PointCloud<PointXYZColorScore>::Ptr map_in_position = pcl::PointCloud<PointXYZColorScore>::Ptr(new pcl::PointCloud<PointXYZColorScore>);
	pcl::transformPointCloud(*new_map, *map_in_position, position);


	pcl::IterativeClosestPoint<PointXYZColorScore, PointXYZColorScore> icp;
	icp.setInputSource(map_in_position);
    icp.setInputTarget(previous_map);
	icp.setMaximumIterations(5);
	icp.setEuclideanFitnessEpsilon (0.005);
	icp.setTransformationEpsilon(1e-5);

	pcl::PointCloud<PointXYZColorScore> registered_map;
	icp.setMaxCorrespondenceDistance (1.0);
	icp.align(registered_map);

	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	

	float ang = (float)-atan2(transformation.coeff(0, 1), transformation.coeff(0,0));
	float dist = pow((transformation.coeff(0,3) - prev_transformation.coeff(0,3)),2) + pow((transformation.coeff(1,3) - prev_transformation.coeff(1,3)),2) + pow((transformation.coeff(2,3) - prev_transformation.coeff(2,3)),2);
	// std::cout << dist << std::endl;
	prev_transformation = transformation;


	position = transformation * position;
	send_position();

	std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
	//*previous_map += registered_map;
	for(PointXYZColorScore new_point : registered_map.points){
		float min_dist=INFINITY;
		int index;
		for(int i=0 ; i<previous_map->size(); i++){
			float dist = sqrt(pow(previous_map->points[i].x-new_point.x,2)+pow(previous_map->points[i].y-new_point.y,2));
			if(dist<min_dist){
				min_dist = dist;
				index = i;
			}

		}

		std::cout << previous_map->size() << std::endl;
		std::cout << min_dist << std::endl;
		if(min_dist<1){
			previous_map->points[index].x = 0.9*previous_map->points[index].x + 0.1*new_point.x;
			previous_map->points[index].y = 0.9*previous_map->points[index].y + 0.1*new_point.y;
		} else {
			previous_map->push_back(new_point);
		}

	}

	sensor_msgs::PointCloud2 new_map_msg;

	pcl::toROSMsg(*previous_map, new_map_msg);
	new_map_msg.header.frame_id = "map";
	map_publisher.publish(new_map_msg);

	// pcl::search::KdTree<PointXYZColorScore>::Ptr tree (new pcl::search::KdTree<PointXYZColorScore>);
	// tree->setInputCloud(previous_map);

	// std::vector<pcl::PointIndices> cluster_indices;
	// pcl::EuclideanClusterExtraction<PointXYZColorScore> ec;
	// ec.setClusterTolerance (1.5);
	// ec.setMinClusterSize (3);
	// ec.setMaxClusterSize (100000);
	// ec.setSearchMethod (tree);
	// ec.setInputCloud (previous_map);
	// ec.extract (cluster_indices);

	// pcl::PointCloud<PointXYZColorScore>::Ptr clustered_points (new pcl::PointCloud<PointXYZColorScore>);
	// for(const auto& cluster : cluster_indices) {
	// 	PointXYZColorScore centro;
	// 	centro.x = 0;
	// 	centro.y = 0;
	// 	centro.z = 0;
	// 	centro.color = 0;
	// 	centro.score = 1;
	// 	for (const auto& idx : cluster.indices) {
	// 		centro.x += (*previous_map)[idx].x;
	// 		centro.y += (*previous_map)[idx].y;
	// 		centro.z = 0;

	// 	}
	// 	centro.x /= cluster.indices.size();
	// 	centro.y /= cluster.indices.size();
	// 	for (const auto& idx : cluster.indices) {
	// 		(*previous_map)[idx].x = centro.x;
	// 		(*previous_map)[idx].y = centro.y;
	// 	}

	// 	clustered_points->push_back(centro);

	// }
	// *allp_clustered = *clustered_points;



	// if(callback_iteration % 10 == 9) {
	// 	std::vector<pcl::PointIndices> cluster_indices;
	// 	pcl::EuclideanClusterExtraction<PointXYZColorScore> ec;
	// 	ec.setClusterTolerance (1.5);
	// 	ec.setMinClusterSize (6);
	// 	ec.setMaxClusterSize (200);
	// 	ec.setSearchMethod (tree);
	// 	ec.setInputCloud (previous_map);
	// 	ec.extract (cluster_indices);



	// 	pcl::PointCloud<PointXYZColorScore>::Ptr clustered_points (new pcl::PointCloud<PointXYZColorScore>);
	// 	pcl::PointCloud<PointXYZColorScore>::Ptr mapa_global (new pcl::PointCloud<PointXYZColorScore>);
	// 	for(const auto& cluster : cluster_indices) {
	// 		int j = 0;
	// 		PointXYZColorScore cono;
	// 		cono.x = (*previous_map)[cluster.indices[0]].x;
	// 		cono.y = (*previous_map)[cluster.indices[0]].y;
	// 		cono.z = 0;
	// 		cono.color = 0;
	// 		cono.score = 1;
	// 		for (const auto& idx : cluster.indices) {
    //                             // This is a botch that works but should be changed. PCL ICP doesn't sopport weighted points,
    //                             // so the more reliable we consider a point to be, the more that are placed on the exact same
    //                             // coordinates (limited in this case to 50).
	// 			if(j < 10)
	// 				clustered_points->push_back((*previous_map)[idx]);
	// 			else
	// 				break;
	// 			j++;
	// 		}
	// 		mapa_global->push_back(cono);
	// 	}

	// 	// sensor_msgs::PointCloud2 map_msg;

	// 	// pcl::toROSMsg(*mapa_global, map_msg);
	// 	// map_msg.header.frame_id = "map";
	// 	// map_publisher.publish(map_msg);

	// 	*previous_map = *clustered_points;
	// }



}

void ICP_handle::send_position() {
	geometry_msgs::TransformStamped transformSt;
	transformSt.header.stamp = ros::Time::now();
	transformSt.header.frame_id = "map";
	transformSt.child_frame_id = "body";
	transformSt.transform.translation.x = position.coeff(12);
	transformSt.transform.translation.y = position.coeff(13);
	tf2::Quaternion q;
	float ang = (float)-atan2(position.coeff(0, 1), position.coeff(0,0));
	q.setRPY(0, 0, ang);
	transformSt.transform.rotation.x = q.x();
	transformSt.transform.rotation.y = q.y();
	transformSt.transform.rotation.z = q.z();
	transformSt.transform.rotation.w = q.w();

	br.sendTransform(transformSt);
}
