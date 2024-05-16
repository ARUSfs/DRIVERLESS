#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "common_msgs/Map.h"
#include "common_msgs/Cone.h"
 #include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>


#include <pcl/common/impl/centroid.hpp>
#include "icp_slam.hpp"


using namespace std;

ICP_handle::ICP_handle(){
	previous_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	allp_clustered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

	position = Eigen::Matrix4f::Identity(4, 4);

	sub = nh.subscribe<common_msgs::Map>("/perception_map", 1000, &ICP_handle::map_callback, this);

	trans_pub = nh.advertise<common_msgs::Map>("/mapa_icp", 10);

	pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/nube", 10);
}

void ICP_handle::map_callback(common_msgs::Map map) {
	i++;
	intensity *= 1;

	int n_cones = map.cones.size();
	pcl::PointCloud<pcl::PointXYZI>::Ptr new_map(new pcl::PointCloud<pcl::PointXYZI>(n_cones, 1));

	for(int i = 0; i < n_cones; i++){
		new_map->points[i].x = map.cones[i].position.x;
		new_map->points[i].y = map.cones[i].position.y;
		new_map->points[i].intensity = intensity;
	}

	if(!has_map){
		*allp_clustered = *new_map;
		*previous_map = *new_map;
		has_map = true;

		send_position();
		return;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr map_in_position = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*new_map, *map_in_position, position);


	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	icp.setInputSource(map_in_position);
	icp.setInputTarget(previous_map);
	icp.setMaximumIterations(9000);
	//icp.setEuclideanFitnessEpsilon (0.05);
	icp.setTransformationEpsilon(1e-5);

	pcl::PointCloud<pcl::PointXYZI> registered_map;
	icp.setMaxCorrespondenceDistance (1.0);
	icp.align(registered_map);

	Eigen::Matrix4f transformation = icp.getFinalTransformation();

	float ang = (float)-atan2(transformation.coeff(0, 1), transformation.coeff(0,0));
	cout << transformation.coeff(12) << ", " << transformation.coeff(13) << ", " << ang << endl;
	if(ang < -0.15 || ang > 0.15)
		return;

	position = transformation * position;
	send_position();

	*previous_map += registered_map;


	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud(previous_map);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance (1.5);
	ec.setMinClusterSize (3);
	ec.setMaxClusterSize (100000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (previous_map);
	ec.extract (cluster_indices);

	pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_points (new pcl::PointCloud<pcl::PointXYZI>);
	for(const auto& cluster : cluster_indices) {
		//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
		//bool exists = false;
		pcl::PointXYZI centro;
		centro.x = 0;
		centro.y = 0;
		centro.z = 0;
		centro.intensity = 0.0f;
		float sum_int = 0;
		for (const auto& idx : cluster.indices) {
			/*if((*previous_map)[idx].intensity == 1){
				exists = true;
				clustered_points->push_back((*previous_map)[idx]);
				break;
			}*/
			//cloud_cluster->push_back((*previous_map)[idx]);
			sum_int += (*previous_map)[idx].intensity;
			centro.x += (*previous_map)[idx].x*(*previous_map)[idx].intensity;
			centro.y += (*previous_map)[idx].y*(*previous_map)[idx].intensity;
			centro.z = 0;

		}
		centro.x /= sum_int;
		centro.y /= sum_int;
		for (const auto& idx : cluster.indices) {
			(*previous_map)[idx].x = centro.x;
			(*previous_map)[idx].y = centro.y;
		}

		clustered_points->push_back(centro);

	}
	*allp_clustered = *clustered_points;

	sensor_msgs::PointCloud2 output;

	pcl::toROSMsg(*previous_map, output);
	output.header.frame_id = "map";
	pcl_pub.publish(output);



	if(i % 10 == 9) {
		cout << "holaa" << endl;

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
		ec.setClusterTolerance (1.5);
		ec.setMinClusterSize (6);
		ec.setMaxClusterSize (200);
		ec.setSearchMethod (tree);
		ec.setInputCloud (previous_map);
		ec.extract (cluster_indices);

		pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_points (new pcl::PointCloud<pcl::PointXYZI>);

		common_msgs::Map mapa_global;
		mapa_global.header.frame_id = "map";
		mapa_global.header.stamp = ros::Time::now();
		for(const auto& cluster : cluster_indices) {
			int j = 0;
			common_msgs::Cone cono;
			cono.position.x = (*previous_map)[cluster.indices[0]].x;
			cono.position.y = (*previous_map)[cluster.indices[0]].y;
			cono.color = "b";
			cono.confidence = 1;
			for (const auto& idx : cluster.indices) {
				if(j < 50)
					clustered_points->push_back((*previous_map)[idx]);
				else
					break;
				j++;
			}
			mapa_global.cones.push_back(cono);

		}
		trans_pub.publish(mapa_global);
		*previous_map = *clustered_points;


	}

}

void ICP_handle::send_position() {
	geometry_msgs::TransformStamped transformSt;
	transformSt.header.stamp = ros::Time::now();
	transformSt.header.frame_id = "map";
	transformSt.child_frame_id = "velodyne";

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
