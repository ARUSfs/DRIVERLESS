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
#include <algorithm>
#include <common_msgs/CarState.h>
#include <std_msgs/Int16.h>


#include <pcl/common/impl/centroid.hpp>
#include "icp_slam.hpp"


using namespace std;

ICP_handle::ICP_handle(){
	prev_t = ros::Time::now();	
	lap_time = ros::Time::now();

	previous_map = pcl::PointCloud<PointXYZColorScore>::Ptr(new pcl::PointCloud<PointXYZColorScore>);
	allp_clustered = pcl::PointCloud<PointXYZColorScore>::Ptr(new pcl::PointCloud<PointXYZColorScore>);

	position = Eigen::Matrix4f::Identity(4, 4);
	prev_transformation = Eigen::Matrix4f::Identity(4, 4);

	perception_sub = nh.subscribe<sensor_msgs::PointCloud2>("/perception_map", 1000, &ICP_handle::map_callback, this);
	state_sub = nh.subscribe<common_msgs::CarState>("/car_state/state", 1000, &ICP_handle::state_callback, this);


	map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/mapa_icp", 10);
	lap_count_publisher = nh.advertise<std_msgs::Int16>("/lap_counter", 10);

}

void ICP_handle::state_callback(common_msgs::CarState state_msg) {
	vx = state_msg.vx;
    yaw_rate = state_msg.r;
}

void ICP_handle::map_callback(sensor_msgs::PointCloud2 map_msg) {

	pcl::PointCloud<PointXYZColorScore>::Ptr new_map(new pcl::PointCloud<PointXYZColorScore>);
	pcl::fromROSMsg(map_msg, *new_map);

	if(!has_map){
		prev_t = ros::Time::now();
		lap_time = ros::Time::now();
		*allp_clustered = *new_map;
		*previous_map = *new_map;
		has_map = true;

		send_position();
		return;
	}

	pcl::PointCloud<PointXYZColorScore>::Ptr map_in_position = pcl::PointCloud<PointXYZColorScore>::Ptr(new pcl::PointCloud<PointXYZColorScore>);
	pcl::transformPointCloud(*new_map, *map_in_position, position);


	pcl::IterativeClosestPoint<PointXYZColorScore, PointXYZColorScore> icp;
	icp.setInputSource(map_in_position);
	if(callback_iteration < 10)
		icp.setInputTarget(previous_map);
	else
		icp.setInputTarget(allp_clustered);
	icp.setMaximumIterations(50);
	icp.setEuclideanFitnessEpsilon(0.005);
	icp.setTransformationEpsilon(1e-5);

	pcl::PointCloud<PointXYZColorScore> registered_map;
	icp.setMaxCorrespondenceDistance (1.0);
	icp.align(registered_map);
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	
	// *previous_map += registered_map;


	float tx = transformation.coeff(0,0)*position.coeff(0,3) + transformation.coeff(0,1)*position.coeff(1,3) + transformation.coeff(0,3) - position.coeff(0,3);
	float ty = transformation.coeff(1,0)*position.coeff(0,3) + transformation.coeff(1,1)*position.coeff(1,3) + transformation.coeff(1,3) - position.coeff(1,3);
	float dist = sqrt(tx*tx + ty*ty);
	// std::cout << dist << std::endl;


	//calcular estimacion
	float dt = ros::Time::now().toSec()-prev_t.toSec();
	float dx = dt*vx;
	float dyaw = dt*yaw_rate; 

	float yaw = (float)-atan2(position.coeff(0, 1), position.coeff(0,0));
	Eigen::Matrix4f estimation = Eigen::Matrix4f::Identity();
	estimation(0,3)=dx*cos(yaw+dyaw)+position(0,3)-cos(dyaw)*position(0,3)+sin(dyaw)*position(1,3);
	estimation(1,3)=dx*sin(yaw+dyaw)+position(1,3)-sin(dyaw)*position(0,3)-cos(dyaw)*position(1,3);
	estimation(0,0)=cos(dyaw);
	estimation(1,0)=sin(dyaw);
	estimation(0,1)=-sin(dyaw);
	estimation(1,1)=cos(dyaw);

	//sigmoide para ponderar icp y estimacion
	float tyaw = (float)-atan2(transformation.coeff(0, 1), transformation.coeff(0,0)); 
	float w;
	if(dist == 0)
		w = 1;
	else
		w = -0.5 + 1.0/(1.0 + std::exp(-(3*std::abs(dist-dx)+3*std::abs(tyaw-dyaw))));
	std::cout << w << std::endl;
	prev_transformation = (estimation*w + transformation*(1-w));
	position = prev_transformation*position;
	pcl::PointCloud<PointXYZColorScore> m;
	pcl::transformPointCloud(*new_map, m, position);
	*previous_map += m;

	// pcl::PointCloud<PointXYZColorScore>::Ptr map_in_position = pcl::PointCloud<PointXYZColorScore>::Ptr(new pcl::PointCloud<PointXYZColorScore>);
	// pcl::transformPointCloud(*new_map, *map_in_position, estimation*position);

	// pcl::IterativeClosestPoint<PointXYZColorScore, PointXYZColorScore> icp;
	// icp.setInputSource(map_in_position);
	// if(callback_iteration < 10)
	// 	icp.setInputTarget(previous_map);
	// else
	// 	icp.setInputTarget(allp_clustered);
	// icp.setMaximumIterations(50);
	// icp.setEuclideanFitnessEpsilon(0.005);
	// icp.setTransformationEpsilon(1e-5);

	// pcl::PointCloud<PointXYZColorScore> registered_map;
	// icp.setMaxCorrespondenceDistance (1.0);
	// icp.align(registered_map);
	// Eigen::Matrix4f transformation = icp.getFinalTransformation();

	// *previous_map += registered_map;
	
	// position = transformation*position;

	prev_t = ros::Time::now();
	send_position();



	pcl::search::KdTree<PointXYZColorScore>::Ptr tree (new pcl::search::KdTree<PointXYZColorScore>);
	tree->setInputCloud(previous_map);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointXYZColorScore> ec;
	ec.setClusterTolerance (1.5);
	ec.setMinClusterSize (3);
	ec.setMaxClusterSize (100000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (previous_map);
	ec.extract (cluster_indices);

	pcl::PointCloud<PointXYZColorScore>::Ptr clustered_points (new pcl::PointCloud<PointXYZColorScore>);
	for(const auto& cluster : cluster_indices) {
		PointXYZColorScore centro;
		centro.x = 0;
		centro.y = 0;
		centro.z = 0;
		centro.color = 0;
		centro.score = 1;
		for (const auto& idx : cluster.indices) {
			centro.x += (*previous_map)[idx].x;
			centro.y += (*previous_map)[idx].y;
			centro.z = 0;

		}
		centro.x /= cluster.indices.size();
		centro.y /= cluster.indices.size();
		for (const auto& idx : cluster.indices) {
			(*previous_map)[idx].x = centro.x;
			(*previous_map)[idx].y = centro.y;
		}

		clustered_points->push_back(centro);

	}
	*allp_clustered = *clustered_points;



	if(callback_iteration % 10 == 9) {
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<PointXYZColorScore> ec;
		ec.setClusterTolerance (1.5);
		ec.setMinClusterSize (6);
		ec.setMaxClusterSize (200);
		ec.setSearchMethod (tree);
		ec.setInputCloud (previous_map);
		ec.extract (cluster_indices);



		pcl::PointCloud<PointXYZColorScore>::Ptr clustered_points (new pcl::PointCloud<PointXYZColorScore>);
		pcl::PointCloud<PointXYZColorScore>::Ptr mapa_global (new pcl::PointCloud<PointXYZColorScore>);
		for(const auto& cluster : cluster_indices) {
			int j = 0;
			PointXYZColorScore cono;
			cono.x = (*previous_map)[cluster.indices[0]].x;
			cono.y = (*previous_map)[cluster.indices[0]].y;
			cono.z = 0;
			cono.color = 0;
			cono.score = 1;
			for (const auto& idx : cluster.indices) {
                                // This is a botch that works but should be changed. PCL ICP doesn't sopport weighted points,
                                // so the more reliable we consider a point to be, the more that are placed on the exact same
                                // coordinates (limited in this case to 50).
				if(j < 10)
					clustered_points->push_back((*previous_map)[idx]);
				else
					break;
				j++;
			}
			mapa_global->push_back(cono);
		}

		// sensor_msgs::PointCloud2 map_msg;

		// pcl::toROSMsg(*mapa_global, map_msg);
		// map_msg.header.frame_id = "map";
		// map_publisher.publish(map_msg);

		*previous_map = *clustered_points;
	}

	sensor_msgs::PointCloud2 new_map_msg;

	pcl::toROSMsg(*allp_clustered, new_map_msg);
	new_map_msg.header.frame_id = "map";
	map_publisher.publish(new_map_msg);

	callback_iteration++;

}

void ICP_handle::send_position() {
	geometry_msgs::TransformStamped transformSt;
	transformSt.header.stamp = ros::Time::now();
	transformSt.header.frame_id = "map";
	transformSt.child_frame_id = "body";
	transformSt.transform.translation.x = position.coeff(0,3);
	transformSt.transform.translation.y = position.coeff(1,3);
	tf2::Quaternion q;
	float ang = (float)-atan2(position.coeff(0, 1), position.coeff(0,0));
	q.setRPY(0, 0, ang);
	transformSt.transform.rotation.x = q.x();
	transformSt.transform.rotation.y = q.y();
	transformSt.transform.rotation.z = q.z();
	transformSt.transform.rotation.w = q.w();

	br.sendTransform(transformSt);

	if (position.coeff(0,3)*position.coeff(0,3)+position.coeff(1,3)*position.coeff(1,3) < 4){
		if(ros::Time::now().toSec()-lap_time.toSec() > 20){
			lap_count += 1;
			lap_time = ros::Time::now();
			std::cout << "Lap count: " << lap_count << std::endl;
		}else{
			lap_time = ros::Time::now();
		}
	} 

	std_msgs::Int16 lap_count_msg;
	lap_count_msg.data = lap_count;
	lap_count_publisher.publish(lap_count_msg);
	
}
