#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "common_msgs/Map.h"
#include "common_msgs/Cone.h"
#include <iostream>

#include <cstdint>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>

using namespace std;

#include "lidar_handle.hpp"

LidarHandle::LidarHandle(){

    nh.getParam("/lidar_perception/lidar_topic",lidar_topic);
    nh.getParam("/lidar_perception/frame_id",frame_id);
    nh.getParam("/lidar_perception/map_topic",map_topic);
    nh.getParam("/lidar_perception/filtered_cloud_topic",filtered_cloud_topic);
    nh.getParam("/lidar_perception/cones_marker_topic",cones_marker_topic);

    nh.getParam("/lidar_perception/MAX_X_FOV",MAX_X_FOV);
    nh.getParam("/lidar_perception/MAX_Y_FOV",MAX_Y_FOV);
    nh.getParam("/lidar_perception/MAX_Z_FOV",MAX_Z_FOV);
    nh.getParam("/lidar_perception/H_FOV",H_FOV);
    H_FOV = H_FOV*(M_PI/180);

    nh.getParam("/lidar_perception/inverted",inverted);
    

    sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1, &LidarHandle::callback, this);

    map_pub = nh.advertise<common_msgs::Map>(map_topic, 1000);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1000);
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>(cones_marker_topic, 1000);
    
};

void LidarHandle::callback(sensor_msgs::PointCloud2 msg){
    ros::Time ini = ros::Time::now();

    //Transformamos el mensaje en una nube de pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *cloud);

    if (inverted) {
        // //Definimos matriz de transformación
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform(1, 1) = -transform(1, 1);  // Invertir el eje Y
        transform(2, 2) = -transform(2, 2);  // Invertir el eje Z

        // // Aplicar la transformación a la nube de puntos
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }
    
    double Mx = MAX_X_FOV;
    double My = MAX_Y_FOV;
    double Mz = MAX_Z_FOV;
    double H = H_FOV;
    //Condición de filtro de puntos
    auto condition = [Mx,My,Mz,H](const pcl::PointXYZI& p) {
        return !(p.x<Mx && abs(p.y)<My && p.z<Mz && abs(atan2(p.y,p.x))<H/2  && (p.x*p.x + p.y*p.y)>4);
    };

    //Aplicamos el filtro
    cloud->erase(std::remove_if(cloud->points.begin(), cloud->points.end(), condition), cloud->points.end());

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (50);
    seg.setDistanceThreshold (0.05);

    // int nr_points = (int) cloud->size ();
    // while (cloud->size () > 0.3 * nr_points){
        // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        // break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud = *cloud_f;
    // }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud (cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (0.5); // 2cm
    ec.setMinClusterSize (3);
    ec.setMaxClusterSize (200);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int i = 0;
    vector<float> X;
    vector<float> Y;
    vector<float> Z;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    common_msgs::Map map;
    for (const auto& cluster : cluster_indices)
    {
        // Crear una nube temporal para el cluster actual
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, cluster, *cluster_cloud);

        // Obtener la caja delimitadora (bounding box) del cluster
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
        float max_x = max_pt.x;
        float min_x = min_pt.x;
        float max_y = max_pt.y;
        float min_y = min_pt.y;
        float max_z = max_pt.z;
        float min_z = min_pt.z;
        
        if((max_z-min_z)>0.05 && (max_z-min_z)<0.4 && (max_x-min_x)<0.4 && (max_y-min_y)<0.4){
            X.push_back((max_x+min_x)/2);
            Y.push_back((max_y+min_y)/2);
            Z.push_back(min_z);
            // for (const auto& idx : cluster.indices) {
            //     p = (*cloud)[idx];
            //     p.intensity = i;
            //     cloud_cluster->push_back(p);
            // }
            common_msgs::Cone cone;
            cone.position.x = (max_x+min_x)/2;
            cone.position.y = (max_y+min_y)/2;
            cone.position.z = min_z;
            cone.color = 'b';
            cone.confidence = 1;
            map.cones.push_back(cone);
        }

        i++;

    }

    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(*cloud,msg2);
        msg2.header.frame_id=frame_id;
        filtered_cloud_pub.publish(msg2);

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all_marker);

    for(int i=0; i<X.size();i++){
        visualization_msgs::Marker cylinder_marker;
        cylinder_marker.header.frame_id = frame_id;
        cylinder_marker.id = i;
        cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
        cylinder_marker.action = visualization_msgs::Marker::ADD;
        cylinder_marker.pose.position.x = X[i];  
        cylinder_marker.pose.position.y = Y[i];      
        cylinder_marker.pose.position.z = Z[i]+0.25;      
        cylinder_marker.scale.x = 0.2;  
        cylinder_marker.scale.y = 0.2;  
        cylinder_marker.scale.z = 0.5;  
        cylinder_marker.color.g = 1.0;
        cylinder_marker.color.a = 1.0;  

        marker_array.markers.push_back(cylinder_marker);
    }

    markers_pub.publish(marker_array);


    map_pub.publish(map);

    ros::Time fin = ros::Time::now();
    std::cout << (fin-ini) << endl;
}








