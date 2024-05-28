#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
#include <sensor_msgs/PointField.h>


using namespace std;

#include "lidar_handle.hpp"

LidarHandle::LidarHandle()
{

    nh.getParam("/new_lidar_perception/lidar_topic", lidar_topic);
    nh.getParam("/new_lidar_perception/frame_id", frame_id);
    nh.getParam("/new_lidar_perception/map_topic", map_topic);
    nh.getParam("/new_lidar_perception/filtered_cloud_topic", filtered_cloud_topic);
    nh.getParam("/new_lidar_perception/cones_marker_topic", cones_marker_topic);

    nh.getParam("/new_lidar_perception/MAX_X_FOV", MAX_X_FOV);
    nh.getParam("/new_lidar_perception/MAX_Y_FOV", MAX_Y_FOV);
    nh.getParam("/new_lidar_perception/MAX_Z_FOV", MAX_Z_FOV);
    nh.getParam("/new_lidar_perception/H_FOV", H_FOV);
    H_FOV = H_FOV * (M_PI / 180);

    nh.getParam("/new_lidar_perception/inverted", inverted);

    sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1, &LidarHandle::callback, this);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1000);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1000);
};



void LidarHandle::callback(sensor_msgs::PointCloud2 cloud_msg)
{
    ros::Time ini = ros::Time::now();

    const double Mx = MAX_X_FOV;
    const double My = MAX_Y_FOV;
    const double Mz = MAX_Z_FOV;
    const double H = H_FOV;

    const size_t offset_x = cloud_msg.fields[0].offset;
    const size_t offset_y = cloud_msg.fields[1].offset;
    const size_t offset_z = cloud_msg.fields[2].offset;
    const size_t offset_i = cloud_msg.fields[3].offset;
    const size_t offset_ring = cloud_msg.fields[4].offset;


    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vector;
    for (int i = 0; i < 32; ++i) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_vector.push_back(cloud);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr colors_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (size_t point_idx = 0; point_idx < cloud_msg.width * cloud_msg.height; ++point_idx) {
        uint8_t* point_data = &cloud_msg.data[point_idx * cloud_msg.point_step];

        float x = *reinterpret_cast<float*>(point_data + offset_x);
        float y = *reinterpret_cast<float*>(point_data + offset_y);
        float z = *reinterpret_cast<float*>(point_data + offset_z);
        float intensity = *reinterpret_cast<float*>(point_data + offset_i);
        uint16_t ring = *reinterpret_cast<uint16_t*>(point_data + offset_ring);

        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = ring;

        cloud_vector[ring]->push_back(p);



    }

    // Condición de filtro de puntos
    auto condition = [Mx, My, Mz, H](const pcl::PointXYZI &p)
    {
        return !(p.x < Mx && abs(p.y) < My && p.z < Mz && abs(atan2(p.y, p.x)) < H / 2 && (p.x * p.x + p.y * p.y) > 1.5);
    };

    int i = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr layer : cloud_vector) {
        // Aplicamos el filtro
        layer->erase(std::remove_if(layer->points.begin(), layer->points.end(), condition), layer->points.end());

        if (layer->width > 0){

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(layer);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(0.5); // 2cm
            ec.setMinClusterSize(3);
            ec.setMaxClusterSize(30);
            ec.setSearchMethod(tree);
            ec.setInputCloud(layer);
            ec.extract(cluster_indices);

            for (const auto &cluster : cluster_indices){
                // Crear una nube temporal para el cluster actual
                pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::copyPointCloud(*layer, cluster, *cluster_cloud);

                for (const auto& idx : cluster.indices) {
                    pcl::PointXYZI p = (*layer)[idx];
                    p.intensity = i;
                    final_cloud->push_back(p);
                }
                i++;
            }
        }

    }



    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(final_cloud);

    std::vector<pcl::PointIndices> final_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.5); // 2cm
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(200);
    ec.setSearchMethod(tree);
    ec.setInputCloud(final_cloud);
    ec.extract(final_indices);


    int j = 0;
    pcl::PointCloud<PointXYZColorScore>::Ptr map_cloud (new pcl::PointCloud<PointXYZColorScore>);
    
    // common_msgs::Map map;
    for (const auto &cluster : final_indices)
    {
        // Crear una nube temporal para el cluster actual
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*final_cloud, cluster, *cluster_cloud);

        // Obtener la caja delimitadora (bounding box) del cluster
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
        float max_x = max_pt.x;
        float min_x = min_pt.x;
        float max_y = max_pt.y;
        float min_y = min_pt.y;
        float max_z = max_pt.z;
        float min_z = min_pt.z;

        if ((max_z - min_z) < 0.4 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
        {
            // for (const auto& idx : cluster.indices) {
            //     p = (*cloud)[idx];
            //     p.intensity = i;
            //     cloud_cluster->push_back(p);
            // }
            PointXYZColorScore cone;
            cone.x = (max_x+min_x)/2;
            cone.y = (max_y+min_y)/2;
            cone.z = 0;
            cone.color = 0;
            cone.score = 1;
            map_cloud->push_back(cone);
        }

        j++;
    }
    



    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(*final_cloud, msg2);
    msg2.header.frame_id = frame_id;
    filtered_cloud_pub.publish(msg2);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud,map_msg);
    map_msg.header.frame_id=frame_id;
    map_pub.publish(map_msg);

    ros::Time fin = ros::Time::now();
    std::cout << (fin - ini) << endl;


}
