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

using namespace std;

#include "lidar_handle.hpp"

LidarHandle::LidarHandle()
{

    nh.getParam("/lidar_perception/lidar_topic", lidar_topic);
    nh.getParam("/lidar_perception/frame_id", frame_id);
    nh.getParam("/lidar_perception/map_topic", map_topic);
    nh.getParam("/lidar_perception/filtered_cloud_topic", filtered_cloud_topic);
    nh.getParam("/lidar_perception/cones_marker_topic", cones_marker_topic);

    nh.getParam("/lidar_perception/MAX_X_FOV", MAX_X_FOV);
    nh.getParam("/lidar_perception/MAX_Y_FOV", MAX_Y_FOV);
    nh.getParam("/lidar_perception/MAX_Z_FOV", MAX_Z_FOV);
    nh.getParam("/lidar_perception/H_FOV", H_FOV);
    H_FOV = H_FOV * (M_PI / 180);

    nh.getParam("/lidar_perception/inverted", inverted);

    sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1, &LidarHandle::callback, this);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1000);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1000);
};

void recostruccion(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_f, std::vector<pcl::PointXYZI> punto)
{
    // Crear el objeto KdTreeFLANN una sola vez fuera del bucle
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_f);

    // Definir el radio de búsqueda
    float search_radius = 0.15; // Puedes ajustar este valor segun sea necesario

    // Desenrollar el bucle externo
    for (size_t idx = 0; idx < punto.size(); ++idx) // Incrementar en 2 en cada iteración
    {
        // Buscar los puntos cercanos a los puntos medio y siguiente
        std::vector<int> point_indices;
        std::vector<float> point_distances;

        // Verificar si se encuentran puntos dentro del radio de búsqueda alrededor de los puntos medio y siguiente
        if (kdtree.radiusSearch(punto[idx], search_radius, point_indices, point_distances) > 0)
        {
            // Iterar sobre los índices de los puntos encontrados
            for (size_t i = 0; i < point_indices.size(); ++i)
            {
                int point_index = point_indices[i];
                cloud->emplace_back(cloud_f->points[point_index]);
            }
        }
    }
}

void obtenerPuntosDeClusters(std::vector<pcl::PointXYZI>& puntos, const std::vector<pcl::PointIndices> &chosen_clusters, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{

    for (const auto &cluster : chosen_clusters)
    {
        // Tomar el primer punto del cluster y agregarlo al vector de puntos
        if (!cluster.indices.empty())
        {
            pcl::PointXYZI punto = (*cloud)[cluster.indices[0]];
            puntos.push_back(punto);
        }
    }
}

void LidarHandle::callback(sensor_msgs::PointCloud2 msg)
{
    ros::Time ini = ros::Time::now();

    // Transformamos el mensaje en una nube de pcl
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *cloud);

    double Mx = MAX_X_FOV;
    double My = MAX_Y_FOV;
    double Mz = MAX_Z_FOV;
    double H = H_FOV;
    // Condición de filtro de puntos
    auto condition = [Mx, My, Mz, H](const pcl::PointXYZI &p)
    {
        return !(p.x < Mx && abs(p.y) < My && p.z < Mz && abs(atan2(p.y, p.x)) < H / 2 && (abs(p.y) > 0.8 || p.x > 2));
    };

    // Aplicamos el filtro
    cloud->erase(std::remove_if(cloud->points.begin(), cloud->points.end(), condition), cloud->points.end());

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(50);
    seg.setDistanceThreshold(0.05);

    // int nr_points = (int) cloud->size ();
    // while (cloud->size () > 0.3 * nr_points){
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        // break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud = *cloud_f;
    // }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.4); // 2cm
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(200);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    //****OPCIONAL****
    std::vector<pcl::PointXYZI> puntos;
    obtenerPuntosDeClusters(puntos, cluster_indices, cloud);
    
    recostruccion(cloud, cloud_plane, puntos);


    int i = 0;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZColorScore>::Ptr map_cloud (new pcl::PointCloud<PointXYZColorScore>);
    
    // common_msgs::Map map;
    for (const auto &cluster : cluster_indices)
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

        if ((max_z - min_z) > 0.1 && (max_z - min_z) < 0.4 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
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

        i++;
    }
    
    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(*cloud, msg2);
    msg2.header.frame_id = frame_id;
    filtered_cloud_pub.publish(msg2);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud,map_msg);
    map_msg.header.frame_id=frame_id;
    map_pub.publish(map_msg);

    ros::Time fin = ros::Time::now();
    std::cout << (fin - ini) << endl;
}
