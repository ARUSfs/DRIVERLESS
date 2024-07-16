#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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
#include <common_msgs/CarState.h>
#include <pcl/point_cloud.h>
#include "pointRT.h"
#include <unordered_map>
#include <unordered_set>

using namespace std;

#include "lidar_handle.hpp"

LidarHandle::LidarHandle()
{

    nh.getParam("/lidar_perception/lidar_topic", lidar_topic);
    nh.getParam("/lidar_perception/car_state_topic", car_state_topic);
    nh.getParam("/lidar_perception/frame_id", frame_id);
    nh.getParam("/lidar_perception/map_topic", map_topic);
    nh.getParam("/lidar_perception/filtered_cloud_topic", filtered_cloud_topic);
    nh.getParam("/lidar_perception/cones_marker_topic", cones_marker_topic);

    nh.getParam("/lidar_perception/MAX_X_FOV", MAX_X_FOV);
    nh.getParam("/lidar_perception/MAX_Y_FOV", MAX_Y_FOV);
    nh.getParam("/lidar_perception/MAX_Z_FOV", MAX_Z_FOV);
    nh.getParam("/lidar_perception/H_FOV", H_FOV);

    nh.getParam("/lidar_perception/FRAMERATE", FRAMERATE);

    H_FOV = H_FOV * (M_PI / 180);
    nh.getParam("/lidar_perception/inverted", inverted);

    sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1, &LidarHandle::callback, this);
    car_state_sub = nh.subscribe<common_msgs::CarState>(car_state_topic, 1, &LidarHandle::CarStateCallback, this);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1000);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1000);
};
/*
void recostruccion(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground,
                   const std::vector<pcl::PointXYZI> &punto,
                   std::vector<pcl::PointIndices> &chosen_clusters)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(ground);
    float search_radius = 0.15;
    pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices);

    for (int i = chosen_clusters.size() - 1; i >= 0; --i)
    { // Iterar de atrás hacia adelante para poder eliminar elementos
        const auto &cluster = chosen_clusters[i];

        // Create a temporary cloud for the current cluster
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, cluster, *cluster_cloud);

        // Retrieve the bounding box of the cluster
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
            for (const auto &p : punto)
            {
                std::vector<int> point_indices;
                std::vector<float> point_distances;
                if (kdtree.radiusSearch(p, search_radius, point_indices, point_distances) > 0)
                {
                    for (const auto &point_index : point_indices)
                    {
                        cloud->emplace_back(ground->points[point_index]);
                    }
                }
            }
        }

        else
        {
            // Registrar índices para eliminación
            indices_to_remove->indices.insert(indices_to_remove->indices.end(), cluster.indices.begin(), cluster.indices.end());
            // Eliminar el cluster que no cumple las condiciones
            chosen_clusters.erase(chosen_clusters.begin() + i);
        }

    }

    // Crear un filtro para eliminar los puntos marcados
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices_to_remove);
    extract.setNegative(true);
    extract.filter(*cloud);

}
*/
void recostruccion(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground,
                   const std::vector<pcl::PointXYZI> &punto,
                   std::vector<pcl::PointIndices> &chosen_clusters)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(ground);
    float search_radius = 0.15;
    pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices);

    for (int i = chosen_clusters.size() - 1; i >= 0; --i)
    { // Iterar de atrás hacia adelante para poder eliminar elementos
        const auto &cluster = chosen_clusters[i];

        // Create a temporary cloud for the current cluster
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, cluster, *cluster_cloud);

        // Retrieve the bounding box of the cluster
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
            for (const auto &p : punto)
            {
                std::vector<int> point_indices;
                std::vector<float> point_distances;
                if (kdtree.radiusSearch(p, search_radius, point_indices, point_distances) > 0)
                {
                    for (const auto &point_index : point_indices)
                    {
                        cloud->emplace_back(ground->points[point_index]);
                    }
                }
            }
        }
        
        else
            {
                // Eliminar el cluster que no cumple las condiciones
                chosen_clusters.erase(chosen_clusters.begin() + i);
            }
        
    }
}

/*
void reconstruccion2(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground,
                     std::vector<pcl::PointIndices> &chosen_clusters)
{
    // KD-Tree para realizar búsquedas por radio en la nube del suelo (cloud_f)
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(ground);
    float search_radius = 0.15;

    // Vector para almacenar los nuevos clústeres
    std::vector<pcl::PointIndices> new_clusters;

    // Itera sobre cada clúster elegido
    for (const auto &cluster : chosen_clusters)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, cluster, *cluster_cloud);

        // Obtener el cuadro delimitador del clúster
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
        float max_x = max_pt.x;
        float min_x = min_pt.x;
        float max_y = max_pt.y;
        float min_y = min_pt.y;
        float max_z = max_pt.z;
        float min_z = min_pt.z;

        // Comprobar las dimensiones del clúster
        if ((max_z - min_z) < 0.4 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
        {
            // Calcular el punto medio del clúster
            pcl::PointXYZI midpoint;
            midpoint.x = (max_x + min_x) / 2.0;
            midpoint.y = (max_y + min_y) / 2.0;
            midpoint.z = (max_z + min_z) / 2.0;

            std::vector<int> point_indices;
            std::vector<float> point_distances;
            if (kdtree.radiusSearch(midpoint, search_radius, point_indices, point_distances) > 0)
            {
                pcl::PointIndices new_cluster_indices;
                for (size_t i = 0; i < point_indices.size(); ++i)
                {
                    int point_index = point_indices[i];
                    // std::cout << point_index << endl;
                    new_cluster_indices.indices.push_back(point_index);
                }
                // Añadir el nuevo clúster al vector de nuevos clústeres
                new_clusters.push_back(new_cluster_indices);
            }
        }
    }

    // Actualizar el vector de clústeres con los nuevos clústeres
    chosen_clusters = new_clusters;
}
*/

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);
        return hash1 ^ hash2;
    }
};

void filter_segments(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                     pcl::PointIndices::Ptr &inliers,
                     pcl::PointIndices::Ptr &outliers,
                     int n_segments,
                     float threshold)
{
    const float PI = 3.14159265358979323846;
    std::unordered_map<std::pair<int, int>, float, pair_hash> min_z_map;

    for (std::size_t i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZI p = cloud->at(i);

        float angle = atan2(p.y, p.x); // Usar atan2 para obtener el ángulo correcto
        if (angle < 0)
        {
            angle += 2 * PI; // Asegurarse de que el ángulo esté en [0, 2*PI)
        }

        int segment = static_cast<int>(angle / (2 * PI / n_segments));
        int distance = static_cast<int>(std::sqrt(p.x * p.x + p.y * p.y));
        std::pair<int, int> key = {segment, distance};

        if (min_z_map.find(key) == min_z_map.end())
        {
            // Si no existe el segmento, agregarlo y mantener el punto actual
            min_z_map[key] = p.z;
            inliers->indices.push_back(i);
        }
        else
        {
            if (p.z >= min_z_map[key] + threshold)
            {
                // Si el valor de z es mayor que el valor mínimo más el umbral, mantener el punto actual
                outliers->indices.push_back(i);
            }
            else
            {
                // Si el valor de z es menor, actualizar el mínimo z y agregar el punto actual a inliers
                min_z_map[key] = p.z;
                inliers->indices.push_back(i);
            }
        }
    }
}

void obtenerPuntosDeClusters(std::vector<pcl::PointXYZI> &puntos, const std::vector<pcl::PointIndices> &chosen_clusters, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{

    for (const auto &cluster : chosen_clusters)
    {
        if (!cluster.indices.empty())
        {
            pcl::PointXYZI punto = (*cloud)[cluster.indices[0]];
            puntos.push_back(punto);
        }
    }
}

void LidarHandle::CarStateCallback(common_msgs::CarState msg)
{
    vx = msg.vx;
    yaw_rate = msg.r;
}

void LidarHandle::callback(sensor_msgs::PointCloud2 msg)
{
    ros::Time ini = ros::Time::now();

    // We transform the message into a pcl cloud.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *cloud);

    double Mx = MAX_X_FOV;
    double My = MAX_Y_FOV;
    double Mz = MAX_Z_FOV;
    double H = H_FOV;
    // Point filter condition
    auto condition = [Mx, My, Mz, H](const pcl::PointXYZI &p)
    {
        return !(p.x < Mx && abs(p.y) < My && p.z < Mz && abs(atan2(p.y, p.x)) < H / 2 && (abs(p.y) > 0.8 || p.x > 2));
    };

    // Apply filter
    cloud->erase(std::remove_if(cloud->points.begin(), cloud->points.end(), condition), cloud->points.end());

    // THIS WOULD BE NECESSARY IF THE LIDAR IS NOT CALIBRATED
    
    Eigen::Vector3d coefficients1;
    coefficients1 << -0.0433816, 0.00200159, 0.999057; // Reemplaza estos valores con los coeficientes que tienes

    // Coeficientes (a, b, c) del modelo
    Eigen::Vector3d plane_normal(coefficients1[0], coefficients1[1], coefficients1[2]);
    plane_normal.normalize();

    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d rotation_axis = plane_normal.cross(z_axis);
    double cosTheta = plane_normal.dot(z_axis);
    double theta = std::acos(cosTheta);

    if (rotation_axis.norm() > 1e-6)
    {
        rotation_axis.normalize();
        Eigen::AngleAxisd angleAxis(theta, rotation_axis);
        Eigen::Matrix3d rotation_matrix = angleAxis.toRotationMatrix();

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto &point : *cloud)
        {
            Eigen::Vector3d p(point.x, point.y, point.z);
            Eigen::Vector3d p_transformed = rotation_matrix * p;
            pcl::PointXYZI transformed_point;
            transformed_point.x = p_transformed.x();
            transformed_point.y = p_transformed.y();
            transformed_point.z = p_transformed.z();
            transformed_point.intensity = point.intensity;
            transformed_cloud->points.push_back(transformed_point);
        }

        *cloud = *transformed_cloud;
    }
    else
    {
        std::cout << "Plane is already aligned with the z-axis." << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

    filter_segments(cloud, inliers, outliers, 16, 0.06);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(outliers);
    extract.filter(*cloud_f);

    extract.setNegative(true);
    extract.filter(*ground_cloud);
    
    ros::Time fin_r = ros::Time::now();
    ros::Duration t_segment = fin_r - ini;

    sensor_msgs::PointCloud2Iterator<float> iter_timestamp(msg, "time");

    for (size_t i = 0; i < outliers->indices.size(); ++i)
    {
        if (i == 0)
        {
            iter_timestamp += outliers->indices[i];
        }
        else
        {
            iter_timestamp += (outliers->indices[i] - outliers->indices[i - 1]);
        }

        float point_time = *iter_timestamp;

        float time_offset = (FRAMERATE - point_time) + /*0.01*/ +t_segment.toSec();
        cloud_f->points[i].x -= vx * time_offset;
        float yaw = -yaw_rate * time_offset;

        double rotationMatrix[2][2] = {
            {cos(yaw), -sin(yaw)},
            {sin(yaw), cos(yaw)}};

        auto point = cloud_f->points[i];
        float x = point.x;
        float y = point.y;

        cloud_f->points[i].x = rotationMatrix[0][0] * x + rotationMatrix[0][1] * y;
        cloud_f->points[i].y = rotationMatrix[1][0] * x + rotationMatrix[1][1] * y;
    }
    /*
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
    tree1->setInputCloud(cloud_f);

    std::vector<pcl::PointIndices> cluster_indices1;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec1;
    ec1.setClusterTolerance(0.4); // 2cm
    ec1.setMinClusterSize(2);
    ec1.setMaxClusterSize(200);
    ec1.setSearchMethod(tree1);
    ec1.setInputCloud(cloud_f);
    ec1.extract(cluster_indices1);
    */
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_f);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.4); // 2cm
    ec.setMinClusterSize(4);
    ec.setMaxClusterSize(200);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_f);
    ec.extract(cluster_indices);
    //****OPTIONAL****
    std::vector<pcl::PointXYZI> puntos;
    obtenerPuntosDeClusters(puntos, cluster_indices, cloud_f);
    // std::cout << "Antes: " << cluster_indices1.size() << endl;
    recostruccion(cloud_f, ground_cloud, puntos, cluster_indices);
    // std::cout << "Despues: " << cluster_indices1.size() << endl;



    int i = 0;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZColorScore>::Ptr map_cloud(new pcl::PointCloud<PointXYZColorScore>);

    // common_msgs::Map map;
    for (const auto &cluster : cluster_indices)
    {
        // Create a temporary cloud for the current cluster
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud_f, cluster, *cluster_cloud);

        // Retrieve the bounding box of the cluster
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
        float max_x = max_pt.x;
        float min_x = min_pt.x;
        float max_y = max_pt.y;
        float min_y = min_pt.y;
        float max_z = max_pt.z;
        float min_z = min_pt.z;

        if ((max_z - min_z) > 0.1 && (max_x - min_x) < 0.4 && (max_y - min_y) < 0.4)
        {
            //  for (const auto& idx : cluster.indices) {
            //      p = (*cloud)[idx];
            //      p.intensity = i;
            //      cloud_cluster->push_back(p);
            //  }
            PointXYZColorScore cone;
            cone.x = (max_x + min_x) / 2;
            cone.y = (max_y + min_y) / 2;
            cone.z = 0;
            cone.color = 0;
            cone.score = 1;
            map_cloud->push_back(cone);
        }
        i++;
    }
    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(*cloud_f, msg2);
    msg2.header.frame_id = frame_id;
    filtered_cloud_pub.publish(msg2);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud, map_msg);
    map_msg.header.frame_id = frame_id;
    map_pub.publish(map_msg);

    ros::Time fin = ros::Time::now();
    std::cout << (fin - ini) << endl;
}
