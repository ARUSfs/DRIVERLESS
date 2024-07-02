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

void recostruccion(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_f, std::vector<pcl::PointXYZI> punto)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_f);
    float search_radius = 0.15;
    for (size_t idx = 0; idx < punto.size(); ++idx)
    {
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        if (kdtree.radiusSearch(punto[idx], search_radius, point_indices, point_distances) > 0)
        {
            for (size_t i = 0; i < point_indices.size(); ++i)
            {
                int point_index = point_indices[i];
                cloud->emplace_back(cloud_f->points[point_index]);
            }
        }
    }
}

// Definición de pair_hash
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

// Función para procesar los datos del LiDAR y separar en suelo y no suelo
void processLidarData(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                      pcl::PointIndices::Ptr &inliers,
                      pcl::PointIndices::Ptr &outliers,
                      int n_segments)
{
    // Ajustar la longitud de los datos
    cloud->points.resize(cloud->points.size() - (cloud->points.size() % n_segments));

    // Dividir los datos en segmentos
    std::vector<std::vector<pcl::PointXYZI>> separated_data;
    for (int i = 0; i < n_segments; ++i)
    {
        separated_data.push_back(std::vector<pcl::PointXYZI>(cloud->points.begin() + i * (cloud->points.size() / n_segments),
                                                             cloud->points.begin() + (i + 1) * (cloud->points.size() / n_segments)));
    }

    // Procesamiento de los datos por segmento
    std::unordered_map<std::pair<int, int>, float, pair_hash> min_z;
    std::vector<bool> is_ground;
    for (int i = 0; i < n_segments; ++i)
    {
        for (const auto &p : separated_data[i])
        {
            int d = 0;
            if (p.x > -100)
            {
                d = static_cast<int>(std::sqrt(p.x * p.x + p.y * p.y));
            }
            auto key = std::make_pair(i, d);
            if (min_z.find(key) != min_z.end())
            {
                min_z[key] = std::min(p.z, min_z[key]);
                if (p.z < min_z[key] + 0.07)
                {
                    is_ground.push_back(true);
                }
                else
                {
                    is_ground.push_back(false);
                }
            }
            else
            {
                min_z[key] = p.z;
                is_ground.push_back(true);
            }
        }
    }

    // Inicializar los índices para inliers y outliers
    inliers.reset(new pcl::PointIndices);
    outliers.reset(new pcl::PointIndices);

    // Selección de los puntos del suelo y los puntos restantes
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (is_ground[i])
        {
            inliers->indices.push_back(i); // Agregar el índice al conjunto de inliers
        }
        else
        {
            outliers->indices.push_back(i); // Agregar el índice al conjunto de outliers
        }
    }
}

void obtenerPuntosDeClusters(std::vector<pcl::PointXYZI> &puntos, const std::vector<pcl::PointIndices> &chosen_clusters, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
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

void LidarHandle::CarStateCallback(common_msgs::CarState msg)
{
    vx = msg.vx;
    yaw_rate = msg.r;
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

    //ESTO SERIA NECESARIO SI LIDAR NO ESTA CALIBRADO
    /*
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZI> seg1;
    seg1.setOptimizeCoefficients(true);
    seg1.setModelType(pcl::SACMODEL_PLANE);
    seg1.setMethodType(pcl::SAC_RANSAC);
    seg1.setDistanceThreshold(0.01);

    seg1.setInputCloud(cloud);
    seg1.segment(*inliers1, *coefficients1);

    if (inliers1->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }

    // Coeficientes del plano (a, b, c, d) del modelo ax + by + cz + d = 0
    Eigen::Vector3d plane_normal(coefficients1->values[0], coefficients1->values[1], coefficients1->values[2]);
    plane_normal.normalize();

    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d rotation_axis = plane_normal.cross(z_axis);
    double cosTheta = plane_normal.dot(z_axis);
    double theta = std::acos(cosTheta);

    if (rotation_axis.norm() > 1e-6)
    { // Verificar que el eje de rotación no sea cero
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

        *cloud = *transformed_cloud; // Actualizar la nube de puntos original con la transformada
    }
    else
    {
        std::cout << "Plane is already aligned with the z-axis." << std::endl;
    }
    */
   
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

    processLidarData(cloud, inliers, outliers, 12);

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*ground_cloud);

    extract.setNegative(true);
    extract.filter(*cloud_f);

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

    // Creating the KdTree object for the search method of the extraction
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

    //****OPCIONAL****
    std::vector<pcl::PointXYZI> puntos;
    obtenerPuntosDeClusters(puntos, cluster_indices, cloud_f);

    recostruccion(cloud_f, ground_cloud, puntos);

    int i = 0;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZColorScore>::Ptr map_cloud(new pcl::PointCloud<PointXYZColorScore>);

    // common_msgs::Map map;
    for (const auto &cluster : cluster_indices)
    {
        // Crear una nube temporal para el cluster actual
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud_f, cluster, *cluster_cloud);

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
