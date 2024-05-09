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

    // nh.getParam("/lidar_perception/inverted", inverted);

    sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 1, &LidarHandle::callback, this);

    map_pub = nh.advertise<common_msgs::Map>(map_topic, 1000);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic, 1000);
    markers_pub = nh.advertise<visualization_msgs::MarkerArray>(cones_marker_topic, 1000);
};

void recostruccion(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_f, std::vector<pcl::PointXYZI> punto)
{
    for (size_t idx = 0; idx < punto.size(); ++idx)
    {
        // Crear el objeto KdTreeFLANN
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(cloud_f);

        // Definir el radio de búsqueda
        float search_radius = 0.30; // Puedes ajustar este valor segun sea necesario

        // Buscar los puntos cercanos al punto medio
        std::vector<int> point_indices;
        std::vector<float> point_distances;

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

void ransacMultiplesmapPlanos(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int numeroDePlanosABuscar)
{
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setInputCloud(cloud);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);

    std::vector<pcl::ModelCoefficients> coefficientsList;
    std::vector<pcl::PointIndices> inliersList;

    // Configurar la búsqueda de planos
    seg.setModelType(pcl::SACMODEL_PLANE);

    // Iterar para buscar múltiples planos
    for (int i = 0; i < numeroDePlanosABuscar; ++i)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        seg.segment(*inliers, *coefficients);

        // Guardar los coeficientes e índices de los puntos inliers
        coefficientsList.push_back(*coefficients);
        inliersList.push_back(*inliers);

        // Remover los inliers encontrados de la nube de puntos para evitar que sean detectados nuevamente
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud);
    }
}

void ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &planoDominante)
{
    // Crear un objeto de segmentación usando RANSAC
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(64);       // Número máximo de iteraciones
    seg.setDistanceThreshold(0.05); // Umbral de distancia para considerar un punto como inlier

    // Establecer la nube de puntos de entrada para la segmentación
    seg.setInputCloud(cloud);

    // Crear un contenedor para los índices de los puntos que pertenecen al plano estimado
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Realizar la segmentación para encontrar el plano dominante
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    // Crear un objeto para extraer los puntos del plano dominante
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);      // Mantener los puntos que pertenecen al plano
    extract.filter(*planoDominante); // Almacenar los puntos del plano dominante

    // Crear una copia de la nube de puntos original
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud, *cloudCopy);

    // Extraer los puntos del plano dominante de la copia de la nube de puntos original
    extract.setNegative(true); // Mantener los puntos que no pertenecen al plano
    extract.filter(*cloud);    // Almacenar los puntos que no pertenecen al plano en la variable cloud
}

struct PolarPoint
{
    float radius;
    float angle;
};

// Función para convertir un punto de coordenadas cartesianas a polares
PolarPoint cartesianToPolar(const pcl::PointXYZI &point)
{
    PolarPoint polar_point;
    polar_point.radius = sqrt(point.x * point.x + point.y * point.y);
    polar_point.angle = atan2(point.y, point.x);
    return polar_point;
}

// Función para encontrar los puntos medios de los conos en un cluster
std::vector<pcl::PointXYZI> findConesMidpoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::vector<pcl::PointIndices> &cluster_indices)
{
    std::vector<pcl::PointXYZI> puntos_medios; // Vector para almacenar los puntos medios
    float minZ = 0.;
    for (const auto &indices : cluster_indices)
    {  
        float max_x = -10.0;
        float min_x = 10.0;
        float max_y = -10.0;
        float min_y = 10.0;
        float max_z = -10.0;
        float min_z = 10.0;
        pcl::PointXYZI p;
        for (const auto& idx : indices.indices) {
            p = (*cloud)[idx];
            max_x = max(p.x,max_x);
            max_y = max(p.y,max_y);
            max_z = max(p.z,max_z);
            min_x = min(p.x,min_x);
            min_y = min(p.y,min_y);
            min_z = min(p.z,min_z);
        }
        if((max_z-min_z)>0.1 && (max_z-min_z)<0.4 && (max_x-min_x)<0.3 && (max_y-min_y)<0.3){

            std::vector<PolarPoint> polar_points;
            for (const auto &index : indices.indices)
            {
                const pcl::PointXYZI &point = cloud->points[index];
                polar_points.push_back(cartesianToPolar(point));

                if (point.z < minZ)
                {
                    minZ = point.z;
                }
            }

            std::sort(polar_points.begin(), polar_points.end(),
                    [](const PolarPoint &p1, const PolarPoint &p2)
                    {
                        return p1.angle > p2.angle;
                    });

            pcl::PointXYZI p1, p2, midpoint;
            p1.x = polar_points[0].radius * cosf(polar_points[0].angle);
            p1.y = polar_points[0].radius * sinf(polar_points[0].angle);
            p1.z = 0.f;
            p1.intensity = 0.f;

            p2.x = polar_points[1].radius * cosf(polar_points[1].angle);
            p2.y = polar_points[1].radius * sinf(polar_points[1].angle);
            p2.z = 0.f;
            p2.intensity = 0.f;

            midpoint.x = (p1.x + p2.x) / 2.;
            midpoint.y = (p1.y + p2.y) / 2.;
            midpoint.z = minZ;
            midpoint.intensity = 0.0;

            puntos_medios.push_back(midpoint); // Agregar el punto medio al vector
        }
    }

    return puntos_medios;
}

struct Punto
{
    double x, y;
};

float scoring(const pcl::PointXYZI cone_pos, const pcl::PointIndices cluster, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float r, float h, float t)
{
    /*
    r = cone_radius
    t = noise of lidar
    h = cone height
    */
    float sum = 0.0;
    float score;

    for (const auto &idx : cluster.indices)
    {
        const pcl::PointXYZI &p = cloud->at(idx);
        pcl::PointXYZI b;
        float distance = 0.0;
        b.x = cone_pos.x;
        b.y = cone_pos.y;
        b.z = -3.5; // Coger el punto mas bajo del cluster para la altura.

        if (p.z < (cone_pos.z + h))
        {
            Punto newP;
            newP.x = sqrt(abs((p.x - b.x) + (p.y - b.y)));
            newP.y = p.z - b.z;

            distance = abs((-(h / r) * newP.x) + (h - newP.y) / sqrt(pow(h / r, 2) + 1));
        }
        else
        {
            pcl::PointXYZI A;
            A.x = b.x;
            A.y = b.y;
            A.z = b.z + h;

            distance = pcl::euclideanDistance(A, p);
        }

        sum += 1.0 - std::min((distance * distance) / (t * t), 1.0f);
    }

    score = sum / cluster.indices.size();

    return score;
}

void filtrarClustersPorDimensiones(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::vector<pcl::PointIndices> &cluster_indices, pcl::PointCloud<pcl::PointXYZI>::Ptr &nube_filtrada, float max_diameter, float max_height)
{
    // Iterar sobre los índices de los clusters
    for (const auto &indices : cluster_indices)
    {
        // Crear una nube temporal para el cluster actual
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, indices, *cluster_cloud);

        // Obtener la caja delimitadora (bounding box) del cluster
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
        float cluster_diameter = max_pt.x - min_pt.x;
        float cluster_height = max_pt.z - min_pt.z;

        // Verificar si el cluster cumple con los requisitos de dimensiones
        if (cluster_diameter <= max_diameter && cluster_height <= max_height)
        {
            // Agregar los puntos del cluster filtrado a la nube filtrada
            *nube_filtrada += *cluster_cloud;
        }
    }
}

void LidarHandle::callback(sensor_msgs::PointCloud2 msg)
{
    ros::Time ini = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg, *cloud);
    pcl::fromROSMsg(msg, *cloud_f);

    double Mx = MAX_X_FOV;
    double My = MAX_Y_FOV;
    double Mz = MAX_Z_FOV;
    double H = H_FOV;

    // Condición de filtro de puntos
    auto condition = [Mx, My, Mz, H](const pcl::PointXYZI &p)
    {
        return !(p.x < Mx && abs(p.y) < My && p.z < Mz && abs(atan2(p.y, p.x)) < H / 2);
    };

    // Aplicamos el filtro
    cloud->erase(std::remove_if(cloud->points.begin(), cloud->points.end(), condition), cloud->points.end());

    // Aplicamos ransac
    ransac(cloud, cloud_f);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.4); // 4cm
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Calculamos punto medio clusters
    std::vector<pcl::PointXYZI> puntoMedio;
    puntoMedio = findConesMidpoint(cloud, cluster_indices);

    ////////////////////////////////
    // Recostruccion              OPCIONAL****
    recostruccion(cloud, cloud_f, puntoMedio);
    ////////////////////////////////

    ////////////////////////////////
    // Filtrar cluster por dimensiones          OPCIONAL****
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    filtrarClustersPorDimensiones(cloud, cluster_indices, cluster_cloud, 2.5f, 3.45f);
    ////////////////////////////////

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all_marker);

    common_msgs::Map map;
    for (size_t i = 0; i < puntoMedio.size(); i++)
    {
        common_msgs::Cone cone;
        cone.position.x = puntoMedio[i].x;
        cone.position.y = puntoMedio[i].y;
        cone.position.z = puntoMedio[i].z;
        cone.color = ' ';
        ////////////////////////////////
        // Enviar scoring del cono               OPCIONAL****
        // cone.confidence = scoring(puntoMedio[i], cluster_indices[i], cloud, 1.14f, 3.25f, 3.0f);
        ////////////////////////////////
        cone.confidence = 0.0;
        map.cones.push_back(cone);

        visualization_msgs::Marker cylinder_marker;
        cylinder_marker.header.frame_id = frame_id;
        cylinder_marker.id = i;
        cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
        cylinder_marker.action = visualization_msgs::Marker::ADD;
        cylinder_marker.pose.position.x = puntoMedio[i].x; 
        cylinder_marker.pose.position.y = puntoMedio[i].y;      
        cylinder_marker.pose.position.z = puntoMedio[i].z;   
        cylinder_marker.scale.x = 0.2;  
        cylinder_marker.scale.y = 0.2;  
        cylinder_marker.scale.z = 0.5;  
        cylinder_marker.color.g = 1.0;
        cylinder_marker.color.a = 1.0;  

        marker_array.markers.push_back(cylinder_marker);
    }
    map_pub.publish(map);
    markers_pub.publish(marker_array);
    // Si se elige Filtrar cluster por dimensiones, enviar como mensaje cluster_cloud


    sensor_msgs::PointCloud2 msg2;
    pcl::toROSMsg(*cloud, msg2);
    msg2.header.frame_id = frame_id;
    filtered_cloud_pub.publish(msg2);

    

    ros::Time fin = ros::Time::now();
    std::cout << (fin - ini) << endl;
}
