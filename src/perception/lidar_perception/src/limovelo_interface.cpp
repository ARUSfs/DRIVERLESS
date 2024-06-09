#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
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
#include <pcl/common/common.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include "PointXYZColorScore.h"
#include <iomanip>

class PointCloudAccumulator
{
public:
    PointCloudAccumulator() :  tfListener(tfBuffer) // Máxima antigüedad en segundos
    {   
        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setMaxIterations(5000);
        seg_.setDistanceThreshold(0.1);

        // Suscribirse al tópico de la nube de puntos
        sub_ = nh_.subscribe("/limovelo/pcl", 1, &PointCloudAccumulator::pointCloudCallback, this);

        timer = nh_.createTimer(ros::Duration(0.5), &PointCloudAccumulator::timerCallback,this);

        // Publicar la nube de puntos acumulada
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/limovelo_points", 1);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/limovelo_map", 1000);
        // Inicializar la nube de puntos acumulada
        accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    void timerCallback(const ros::TimerEvent&){
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);

        seg_.setInputCloud(accumulated_cloud_);
        seg_.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(accumulated_cloud_);
        extract.setIndices(inliers);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);


        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_f);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.3); // 2cm
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(200);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_f);
        ec.extract(cluster_indices);

        
        pcl::PointCloud<PointXYZColorScore>::Ptr map_cloud(new pcl::PointCloud<PointXYZColorScore>);
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

            if ((max_z - min_z) > 0.1 && (max_z - min_z) < 0.6 && (max_x - min_x) < 0.5 && (max_y - min_y) < 0.5)
            {
                PointXYZColorScore cone;
                cone.x = (max_x + min_x) / 2;
                cone.y = (max_y + min_y) / 2;
                cone.z = (max_z + min_z) / 2;
                cone.color = 0;
                cone.score = 1;
                map_cloud->push_back(cone);
            }
        }

        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*map_cloud, map_msg);
        map_msg.header.frame_id = "map";
        map_pub_.publish(map_msg);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_f, output);
        output.header.frame_id = "map";
        output.header.stamp = ros::Time::now();
        pub_.publish(output);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
    {

        // Convertir el mensaje de nube de puntos a una nube de puntos PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *transformed_cloud);


        // Agregar la nube de puntos actual a la cola de nubes
        for (auto &point : transformed_cloud->points)
        {   
            
            accumulated_cloud_->points.push_back(point);
            
        }
        
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher map_pub_;
    ros::Timer timer;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;
    pcl::SACSegmentation<pcl::PointXYZI> seg_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "limovelo_interface");
    PointCloudAccumulator pca;
    ros::spin();
    return 0;
}
