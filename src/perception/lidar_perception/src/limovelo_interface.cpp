#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/point_cloud_conversion.h>
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
#include <std_msgs/Float32MultiArray.h>


#include <sensor_msgs/PointCloud2.h>

#include "PointXYZColorScore.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>

#include <iomanip>

using namespace Eigen;

class PointCloudAccumulator
{
public:
    PointCloudAccumulator()
    {   
        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setMaxIterations(5000);
        seg_.setDistanceThreshold(0.1);

        // Inicializar la nube de puntos acumulada
        accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        R = Matrix3d::Identity();

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/limovelo_points", 1);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/limovelo_map", 1);
        rot_pub = nh_.advertise<std_msgs::Float32MultiArray>("/limovelo/rotation", 1);

        sub_ = nh_.subscribe("/limovelo/pcl", 1, &PointCloudAccumulator::pointCloudCallback, this);

        timer = nh_.createTimer(ros::Duration(0.5), &PointCloudAccumulator::timerCallback,this);

    }

    void timerCallback(const ros::TimerEvent&){
        if (accumulated_cloud_->points.size() == 0) {
            return;
        }
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);

        seg_.setInputCloud(accumulated_cloud_);
        seg_.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        } else {
            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud(accumulated_cloud_);
            extract.setIndices(inliers);

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);

            // Get rotation matrix from floor coefficients
            Vector3d v(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            Vector3d vNorm = v / v.norm();
            Vector3d wNorm(0, 0, 1);
            Vector3d k = vNorm.cross(wNorm);
            double cosTheta = vNorm.dot(wNorm);
            double theta = std::acos(cosTheta);
            AngleAxisd angleAxis(theta, k.normalized());
            R = angleAxis.toRotationMatrix();
        }

        
        

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_f);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.3);
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
                Vector3d p((max_x + min_x)/2, (max_y + min_y)/2, (max_z + min_z)/2);
                Vector3d pRotated = R * p;
                PointXYZColorScore cone;
                cone.x = pRotated[0];
                cone.y = - pRotated[1];
                cone.z = - pRotated[2];
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

   
        std_msgs::Float32MultiArray matrix_msg;
        matrix_msg.data.resize(9);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                matrix_msg.data[i * 3 + j] = R(i, j);
            }
        }
        rot_pub.publish(matrix_msg);

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
    ros::Publisher rot_pub;
    ros::Timer timer;

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;
    pcl::SACSegmentation<pcl::PointXYZI> seg_;
    Matrix3d R;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "limovelo_interface");
    PointCloudAccumulator pca;
    ros::spin();
    return 0;
}
