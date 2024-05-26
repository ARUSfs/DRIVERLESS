#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tf2_ros/transform_listener.h>


class PurePursuit {
    private:
        pcl::PointCloud<pcl::PointXY> path;
        std::vector<float> distance_along_curve;
        bool path_updated = false;

        const std::string global_frame;
        const std::string car_frame;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;


    public:
        PurePursuit(std::string global_f, std::string car_f)
            : global_frame(global_f)
            , car_frame(car_f)
            , tfListener(tfBuffer) { }

        void update_path(pcl::PointCloud<pcl::PointXY>::Ptr new_path);
        pcl::PointXY search_pursuit_point(float look_ahead_distance);

};
