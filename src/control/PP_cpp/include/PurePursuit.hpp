#include <string>

#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>


class PurePursuit {
    private:
        std::vector<pcl::PointXY> path;
        std::vector<float> distance_along_curve;
        bool path_updated = false;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        pcl::PointXY search_pursuit_point(float look_ahead_distance,
                                          const PointXY car_position);

    public:
        PurePursuit()
            : tfListener(tfBuffer) { }

        void update_path(const std::vector<pcl::PointXY> &new_path);
        float get_steering_angle();

};
