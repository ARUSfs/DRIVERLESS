#include <string>

#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>


class PurePursuit {
    private:

        std::vector<float> distance_along_curve;
        bool path_updated = false;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener{tfBuffer};

        pcl::PointXY search_pursuit_point(float look_ahead_distance,
                                          const pcl::PointXY car_position);

    public:
        std::vector<pcl::PointXY> path;
        size_t pursuit_index;
        PurePursuit();
            // : tfListener(tfBuffer) { }

        void update_path(const std::vector<pcl::PointXY> &new_path);
        float get_steering_angle();

};
