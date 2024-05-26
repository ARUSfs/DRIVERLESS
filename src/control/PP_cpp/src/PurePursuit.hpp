#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class PurePursuit {
    private:
        pcl::PointCloud<pcl::PointXY> path;
        bool path_updated = false;

        std::string global_frame;
        std::string car_frame;


    public:
        void update_path(pcl::PointCloud<pcl::PointXY>::Ptr new_path);
        pcl::PointXY search_pursuit_point(float look_ahead_distance);

};
