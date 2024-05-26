#include "PurePursuit.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
using namespace pcl;

void PurePursuit::update_path(PointCloud<PointXY>::Ptr new_path) {
    // Checking if path is empty since Delaunay sometimes returns empty paths
    // In this case just ignore it, the last path should be enough.
    if(new_path->size() == 0) {
        return;
    }

    path = *new_path;
    path_updated = true;
}

PointXY search_pursuit_point(float look_ahead_distance) {
}
