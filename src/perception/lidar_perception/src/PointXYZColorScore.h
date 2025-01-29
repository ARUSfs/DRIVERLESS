#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>




struct PointXYZColorScore {
    PCL_ADD_POINT4D;  
    int color;
    float score;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructors
    PointXYZColorScore() : color(0), score(0) {}
    PointXYZColorScore(float x, float y, float z, int color, float score) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->color = color;
        this->score = score;
        this->data[3] = 1.0f; 
    }
};


POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZColorScore,       
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int, color, color)
                                   (float, score, score)
)