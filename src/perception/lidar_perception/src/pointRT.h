#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

struct pointRT
{
    PCL_ADD_POINT4D; // Macro para definir los campos x, y, z, y padding
    float intensity; // Intensidad del punto
    float time;      // Tiempo de captura del punto
    uint16_t ring;   // Índice del anillo del LiDAR

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Macro para la alineación adecuada en Eigen

    pointRT() : x(0), y(0), z(0), intensity(0), time(0), ring(0)
    {
    }
    pointRT(float x, float y, float z, float intensity, float time, uint16_t ring)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->intensity = intensity;
        this->time = time;
        this->ring = ring;
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(pointRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, time, time)
                                  (std::uint16_t, ring, ring))