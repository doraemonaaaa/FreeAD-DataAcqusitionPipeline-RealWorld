#ifndef PCL_TYPE_BASE_H
#define PCL_TYPE_BASE_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    // PCL_ADD_INTENSITY
    float intensity;
    uint16_t ring;
    float timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
(float,x,x)
(float,y,y)
(float,z,z)
(float, intensity, intensity)
(uint16_t,ring,ring)
(float ,timestamp,timestamp)
)

typedef PointXYZIRT PointType;

#endif
