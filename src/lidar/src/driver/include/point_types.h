#ifndef INCLUDE_POINT_TYPES_H_
#define INCLUDE_POINT_TYPES_H_

#include <pcl/point_types.h>

struct PointXYZIT {
  PCL_ADD_POINT4D   //添加pcl里xyz
  float intensity;
  double timestamp;
  uint16_t ring;                   ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned,确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16;                   // 强制SSE填充以正确对齐内存

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

typedef PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;

#endif  // INCLUDE_POINT_TYPES_H_
