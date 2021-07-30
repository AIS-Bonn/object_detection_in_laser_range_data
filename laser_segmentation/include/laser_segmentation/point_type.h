/* -*- mode: C++ -*-
 *
 *  WARNING: If you use this header, always include it first.
 *           It defines PCL_NO_PRECOMPILE which has to be defined in order
 *           to use templated pcl functions. If you use the latter, make
 *           sure to include their .hpp files as well.
 */

#ifndef __LASER_SEGMENTATION_POINT_TYPES_H
#define __LASER_SEGMENTATION_POINT_TYPES_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace laser_segmentation
{
/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float intensity;                    ///< laser intensity reading
  uint16_t ring;                      ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

/** Euclidean Velodyne coordinate, including intensity, distance and ring number. */
struct PointXYZIDR
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float intensity;                    ///< laser intensity reading
  float distance;                     ///< distance of point to sensor
  uint16_t ring;                      ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

/** Euclidean Velodyne coordinate, including object segmentation flag and ring number. */
struct PointXYZSegmentation
{
  PCL_ADD_POINT4D;                    ///< quad-word XYZ
  uint8_t segmentation;               ///< segmentation flag
  uint16_t ring;                      ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace laser_segmentation

POINT_CLOUD_REGISTER_POINT_STRUCT(laser_segmentation::PointXYZIR,
                                  (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(laser_segmentation::PointXYZIDR,
                                  (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, distance, distance)
                                    (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(laser_segmentation::PointXYZSegmentation,
                                  (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (uint8_t, segmentation, segmentation)
                                    (uint16_t, ring, ring))

#endif // __LASER_SEGMENTATION_POINT_TYPES_H

