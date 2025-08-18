#ifndef POINTS_DOWNSAMPLER_H
#define POINTS_DOWNSAMPLER_H

#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

static pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
  pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
  narrowed_scan.header = scan.header;

  if (min_range >= max_range) {
    RCLCPP_ERROR(rclcpp::get_logger("points_downsampler"), "min_range>=max_range @(%lf, %lf)", min_range, max_range);
    return scan;
  }

  double square_min_range = min_range * min_range;
  double square_max_range = max_range * max_range;

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
  {
    const pcl::PointXYZ &p = *iter;
    double square_distance = p.x * p.x + p.y * p.y;
    if (square_min_range <= square_distance && square_distance <= square_max_range) {
      narrowed_scan.points.push_back(p);
    }
  }

  return narrowed_scan;
}

#endif // POINTS_DOWNSAMPLER_H
