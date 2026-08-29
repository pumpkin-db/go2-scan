#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace stair_perception {

struct DetectorParams {
  double voxel_size{0.08};
  double min_range{0.8};
  double max_range{7.0};
  double min_slope_deg{15.0};
  double max_slope_deg{45.0};
  double min_width{0.55};
  double max_width{3.0};
  double min_length{0.8};
  double min_rise{0.35};
  double corridor_half_width{0.8};
  double ransac_distance{0.12};
  int min_support{30};
  int sector_count{36};
};

struct Candidate {
  Eigen::Vector3f entry{Eigen::Vector3f::Zero()};
  Eigen::Vector3f exit{Eigen::Vector3f::Zero()};
  Eigen::Vector3f heading{Eigen::Vector3f::UnitX()};
  float slope{0.0F};
  float width{0.0F};
  float rise{0.0F};
  float confidence{0.0F};
  pcl::PointCloud<pcl::PointXYZ>::Ptr support{new pcl::PointCloud<pcl::PointXYZ>};
};

class DetectorCore {
 public:
  explicit DetectorCore(DetectorParams params = DetectorParams());
  std::vector<Candidate> detect(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                const Eigen::Vector3f& robot_position) const;

 private:
  DetectorParams params_;
};

}  // namespace stair_perception
