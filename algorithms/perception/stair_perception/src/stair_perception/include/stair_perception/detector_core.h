#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
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
  double max_landing_height_gap{0.9};
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

enum class RejectReason {
  SEARCH_SECTOR,
  GROUND_CONNECTION,
  RANSAC,
  SLOPE,
  WIDTH,
  LENGTH,
  RISE,
  SUPPORT,
  DUPLICATE,
  COUNT
};

const char* rejectReasonName(RejectReason reason);

struct RejectedCandidate {
  RejectReason reason{RejectReason::RANSAC};
  int sector{-1};
  int attempt{-1};
  float yaw{0.0F};
  float slope_deg{0.0F};
  float width{0.0F};
  float length{0.0F};
  float rise{0.0F};
  float landing_gap{0.0F};
  int support{0};
};

struct DetectionDiagnostics {
  size_t local_points{0};
  size_t filtered_points{0};
  size_t near_ground_points{0};
  float ground_z{0.0F};
  bool ground_valid{false};
  std::array<size_t, static_cast<size_t>(RejectReason::COUNT)> reject_counts{};
  std::vector<RejectedCandidate> rejected;
};

class DetectorCore {
 public:
  explicit DetectorCore(DetectorParams params = DetectorParams());
  std::vector<Candidate> detect(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                const Eigen::Vector3f& robot_position,
                                DetectionDiagnostics* diagnostics = nullptr) const;

 private:
  DetectorParams params_;
};

}  // namespace stair_perception
