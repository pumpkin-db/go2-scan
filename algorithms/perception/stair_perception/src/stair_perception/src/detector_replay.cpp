#include "stair_perception/detector_core.h"

#include <pcl/io/pcd_io.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: detector_replay CLOUD.pcd ROBOT_X ROBOT_Y ROBOT_Z\n";
    return 2;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(argv[1], *cloud) != 0) return 3;
  const Eigen::Vector3f robot(std::strtof(argv[2], nullptr), std::strtof(argv[3], nullptr),
                              std::strtof(argv[4], nullptr));
  stair_perception::DetectionDiagnostics diagnostics;
  stair_perception::DetectorCore detector;
  const auto candidates = detector.detect(cloud, robot, &diagnostics);

  std::cout << std::fixed << std::setprecision(3)
            << "points local=" << diagnostics.local_points
            << " filtered=" << diagnostics.filtered_points
            << " near=" << diagnostics.near_ground_points
            << " ground_z=" << diagnostics.ground_z << '\n';
  for (size_t i = 0; i < diagnostics.reject_counts.size(); ++i) {
    const auto reason = static_cast<stair_perception::RejectReason>(i);
    std::cout << "reject " << stair_perception::rejectReasonName(reason)
              << '=' << diagnostics.reject_counts[i] << '\n';
  }
  for (const auto& c : candidates) {
    std::cout << "accept entry=" << c.entry.transpose() << " exit=" << c.exit.transpose()
              << " heading=" << c.heading.transpose()
              << " slope_deg=" << c.slope * 180.0 / 3.14159265358979323846
              << " width=" << c.width << " rise=" << c.rise
              << " support=" << c.support->size() << '\n';
  }
  for (const auto& r : diagnostics.rejected) {
    if (r.reason == stair_perception::RejectReason::SEARCH_SECTOR ||
        r.reason == stair_perception::RejectReason::SLOPE) continue;
    std::cout << "detail reason=" << stair_perception::rejectReasonName(r.reason)
              << " sector=" << r.sector << " attempt=" << r.attempt
              << " yaw=" << r.yaw << " slope=" << r.slope_deg
              << " width=" << r.width << " length=" << r.length
              << " rise=" << r.rise << " gap=" << r.landing_gap
              << " support=" << r.support << '\n';
  }
  return 0;
}
