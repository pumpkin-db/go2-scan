#include "stair_perception/detector_core.h"

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace stair_perception {
namespace {

constexpr double kPi = 3.14159265358979323846;

float angleBetween(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
  const float dot = std::max(-1.0F, std::min(1.0F, a.normalized().dot(b.normalized())));
  return std::acos(dot);
}

float planeHeight(const Eigen::Vector4f& plane, float x, float y) {
  return -(plane.x() * x + plane.y() * y + plane.w()) / plane.z();
}

}  // namespace

DetectorCore::DetectorCore(DetectorParams params) : params_(std::move(params)) {}

std::vector<Candidate> DetectorCore::detect(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    const Eigen::Vector3f& robot_position) const {
  std::vector<Candidate> candidates;
  if (!cloud || cloud->empty()) return candidates;

  pcl::PointCloud<pcl::PointXYZ>::Ptr local(new pcl::PointCloud<pcl::PointXYZ>);
  local->reserve(cloud->size());
  for (const auto& p : cloud->points) {
    if (!pcl::isFinite(p)) continue;
    const float dx = p.x - robot_position.x();
    const float dy = p.y - robot_position.y();
    const float r = std::hypot(dx, dy);
    if (r < params_.min_range || r > params_.max_range) continue;
    if (p.z < robot_position.z() - 0.8 || p.z > robot_position.z() + 3.2) continue;
    local->push_back(p);
  }
  if (local->size() < static_cast<size_t>(params_.min_support)) return candidates;

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(local);
  voxel.setLeafSize(params_.voxel_size, params_.voxel_size, params_.voxel_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  voxel.filter(*filtered);

  // Robust local ground reference: lower quartile near the robot. The actual stair
  // classification below still uses a RANSAC plane and gravity-aligned slope.
  std::vector<float> near_heights;
  for (const auto& p : filtered->points) {
    if (std::hypot(p.x - robot_position.x(), p.y - robot_position.y()) < 2.0)
      near_heights.push_back(p.z);
  }
  if (near_heights.size() < 10) return candidates;
  const size_t q = near_heights.size() / 4;
  std::nth_element(near_heights.begin(), near_heights.begin() + q, near_heights.end());
  const float ground_z = near_heights[q];

  const double sector_width = 2.0 * kPi / static_cast<double>(params_.sector_count);
  for (int sector = 0; sector < params_.sector_count; ++sector) {
    const double yaw = -kPi + (sector + 0.5) * sector_width;
    const Eigen::Vector2f ray(std::cos(yaw), std::sin(yaw));
    pcl::PointCloud<pcl::PointXYZ>::Ptr corridor(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& p : filtered->points) {
      const Eigen::Vector2f rel(p.x - robot_position.x(), p.y - robot_position.y());
      const float along = rel.dot(ray);
      const float across = std::abs(rel.x() * ray.y() - rel.y() * ray.x());
      if (along < params_.min_range || along > params_.max_range ||
          across > params_.corridor_half_width)
        continue;
      if (p.z < ground_z + 0.10F || p.z > ground_z + 3.0F) continue;
      corridor->push_back(p);
    }
    if (corridor->size() < static_cast<size_t>(params_.min_support)) continue;

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*corridor));
    Candidate candidate;
    bool accepted = false;
    for (int attempt = 0; attempt < 3 &&
                          remaining->size() >= static_cast<size_t>(params_.min_support);
         ++attempt) {
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(120);
      seg.setDistanceThreshold(params_.ransac_distance);
      seg.setInputCloud(remaining);
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      pcl::ModelCoefficients coefficients;
      seg.segment(*indices, coefficients);
      if (indices->indices.size() < static_cast<size_t>(params_.min_support) ||
          coefficients.values.size() != 4)
        break;

      Eigen::Vector4f plane(coefficients.values[0], coefficients.values[1],
                            coefficients.values[2], coefficients.values[3]);
      const float norm = plane.head<3>().norm();
      if (norm < 1e-5F) break;
      plane /= norm;
      if (plane.z() < 0.0F) plane = -plane;
      const float slope = std::acos(std::max(0.0F, std::min(1.0F, plane.z())));
      const float slope_deg = slope * 180.0F / static_cast<float>(kPi);

      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(remaining);
      extract.setIndices(indices);
      if (slope_deg >= params_.min_slope_deg && slope_deg <= params_.max_slope_deg &&
          std::abs(plane.z()) > 1e-4F) {
        extract.setNegative(false);
        extract.filter(*candidate.support);
        Eigen::Vector3f heading(-plane.x() / plane.z(), -plane.y() / plane.z(), 0.0F);
        if (heading.head<2>().norm() < 1e-4F) break;
        heading.normalize();

        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        for (const auto& p : candidate.support->points) center += Eigen::Vector3f(p.x, p.y, p.z);
        center /= static_cast<float>(candidate.support->size());
        const Eigen::Vector3f lateral(-heading.y(), heading.x(), 0.0F);
        float min_along = std::numeric_limits<float>::max();
        float max_along = -std::numeric_limits<float>::max();
        float min_lat = std::numeric_limits<float>::max();
        float max_lat = -std::numeric_limits<float>::max();
        for (const auto& p : candidate.support->points) {
          const Eigen::Vector3f rel(p.x - center.x(), p.y - center.y(), 0.0F);
          const float along = rel.dot(heading);
          const float lat = rel.dot(lateral);
          min_along = std::min(min_along, along);
          max_along = std::max(max_along, along);
          min_lat = std::min(min_lat, lat);
          max_lat = std::max(max_lat, lat);
        }
        const float length = max_along - min_along;
        const float width = max_lat - min_lat;
        const float rise = length * std::tan(slope);
        if (length >= params_.min_length && width >= params_.min_width &&
            width <= params_.max_width && rise >= params_.min_rise) {
          candidate.entry = center + heading * min_along;
          candidate.exit = center + heading * max_along;
          candidate.entry.z() = planeHeight(plane, candidate.entry.x(), candidate.entry.y());
          candidate.exit.z() = planeHeight(plane, candidate.exit.x(), candidate.exit.y());
          candidate.heading = heading;
          candidate.slope = slope;
          candidate.width = width;
          candidate.rise = candidate.exit.z() - candidate.entry.z();
          const float support_score = std::min(1.0F, candidate.support->size() / 120.0F);
          const float ratio_score = std::min(1.0F, candidate.support->size() /
                                                       static_cast<float>(corridor->size()));
          candidate.confidence = 0.5F * support_score + 0.5F * ratio_score;
          accepted = true;
        }
        break;
      }

      // The dominant plane may be a wall. Remove it and inspect the next plane.
      pcl::PointCloud<pcl::PointXYZ>::Ptr next(new pcl::PointCloud<pcl::PointXYZ>);
      extract.setNegative(true);
      extract.filter(*next);
      remaining = next;
    }
    if (!accepted) continue;

    bool duplicate = false;
    for (auto& existing : candidates) {
      const float distance = (existing.entry.head<2>() - candidate.entry.head<2>()).norm();
      if (distance < 0.9F && angleBetween(existing.heading, candidate.heading) < 0.35F) {
        duplicate = true;
        if (candidate.confidence > existing.confidence) existing = candidate;
        break;
      }
    }
    if (!duplicate) candidates.push_back(candidate);
  }
  return candidates;
}

}  // namespace stair_perception
