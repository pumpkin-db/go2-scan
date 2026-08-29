#include "stair_perception/detector_core.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr makeGround(float z = 0.0F) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (float x = -2.0F; x <= 7.0F; x += 0.08F)
    for (float y = -2.0F; y <= 2.0F; y += 0.08F)
      cloud->push_back(pcl::PointXYZ(x, y, z));
  return cloud;
}

void addStaircase(pcl::PointCloud<pcl::PointXYZ>* cloud, float base_z = 0.0F) {
  constexpr float kStart = 2.0F;
  constexpr float kRun = 0.30F;
  constexpr float kRise = 0.16F;
  for (float x = kStart; x <= 5.0F; x += 0.04F) {
    const int step = static_cast<int>((x - kStart) / kRun);
    const float z = base_z + (step + 1) * kRise;
    for (float y = -0.65F; y <= 0.65F; y += 0.05F)
      cloud->push_back(pcl::PointXYZ(x, y, z));
  }
}

}  // namespace

TEST(DetectorCore, RejectsFlatGround) {
  stair_perception::DetectorCore detector;
  const auto result = detector.detect(makeGround(), Eigen::Vector3f(0.0F, 0.0F, 0.35F));
  EXPECT_TRUE(result.empty());
}

TEST(DetectorCore, DetectsSteppedFlight) {
  auto cloud = makeGround();
  addStaircase(cloud.get());
  stair_perception::DetectorCore detector;
  const auto result = detector.detect(cloud, Eigen::Vector3f(0.0F, 0.0F, 0.35F));
  ASSERT_FALSE(result.empty());
  const auto best = std::max_element(result.begin(), result.end(),
                                     [](const auto& a, const auto& b) {
                                       return a.confidence < b.confidence;
                                     });
  EXPECT_GT(best->rise, 0.8F);
  EXPECT_GT(best->width, 0.8F);
  EXPECT_NEAR(best->heading.x(), 1.0F, 0.25F);
}

TEST(DetectorCore, RejectsFlightDisconnectedFromCurrentFloor) {
  auto cloud = makeGround();
  addStaircase(cloud.get(), 2.0F);
  stair_perception::DetectorCore detector;
  const auto result = detector.detect(cloud, Eigen::Vector3f(0.0F, 0.0F, 0.35F));
  EXPECT_TRUE(result.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
