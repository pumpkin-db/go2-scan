#include "stair_perception/tracker_core.h"

#include <gtest/gtest.h>

namespace {

stair_perception::StairObservation observation(double entry_y, double exit_y, float rise,
                                                float confidence = 0.95F) {
  stair_perception::StairObservation o;
  o.entry_pose.position.y = entry_y;
  o.exit_pose.position.y = exit_y;
  o.heading.y = -1.0;
  o.rise = rise;
  o.confidence = confidence;
  return o;
}

}  // namespace

TEST(TrackerCore, PartialViewCannotShrinkConfirmedFlightExtent) {
  stair_perception::StairTrack track;
  const auto complete = observation(3.14, 0.16, 2.93F);
  track.entry_pose = complete.entry_pose;
  track.exit_pose = complete.exit_pose;
  track.heading = complete.heading;
  track.rise = complete.rise;
  track.confidence = complete.confidence;

  stair_perception::ExtentProposal proposal;
  const auto partial = observation(2.84, 0.79, 2.04F);
  stair_perception::fuseObservation(0.2, partial, &track);
  stair_perception::considerExtent(partial, &track, &proposal);
  EXPECT_FLOAT_EQ(track.rise, 2.93F);
  EXPECT_DOUBLE_EQ(track.entry_pose.position.y, 3.14);
  EXPECT_DOUBLE_EQ(track.exit_pose.position.y, 0.16);
}

TEST(TrackerCore, RepeatedConsistentExtentCanExpandFlight) {
  stair_perception::StairTrack track;
  const auto partial = observation(2.55, 0.88, 1.65F);
  track.entry_pose = partial.entry_pose;
  track.exit_pose = partial.exit_pose;
  track.heading = partial.heading;
  track.rise = partial.rise;
  track.confidence = partial.confidence;

  stair_perception::ExtentProposal proposal;
  EXPECT_FALSE(stair_perception::considerExtent(
      observation(3.10, 0.20, 2.85F, 0.60F), &track, &proposal));
  EXPECT_TRUE(stair_perception::considerExtent(
      observation(3.14, 0.16, 2.93F, 0.60F), &track, &proposal));
  EXPECT_FLOAT_EQ(track.rise, 2.93F);
  EXPECT_DOUBLE_EQ(track.entry_pose.position.y, 3.14);
  EXPECT_DOUBLE_EQ(track.exit_pose.position.y, 0.16);
}

TEST(TrackerCore, SingleLongObservationCannotExpandFlight) {
  stair_perception::StairTrack track;
  const auto partial = observation(2.55, 0.88, 1.65F);
  track.entry_pose = partial.entry_pose;
  track.exit_pose = partial.exit_pose;
  track.heading = partial.heading;
  track.rise = partial.rise;
  stair_perception::ExtentProposal proposal;
  EXPECT_FALSE(stair_perception::considerExtent(
      observation(3.14, 0.16, 2.93F, 0.60F), &track, &proposal));
  EXPECT_FLOAT_EQ(track.rise, 1.65F);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
