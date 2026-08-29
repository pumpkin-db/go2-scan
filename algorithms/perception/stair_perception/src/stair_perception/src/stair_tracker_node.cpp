#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <stair_perception/StairObservationArray.h>
#include <stair_perception/StairTrack.h>
#include <stair_perception/StairTrackArray.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace stair_perception {

class TrackerNode {
 public:
  TrackerNode() : pnh_("~") {
    pnh_.param("association_distance", association_distance_, 1.0);
    pnh_.param("association_heading_deg", association_heading_deg_, 25.0);
    pnh_.param("association_slope_deg", association_slope_deg_, 10.0);
    pnh_.param("confirm_observations", confirm_observations_, 3);
    pnh_.param("track_timeout", track_timeout_, 5.0);
    observation_sub_ = nh_.subscribe("/stair_perception/observations", 5,
                                     &TrackerNode::observationCallback, this);
    track_pub_ = nh_.advertise<StairTrackArray>("/stair_perception/tracks", 2, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/stair_perception/debug/tracks", 1, true);
  }

 private:
  static double headingAngle(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    const double dot = a.x * b.x + a.y * b.y;
    const double na = std::hypot(a.x, a.y);
    const double nb = std::hypot(b.x, b.y);
    if (na < 1e-6 || nb < 1e-6) return M_PI;
    return std::acos(std::max(-1.0, std::min(1.0, dot / (na * nb))));
  }

  int associate(const StairObservation& o) const {
    int best = -1;
    double best_distance = association_distance_;
    for (size_t i = 0; i < tracks_.size(); ++i) {
      const auto& t = tracks_[i];
      const double distance = std::hypot(t.entry_pose.position.x - o.entry_pose.position.x,
                                         t.entry_pose.position.y - o.entry_pose.position.y);
      const double heading = headingAngle(t.heading, o.heading) * 180.0 / M_PI;
      const double slope = std::abs(t.slope - o.slope) * 180.0 / M_PI;
      if (distance < best_distance && heading <= association_heading_deg_ &&
          slope <= association_slope_deg_) {
        best = static_cast<int>(i);
        best_distance = distance;
      }
    }
    return best;
  }

  static void blend(double alpha, const StairObservation& o, StairTrack* t) {
    auto mix = [alpha](double old_value, double new_value) {
      return (1.0 - alpha) * old_value + alpha * new_value;
    };
    t->entry_pose.position.x = mix(t->entry_pose.position.x, o.entry_pose.position.x);
    t->entry_pose.position.y = mix(t->entry_pose.position.y, o.entry_pose.position.y);
    t->entry_pose.position.z = mix(t->entry_pose.position.z, o.entry_pose.position.z);
    t->exit_pose.position.x = mix(t->exit_pose.position.x, o.exit_pose.position.x);
    t->exit_pose.position.y = mix(t->exit_pose.position.y, o.exit_pose.position.y);
    t->exit_pose.position.z = mix(t->exit_pose.position.z, o.exit_pose.position.z);
    t->heading.x = mix(t->heading.x, o.heading.x);
    t->heading.y = mix(t->heading.y, o.heading.y);
    const double norm = std::hypot(t->heading.x, t->heading.y);
    if (norm > 1e-6) { t->heading.x /= norm; t->heading.y /= norm; }
    t->slope = mix(t->slope, o.slope);
    t->width = mix(t->width, o.width);
    t->rise = mix(t->rise, o.rise);
    t->confidence = mix(t->confidence, o.confidence);
  }

  void observationCallback(const StairObservationArray::ConstPtr& msg) {
    const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const StairTrack& t) {
                    return (now - t.last_seen).toSec() > track_timeout_;
                  }), tracks_.end());

    for (const auto& o : msg->observations) {
      const int match = associate(o);
      if (match < 0) {
        StairTrack t;
        t.header = msg->header;
        t.id = next_id_++;
        t.state = StairTrack::DETECTED;
        t.entry_pose = o.entry_pose;
        t.exit_pose = o.exit_pose;
        t.heading = o.heading;
        t.slope = o.slope;
        t.width = o.width;
        t.rise = o.rise;
        t.confidence = o.confidence;
        t.observation_count = 1;
        t.last_seen = now;
        tracks_.push_back(t);
      } else {
        auto& t = tracks_[match];
        blend(1.0 / std::min(5U, t.observation_count + 1), o, &t);
        ++t.observation_count;
        t.last_seen = now;
        t.header = msg->header;
        if (t.observation_count >= static_cast<uint32_t>(confirm_observations_))
          t.state = StairTrack::CONFIRMED;
      }
    }
    publish(msg->header);
  }

  void publish(const std_msgs::Header& header) {
    StairTrackArray out;
    out.header = header;
    out.tracks = tracks_;
    track_pub_.publish(out);

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);
    for (const auto& t : tracks_) {
      visualization_msgs::Marker arrow;
      arrow.header = header;
      arrow.ns = "stair_tracks";
      arrow.id = t.id;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point a = t.entry_pose.position;
      geometry_msgs::Point b = t.exit_pose.position;
      a.z += 0.25; b.z += 0.25;
      arrow.points = {a, b};
      arrow.scale.x = 0.12; arrow.scale.y = 0.22; arrow.scale.z = 0.22;
      arrow.color.a = 0.95;
      if (t.state == StairTrack::CONFIRMED) {
        arrow.color.g = 1.0;
      } else {
        arrow.color.r = 1.0; arrow.color.g = 0.65;
      }
      markers.markers.push_back(arrow);

      visualization_msgs::Marker label;
      label.header = header;
      label.ns = "stair_track_ids";
      label.id = t.id;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::Marker::ADD;
      label.pose.position = t.entry_pose.position;
      label.pose.position.z += 0.55;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.28;
      label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0; label.color.a = 1.0;
      label.text = "stair " + std::to_string(t.id) +
                   (t.state == StairTrack::CONFIRMED ? " CONFIRMED" : " DETECTED");
      markers.markers.push_back(label);
    }
    marker_pub_.publish(markers);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber observation_sub_;
  ros::Publisher track_pub_, marker_pub_;
  std::vector<StairTrack> tracks_;
  uint32_t next_id_{1};
  double association_distance_{1.0};
  double association_heading_deg_{25.0};
  double association_slope_deg_{10.0};
  double track_timeout_{5.0};
  int confirm_observations_{3};
};

}  // namespace stair_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "stair_tracker");
  stair_perception::TrackerNode node;
  ros::spin();
  return 0;
}
