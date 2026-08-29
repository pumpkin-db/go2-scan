#include "stair_perception/detector_core.h"

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <stair_perception/StairObservation.h>
#include <stair_perception/StairObservationArray.h>

#include <mutex>

namespace stair_perception {

class DetectorNode {
 public:
  DetectorNode() : pnh_("~"), core_(loadParams()) {
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/mid360_points");
    pnh_.param<std::string>("pose_topic", pose_topic_, "/quad_0/body_pose");
    pnh_.param<double>("max_rate", max_rate_, 2.0);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &DetectorNode::cloudCallback, this);
    pose_sub_ = nh_.subscribe(pose_topic_, 5, &DetectorNode::poseCallback, this);
    observation_pub_ = nh_.advertise<StairObservationArray>("/stair_perception/observations", 2);
    support_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/stair_perception/debug/support", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/stair_perception/debug/candidates", 1);
    ROS_INFO("[stair_detector] cloud=%s pose=%s (world-frame contract)",
             cloud_topic_.c_str(), pose_topic_.c_str());
  }

 private:
  DetectorParams loadParams() {
    DetectorParams p;
    pnh_.param("voxel_size", p.voxel_size, p.voxel_size);
    pnh_.param("min_range", p.min_range, p.min_range);
    pnh_.param("max_range", p.max_range, p.max_range);
    pnh_.param("min_slope_deg", p.min_slope_deg, p.min_slope_deg);
    pnh_.param("max_slope_deg", p.max_slope_deg, p.max_slope_deg);
    pnh_.param("min_width", p.min_width, p.min_width);
    pnh_.param("max_width", p.max_width, p.max_width);
    pnh_.param("min_length", p.min_length, p.min_length);
    pnh_.param("min_rise", p.min_rise, p.min_rise);
    pnh_.param("max_landing_height_gap", p.max_landing_height_gap,
               p.max_landing_height_gap);
    pnh_.param("corridor_half_width", p.corridor_half_width, p.corridor_half_width);
    pnh_.param("ransac_distance", p.ransac_distance, p.ransac_distance);
    pnh_.param("min_support", p.min_support, p.min_support);
    pnh_.param("sector_count", p.sector_count, p.sector_count);
    return p;
  }

  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_position_ = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                     msg->pose.pose.position.z);
    pose_frame_ = msg->header.frame_id;
    have_pose_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    Eigen::Vector3f robot;
    std::string pose_frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_pose_) return;
      robot = robot_position_;
      pose_frame = pose_frame_;
    }
    if (msg->header.frame_id.empty() || msg->header.frame_id != pose_frame) {
      ROS_WARN_THROTTLE(5.0, "[stair_detector] cloud frame '%s' != pose frame '%s'; dropping",
                        msg->header.frame_id.c_str(), pose_frame.c_str());
      return;
    }
    if (!last_run_.isZero() && (msg->header.stamp - last_run_).toSec() < 1.0 / max_rate_) return;
    last_run_ = msg->header.stamp;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    const auto candidates = core_.detect(cloud, robot);
    publish(msg->header, candidates);
  }

  void publish(const std_msgs::Header& header, const std::vector<Candidate>& candidates) {
    StairObservationArray array;
    array.header = header;
    pcl::PointCloud<pcl::PointXYZ> support;
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (const auto& c : candidates) {
      StairObservation o;
      o.header = header;
      o.entry_pose.position.x = c.entry.x();
      o.entry_pose.position.y = c.entry.y();
      o.entry_pose.position.z = c.entry.z();
      o.entry_pose.orientation.w = 1.0;
      o.exit_pose.position.x = c.exit.x();
      o.exit_pose.position.y = c.exit.y();
      o.exit_pose.position.z = c.exit.z();
      o.exit_pose.orientation.w = 1.0;
      o.heading.x = c.heading.x();
      o.heading.y = c.heading.y();
      o.heading.z = c.heading.z();
      o.slope = c.slope;
      o.width = c.width;
      o.rise = c.rise;
      o.confidence = c.confidence;
      o.support_count = c.support->size();
      array.observations.push_back(o);
      support += *c.support;

      visualization_msgs::Marker arrow;
      arrow.header = header;
      arrow.ns = "stair_candidates";
      arrow.id = marker_id++;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point a, b;
      a.x = c.entry.x(); a.y = c.entry.y(); a.z = c.entry.z() + 0.15;
      b.x = c.exit.x(); b.y = c.exit.y(); b.z = c.exit.z() + 0.15;
      arrow.points = {a, b};
      arrow.scale.x = 0.09; arrow.scale.y = 0.16; arrow.scale.z = 0.16;
      arrow.color.r = 0.1; arrow.color.g = 0.7; arrow.color.b = 1.0; arrow.color.a = 0.9;
      arrow.lifetime = ros::Duration(1.0);
      markers.markers.push_back(arrow);
    }
    observation_pub_.publish(array);
    marker_pub_.publish(markers);
    sensor_msgs::PointCloud2 support_msg;
    pcl::toROSMsg(support, support_msg);
    support_msg.header = header;
    support_pub_.publish(support_msg);
    ROS_INFO_THROTTLE(5.0, "[stair_detector] candidates=%zu", candidates.size());
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber cloud_sub_, pose_sub_;
  ros::Publisher observation_pub_, support_pub_, marker_pub_;
  DetectorCore core_;
  std::mutex mutex_;
  Eigen::Vector3f robot_position_{Eigen::Vector3f::Zero()};
  std::string cloud_topic_, pose_topic_, pose_frame_;
  bool have_pose_{false};
  double max_rate_{2.0};
  ros::Time last_run_;
};

}  // namespace stair_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "stair_detector");
  stair_perception::DetectorNode node;
  ros::spin();
  return 0;
}
