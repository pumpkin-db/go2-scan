#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
class Go2KinematicModelPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    if (!ros::isInitialized())
    {
      gzerr << "[go2_kinematic_model_plugin] ROS is not initialized\n";
      return;
    }

    nh_.reset(new ros::NodeHandle(model_->GetName() + "/kinematic_model"));
    if (sdf->HasElement("cmdTimeout")) cmd_timeout_ = sdf->Get<double>("cmdTimeout");
    if (sdf->HasElement("maxVx")) max_vx_ = sdf->Get<double>("maxVx");
    if (sdf->HasElement("maxVy")) max_vy_ = sdf->Get<double>("maxVy");
    if (sdf->HasElement("maxVyaw")) max_vyaw_ = sdf->Get<double>("maxVyaw");
    if (sdf->HasElement("maxVz")) max_vz_ = sdf->Get<double>("maxVz");
    if (sdf->HasElement("lidarX")) lidar_x_ = sdf->Get<double>("lidarX");
    if (sdf->HasElement("lidarZ")) lidar_z_ = sdf->Get<double>("lidarZ");

    cmd_sub_ = nh_->subscribe("/cmd_vel", 1, &Go2KinematicModelPlugin::OnCmd, this);
    z_target_sub_ = nh_->subscribe("/sim/body_z_target", 1,
                                   &Go2KinematicModelPlugin::OnZTarget, this);
    body_pub_ = nh_->advertise<nav_msgs::Odometry>("/quad_0/body_pose", 10);
    lidar_pub_ = nh_->advertise<nav_msgs::Odometry>("/quad_0/lidar_pose", 10);

    const auto pose = model_->WorldPose();
    x_ = pose.Pos().X();
    y_ = pose.Pos().Y();
    z_ = pose.Pos().Z();
    yaw_ = pose.Rot().Yaw();
    last_update_ = world_->SimTime();
    last_cmd_ = last_update_;
    CacheStandingJoints();
    update_connection_ = event::Events::ConnectWorldUpdateEnd(
        std::bind(&Go2KinematicModelPlugin::OnUpdate, this));
    ROS_INFO("[go2_kinematic_model_plugin] active; sole model pose writer");
  }

private:
  static double Clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  void OnCmd(const geometry_msgs::TwistConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    vx_cmd_ = Clamp(msg->linear.x, -max_vx_, max_vx_);
    vy_cmd_ = Clamp(msg->linear.y, -max_vy_, max_vy_);
    wz_cmd_ = Clamp(msg->angular.z, -max_vyaw_, max_vyaw_);
    last_cmd_ = world_->SimTime();
  }

  void OnZTarget(const std_msgs::Float64ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    z_target_ = msg->data;
    last_z_target_ = world_->SimTime();
    have_z_target_ = std::isfinite(z_target_);
  }

  void CacheStandingJoints()
  {
    const std::array<std::pair<const char *, double>, 12> standing = {{
      {"FL_hip_joint", 0.05}, {"FL_thigh_joint", 0.82}, {"FL_calf_joint", -1.58},
      {"FR_hip_joint", -0.05}, {"FR_thigh_joint", 0.82}, {"FR_calf_joint", -1.58},
      {"RL_hip_joint", 0.05}, {"RL_thigh_joint", 0.95}, {"RL_calf_joint", -1.62},
      {"RR_hip_joint", -0.05}, {"RR_thigh_joint", 0.95}, {"RR_calf_joint", -1.62}}};
    for (const auto &entry : standing)
    {
      auto joint = model_->GetJoint(entry.first);
      if (joint)
      {
        joint->SetPosition(0, entry.second, true);
        joint->SetLowerLimit(0, entry.second);
        joint->SetUpperLimit(0, entry.second);
        standing_joints_.push_back(std::make_pair(joint, entry.second));
      }
    }
  }

  void OnUpdate()
  {
    const common::Time sim_time = world_->SimTime();
    const double dt = (sim_time - last_update_).Double();
    last_update_ = sim_time;
    if (dt <= 0.0 || dt > 0.2) return;

    double vx, vy, wz;
    bool use_z_target = false;
    double z_target = z_;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vx = vx_cmd_; vy = vy_cmd_; wz = wz_cmd_;
      if ((sim_time - last_cmd_).Double() > cmd_timeout_) vx = vy = wz = 0.0;
      use_z_target = have_z_target_ && (sim_time - last_z_target_).Double() <= cmd_timeout_;
      z_target = z_target_;
    }

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    const double vx_world = c * vx - s * vy;
    const double vy_world = s * vx + c * vy;
    x_ += vx_world * dt;
    y_ += vy_world * dt;
    yaw_ = std::atan2(std::sin(yaw_ + wz * dt), std::cos(yaw_ + wz * dt));
    const double old_z = z_;
    if (use_z_target)
      z_ += Clamp(z_target - z_, -max_vz_ * dt, max_vz_ * dt);

    model_->SetWorldPose(ignition::math::Pose3d(x_, y_, z_, 0.0, 0.0, yaw_));
    for (const auto &entry : standing_joints_)
    {
      entry.first->SetPosition(0, entry.second, true);
      entry.first->SetVelocity(0, 0.0);
    }
    PublishPoses(sim_time, vx_world, vy_world, (z_ - old_z) / dt, wz);
  }

  void PublishPoses(const common::Time &sim_time, double vx_world, double vy_world,
                    double vz_world, double wz)
  {
    nav_msgs::Odometry body;
    body.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
    body.header.frame_id = "world";
    body.child_frame_id = "base";
    body.pose.pose.position.x = x_;
    body.pose.pose.position.y = y_;
    body.pose.pose.position.z = z_;
    body.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
    body.twist.twist.linear.x = vx_world;
    body.twist.twist.linear.y = vy_world;
    body.twist.twist.linear.z = vz_world;
    body.twist.twist.angular.z = wz;
    body_pub_.publish(body);

    nav_msgs::Odometry lidar = body;
    lidar.child_frame_id = "mid360";
    lidar.pose.pose.position.x += std::cos(yaw_) * lidar_x_;
    lidar.pose.pose.position.y += std::sin(yaw_) * lidar_x_;
    lidar.pose.pose.position.z += lidar_z_;
    lidar_pub_.publish(lidar);

    geometry_msgs::TransformStamped transform;
    transform.header = body.header;
    transform.child_frame_id = "base";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = z_;
    transform.transform.rotation = body.pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber cmd_sub_, z_target_sub_;
  ros::Publisher body_pub_, lidar_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex cmd_mutex_;
  common::Time last_update_, last_cmd_, last_z_target_;
  double x_ = 0.0, y_ = 0.0, z_ = 0.0, yaw_ = 0.0;
  double vx_cmd_ = 0.0, vy_cmd_ = 0.0, wz_cmd_ = 0.0;
  double max_vx_ = 0.8, max_vy_ = 0.5, max_vyaw_ = 1.0, cmd_timeout_ = 0.3;
  double max_vz_ = 0.5, z_target_ = 0.0;
  bool have_z_target_ = false;
  double lidar_x_ = 0.2, lidar_z_ = 0.2077;
  std::vector<std::pair<physics::JointPtr, double>> standing_joints_;
};

GZ_REGISTER_MODEL_PLUGIN(Go2KinematicModelPlugin)
}  // namespace gazebo
