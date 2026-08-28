#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
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
    if (sdf->HasElement("lidarX")) lidar_x_ = sdf->Get<double>("lidarX");
    if (sdf->HasElement("lidarZ")) lidar_z_ = sdf->Get<double>("lidarZ");

    // 地形跟随（2026-08-28 自 kinematic_sim 迁移）：rosparam 全局开关，
    // 由场景 launch 提供（depot env.sh: terrain_follow:=true gt_elev_file:=...）。
    // indoor_1 不设 → 默认关闭，基线行为不变。
    nh_->param<bool>("/terrain_follow", terrain_follow_, false);
    nh_->param<double>("/terrain_body_height", body_clearance_, 0.25);
    nh_->param<double>("/terrain_max_dz_rate", max_dz_rate_, 0.8);
    nh_->param<double>("/terrain_follow_max_jump", follow_max_jump_, 1.0);
    nh_->param<std::string>("/gt_elev_file", gt_elev_file_, "");
    if (terrain_follow_ && !gt_elev_file_.empty())
    {
      terrain_follow_ = LoadGtElev(gt_elev_file_);
    }
    else if (terrain_follow_)
    {
      ROS_WARN("[go2_kinematic_model_plugin] terrain_follow on but gt_elev_file empty — disabled");
      terrain_follow_ = false;
    }
    if (terrain_follow_)
    {
      ROS_INFO("[go2_kinematic_model_plugin] terrain follow ON (clearance=%.2f dz_rate=%.2f jump=%.2f)",
               body_clearance_, max_dz_rate_, follow_max_jump_);
    }

    cmd_sub_ = nh_->subscribe("/cmd_vel", 1, &Go2KinematicModelPlugin::OnCmd, this);
    body_pub_ = nh_->advertise<nav_msgs::Odometry>("/quad_0/body_pose", 10);
    lidar_pub_ = nh_->advertise<nav_msgs::Odometry>("/quad_0/lidar_pose", 10);
    // 楼梯 committed 单调锁（P6）：stair_mission_manager 发布
    lock_sub_ = nh_->subscribe("/stair_traverse_lock", 2, &Go2KinematicModelPlugin::OnLock, this);
    dir_sub_ = nh_->subscribe("/stair_traverse_dir", 2, &Go2KinematicModelPlugin::OnDir, this);
    // committed 期间 z 剖面（P3b）：2.5D GT 在楼梯区有多层歧义（下层报地面高），
    // 改跟注册表楼梯几何：z_target = entry_z + (exit_z-entry_z)*s/s_exit + clearance
    zprof_sub_ = nh_->subscribe("/stair_traverse_zprof", 2, &Go2KinematicModelPlugin::OnZProf, this);

    const auto pose = model_->WorldPose();
    x_ = pose.Pos().X();
    y_ = pose.Pos().Y();
    z_ = pose.Pos().Z();
    yaw_ = pose.Rot().Yaw();
    s_prev_ = x_ * stair_dx_ + y_ * stair_dy_;
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

  void OnLock(const std_msgs::BoolConstPtr &msg)
  {
    stair_lock_ = msg->data;
    if (stair_lock_)
    {
      s_prev_ = x_ * stair_dx_ + y_ * stair_dy_;
      ROS_INFO("[go2_kinematic_model_plugin] stair monotonic lock ENGAGED d=(%.3f,%.3f) s0=%.2f",
               stair_dx_, stair_dy_, s_prev_);
    }
    else
    {
      ROS_INFO("[go2_kinematic_model_plugin] stair monotonic lock RELEASED");
    }
  }

  void OnDir(const geometry_msgs::Vector3ConstPtr &msg)
  {
    const double n = std::hypot(msg->x, msg->y);
    if (n > 1e-6)
    {
      stair_dx_ = msg->x / n;
      stair_dy_ = msg->y / n;
    }
  }

  void OnZProf(const geometry_msgs::Vector3ConstPtr &msg)
  {
    zprof_entry_z_ = msg->x;
    zprof_exit_z_ = msg->y;
    zprof_s_exit_ = std::max(0.3, msg->z);
    zprof_valid_ = true;
  }

  // ---- GT 地形跟随（自 go2_kinematic_sim.cpp 原样迁移）----
  struct GtElev
  {
    int nx = 0, ny = 0;
    float x0 = 0.f, y0 = 0.f, res = 0.05f;
    std::vector<float> h;  // h[iy*nx + ix]
    bool ok = false;
  } gt_elev_;

  bool LoadGtElev(const std::string &path)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f)
    {
      ROS_ERROR("[go2_kinematic_model_plugin] GT 高程文件打不开: %s", path.c_str());
      return false;
    }
    int32_t nx = 0, ny = 0;
    f.read(reinterpret_cast<char *>(&nx), 4);
    f.read(reinterpret_cast<char *>(&ny), 4);
    f.read(reinterpret_cast<char *>(&gt_elev_.x0), 4);
    f.read(reinterpret_cast<char *>(&gt_elev_.y0), 4);
    f.read(reinterpret_cast<char *>(&gt_elev_.res), 4);
    if (!f || nx <= 0 || ny <= 0 || gt_elev_.res <= 0.f)
    {
      ROS_ERROR("[go2_kinematic_model_plugin] GT 高程文件头非法: %s", path.c_str());
      return false;
    }
    gt_elev_.h.resize(static_cast<size_t>(nx) * ny);
    f.read(reinterpret_cast<char *>(gt_elev_.h.data()),
           static_cast<std::streamsize>(gt_elev_.h.size()) * 4);
    if (!f)
    {
      ROS_ERROR("[go2_kinematic_model_plugin] GT 高程文件体不全: %s", path.c_str());
      return false;
    }
    gt_elev_.nx = nx;
    gt_elev_.ny = ny;
    gt_elev_.ok = true;
    ROS_WARN("[go2_kinematic_model_plugin] GT 高程已载入: %s（%dx%d, res=%.3f, 原点 %.2f,%.2f）",
             path.c_str(), nx, ny, gt_elev_.res, gt_elev_.x0, gt_elev_.y0);
    return true;
  }

  double SampleGtElev(double px, double py) const
  {
    if (!gt_elev_.ok)
      return std::numeric_limits<double>::quiet_NaN();
    const int ix = static_cast<int>(std::floor((px - gt_elev_.x0) / gt_elev_.res));
    const int iy = static_cast<int>(std::floor((py - gt_elev_.y0) / gt_elev_.res));
    if (ix < 0 || ix >= gt_elev_.nx || iy < 0 || iy >= gt_elev_.ny)
      return std::numeric_limits<double>::quiet_NaN();
    const float v = gt_elev_.h[static_cast<size_t>(iy) * gt_elev_.nx + ix];
    if (std::isnan(v))
      return std::numeric_limits<double>::quiet_NaN();
    return static_cast<double>(v);
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
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vx = vx_cmd_; vy = vy_cmd_; wz = wz_cmd_;
      if ((sim_time - last_cmd_).Double() > cmd_timeout_) vx = vy = wz = 0.0;
    }

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);
    double vx_world = c * vx - s * vy;
    double vy_world = s * vx + c * vy;

    // 楼梯 committed 单调锁：去掉沿楼梯反方向的速度分量（P6）
    if (stair_lock_)
    {
      const double v_progress = vx_world * stair_dx_ + vy_world * stair_dy_;
      if (v_progress < 0.0)
      {
        vx_world -= stair_dx_ * v_progress;
        vy_world -= stair_dy_ * v_progress;
      }
    }

    x_ += vx_world * dt;
    y_ += vy_world * dt;
    yaw_ = std::atan2(std::sin(yaw_ + wz * dt), std::cos(yaw_ + wz * dt));

    // 地形跟随：committed 期间跟注册表楼梯几何 z 剖面（绕开 2.5D GT 楼梯歧义，
    // 且不受 jump-clamp 限制——几何剖面是权威，只保留速率限制）；
    // 非 committed 跟 GT 高程图（indoor_1 默认关闭，行为不变）
    double h = std::numeric_limits<double>::quiet_NaN();
    bool geometric = false;
    if (stair_lock_ && zprof_valid_)
    {
      const double s_now = x_ * stair_dx_ + y_ * stair_dy_;
      const double f = Clamp(s_now / zprof_s_exit_, 0.0, 1.0);
      h = zprof_entry_z_ + (zprof_exit_z_ - zprof_entry_z_) * f;
      geometric = true;
    }
    else if (terrain_follow_)
    {
      h = SampleGtElev(x_, y_);
    }
    if (!std::isnan(h))
    {
      const double z_target = h + body_clearance_;
      const double dz = z_target - z_;
      const double dz_max = max_dz_rate_ * dt;
      if (geometric)
      {
        z_ += Clamp(dz, -dz_max, dz_max);
      }
      else if (std::fabs(dz) <= follow_max_jump_)
      {
        z_ += Clamp(dz, -dz_max, dz_max);
      }
    }

    // 位置级单调保护：committed 期间 s 只增不减
    if (stair_lock_)
    {
      const double s_new = x_ * stair_dx_ + y_ * stair_dy_;
      if (s_new < s_prev_)
      {
        const double dback = s_prev_ - s_new;
        x_ += stair_dx_ * dback;
        y_ += stair_dy_ * dback;
      }
      s_prev_ = x_ * stair_dx_ + y_ * stair_dy_;
    }

    model_->SetWorldPose(ignition::math::Pose3d(x_, y_, z_, 0.0, 0.0, yaw_));
    for (const auto &entry : standing_joints_)
    {
      entry.first->SetPosition(0, entry.second, true);
      entry.first->SetVelocity(0, 0.0);
    }
    PublishPoses(sim_time, vx_world, vy_world, wz);
  }

  void PublishPoses(const common::Time &sim_time, double vx_world, double vy_world, double wz)
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
  ros::Subscriber cmd_sub_, lock_sub_, dir_sub_, zprof_sub_;
  ros::Publisher body_pub_, lidar_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex cmd_mutex_;
  common::Time last_update_, last_cmd_;
  double x_ = 0.0, y_ = 0.0, z_ = 0.0, yaw_ = 0.0;
  double vx_cmd_ = 0.0, vy_cmd_ = 0.0, wz_cmd_ = 0.0;
  double max_vx_ = 0.8, max_vy_ = 0.5, max_vyaw_ = 1.0, cmd_timeout_ = 0.3;
  double lidar_x_ = 0.2, lidar_z_ = 0.2077;
  std::vector<std::pair<physics::JointPtr, double>> standing_joints_;

  // 地形跟随
  bool terrain_follow_ = false;
  double body_clearance_ = 0.25, max_dz_rate_ = 0.8, follow_max_jump_ = 1.0;
  std::string gt_elev_file_;

  // 楼梯 committed 单调锁
  bool stair_lock_ = false;
  double stair_dx_ = 1.0, stair_dy_ = 0.0, s_prev_ = 0.0;

  // committed 几何 z 剖面
  bool zprof_valid_ = false;
  double zprof_entry_z_ = 0.0, zprof_exit_z_ = 0.0, zprof_s_exit_ = 2.8;
};

GZ_REGISTER_MODEL_PLUGIN(Go2KinematicModelPlugin)
}  // namespace gazebo
