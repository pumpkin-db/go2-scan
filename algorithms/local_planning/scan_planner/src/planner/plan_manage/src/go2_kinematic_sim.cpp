#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace
{
constexpr double kMaxVYawLimit = 1.0;

ros::Publisher odom_pub;
ros::Subscriber cmd_sub;
ros::Subscriber elevation_sub;
ros::Timer sim_timer;
tf::TransformBroadcaster *tf_broadcaster = nullptr;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double yaw = 0.0;

double vx_cmd = 0.0;
double vy_cmd = 0.0;
double vyaw_cmd = 0.0;
double vx_world = 0.0;
double vy_world = 0.0;

double max_vx = 0.8;
double max_vy = 0.5;
double max_vyaw = kMaxVYawLimit;
double cmd_timeout = 0.3;
double sim_rate = 100.0;
bool publish_tf = false;
std::string frame_id = "world";
std::string child_frame_id = "base";
std::string body_pose_topic = "/quad_0/body_pose";

// ---- 地形跟随 z（楼梯/多层仿真，2026-08-25）----
// 开启后 z 不再恒为 init_z：积分 x/y 后查高程图 f(x,y)，z 向 h+body_height 限速趋近。
// 高程图无效（未收到/越界/NaN）时保持上一 z（悬空保持，不跳变）。
bool terrain_follow = false;
std::string elevation_topic = "/elevation_mapping/elevation_map";
double body_height = 0.25;   // 站高：狗体心离地高度
double max_dz_rate = 0.8;    // z 变化限速 m/s（楼梯台阶的平滑下限）
double follow_max_jump = 1.0; // 单步跟随的最大 z 跳变（滤掉「另一层楼」的地表）

// 最新高程图快照（grid_map_msgs 手工采样，避免引入 grid_map_ros 运行时依赖）
grid_map_msgs::GridMap::ConstPtr last_elevation;

double sampleElevation(double px, double py);   // 前向声明（simCallback 先用）

ros::Time last_cmd_time;
ros::Time last_sim_time;

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void loadParamWithFallback(const ros::NodeHandle &nh, const std::string &private_name,
                           const std::string &fallback_name, double &value, double default_value)
{
  if (nh.getParam(private_name, value))
    return;
  if (ros::param::get(fallback_name, value))
    return;
  value = default_value;
}

void cmdCallback(const geometry_msgs::TwistConstPtr &msg)
{
  vx_cmd = clamp(msg->linear.x, -max_vx, max_vx);
  vy_cmd = clamp(msg->linear.y, -max_vy, max_vy);
  vyaw_cmd = clamp(msg->angular.z, -max_vyaw, max_vyaw);
  last_cmd_time = ros::Time::now();
}

void publishOdom(const ros::Time &stamp)
{
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx_world;
  odom.twist.twist.linear.y = vy_world;
  odom.twist.twist.angular.z = vyaw_cmd;
  odom_pub.publish(odom);

  if (!publish_tf || tf_broadcaster == nullptr)
    return;

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;
  tf_msg.transform.rotation = q;
  tf_broadcaster->sendTransform(tf_msg);
}

void simCallback(const ros::TimerEvent &)
{
  const ros::Time now = ros::Time::now();
  double dt = (now - last_sim_time).toSec();
  last_sim_time = now;
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  double vx = vx_cmd;
  double vy = vy_cmd;
  double wz = vyaw_cmd;
  if ((now - last_cmd_time).toSec() > cmd_timeout)
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_world = c * vx - s * vy;
  vy_world = s * vx + c * vy;

  x += vx_world * dt;
  y += vy_world * dt;
  yaw = normalizeAngle(yaw + wz * dt);

  // 地形跟随：查高程图，z 限速趋近 h+body_height（默认关闭，indoor_1 行为不变）
  if (terrain_follow)
  {
    const double h = sampleElevation(x, y);
    if (!std::isnan(h))
    {
      const double z_target = h + body_height;
      // 跳变限幅：目标与当前 z 差超过 follow_max_jump 视为「另一层楼的地表」
      //（2.5D 高程图在夹层下方会报上层高度），保持当前 z 不跟。
      // 楼梯台阶 0.16m 级差远小于限幅，不受影响。
      if (std::abs(z_target - z) <= follow_max_jump && dt > 1e-4)
      {
        const double dz = z_target - z;
        const double dz_max = max_dz_rate * dt;
        z += std::max(-dz_max, std::min(dz_max, dz));
      }
    }
  }

  publishOdom(now);
}

void elevationCallback(const grid_map_msgs::GridMap::ConstPtr &msg)
{
  last_elevation = msg;
}

// 采样 grid_map 的 elevation 层（最近邻足够；越界/无数据返回 NaN）。
// grid_map 布局：position 是地图中心；Index(0,0) 是 x 最小、y 最小的格；
// data 按列主序（Eigen col-major）：data[row + col*rows]，row↔x、col↔y。
double sampleElevation(double px, double py)
{
  if (last_elevation == nullptr)
    return std::numeric_limits<double>::quiet_NaN();

  const auto &gm = *last_elevation;
  // 找 elevation 层
  int layer = -1;
  for (size_t i = 0; i < gm.layers.size(); ++i)
    if (gm.layers[i] == "elevation")
    {
      layer = (int)i;
      break;
    }
  if (layer < 0)
    return std::numeric_limits<double>::quiet_NaN();

  const float res = gm.info.resolution;
  const int rows = gm.data[layer].layout.dim[0].size;   // x 方向格数
  const int cols = gm.data[layer].layout.dim[1].size;   // y 方向格数
  const double half_x = rows * res * 0.5;
  const double half_y = cols * res * 0.5;
  const double x0 = gm.info.pose.position.x - half_x;   // 地图 x 下缘
  const double y0 = gm.info.pose.position.y - half_y;

  const int ix = (int)std::floor((px - x0) / res);
  const int iy = (int)std::floor((py - y0) / res);
  if (ix < 0 || ix >= rows || iy < 0 || iy >= cols)
    return std::numeric_limits<double>::quiet_NaN();

  const float v = gm.data[layer].data[ix + iy * rows];  // col-major
  if (std::isnan(v))
    return std::numeric_limits<double>::quiet_NaN();
  return (double)v;
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go2_kinematic_sim");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::param::param("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ros::param::param("/init_x", x, 0.0);
  ros::param::param("/init_y", y, 0.0);
  ros::param::param("/init_z", z, 0.3);
  nh.param("init_yaw", yaw, 0.0);
  loadParamWithFallback(nh, "max_vx", "/closed_loop_controller/max_vx", max_vx, 0.8);
  loadParamWithFallback(nh, "max_vy", "/closed_loop_controller/max_vy", max_vy, 0.5);
  loadParamWithFallback(nh, "max_vyaw", "/closed_loop_controller/max_vyaw", max_vyaw, kMaxVYawLimit);
  if (max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[Go2 kinematic sim] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  nh.param("cmd_timeout", cmd_timeout, 0.3);
  nh.param("sim_rate", sim_rate, 100.0);
  nh.param("publish_tf", publish_tf, false);
  nh.param("frame_id", frame_id, std::string("world"));
  nh.param("child_frame_id", child_frame_id, std::string("base"));
  // 地形跟随（楼梯/多层）：默认关。Depot 等多层场景 launch 里开 terrain_follow:=true
  nh.param("terrain_follow", terrain_follow, false);
  nh.param("elevation_topic", elevation_topic, std::string("/elevation_mapping/elevation_map"));
  nh.param("body_height", body_height, 0.25);
  nh.param("max_dz_rate", max_dz_rate, 0.8);
  nh.param("follow_max_jump", follow_max_jump, 1.0);
  if (terrain_follow)
  {
    elevation_sub = node.subscribe(elevation_topic, 1, elevationCallback);
    ROS_WARN("[Go2 kinematic sim] terrain_follow ON: z 跟随 %s (body_height=%.2f, dz<=%.2f m/s)",
             elevation_topic.c_str(), body_height, max_dz_rate);
  }

  tf::TransformBroadcaster broadcaster;
  tf_broadcaster = &broadcaster;

  odom_pub = node.advertise<nav_msgs::Odometry>(body_pose_topic, 100);
  cmd_sub = node.subscribe("cmd_vel", 20, cmdCallback, ros::TransportHints().tcpNoDelay());

  last_cmd_time = ros::Time::now();
  last_sim_time = ros::Time::now();
  sim_timer = node.createTimer(ros::Duration(1.0 / sim_rate), simCallback);

  ROS_WARN("[Go2 kinematic sim] ready.");

  ros::spin();
  return 0;
}
