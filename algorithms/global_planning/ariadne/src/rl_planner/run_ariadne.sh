#!/bin/bash
# 启动 ARiADNE rl_planner 的 conda 包装
#
# 为什么需要：rl_planner.py 的 shebang 是 `#!/usr/bin/env python`，而 launch_gazebo_sim.sh 第 1 步
# 会把 conda 从 PATH 清掉，于是该节点会落到「无 torch」的 Ubuntu 系统 python3.8 上，直接 `import torch` 失败。
# 这里强制用 conda(ariadne) 的 python3.8（torch 2.3.1+cpu 装在里面），并补上 ROS 的 dist-packages，
# 让 rospy / 消息类型能在该 python 里 import。torch 与 ROS 系统 numpy 共存已实测验证通过。
#
# 供 roslaunch 用：<node pkg="rl_planner" type="run_ariadne.sh" .../>
set -e

# 包根（=本脚本所在目录），rl_planner.py 在 scripts/ 下
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS Python 路径：只挂 /opt/ros（rospy/roslib/genpy/消息都在这）。
# 千万不要加 /usr/lib/python3/dist-packages —— 它含系统 numpy 1.17，会抢先被 skimage/torch 用，
# 导致 numpy ABI 冲突（Expected 88 got 80）。rospkg/catkin_pkg/yaml 已改由本 conda 环境提供。
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH

# 不缓冲 stdout/stderr：Timer 线程的异常 traceback 只上 stderr，缓冲会让它"消失"
# （2026-08-24 run3 教训：线程死了日志却毫无痕迹）。必须实时可见。
export PYTHONUNBUFFERED=1

# conda(ariadne) 的 python3.8，torch 在此
exec /home/pumpkin-db/miniconda3/envs/ariadne/bin/python "$PKG_DIR/scripts/rl_planner.py" "$@"
