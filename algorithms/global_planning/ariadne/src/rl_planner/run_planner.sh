#!/bin/bash
# rl_planner 启动包装（roslaunch type 入口）。
# 绕开两个已核实的环境坑（详见 third_party.md）：
#  1) conda activate 在本机会段错误 → 直接用 env 的绝对路径 python
#  2) 系统 python3.8 无 torch、conda base 是 3.13 进不了 ROS noetic(py3.8 ABI)
#     → 用 ariadne env（py3.8 + torch 2.3.1+cpu，机器上唯一非 base env）
# PYTHONPATH 只挂 ROS 自带 dist-packages；严禁挂 /usr/lib/python3/dist-packages
# （numpy 版本与 conda env 冲突的 ABI 坑）。
export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH}"
# Timer 线程异常只上 stderr 且缓冲，线程死掉后日志无痕——必须 UNBUFFERED 才能看到
export PYTHONUNBUFFERED=1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /home/pumpkin-db/miniconda3/envs/ariadne/bin/python "$SCRIPT_DIR/scripts/rl_planner.py"
