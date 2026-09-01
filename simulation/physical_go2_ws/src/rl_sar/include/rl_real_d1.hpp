/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_D1_HPP
#define RL_REAL_D1_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_d1.hpp"

// Agibot D1 SDK
#include "zsl-1/lowlevel.h"
#include "zsl-1/highlevel.h"

#if defined(USE_ROS1) && defined(USE_ROS)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#elif defined(USE_ROS2) && defined(USE_ROS)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#endif

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();

#if defined(USE_ROS2) && defined(USE_ROS)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // Agibot D1 SDK interface
    mc_sdk::LowLevel d1_lowlevel;
    mc_sdk::zsl_1::HighLevel d1_highlevel;
    mc_sdk::motorCmd d1_motor_cmd;
    mc_sdk::motorState* d1_motor_state;
    bool lowlevel_mode = false;  // false = highlevel, true = lowlevel

    // Mode switch and status
    void SwitchToLowLevel();
    void SwitchToHighLevel();
    uint32_t GetBatteryPower();
    uint32_t GetCurrentCtrlMode();

    // Joint mapping helper functions
    // SDK order: FL[0], FR[1], RL[2], RR[3] for each joint type (abad, hip, knee)
    // rl_sar order: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
    void MapStateFromSDK();
    void MapCommandToSDK();

    // others
    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;
    std::vector<float> mapped_joint_torques;

#if defined(USE_ROS1) && defined(USE_ROS)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#elif defined(USE_ROS2) && defined(USE_ROS)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

#endif // RL_REAL_D1_HPP
