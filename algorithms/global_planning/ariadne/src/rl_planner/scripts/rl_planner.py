#!/usr/bin/env python
# -*- coding: utf-8 -*-
import warnings
warnings.simplefilter("ignore", UserWarning)

import rospy
import rospkg
import numpy as np
import torch
import os
import time
from std_msgs.msg import Float32, Header
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from agent import Agent
from model import PolicyNet
from node_manager import NodeManager
from utils import *
import parameter


class Runner:
    def __init__(self):
        self.map_info = None
        self.device = 'cpu'
        self.step = 0

        # visualization
        self.publish_graph = rospy.get_param('~publish_graph', True)

        # map related
        parameter.CELL_SIZE = rospy.get_param('~map_resolution', parameter.CELL_SIZE)
        parameter.FREE = rospy.get_param('~map_free_value', parameter.FREE)
        parameter.OCCUPIED = rospy.get_param('~map_occupied_value', parameter.OCCUPIED)
        parameter.UNKNOWN = rospy.get_param('~map_unknown_value', parameter.UNKNOWN)

        # utility related
        parameter.SENSOR_RANGE = rospy.get_param('~sensor_range', parameter.SENSOR_RANGE)
        parameter.UTILITY_RANGE = rospy.get_param('~utility_range_factor', 0.5) * parameter.SENSOR_RANGE
        parameter.MIN_UTILITY = rospy.get_param('~min_utility', parameter.MIN_UTILITY)
        # go2-scan 四足适配：真实传感器投影下门洞前沿不可见问题（run2/run4 过早完成），
        # 开启后效用归因视线忽略 unknown（只挡 occupied）。见 third_party.md。
        parameter.LOS_IGNORE_UNKNOWN = rospy.get_param('~los_ignore_unknown', parameter.LOS_IGNORE_UNKNOWN)
        parameter.FRONTIER_CELL_SIZE = rospy.get_param('~frontier_downsample_factor', 1) * parameter.CELL_SIZE

        # graph related
        parameter.NODE_RESOLUTION = rospy.get_param('~node_resolution', parameter.NODE_RESOLUTION)
        parameter.CLUSTER_RANGE = rospy.get_param('~frontier_cluster_range', parameter.CLUSTER_RANGE)
        parameter.THR_NEXT_WAYPOINT = rospy.get_param('~next_waypoint_threshold', parameter.THR_NEXT_WAYPOINT)
        parameter.THR_GRAPH_HARD_UPDATE = rospy.get_param('~hard_update_threshold', parameter.THR_GRAPH_HARD_UPDATE)

        # replanning related
        parameter.THR_TO_WAYPOINT = rospy.get_param('~waypoint_threshold', parameter.THR_TO_WAYPOINT)
        parameter.AVOID_OSCILLATION = rospy.get_param('~avoid_waypoint_oscillation', parameter.AVOID_OSCILLATION)
        parameter.ENABLE_SAVE_MODE = rospy.get_param('~enable_save_mode', parameter.ENABLE_SAVE_MODE)
        parameter.ENABLE_DSTARLITE = rospy.get_param('~enable_dstarlite', parameter.ENABLE_DSTARLITE)
        frequency = rospy.get_param('~replanning_frequency', 2.5)

        # network model file
        self.model_file = "checkpoint.pth"

        # robot coordination wrt map frame
        self.robot_location = None

        # the grid occupied by the robot
        self.robot_cell = None

        # initialize robot planner
        self.robot = None
        self.init_agent()
        self.start = None

        # waypoint
        self.next_waypoint_list = []
        self.history_waypoint_list = []
        self.next_waypoint = None

        # termination status
        self.done = False

        # go2-scan 四足适配：停滞式完成判定。效用全零 ≠ 立即完成（真实投影下门洞前沿
        # 常常不可见，会过早冻结）；只有「效用全零 且 地图连续 stagnant_done_sec 秒无增长」
        # 才算真完成。期间继续正常规划，狗持续尝试。
        self.last_free_count = None
        self.last_map_change_time = None
        self.stagnant_done_sec = float(rospy.get_param('~stagnant_done_sec', 20.0))

        # save mode
        self.save_mode = False

        # subscribers
        rospy.Subscriber('/projected_map', OccupancyGrid, self.get_map_callback, queue_size=1)
        rospy.Subscriber('/state_estimation', Odometry, self.get_loc_callback, queue_size=1)

        # publishers
        self.waypoint_pub = rospy.Publisher('/way_point', PointStamped, queue_size=1)
        self.run_time_pub = rospy.Publisher('/runtime', Float32, queue_size=1)
        self.edge_pub = rospy.Publisher('/edge', Marker, queue_size=1)
        self.node_pub = rospy.Publisher('/node', PointCloud2, queue_size=1)
        self.frontier_pub = rospy.Publisher('/frontier', PointCloud2, queue_size=1)
        
        # get map and robot location
        # 四足适配（go2-scan）：纯 pass 忙等会几乎独占 GIL，饿死正在等地图/位姿消息的
        # 回调线程——而标志位恰恰要靠那些回调来置位，形成自锁。sleep 让出 GIL。
        while self.map_info is None or self.robot_location is None:
            rospy.sleep(0.05)

        rate = rospy.Rate(20)
        rospy.Timer(rospy.Duration(1 / frequency), self.run)
        try:
            rate.sleep()
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def get_map_callback(self, msg):
        t1 = time.time()
        delta = msg.info.resolution
        map_origin_x = msg.info.origin.position.x
        map_origin_y = msg.info.origin.position.y
        
        map_width = msg.info.width
        map_height = msg.info.height
        ros_map = np.array(np.array(msg.data).reshape(map_height, map_width).astype(np.int8))

        # padding the map with unknown area to avoid a frontier calculation issue
        pad_size = int(parameter.NODE_RESOLUTION // parameter.CELL_SIZE + 1)
        processed_map = np.pad(ros_map, ((pad_size, pad_size), (pad_size, pad_size)), 'constant', constant_values=parameter.UNKNOWN)
        map_origin_x -= delta * pad_size
        map_origin_y -= delta * pad_size
        robot_belief_map = processed_map

        self.map_info = MapInfo(robot_belief_map, map_origin_x, map_origin_y, delta)

        # go2-scan 四足适配：地图增长监测（停滞式完成判定的依据）
        free_count = int((robot_belief_map == parameter.FREE).sum())
        now = rospy.get_rostime()
        if self.last_free_count is None or free_count != self.last_free_count:
            self.last_free_count = free_count
            self.last_map_change_time = now
        t2 = time.time()
        # print("process map using {}".format(t2 - t1))

    def get_loc_callback(self, msg):
        if self.map_info is None:
            return
        self.robot_location = np.around(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]), 1)
        if self.start is None:

            # 四足适配（go2-scan）：
            # ① 原版只在 NODE_RESOLUTION 网格 4 个角点找起点，开机 octomap 未清图时 assert 死循环；
            # ② 【关键】起点必须落在【全局 NODE_RESOLUTION 格栅】（NODE_RESOLUTION 的整数倍）上！
            #    get_updating_node_coords 每拍生成的候选节点全部在该格栅上；若 start 偏离格栅
            #   （如机器人坐标 (-7.5,0.5) 这类半格），start 节点永远连不上图，
            #    remove_unconnected_nodes(start) 会把其余节点【全部清光】→ 图退化成孤点 →
            #    效用恒零 → waypoint=自身位置 → 狗站桩（2026-08-22 离线复现：19 节点建好后清除仅剩 1）。
            #    故候选直接取「机器人位置四舍五入到格栅 ± 6 格」，按距离排序逐个验 free。
            # ③ 找不到不崩，throttle 告警等下一帧地图。
            res = parameter.NODE_RESOLUTION
            base_x = int(round(float(self.robot_location[0]) / res))
            base_y = int(round(float(self.robot_location[1]) / res))
            cand = []
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    px, py = (base_x + dx) * res, (base_y + dy) * res
                    cand.append((((base_x + dx) * res - self.robot_location[0]) ** 2
                                 + ((base_y + dy) * res - self.robot_location[1]) ** 2, px, py))
            cand.sort()

            map_h, map_w = self.map_info.map.shape
            found_start = None
            for _, px, py in cand:
                cell = get_cell_position_from_coords(np.array([px, py]), self.map_info, check_negative=False)
                if 0 <= cell[0] < map_w and 0 <= cell[1] < map_h \
                        and self.map_info.map[cell[1], cell[0]] == parameter.FREE:
                    found_start = np.array([px, py])
                    break

            if found_start is None:
                rospy.logwarn_throttle(
                    10.0, "[go2-scan] 机器人 %s 周围暂无 free 格（octomap 尚未清图），等待中…"
                    % str(self.robot_location))
                return

            self.start = np.around(found_start, 1)
            self.robot.node_manager = NodeManager(self.start)
            print("initialize quad tree at", self.start)
            print("initialize robot location at", self.robot_location)
        self.robot_cell = get_cell_position_from_coords(self.robot_location, self.map_info)

    def waypoint_wrapper(self, loc):
        way_point = PointStamped()
        way_point.header.frame_id = "world"
        way_point.header.stamp = rospy.Time.now()
        way_point.point.x = loc[0]
        way_point.point.y = loc[1]
        return way_point

    def init_agent(self):
        policy_net = PolicyNet(parameter.NODE_INPUT_DIM, parameter.EMBEDDING_DIM).to(self.device)
        model_folder = os.path.join(rospkg.RosPack().get_path('rl_planner'), 'scripts/model')
        model_file = os.path.join(model_folder, self.model_file)
        policy_net.load_state_dict(torch.load(model_file, map_location=self.device)['policy_model'])

        self.robot = Agent(policy_net, self.device, self.publish_graph)

    def _find_recovery_waypoint(self):
        # go2-scan 四足适配（2026-08-24）：效用全零时的恢复导航目标。
        # 取最近的若干前沿点，沿「前沿→机器人」方向回退采样，返回第一个
        # 带净空的自由格（is_free_with_margin 保证与消毒器兼容；
        # 视线用 ignore_unknown 变体——朝未知边界走本就是探索的本意，
        # 只要求中间没有实墙）。每拍重算：前沿消失或抵达后自动更新。
        if self.map_info is None or self.robot_location is None:
            return None
        frontiers = get_frontier_in_map(self.map_info)
        if len(frontiers) == 0:
            return None
        frontiers = np.array(list(frontiers)).reshape(-1, 2)
        order = np.argsort(np.linalg.norm(frontiers - self.robot_location, axis=-1))
        for idx in order[:8]:
            f = frontiers[idx]
            dirv = self.robot_location - f
            dist = float(np.linalg.norm(dirv))
            if dist < 1e-6:
                continue
            diru = dirv / dist
            for step in np.arange(0.4, min(dist, 6.0), 0.4):
                cand = np.around(f + diru * step, 1)
                if is_free_with_margin(cand, self.map_info) \
                        and np.linalg.norm(cand - self.robot_location) > 1.0 \
                        and not check_collision_ignore_unknown(cand, self.robot_location, self.map_info):
                    return cand
        return None

    def run(self, event=None):
        # 四足适配（go2-scan）：Timer 回调里的异常只会打到 stderr（缓冲后迟迟不可见），
        # 且会永久杀死定时器线程——表现为"再无 waypoint、日志毫无痕迹"（run2/run3 实测）。
        # 这里兜底捕获并把完整 traceback 写进 rosout/日志文件，保证可诊断。
        try:
            self._run_impl(event)
        except Exception:
            import traceback
            rospy.logerr("[go2-scan] run() 异常，Timer 线程保住：\n" + traceback.format_exc())

    def _run_impl(self, event=None):
        # no more planning if exploration is completed
        t1 = time.time()
        if self.done:
             return

        # 四足适配（go2-scan）：起点还没找到时跳过本拍。原版会往下走到 self.start[0]，
        # start=None 时抛 TypeError 把 Timer 线程打死（stderr 只上屏、不进日志文件），
        # 表现为"永远不发 waypoint、狗一动不动"且日志里毫无痕迹（2026-08-21 实测）。
        if self.start is None:
            return

        if self.save_mode:
            if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                return
            else:
                if len(self.next_waypoint_list) > 0:
                    next_waypoint = self.next_waypoint_list.pop(0)
                    while check_collision(self.robot_location, np.array(next_waypoint), self.map_info) is False\
                            and np.linalg.norm(self.robot_location - np.array(next_waypoint)) < (parameter.THR_NEXT_WAYPOINT + parameter.NODE_RESOLUTION)\
                            and len(self.next_waypoint_list) > 0:
                        next_waypoint = self.next_waypoint_list.pop(0)
                    self.next_waypoint = next_waypoint

                    self.history_waypoint_list.append((self.next_waypoint[0], self.next_waypoint[1]))
                    waypoint_msg = self.waypoint_wrapper(self.next_waypoint)
                    self.waypoint_pub.publish(waypoint_msg)
                    run_time = Float32()
                    run_time.data = time.time() - t1

                    # publish
                    self.run_time_pub.publish(run_time)
                    return
                else:
                    self.save_mode = False
                    rospy.logwarn("Switch back to RL")


        # check and solve oscillation between two waypoints
        if parameter.AVOID_OSCILLATION and len(self.history_waypoint_list) > 4:
            if self.history_waypoint_list[-1] == self.history_waypoint_list[-3] and self.history_waypoint_list[-2] == self.history_waypoint_list[-4]:
                self.next_waypoint_list = []
                if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                    return

        # if planned one more step, use it
        if len(self.next_waypoint_list) > 0:
            if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                pass
            else:
                self.robot_location = self.next_waypoint
                self.next_waypoint = self.next_waypoint_list.pop(0)
                waypoint_msg = self.waypoint_wrapper(self.next_waypoint)
                self.waypoint_pub.publish(waypoint_msg)
        self.next_waypoint_list = []
        # print("robot location at", self.robot_location)

        # remove nodes on obstacles if any
        self.robot.node_manager.check_valid_node(self.robot_location, self.map_info)

        # find nearest node to the robot
        robot_node_location = self.robot_location
        if self.robot_location[0] != self.start[0] or self.robot_location[1] != self.start[1]:
            if self.robot.node_manager.nodes_dict.__len__() == 0:
                robot_node_location = self.start
            else:
                nearest_node = self.robot.node_manager.nodes_dict.nearest_neighbors(self.robot_location.tolist(), 1)[0]
                node_coords = nearest_node.data.coords
                robot_node_location = node_coords

        # updating planning graph
        self.robot.update_planning_state(self.map_info, robot_node_location)

        # check the termination status
        # go2-scan 四足适配（2026-08-24，停滞式完成判定）：
        # 上游「效用全零 => 立即 done」在真实传感器投影下会过早冻结——门洞前沿只有
        # 1~2 格、紧邻门框锯齿格或被墙遮挡时，全图节点可能同时零效用而世界远未探完
        # （run2/run4/run5 实录：~16% 覆盖即停）。改为：
        #   完成 = 效用全零 且 地图连续 stagnant_done_sec 秒无任何增长。
        # 未达条件时不 return，继续正常规划让狗持续尝试（地图一旦再增长即重置计时）。
        if sum(self.robot.key_utility) == 0:
            now = rospy.get_rostime()
            stagnant_for = (now - self.last_map_change_time).to_sec() \
                if self.last_map_change_time is not None else 0.0
            if stagnant_for >= self.stagnant_done_sec:
                g = "\033[92m"
                n = "\033[0m"
                rospy.loginfo(f"{g}Exploration Completed{n} "
                              f"(utility 全零且地图停滞 {stagnant_for:.0f}s)")
                self.done = True
                run_time = Float32()
                run_time.data = 0
                self.run_time_pub.publish(run_time)
                return
            rospy.logwarn_throttle(
                10.0, "[go2-scan] utility 全零但未达停滞阈值(%.0fs/%.0fs)，继续探索"
                % (stagnant_for, self.stagnant_done_sec))
            # 恢复导航：策略常反复选中贴障碍节点、被消毒器逐拍抑制 → 狗原地不动、
            # 地图永不增长 → 停滞计时走完误判完成（run7/run9 实录）。效用全零期间
            # 直接朝「最近前沿的可达自由格」发恢复航点；前沿消失或抵达后自动重算。
            recovery = self._find_recovery_waypoint()
            if recovery is not None:
                rospy.logwarn_throttle(5.0, "[go2-scan] 恢复导航 -> %s" % str(np.around(recovery, 2)))
                self.next_waypoint = recovery
                self.waypoint_pub.publish(self.waypoint_wrapper(recovery))
                return

        # get rl observation
        t2 = time.time()
        observation = self.robot.get_observation(self.robot_location)
        t3 = time.time()

        # network inference to get next waypoint
        next_location, next_node_index = self.robot.select_next_waypoint(observation)

        self.next_waypoint_list.append(next_location)
        if len(self.history_waypoint_list) > 0:
            if (next_location[0], next_location[1]) != self.history_waypoint_list[-1]:
                self.history_waypoint_list.append((next_location[0], next_location[1]))
        else:
            self.history_waypoint_list.append((next_location[0], next_location[1]))

        # planning one more step if next node's utility is zero
        if self.robot.node_manager.nodes_dict.find(next_location.tolist()).data.utility == 0:
            next_observation = self.robot.get_next_observation(next_node_index, observation)
            next_next_location, _ = self.robot.select_next_waypoint(next_observation)

            # if next waypoint is too close, go to the next next waypoint
            if np.linalg.norm(next_location - self.robot_location) < parameter.NODE_RESOLUTION:
                self.next_waypoint_list = []

            self.next_waypoint_list.append(next_next_location)

        t4 = time.time()
        # print("next waypoint at", next_location)
        # print("update planning state using {}".format(t2 - t1))
        # print("prepare tensor input using {}".format(t3 - t2))
        # print("neural network inference using {}".format(t4-t3))

        # if rl gets stuck, go to nearest frontier
        if parameter.ENABLE_SAVE_MODE:
            if self.detect_waypoint_loop():
                self.next_waypoint_list = self.robot.node_manager.path_to_nearest_frontier
                self.save_mode = True
                rospy.logwarn("Switch to save mode")

        # get waypoint message
        self.next_waypoint = self.next_waypoint_list.pop(0)

        # 四足适配（go2-scan）目标消毒：ARiADNE 的 0.4m 投影图分辨率粗，RL 选出的节点可能
        # 落在离墙脸仅 0.1~0.3m 的格子里（本格 free、邻格全是墙）。下游 SCAN-Planner 的精细
        # 地图+膨胀会判该目标 "occupied"，并把目标回退到机器人自身位置 → 狗站桩；ARiADNE 收不到
        # 任何拒收反馈，utility 全 0 下每拍重发同一目标 → 永久死锁（2026-08-22 实测：
        # 目标(-5.5,0.5)距墙脸0.1m，狗冻结在(-6.2,0.5)，SCAN 刷屏 adjustGlobalTargetIfOccupied）。
        # 消毒规则：目标及其 3×3 邻域（0.4m 图上 ≈±0.4m 净空）全 free 才放行；
        # 否则改发「沿线最后一个带净空的逼近点」；连逼近点都没有则本拍不发。
        # 注：上游 save_mode 兜底在此场景无效——path_to_nearest_frontier 只对 utility>0
        #     的节点计算（node_manager.get_rarefied_graph），utility 全 0 时恒为 None。
        if not is_free_with_margin(self.next_waypoint, self.map_info):
            approach = find_approach_point(np.asarray(self.robot_location, dtype=float),
                                           np.asarray(self.next_waypoint, dtype=float), self.map_info)
            if approach is None or np.linalg.norm(approach - self.robot_location) < 0.8:
                rospy.logwarn_throttle(
                    10.0, "[go2-scan] 目标 %s 太贴障碍且无安全逼近点，本拍不发" % str(np.around(self.next_waypoint, 2)))
                return
            rospy.logwarn_throttle(
                10.0, "[go2-scan] 目标 %s 太贴障碍，改发逼近点 %s"
                % (str(np.around(self.next_waypoint, 2)), str(approach)))
            self.next_waypoint = approach
        waypoint_msg = self.waypoint_wrapper(self.next_waypoint)

        # get planning time message
        run_time = Float32()
        run_time.data = t4 - t1

        # publish
        self.run_time_pub.publish(run_time)
        self.waypoint_pub.publish(waypoint_msg)

        self.step += 1
        if self.publish_graph:
            self.visualize_graph()

    def detect_waypoint_loop(self, max_length=6):
        if len(self.history_waypoint_list) < max_length:
            return False

        waypoint_list_to_check = self.history_waypoint_list[-max_length:]
        loop =[]
        for i, waypoint in enumerate(waypoint_list_to_check[:-1]):
            if waypoint == waypoint_list_to_check[-1]:
                loop = waypoint_list_to_check[i:]

        if loop:
            loop_length = len(loop)
            if len(self.history_waypoint_list) < 2 * loop_length + 1:
                return False
            waypoint_list_to_check2 = self.history_waypoint_list[-max_length-loop_length+1:-loop_length+1]
            # print("length check", waypoint_list_to_check2, loop)
            loop2 = []
            for i, waypoint in enumerate(waypoint_list_to_check2[:-1]):
                if waypoint == waypoint_list_to_check2[-1]:
                    loop2 = waypoint_list_to_check2[i:]
                    break
            if loop2:
                return True
            else:
                return False

    def visualize_graph(self):
        # visualize edges
        edges = Marker()
        edges.header.frame_id = 'world'
        edges.header.stamp = rospy.Time.now()
        edges.type = Marker.LINE_LIST
        edges.scale.x = 0.1
        edges.color.r = 0.0
        edges.color.g = 0.6
        edges.color.b = 0.0
        edges.color.a = 1.0
        edges.pose.orientation.x = 0.0
        edges.pose.orientation.y = 0.0
        edges.pose.orientation.z = 0.0
        edges.pose.orientation.w = 1.0

        for coords in self.robot.key_node_coords:
            node = self.robot.node_manager.key_node_dict[(coords[0], coords[1])]
            for neighbor_coords in node.neighbor_set:
                start = Point()
                start.x = coords[0]
                start.y = coords[1]
                end_coords = (neighbor_coords - coords) / 2 + coords
                end = Point()
                end.x = end_coords[0]
                end.y = end_coords[1]
                edges.points.append(start)
                edges.points.append(end)

        self.edge_pub.publish(edges)

        # visualize nodes
        nodes = []
        for node_coords, utility in zip(self.robot.key_node_coords, self.robot.key_utility):
            nodes.append((node_coords[0], node_coords[1], 0.0, utility))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        nodes = point_cloud2.create_cloud(header, fields, nodes)
        self.node_pub.publish(nodes)

        # visualize frontiers
        frontiers = []
        for frontier in self.robot.frontier:
            frontiers.append((frontier[0], frontier[1], 0))
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        frontiers = point_cloud2.create_cloud(header, fields, frontiers)
        self.frontier_pub.publish(frontiers)
        

if __name__ == '__main__':
    rospy.init_node('rl_planner', anonymous=True)
    rl_runner = Runner()
