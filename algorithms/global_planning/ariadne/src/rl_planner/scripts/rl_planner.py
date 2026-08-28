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
        self.enable_escape_recovery = rospy.get_param('~enable_escape_recovery', True)
        self.escape_min_distance = rospy.get_param(
            '~escape_min_distance', 2 * parameter.NODE_RESOLUTION)
        self.escape_required_displacement = rospy.get_param(
            '~escape_required_displacement', 3 * parameter.NODE_RESOLUTION)
        self.escape_required_map_growth = rospy.get_param(
            '~escape_required_map_growth', 20)
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

        # 停滞式完成判定状态（2026-08-25）：地图签名与最后变化时刻
        self._last_map_signature = None
        self._last_map_change_time = time.time()
        self.stalled_complete_seconds = rospy.get_param('~stalled_complete_seconds', 20.0)

        # save mode
        self.save_mode = False
        self.escape_mode = False
        self.escape_arrived = False
        self.escape_origin = None
        self.escape_start_known_cells = 0
        self.escape_excluded_nodes = set()
        self.policy_blocked_nodes = set()
        # STOP_REASON 诊断（2026-08-28，纯日志不改变行为）
        self._stop_state = {'reason': None, 't': 0.0}
        self._done_reason = ''

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
        while self.map_info is None or self.robot_location is None:
            pass

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

        # 停滞式完成判定（第一杠杆，2026-08-25）：记录地图最后一次「增长」的时刻。
        # 已知区域格数变化才算增长；纯 utility 判完成会在门洞前沿不可见时假停。
        known_cells = int((ros_map != parameter.UNKNOWN).sum())
        sig = (map_width, map_height, known_cells)
        if sig != self._last_map_signature:
            self._last_map_signature = sig
            self._last_map_change_time = time.time()

        t2 = time.time()
        # print("process map using {}".format(t2 - t1))

    def get_loc_callback(self, msg):
        if self.map_info is None:
            return
        self.robot_location = np.around(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]), 1)
        if self.start is None:

            x = np.array([(self.robot_location[0] // parameter.NODE_RESOLUTION) * parameter.NODE_RESOLUTION, (self.robot_location[0] // parameter.NODE_RESOLUTION + 1) * parameter.NODE_RESOLUTION])
            y = np.array([(self.robot_location[1] // parameter.NODE_RESOLUTION) * parameter.NODE_RESOLUTION, (self.robot_location[1] // parameter.NODE_RESOLUTION + 1) * parameter.NODE_RESOLUTION])
            t1, t2 = np.meshgrid(x, y)
            candidate_starts = np.vstack([t1.T.ravel(), t2.T.ravel()]).T
            dis_robot = np.linalg.norm(candidate_starts - self.robot_location, axis=1)
            sorted_candidate_starts = candidate_starts[np.argsort(dis_robot)]

            for start in sorted_candidate_starts:
                if is_free(start, self.map_info):
                    self.start = start
                    break

            assert self.start is not None, rospy.logwarn("can not find valid start point")

            self.start = np.around(self.start, 1)
            self.robot.node_manager = NodeManager(self.start)
            print("initialize quad tree at", self.start)
            print("initialize robot location at", self.robot_location)
        self.robot_cell = get_cell_position_from_coords(self.robot_location, self.map_info)

    def waypoint_wrapper(self, loc):
        way_point = PointStamped()
        way_point.header.frame_id = "map"
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

    def _log_stop(self, reason, robot_node_location=None):
        """STOP_REASON 诊断：ARiADNE 本拍不发布航点时记录原因与决策上下文。
        同原因 5s 节流；换原因立即记。纯日志，不改变任何控制流。"""
        now = time.time()
        if reason == self._stop_state['reason'] and now - self._stop_state['t'] < 5.0:
            return
        self._stop_state['reason'] = reason
        self._stop_state['t'] = now
        util_pos = -1
        valid = -1
        try:
            util_pos = int(sum(1 for u in self.robot.key_utility if u > 0))
        except Exception:
            pass
        try:
            if robot_node_location is not None:
                entry = self.robot.node_manager.nodes_dict.find(list(robot_node_location))
                if entry is not None:
                    valid = len(getattr(entry.data, 'neighbor_set', set()) or set())
        except Exception:
            valid = -2
        try:
            rospy.loginfo(
                "STOP_REASON=%s util_pos=%d valid_actions=%d blocked=%d "
                "stalled=%.1fs stall_done=%s escape_mode=%s escape_arrived=%s excluded=%d",
                reason, util_pos, valid, len(self.policy_blocked_nodes),
                now - self._last_map_change_time, self._done_reason,
                self.escape_mode, self.escape_arrived, len(self.escape_excluded_nodes))
        except Exception:
            pass

    def run(self, event=None):
        # no more planning if exploration is completed
        t1 = time.time()
        if self.done:
             self._log_stop('done_' + (self._done_reason or 'unknown'))
             return

        if self.escape_mode:
            if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                self._log_stop('escape_walking')
                return
            if self.next_waypoint_list:
                next_waypoint = self.next_waypoint_list.pop(0)
                while check_collision(self.robot_location, np.asarray(next_waypoint), self.map_info) is False \
                        and np.linalg.norm(self.robot_location - np.asarray(next_waypoint)) < \
                        (parameter.THR_NEXT_WAYPOINT + parameter.NODE_RESOLUTION) \
                        and self.next_waypoint_list:
                    next_waypoint = self.next_waypoint_list.pop(0)
                self.next_waypoint = np.asarray(next_waypoint)
                self.waypoint_pub.publish(self.waypoint_wrapper(self.next_waypoint))
                return

            self.escape_mode = False
            self.escape_arrived = True

        if self.save_mode:
            if np.linalg.norm(self.next_waypoint - self.robot_location) > parameter.THR_TO_WAYPOINT:
                self._log_stop('save_mode')
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
                    self._log_stop('oscillation_break')
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

        if self.escape_arrived:
            known_cells = int((self.map_info.map != parameter.UNKNOWN).sum())
            map_growth = known_cells - self.escape_start_known_cells
            displacement = np.linalg.norm(self.robot_location - self.escape_origin)
            if map_growth >= self.escape_required_map_growth \
                    and displacement >= self.escape_required_displacement:
                self.escape_arrived = False
                self.escape_origin = None
                self.escape_excluded_nodes = set()
                self.history_waypoint_list = []
                rospy.logwarn(
                    "Escape recovery completed: displacement=%.1fm, map_growth=%d; "
                    "switch back to RL", displacement, map_growth)
            else:
                min_origin_distance = max(
                    self.escape_required_displacement,
                    displacement + parameter.NODE_RESOLUTION)
                escape_path, escape_target, path_distance, frontier_distance = \
                    self.robot.node_manager.find_escape_path(
                        self.robot.location, self.escape_excluded_nodes,
                        self.escape_min_distance, self.escape_origin,
                        min_origin_distance)
                if escape_path:
                    self.escape_excluded_nodes.add(tuple(escape_target))
                    self.next_waypoint_list = escape_path
                    self.next_waypoint = np.asarray(self.next_waypoint_list.pop(0))
                    self.escape_mode = True
                    self.escape_arrived = False
                    rospy.logwarn(
                        "Continue escape recovery: target=(%.1f, %.1f), path=%.1fm, "
                        "frontier_offset=%.1fm, displacement=%.1fm, map_growth=%d",
                        escape_target[0], escape_target[1], path_distance,
                        frontier_distance, displacement, map_growth)
                    self.waypoint_pub.publish(self.waypoint_wrapper(self.next_waypoint))
                    return

                self.escape_arrived = False
                self.escape_origin = None
                self.escape_excluded_nodes = set()
                self.history_waypoint_list = []
                rospy.logwarn(
                    "Escape recovery stopped: no farther frontier-backed node "
                    "(displacement=%.1fm, map_growth=%d)", displacement, map_growth)

        # check the termination status
        if sum(self.robot.key_utility) == 0:
            # 停滞式完成判定（2026-08-25）：效用全零还不够，必须地图也静止满
            # STALLED_COMPLETE_SECONDS 秒才算完成。否则视为暂时停滞（门洞前沿不可见等），
            # 等地图更新后前沿自然重现。
            stalled_seconds = time.time() - self._last_map_change_time
            if stalled_seconds < self.stalled_complete_seconds:
                self._log_stop('utility_zero_waiting_map_growth', robot_node_location)
                return

            g = "\033[92m"
            n= "\033[0m"
            rospy.loginfo(f"{g}Exploration Completed{n}")
            self._done_reason = 'stalled_complete' if self.stalled_complete_seconds > 0 else 'utility_zero_immediate'
            self._log_stop('exploration_completed')
            self.done = True
            run_time = Float32()
            run_time.data = 0
            self.run_time_pub.publish(run_time)
            return

        # get rl observation
        t2 = time.time()
        observation = self.robot.get_observation(self.robot_location)
        t3 = time.time()

        # network inference to get next waypoint
        next_location, next_node_index = self.robot.select_next_waypoint(
            observation, excluded_positions=self.policy_blocked_nodes)

        self.next_waypoint_list.append(next_location)
        if len(self.history_waypoint_list) > 0:
            if (next_location[0], next_location[1]) != self.history_waypoint_list[-1]:
                self.history_waypoint_list.append((next_location[0], next_location[1]))
        else:
            self.history_waypoint_list.append((next_location[0], next_location[1]))

        # planning one more step if next node's utility is zero
        if self.robot.node_manager.nodes_dict.find(next_location.tolist()).data.utility == 0:
            next_observation = self.robot.get_next_observation(next_node_index, observation)
            next_next_location, _ = self.robot.select_next_waypoint(
                next_observation, excluded_positions=self.policy_blocked_nodes)

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

        if self.enable_escape_recovery and self.detect_waypoint_loop():
            loop_nodes = set(self.history_waypoint_list[-6:])
            self.policy_blocked_nodes.update(loop_nodes)
            if len(self.policy_blocked_nodes) > 32:
                self.policy_blocked_nodes = set(loop_nodes)
            self.escape_origin = self.robot_location.copy()
            self.escape_start_known_cells = int(
                (self.map_info.map != parameter.UNKNOWN).sum())
            self.escape_excluded_nodes = self.policy_blocked_nodes.copy()
            escape_path, escape_target, path_distance, frontier_distance = \
                self.robot.node_manager.find_escape_path(
                    self.robot.location, self.escape_excluded_nodes,
                    self.escape_min_distance, self.escape_origin,
                    self.escape_min_distance)
            if escape_path:
                self.escape_excluded_nodes.add(tuple(escape_target))
                self.next_waypoint_list = escape_path
                self.escape_mode = True
                rospy.logwarn(
                    "Switch to escape recovery: target=(%.1f, %.1f), path=%.1fm, "
                    "frontier_offset=%.1fm, steps=%d",
                    escape_target[0], escape_target[1], path_distance,
                    frontier_distance, len(escape_path))

        # get waypoint message
        self.next_waypoint = self.next_waypoint_list.pop(0)
        waypoint_msg = self.waypoint_wrapper(self.next_waypoint)

        # get planning time message
        run_time = Float32()
        run_time.data = t4 - t1

        # publish
        self.run_time_pub.publish(run_time)
        self.waypoint_pub.publish(waypoint_msg)
        self._stop_state['reason'] = None  # 恢复发布：下一次停止立即记录

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
        return False

    def visualize_graph(self):
        # visualize edges
        edges = Marker()
        edges.header.frame_id = 'map'
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
        header.frame_id = "map"
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
        header.frame_id = "map"
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
