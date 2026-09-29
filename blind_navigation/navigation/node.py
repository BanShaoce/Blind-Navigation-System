"""ROS 2 导航节点与避障状态机。"""

from __future__ import annotations

import math
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import Costmap
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from blind_navigation.common.protocol import NavigationEvent, UdpEndpoints
from blind_navigation.common.settings import NavigationSettings
from blind_navigation.navigation.costmap import LocalCostmap
from blind_navigation.navigation.geometry import (
    distance_to_segment,
    normalize_angle,
    transform_point,
)
from blind_navigation.navigation.models import NavigationState, compress_path
from blind_navigation.navigation.motors import MotorController


class BlindGuideNode(Node):
    def __init__(
        self,
        motors: MotorController,
        settings: NavigationSettings | None = None,
        endpoints: UdpEndpoints | None = None,
    ):
        super().__init__("blind_guide_node")
        defaults = settings or NavigationSettings()
        self.settings = NavigationSettings(
            angle_threshold_deg=self._float_parameter(
                "guidance.angle_threshold_deg", defaults.angle_threshold_deg
            ),
            direction_threshold_deg=self._float_parameter(
                "guidance.direction_threshold_deg", defaults.direction_threshold_deg
            ),
            waypoint_reach_threshold=self._float_parameter(
                "guidance.waypoint_reach_threshold",
                defaults.waypoint_reach_threshold,
            ),
            off_track_limit=self._float_parameter(
                "guidance.off_track_limit", defaults.off_track_limit
            ),
            off_track_check_interval=self._float_parameter(
                "guidance.off_track_check_interval",
                defaults.off_track_check_interval,
            ),
            path_sample_distance=self._float_parameter(
                "guidance.path_sample_distance", defaults.path_sample_distance
            ),
        )
        self.endpoints = endpoints or UdpEndpoints()
        self.motors = motors

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()

        self.nav_lock = threading.Lock()
        self.path_segments =[]
        self.current_segment_index = 0
        self.is_navigating = False
        self.waiting_for_turn = False
        self.is_waiting_for_clear = False
        self.final_goal_pose = None
        self.trigger_obstacle_pt = None   # 记录触发避障的障碍物点 (odom 系)
        self.last_turn_direction = None
        self.last_off_track_check_time = 0.0
        self.has_warned_off_track = False
        self.current_goal_pose = None
        self.current_vibrate_state = None
        self.last_collision_state = False
        self.last_collision_direction = None
        # 局部代价地图（用于实时避障）
        self.avoidance_clear_count = 0   # 连续无碰撞次数
        self.temp_goal_active = False
        self.original_goal_pose = None
        self.best_avoid_direction = None
        self._nav_start_lock = False
        self.last_obstacle_clear_time = 0.0
        self.waiting_clear_count = 0
        self.temp_clear_count = 0
        self.off_track_fail_count = 0
        self.max_off_track_retries = 3
        self.local_costmap = LocalCostmap()
        self.costmap_sub = self.create_subscription(
            Costmap,
            '/local_costmap/costmap_raw',
            self.local_costmap.update,
            10
        )
        # 状态机
        self.nav_state = NavigationState.IDLE
        self.audio_blocked = False        # True 时禁止 speak()
        self.perception_active = False    # 感知模块正在对话
        # UDP 发送 socket（向感知模块发送状态）
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_dest = (self.endpoints.host, self.endpoints.event_port)

    def _float_parameter(self, name: str, default: float) -> float:
        self.declare_parameter(name, default)
        return float(self.get_parameter(name).value)

    def send_status(self, status: str | NavigationEvent) -> None:
        try:
            self.udp_sock.sendto(str(status).encode(), self.udp_dest)
        except OSError as exc:
            self.get_logger().warning(f"导航事件发送失败: {exc}")

    def vibrate_pattern(self, pattern_type: str) -> None:
        if pattern_type == "stop":
            self.motors.stop()
            self.current_vibrate_state = "stop"
            return
        if self.nav_state is not NavigationState.NAVIGATING:
            return
        if self.current_vibrate_state == pattern_type and pattern_type != "replan":
            return

        self.current_vibrate_state = pattern_type
        self.get_logger().info(f"触觉指引切换为: {pattern_type}")
        threading.Thread(
            target=self.motors.activate,
            args=(pattern_type,),
            daemon=True,
            name="motor-guidance",
        ).start()

    def transform_point_map_to_odom(self, x_map, y_map):
        """通过 TF 查询 map->odom 变换，手动将坐标转换到 odom 坐标系"""
        try:
            # 获取 map 在 odom 坐标系中的位姿
            t = self.tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            # 计算旋转角度
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            # 坐标转换：先旋转，再平移
            return transform_point((x_map, y_map), (tx, ty), yaw)
        except TransformException:
            # TF 变换尚未就绪
            return None, None
        except Exception as exc:
            self.get_logger().warning(f"map 到 odom 坐标转换失败: {exc}")
            return None, None

    def speak(self, text_or_code: str | NavigationEvent) -> None:
        """
        统一语音播报接口 —— 现在改为发送 UDP 事件码给感知模块。
        如果传入的是已知事件码（如 ARRIVED），直接发送；
        如果是普通文本，将作为通用语音请求发送（仅调试用）。
        """
        if self.audio_blocked:
            self.get_logger().debug("感知正在播报，导航事件将由感知进程排队处理")
        self.get_logger().info(f"通知感知模块: {text_or_code}")
        self.send_status(text_or_code)

    def get_current_pose(self):
        """ 通过TF查询base_link 在 map 坐标系下的位姿（x, y, 航向角 yaw） """
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except TransformException:
            return None, None, None
        except Exception as exc:
            self.get_logger().debug(f"map 位姿查询失败: {exc}")
            return None, None, None

    def get_current_pose_odom(self):
        """返回 odom 坐标系下的 base_link 位姿，用于查询 local_costmap"""
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except TransformException:
            return None, None, None
        except Exception as exc:
            self.get_logger().debug(f"odom 位姿查询失败: {exc}")
            return None, None, None

    def process_path(self, nav_path):
        """ 找路，把大量路径点压缩成关键点，合并角度变换小的段，计算每个段的距离和方向 """
        if not nav_path or len(nav_path.poses) < 2:
            return False
        points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in nav_path.poses
        ]
        self.path_segments = compress_path(
            points,
            sample_distance=self.settings.path_sample_distance,
            angle_threshold_deg=self.settings.angle_threshold_deg,
        )
        return bool(self.path_segments)

    def navigation_loop(self):
        OBSTACLE_COOLDOWN = 1.5            # 避障退出后至少等待几秒再检测，防止反复触发

        while rclpy.ok():
            if not self.is_navigating:
                time.sleep(0.5)
                continue
            if self.nav_state is NavigationState.PERCEPTION_ACTIVE:
                time.sleep(0.5)
                continue
            # ====== 临时目标到达检测 ======
            if self.temp_goal_active:
                # 先获取一次位姿，后面只用这个值（避免多次查询）
                x, y, yaw = self.get_current_pose()
                if x is None:
                    time.sleep(0.2)
                    continue

                # ---- 1. 障碍物消失检测（最高优先级） ----
                if self.trigger_obstacle_pt is not None:
                    trig_cost = self.local_costmap.cost_at(*self.trigger_obstacle_pt)
                    if trig_cost >= 0 and trig_cost < LocalCostmap.SAFE_COST:
                        self.temp_clear_count += 1
                        if self.temp_clear_count >= 3:
                            self.get_logger().info("✅ 临时导航中，障碍物已消失，返回原路径")
                            self.temp_goal_active = False
                            self.temp_clear_count = 0
                            self.trigger_obstacle_pt = None
                            if self.original_goal_pose:
                                self.current_goal_pose = self.original_goal_pose
                                self.original_goal_pose = None
                                self.start_navigation(self.current_goal_pose, is_replan=True)
                            continue
                    else:
                        self.temp_clear_count = 0          # 代价仍高，清零计数
                    # 仅当地图本身有效且障碍物点滚出局部地图时，才恢复全局导航。
                    if (
                        trig_cost == LocalCostmap.UNKNOWN
                        and self.local_costmap.ready
                        and not self.local_costmap.contains(*self.trigger_obstacle_pt)
                    ):
                        self.get_logger().info("❗ 临时导航中，障碍物点已离开局部地图，返回全局导航")
                        self.temp_goal_active = False
                        self.temp_clear_count = 0
                        self.trigger_obstacle_pt = None
                        if self.original_goal_pose:
                            original_goal = self.original_goal_pose
                            self.original_goal_pose = None
                            self.start_navigation(original_goal, is_replan=True)
                        continue
                # ---- 2. 临时路径段已走完 ----
                if self.current_segment_index >= len(self.path_segments):
                    self.temp_goal_active = False
                    self.get_logger().info("✅ 到达临时点，重新规划至最终目标")
                    if self.original_goal_pose:
                        original_goal = self.original_goal_pose
                        self.original_goal_pose = None
                        self.start_navigation(original_goal, is_replan=True)
                    else:
                        self.finish_navigation()
                    continue

                # ---- 3. 正常引导：转弯 / 直行 ----
                current_seg = self.path_segments[self.current_segment_index]

                # 3a. 转弯引导
                if self.waiting_for_turn:
                    diff = normalize_angle(current_seg.heading - yaw)
                    diff_deg = math.degrees(diff)
                    if abs(diff_deg) <= self.settings.direction_threshold_deg:
                        # 转弯完成
                        self.waiting_for_turn = False
                        self.vibrate_pattern('straight')
                        self.last_turn_direction = None
                        # 完成转弯后至少要等一小段时间再继续，防止立刻误触段更新
                        time.sleep(0.2)
                        continue
                    else:
                        # 继续引导转弯
                        new_dir = 'left' if diff_deg > 0 else 'right'
                        # 滞回：只有当方向确实改变且偏差 >30° 才切换
                        if self.last_turn_direction is None or (
                            new_dir != self.last_turn_direction
                            and abs(diff_deg) > 30.0
                        ):
                            self.last_turn_direction = new_dir
                            self.vibrate_pattern(new_dir)
                        time.sleep(0.2)
                        continue

                # 3b. 直行 → 检查是否到达当前段终点
                dist_to_end = math.dist((x, y), current_seg.end_point)
                if dist_to_end < self.settings.waypoint_reach_threshold:
                    self.current_segment_index += 1
                    self.has_warned_off_track = False
                    # 如果还有下一段，播放新段的引导指令
                    if self.current_segment_index < len(self.path_segments):
                        self.play_next_segment_instruction(yaw)
                    continue

                # 3c. 什么都没有发生，继续当前方向
                time.sleep(0.2)
                continue   # 继续保持在临时导航模式
            x, y, yaw = self.get_current_pose()
            if x is None:
                time.sleep(0.2)
                continue

            now = time.time()

            if self.is_waiting_for_clear:
                x_odom, y_odom, _ = self.get_current_pose_odom()
                if x_odom is not None:
                    collision, _, _ = self._check_path_collision(x_odom, y_odom)
                    trig_cost = (
                        self.local_costmap.cost_at(*self.trigger_obstacle_pt)
                        if self.trigger_obstacle_pt
                        else LocalCostmap.UNKNOWN
                    )
                    if (
                        not collision
                        and trig_cost != LocalCostmap.UNKNOWN
                        and trig_cost < LocalCostmap.SAFE_COST
                    ):
                        self.waiting_clear_count += 1
                        if self.waiting_clear_count >= 3:
                            self.is_waiting_for_clear = False
                            self.waiting_clear_count = 0
                            self.get_logger().info("✅ 路径已畅通，重新规划全局路径")
                            self.speak(NavigationEvent.PATH_CLEAR)
                            if self.final_goal_pose:
                                success = self.start_navigation(self.final_goal_pose)
                            else:
                                success = self.start_navigation(self.current_goal_pose)
                            if not success:
                                self.speak(NavigationEvent.PLAN_FAILED)
                                self.is_navigating = False
                                self.vibrate_pattern('stop')
                            continue
                    else:
                        self.waiting_clear_count = 0
                time.sleep(0.2)
                continue

            # ============ 模式 2：正常路径引导 ============
            # 检查是否已走完所有段
            if self.current_segment_index >= len(self.path_segments):
                self.finish_navigation()
                continue
            current_seg = self.path_segments[self.current_segment_index]
            # --- 处理转弯等待状态（不受冷却影响） ---
            if self.waiting_for_turn:
                diff = normalize_angle(current_seg.heading - yaw)
                diff_deg = math.degrees(diff)
                if abs(diff_deg) <= self.settings.direction_threshold_deg:
                    self.waiting_for_turn = False
                    self.vibrate_pattern('straight')
                else:
                    if diff_deg > 0:
                        self.vibrate_pattern('left')
                    else:
                        self.vibrate_pattern('right')
                    time.sleep(0.2)
                    continue    # 转弯中，跳过后续处理
            # --- 偏离检测（不受冷却影响） ---
            if self.check_if_off_track((x, y), current_seg, yaw):
                self.handle_off_track()
                continue
            # --- 局部障碍检测（仅在冷却期过后进行） ---
            if not self.waiting_for_turn and not self.temp_goal_active:
                if now - self.last_obstacle_clear_time >= OBSTACLE_COOLDOWN:
                    x_odom, y_odom, _ = self.get_current_pose_odom()
                    if x_odom is not None:
                        collision, direction, obstacle_pt = self._check_path_collision(
                            x_odom, y_odom
                        )
                        # 只在状态变化时打印
                        if (
                            collision != self.last_collision_state
                            or direction != self.last_collision_direction
                        ):
                            self.last_collision_state = collision
                            self.last_collision_direction = direction
                            if collision:
                                self.get_logger().info(f"🚨 路径被阻挡，建议向 {direction} 绕行")
                            else:
                                self.get_logger().info("路径畅通")
                        if collision:
                            self.trigger_obstacle_pt = obstacle_pt
                            # 获取当前路径段的方向
                            if self.current_segment_index < len(self.path_segments):
                                path_heading = self.path_segments[
                                    self.current_segment_index
                                ].heading
                            else:
                                path_heading = math.atan2(
                                    self.current_goal_pose.pose.position.y - y,
                                    self.current_goal_pose.pose.position.x - x
                                )
                            # 启动临时绕行（内部会自动搜索两个方向）
                            if self._start_temp_navigation(obstacle_pt, path_heading, direction):
                                continue
                            else:
                                # 两侧都无法绕行 → 进入等待
                                self.get_logger().info("⚠️ 暂时无法通过，请等待")
                                self.speak(NavigationEvent.BLOCKED)
                                self.vibrate_pattern('stop')
                                self.final_goal_pose = self.current_goal_pose
                                self.is_waiting_for_clear = True
                                self.trigger_obstacle_pt = obstacle_pt
                                self.last_obstacle_clear_time = time.time() + 2.0
                                continue
                # 如果冷却未到，不做任何障碍检测，直接继续后面的段完成检查
            # --- 检查是否到达当前段终点 ---
            dist_to_end = math.dist((x, y), current_seg.end_point)
            if dist_to_end < self.settings.waypoint_reach_threshold:
                self.current_segment_index += 1
                self.has_warned_off_track = False
                if self.temp_goal_active and self.current_segment_index >= len(self.path_segments):
                    pass
                else:
                    self.play_next_segment_instruction(yaw)
                continue
            # 正常监测间隔
            time.sleep(0.2)


    def transform_point_odom_to_map(self, x_odom, y_odom):
        """将 odom 系下的点转换到 map 系，利用 TF 查询 odom->map"""
        try:
            # 注意：这里查询的是 odom 在 map 下的变换，所以是 lookup_transform('map', 'odom', ...)
            t = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            return transform_point((x_odom, y_odom), (tx, ty), yaw)
        except Exception as exc:
            self.get_logger().debug(f"odom 到 map 坐标转换失败: {exc}")
            return None

    def _make_temp_pose(self, x, y, heading):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # 方向沿用原路径方向，不强制对齐，避免盲人原地转圈
        quat = self._yaw_to_quaternion(heading)
        pose.pose.orientation = quat
        return pose

    def _yaw_to_quaternion(self, yaw):
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.w = math.cos(yaw/2)
        q.z = math.sin(yaw/2)
        return q

    def _start_temp_navigation(self, obstacle_pt, path_heading, preferred_direction):
        """ 尝试从 preferred_direction 开始寻找安全点并规划，若失败则自动尝试反方向。返回 True 表示成功启动临时导航。 """
        # 优先方向
        temp_pt = self.local_costmap.find_avoid_point(
            obstacle_pt, path_heading, preferred_direction
        )
        if temp_pt is None:
            # 反方向
            opposite = 'right' if preferred_direction == 'left' else 'left'
            temp_pt = self.local_costmap.find_avoid_point(
                obstacle_pt, path_heading, opposite
            )
            direction_used = opposite
        else:
            direction_used = preferred_direction

        if temp_pt is None:
            return False   # 两侧都找不到安全点

        # 转换到 map 坐标系
        map_pt = self.transform_point_odom_to_map(temp_pt[0], temp_pt[1])
        if map_pt is None:
            return False

        temp_pose = self._make_temp_pose(map_pt[0], map_pt[1], path_heading)
        saved_original = self.current_goal_pose

        # 尝试规划并导航
        if self.start_navigation(temp_pose, is_replan=True):
            self.get_logger().info(f"✅ 临时绕行点已启动: ({map_pt[0]:.2f}, {map_pt[1]:.2f})")
            self.original_goal_pose = saved_original
            self.temp_goal_active = True
            return True

        # 第一次尝试失败（可能被锁或规划器拒绝）
        # 若之前用的是首选方向，再试试反方向
        if direction_used == preferred_direction:
            opposite = 'right' if preferred_direction == 'left' else 'left'
            temp_pt2 = self.local_costmap.find_avoid_point(
                obstacle_pt, path_heading, opposite
            )
            if temp_pt2 is not None:
                map_pt2 = self.transform_point_odom_to_map(temp_pt2[0], temp_pt2[1])
                if map_pt2 is not None:
                    temp_pose2 = self._make_temp_pose(map_pt2[0], map_pt2[1], path_heading)
                    if self.start_navigation(temp_pose2, is_replan=True):
                        self.original_goal_pose = saved_original
                        self.temp_goal_active = True
                        return True

        return False   # 所有尝试均失败

    def _check_path_collision(self, robot_x_odom, robot_y_odom):
        if self.current_segment_index >= len(self.path_segments):
            return False, None, None

        seg = self.path_segments[self.current_segment_index]
        end_x_odom, end_y_odom = self.transform_point_map_to_odom(
            seg.end_point[0], seg.end_point[1])

        if end_x_odom is None:
            self.get_logger().warning("TF 转换失败，本次碰撞检测跳过")
            return False, None, None

        dx = end_x_odom - robot_x_odom
        dy = end_y_odom - robot_y_odom
        total_dist = math.sqrt(dx*dx + dy*dy)
        if total_dist < 0.3:
            return False, None, None

        path_angle = math.atan2(dy, dx)
        step = 0.2
        num_points = min(int(total_dist / step), 20)
        if num_points == 0:
            return False, None, None

        MIN_CHECK_DIST = 0.5

        lateral_shifts = []       # 记录每一个障碍物采样点的横向偏移
        first_obstacle_pt = None

        for i in range(1, num_points + 1):
            frac = i * step / total_dist
            dist_from_robot = frac * total_dist
            if dist_from_robot < MIN_CHECK_DIST:
                continue

            px = robot_x_odom + dx * frac
            py = robot_y_odom + dy * frac
            center_cost = self.local_costmap.cost_at(px, py)

            if center_cost > LocalCostmap.OBSTACLE_COST:
                # 计算相对机器人的向量
                vec_x = px - robot_x_odom
                vec_y = py - robot_y_odom
                # 横向偏移 = 向量与路径左向量的点积（左为正）
                perp_left_x = -math.sin(path_angle)
                perp_left_y =  math.cos(path_angle)
                lateral = vec_x * perp_left_x + vec_y * perp_left_y
                lateral_shifts.append(lateral)

                if first_obstacle_pt is None:
                    first_obstacle_pt = (px, py)

        if lateral_shifts:
            # 平均偏移 > 0 表示障碍物偏左 → 应向右绕
            avg_lateral = sum(lateral_shifts) / len(lateral_shifts)
            direction = 'right' if avg_lateral > 0 else 'left'
            return True, direction, first_obstacle_pt
        else:
            return False, None, None

    def play_next_segment_instruction(self, current_yaw):
        if len(self.path_segments) == 0:
            self.get_logger().error("❌ path_segments 意外为空，取消当前导航")
            self.finish_navigation()
            return
        if self.current_segment_index >= len(self.path_segments):
            self.finish_navigation()
            return
        next_seg = self.path_segments[self.current_segment_index]
        diff = normalize_angle(next_seg.heading - current_yaw)
        diff_deg = math.degrees(diff)

        if abs(diff_deg) <= self.settings.angle_threshold_deg:
            self.vibrate_pattern('straight')
            self.waiting_for_turn = False
            self.last_turn_direction = None
        else:
            new_dir = 'left' if diff_deg > 0 else 'right'
            # 滞回：只有当方向真正改变且偏差足够大时才切换振动
            if self.last_turn_direction is None or (
                new_dir != self.last_turn_direction and abs(diff_deg) > 15.0
            ):
                self.last_turn_direction = new_dir
                self.vibrate_pattern(new_dir)
            self.waiting_for_turn = True

    def finish_navigation(self):
        self.nav_state = NavigationState.IDLE
        self.perception_active = False
        self.temp_goal_active = False
        self.original_goal_pose = None
        self.vibrate_pattern('stop')
        self.speak(NavigationEvent.ARRIVED)
        self.is_navigating = False

    def abort_navigation(self, event: NavigationEvent) -> None:
        self.nav_state = NavigationState.IDLE
        self.perception_active = False
        self.temp_goal_active = False
        self.original_goal_pose = None
        self.is_navigating = False
        self.vibrate_pattern("stop")
        self.speak(event)

    def check_if_off_track(self, user_pos, segment, current_yaw):
        if not self.path_segments:
            return False
        now = time.time()
        if now - self.last_off_track_check_time < self.settings.off_track_check_interval:
            return False
        self.last_off_track_check_time = now
        if self.has_warned_off_track:
            return False

        dist = distance_to_segment(user_pos, segment.start_point, segment.end_point)

        # 1. 与当前路径段的距离是否超过配置阈值
        if dist > self.settings.off_track_limit:
            return True

        # 2. 如果盲人没在转弯，但他面朝的方向跟路线差了超过 90 度（走反了）
        if not self.waiting_for_turn:
            diff = normalize_angle(segment.heading - current_yaw)
            if abs(math.degrees(diff)) > 90.0:
                self.get_logger().warning("检测到用户方向与路线相反，开始重规划")
                return True

        return False

    def handle_off_track(self):
        self.has_warned_off_track = True
        self.get_logger().warning("检测到路线偏离，开始重规划")
        if self.current_goal_pose:
            success = self.start_navigation(self.current_goal_pose, is_replan=True)
            if success:
                self.off_track_fail_count = 0
            else:
                self.off_track_fail_count += 1
                if self.off_track_fail_count >= self.max_off_track_retries:
                    self.abort_navigation(NavigationEvent.OFF_TRACK_STOP)

    def start_navigation(self, goal_pose, is_replan=False):
        if self._nav_start_lock:
            self.get_logger().warning("start_navigation 重入被阻止")
            return False
        self._nav_start_lock = True
        self.waiting_clear_count = 0

        try:
            # 重置避障冷却与状态，确保新导航可以立即检测障碍物
            # 但不要清空 trigger_obstacle_pt，除非是真正的新导航（非 replan）
            self.last_obstacle_clear_time = 0.0
            self.avoidance_clear_count = 0
            self.best_avoid_direction = None
            self.last_collision_state = False
            self.last_collision_direction = None
            self.is_waiting_for_clear = False
            self.final_goal_pose = None
            # 只有在完全新的导航指令（非重规划）时，才清除记录的障碍物点
            if not is_replan:
                self.trigger_obstacle_pt = None

            # 如果正在执行临时避障，取消临时导航
            if self.temp_goal_active:
                self.temp_goal_active = False
                self.original_goal_pose = None

            self.current_goal_pose = goal_pose
            x, y, yaw = self.get_current_pose()
            if x is None:
                self.speak(NavigationEvent.PLAN_FAILED)
                return False
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'map'
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose.position.x = x
            start_pose.pose.position.y = y
            start_pose.pose.orientation.w = 1.0
            # 如果是重规划（含临时导航、障碍消失后回归），先取消前一个规划任务
            if is_replan:
                self.last_obstacle_clear_time = time.time()
                try:
                    self.navigator.cancelNav()
                    time.sleep(0.5)   # 给 planner 足够的时间释放
                except Exception as exc:
                    self.get_logger().warning(f"取消旧规划任务失败: {exc}")
            with self.nav_lock:
                path = self.navigator.getPath(start_pose, goal_pose)
            if not path or len(path.poses) == 0:
                self.speak(NavigationEvent.PLAN_FAILED)
                # 规划失败时，重置锁，让它过几秒还能继续尝试自救
                self.has_warned_off_track = False
                if not is_replan:
                    self.is_navigating = False
                    self.vibrate_pattern('stop')
                return False
            if self.process_path(path):
                self.current_segment_index = 0
                self.is_navigating = True
                self.nav_state = NavigationState.NAVIGATING
                self.perception_active = False
                # 要拿到了新路线，立刻重置防抖锁
                self.has_warned_off_track = False
                if not is_replan:
                    self.speak(NavigationEvent.NAV_STARTED)
                self.play_next_segment_instruction(yaw)
                return True
            else:
                #self.speak("目标点就在附近")
                self.has_warned_off_track = False
                return False
        finally:
            self._nav_start_lock = False

    def stop_navigation(self) -> None:
        self.nav_state = NavigationState.IDLE
        self.is_navigating = False
        try:
            self.navigator.cancelNav()
        except Exception as exc:
            self.get_logger().warning(f"取消导航任务失败: {exc}")
        self.vibrate_pattern("stop")
        self.speak(NavigationEvent.NAV_STOPPED)

    def pause_navigation(self) -> None:
        self.nav_state = NavigationState.PERCEPTION_ACTIVE
        self.perception_active = True
        self.vibrate_pattern("stop")
        self.temp_goal_active = False
        self.original_goal_pose = None
        self.waiting_for_turn = False
        self.is_waiting_for_clear = False
        self.waiting_clear_count = 0
        self.get_logger().info("感知对话已接管，导航指引暂停")

    def resume_navigation(self) -> None:
        if not self.is_navigating:
            self.get_logger().info("当前没有进行中的导航任务")
            return
        self.nav_state = NavigationState.NAVIGATING
        self.perception_active = False
        _, _, yaw = self.get_current_pose()
        if yaw is not None and self.current_segment_index < len(self.path_segments):
            self.play_next_segment_instruction(yaw)
        self.get_logger().info("导航指引已恢复")

    def set_audio_blocked(self, blocked: bool) -> None:
        self.audio_blocked = blocked

    def close(self) -> None:
        self.motors.stop()
        self.udp_sock.close()
