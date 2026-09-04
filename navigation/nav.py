# 可用一键启动的马达，拆开语音控制，重定向更新

#! /usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from nav2_msgs.msg import Costmap
from rclpy.time import Time
import sys
import time
import math
import os
import threading
import socket

# ====== 硬件马达 GPIO 库 ======
import Jetson.GPIO as GPIO

LEFT_PIN = 7
FRONT_PIN = 29
RIGHT_PIN = 31

# ====== 音频配置参数 ======
MIC_SAMPLE_RATE = 48000      
MIC_DEVICE_INDEX = 24        


# =================配置参数=================
METERS_PER_STEP = 0.5           #步长
ANGLE_THRESHOLD_DEG = 10.0      #转角合并阈值 
DIRECTION_CHECK_THRESHOLD = 10.0  #转向判定阈值
WAYPOINT_REACH_THRESHOLD = 0.4   #到达判定阈值
OFF_TRACK_LIMIT = 0.4            #偏离阈值
OFF_TRACK_CHECK_INTERVAL = 1.5   #偏离检测频率（s）

class PathSegmentInfo:
    def __init__(self):
        self.distance = 0.0      
        self.heading = 0.0       
        self.start_point = None  
        self.end_point = None    
        self.is_last_segment = False

def init_motors():  #震动马达
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup([LEFT_PIN, FRONT_PIN, RIGHT_PIN], GPIO.OUT, initial=GPIO.LOW)

def stop_all_motors():
    GPIO.output(LEFT_PIN, GPIO.LOW)
    GPIO.output(FRONT_PIN, GPIO.LOW)
    GPIO.output(RIGHT_PIN, GPIO.LOW)

class BlindGuideNode(Node):
    def __init__(self):
        super().__init__('blind_guide_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        
        self.nav_lock = threading.Lock()
        self.audio_lock = threading.Lock()
        self.motor_lock = threading.Lock() 
        self.current_audio_process = None

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
        self.latest_costmap = None
        self._nav_start_lock = False
        self.last_obstacle_clear_time = 0.0
        self.waiting_clear_count = 0
        self.temp_clear_count = 0
        self.off_track_fail_count = 0
        self.max_off_track_retries = 3
        self.costmap_lock = threading.Lock()
        self.costmap_sub = self.create_subscription(
            Costmap,
            '/local_costmap/costmap_raw',
            self.costmap_callback,
            10
        ) 
        # 状态机
        self.nav_state = 'IDLE'           # IDLE, NAVIGATING, PERCEPTION_ACTIVE
        self.audio_blocked = False        # True 时禁止 speak()
        self.perception_active = False    # 感知模块正在对话
# UDP 发送 socket（向感知模块发送状态）
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_dest = ('127.0.0.1', 5001)

    #def sample_obstacle_in_front(self, robot_x, robot_y, robot_yaw, distance=1.0, half_width=0.3):
        """
                在前方 distance 米处，沿垂直于路径方向采样 3~5 个点，
                返回代价最高的点的值和它在路径左右侧的信息。
        """
    #    check_angles = [0, -15, 15, -30, 30]   # 相对于 robot_yaw 的角度（度）
    #    max_cost = 0
    #    for angle_deg in check_angles:
    #        rad = math.radians(angle_deg)
    #        wx = robot_x + distance * math.cos(robot_yaw + rad)
    #        wy = robot_y + distance * math.sin(robot_yaw + rad)
    #        cost = self.get_cost_at(wx, wy)
    #        if cost > max_cost:
    #            max_cost = cost
    #    return max_cost
    
    def send_status(self, status):
        try:
            self.udp_sock.sendto(status.encode(), self.udp_dest)
        except:
            pass
    
    def find_temp_avoid_point(self, obstacle_pt, path_angle, avoid_side, step=0.1):
        """
                    从障碍物位置开始，向侧面搜索安全点。对每个候选点，用 _is_point_safe 检查格点代价是否安全，
                    再用 _can_move_forward 检查能否继续向前走（避免卡在墙角）。
    
                    参数:
            obstacle_pt: (x, y) 在 odom 系下的障碍物投影点
            path_angle:  原路径方向（弧度）
            avoid_side: 'left' 或 'right'
                    返回:
            (wx, wy) 或 None
        """
        if obstacle_pt is None:
            return None

        obs_x, obs_y = obstacle_pt
        perp_angle = path_angle + (math.pi/2 if avoid_side == 'left' else -math.pi/2)
        SAFE_THRESH = 30
        MAX_LATERAL = 2.0      # 最大侧向搜索距离
        MIN_LATERAL = 0.3      # 至少离开障碍物 0.3m

        for lateral_m in np.arange(MIN_LATERAL, MAX_LATERAL + step, step):
            wx = obs_x + lateral_m * math.cos(perp_angle)
            wy = obs_y + lateral_m * math.sin(perp_angle)
            if self._is_point_safe(wx, wy, margin=0.2):
                # 进一步检查该点是否至少能向前走一点（避免卡在墙角）
                if self._can_move_forward(wx, wy, path_angle):
                    return (wx, wy)
        return None

    def _can_move_forward(self, x, y, path_angle, check_dist=0.5):
        """检查从 (x,y) 沿 path_angle 方向是否可通行"""
        SAFE_THRESH = 30
        for d in [0.2, 0.4, 0.6]:
            cx = x + d * math.cos(path_angle)
            cy = y + d * math.sin(path_angle)
            if self.get_cost_at(cx, cy) >= SAFE_THRESH:
                return False
        return True
        
    def _is_point_safe(self, x, y, margin=0.2):
        """检查点 (x,y) 及其 margin 范围内所有代价是否均低于阈值"""
        SAFE_THRESH = 30
        # 先快速检测中心点
        if self.get_cost_at(x, y) >= SAFE_THRESH:
            return False
        # 再检查四个方向各 margin 米处的点（十字形，避免过多采样）
        for angle in [0, math.pi/2, math.pi, 3*math.pi/2]:
            cx = x + margin * math.cos(angle)
            cy = y + margin * math.sin(angle)
            if self.get_cost_at(cx, cy) >= SAFE_THRESH:
                return False
        return True
    
    def vibrate_pattern(self, pattern_type):
        if pattern_type == 'stop':
            stop_all_motors()
            self.current_vibrate_state = 'stop'
            return
        if self.nav_state != 'NAVIGATING':
            return
        if self.current_vibrate_state == pattern_type and pattern_type != 'replan':
            return
            
        self.current_vibrate_state = pattern_type

        def run():
            with self.motor_lock:
                stop_all_motors()
                
                if pattern_type == 'left':
                    print("\n👉[状态切换] 需要左转 -> 启动【左马达】")
                    GPIO.output(LEFT_PIN, GPIO.HIGH)
                elif pattern_type == 'right':
                    print("\n👉 [状态切换] 需要右转 -> 启动【右马达】")
                    GPIO.output(RIGHT_PIN, GPIO.HIGH)
                elif pattern_type == 'straight':
                    print("\n⬆️[状态切换] 方向正确/直行 -> 启动【前马达】")
                    GPIO.output(FRONT_PIN, GPIO.HIGH)
                elif pattern_type == 'stop':
                    print("\n🛑[状态切换] 停止导航 -> 关闭所有马达")
                    pass 

        threading.Thread(target=run, daemon=True).start()

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
            x_odom = x_map * math.cos(yaw) - y_map * math.sin(yaw) + tx
            y_odom = x_map * math.sin(yaw) + y_map * math.cos(yaw) + ty
            return x_odom, y_odom
        except TransformException:
            # TF 变换尚未就绪
            return None, None
        except Exception as e:
            print(f"[ERROR] map->odom transform failed: {e}")
            return None, None
    
    def get_cost_at(self, wx, wy):
        """返回世界坐标 (wx, wy) 处的代价，若无法获取返回 -1"""
        with self.costmap_lock:
            if self.latest_costmap is None:
                return -1
            meta = self.latest_costmap.metadata
            # 计算栅格索引
            gx = int((wx - meta.origin.position.x) / meta.resolution)
            gy = int((wy - meta.origin.position.y) / meta.resolution)
            if gx < 0 or gx >= meta.size_x or gy < 0 or gy >= meta.size_y:
                return -1   # 超出范围
            index = gy * meta.size_x + gx
            if index >= len(self.latest_costmap.data):
                return -1
            return self.latest_costmap.data[index]
    
    def speak(self, text_or_code: str):
        """
        统一语音播报接口 —— 现在改为发送 UDP 事件码给感知模块。
        如果传入的是已知事件码（如 ARRIVED），直接发送；
        如果是普通文本，将作为通用语音请求发送（仅调试用）。
        """
        print(f"🔊 通知感知模块: {text_or_code}")
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
        except Exception:
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
        except Exception:
            return None, None, None
    
    def process_path(self, nav_path):
        """ 找路，把大量路径点压缩成关键点，合并角度变换小的段，计算每个段的距离和方向 """
        if not nav_path or len(nav_path.poses) < 2: return False
        self.path_segments.clear()
        poses = nav_path.poses

        key_points =[]
        key_points.append((poses[0].pose.position.x, poses[0].pose.position.y))
        accumulated_dist = 0.0
        SAMPLE_DIST = 0.5

        for i in range(1, len(poses)):
            p_prev = poses[i - 1].pose.position
            p_curr = poses[i].pose.position
            dist = math.sqrt((p_curr.x - p_prev.x) ** 2 + (p_curr.y - p_prev.y) ** 2)
            accumulated_dist += dist
            if i == len(poses) - 1 or accumulated_dist >= SAMPLE_DIST:
                key_points.append((p_curr.x, p_curr.y))
                accumulated_dist = 0.0

        if len(key_points) < 2: return False

        for i in range(len(key_points) - 1):
            p_start = key_points[i]
            p_end = key_points[i + 1]
            dx = p_end[0] - p_start[0]
            dy = p_end[1] - p_start[1]
            dist = math.sqrt(dx ** 2 + dy ** 2)
            segment_heading = math.atan2(dy, dx)

            if len(self.path_segments) > 0:
                last_seg = self.path_segments[-1]
                heading_diff = segment_heading - last_seg.heading
                while heading_diff > math.pi: heading_diff -= 2 * math.pi
                while heading_diff < -math.pi: heading_diff += 2 * math.pi
                
                if abs(math.degrees(heading_diff)) <= ANGLE_THRESHOLD_DEG:
                    last_seg.distance += dist
                    last_seg.end_point = p_end
                    new_dx = p_end[0] - last_seg.start_point[0]
                    new_dy = p_end[1] - last_seg.start_point[1]
                    last_seg.heading = math.atan2(new_dy, new_dx)
                    continue

            seg = PathSegmentInfo()
            seg.start_point = p_start
            seg.end_point = p_end
            seg.distance = dist
            seg.heading = segment_heading
            self.path_segments.append(seg)

        if len(self.path_segments) > 0:
            self.path_segments[-1].is_last_segment = True
        return True

    def navigation_loop(self):
        OBSTACLE_CHECK_DIST = 0.8          # 避障检测距离（米）
        OBSTACLE_COST_THRESH = 80          # 代价大于此值视为需要避障
        SAFE_COST_THRESH = 30              # 代价低于此值认为可通行
        OBSTACLE_COOLDOWN = 1.5            # 避障退出后至少等待几秒再检测，防止反复触发
        
        while rclpy.ok():
            if not self.is_navigating:
                time.sleep(0.5)
                continue       
            if self.nav_state == 'PERCEPTION_ACTIVE':
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
                    trig_cost = self.get_cost_at(*self.trigger_obstacle_pt)
                    if trig_cost >= 0 and trig_cost < SAFE_COST_THRESH:
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
                    # 障碍物离开局部地图也算消失
                    if trig_cost == -1:
                        self.get_logger().info("❗ 临时导航中，障碍物点已离开局部地图，返回全局导航")
                        self.temp_goal_active = False
                        self.temp_clear_count = 0
                        self.trigger_obstacle_pt = None
                        if self.original_goal_pose:
                            self.start_navigation(self.original_goal_pose, is_replan=True)
                        continue
                # ---- 2. 临时路径段已走完 ----
                if self.current_segment_index >= len(self.path_segments):
                    self.temp_goal_active = False
                    self.get_logger().info("✅ 到达临时点，重新规划至最终目标")
                    if self.original_goal_pose:
                        self.start_navigation(self.original_goal_pose, is_replan=True)
                    else:
                        self.finish_navigation()
                    continue

                # ---- 3. 正常引导：转弯 / 直行 ----
                current_seg = self.path_segments[self.current_segment_index]

                # 3a. 转弯引导
                if self.waiting_for_turn:
                    diff = current_seg.heading - yaw
                    while diff > math.pi: diff -= 2 * math.pi
                    while diff < -math.pi: diff += 2 * math.pi
                    diff_deg = math.degrees(diff)
                    if abs(diff_deg) <= DIRECTION_CHECK_THRESHOLD:
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
                        if (self.last_turn_direction is None) or \
                            (new_dir != self.last_turn_direction and abs(diff_deg) > 30.0):
                            self.last_turn_direction = new_dir
                            self.vibrate_pattern(new_dir)
                        time.sleep(0.2)
                        continue

                # 3b. 直行 → 检查是否到达当前段终点
                dist_to_end = math.sqrt((x - current_seg.end_point[0])**2 + (y - current_seg.end_point[1])**2)
                if dist_to_end < WAYPOINT_REACH_THRESHOLD:
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
                x_odom, y_odom, yaw_odom = self.get_current_pose_odom()
                if x_odom is not None:
                    collision, _, _ = self._check_path_collision(x_odom, y_odom)
                    trig_cost = self.get_cost_at(*self.trigger_obstacle_pt) if self.trigger_obstacle_pt else -1
                    if not collision and (trig_cost == -1 or trig_cost < SAFE_COST_THRESH):
                        self.waiting_clear_count += 1
                        if self.waiting_clear_count >= 3:
                            self.is_waiting_for_clear = False
                            self.waiting_clear_count = 0
                            self.get_logger().info("✅ 路径已畅通，重新规划全局路径")
                            self.speak("PATH_CLEAR")
                            if self.final_goal_pose:
                                success = self.start_navigation(self.final_goal_pose)
                            else:
                                success = self.start_navigation(self.current_goal_pose)
                            if not success:
                                self.speak("PLAN_FAILED")
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
                diff = current_seg.heading - yaw
                while diff > math.pi: diff -= 2 * math.pi
                while diff < -math.pi: diff += 2 * math.pi
                diff_deg = math.degrees(diff)
                if abs(diff_deg) <= DIRECTION_CHECK_THRESHOLD:
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
                    x_odom, y_odom, yaw_odom = self.get_current_pose_odom()
                    if x_odom is not None:
                        collision, direction, obstacle_pt = self._check_path_collision(x_odom, y_odom)
                        # 只在状态变化时打印
                        if collision != self.last_collision_state or direction != self.last_collision_direction:
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
                                path_heading = self.path_segments[self.current_segment_index].heading
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
                                self.speak("BLOCKED")
                                self.vibrate_pattern('stop')
                                self.final_goal_pose = self.current_goal_pose
                                self.is_waiting_for_clear = True
                                self.trigger_obstacle_pt = obstacle_pt
                                self.last_obstacle_clear_time = time.time() + 2.0
                                continue
                # 如果冷却未到，不做任何障碍检测，直接继续后面的段完成检查
            # --- 检查是否到达当前段终点 ---
            dist_to_end = math.sqrt((x - current_seg.end_point[0])**2 + (y - current_seg.end_point[1])**2)
            if dist_to_end < WAYPOINT_REACH_THRESHOLD:
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
            x_map = x_odom * math.cos(yaw) - y_odom * math.sin(yaw) + tx
            y_map = x_odom * math.sin(yaw) + y_odom * math.cos(yaw) + ty
            return x_map, y_map
        except Exception:
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
        temp_pt = self.find_temp_avoid_point(obstacle_pt, path_heading, preferred_direction)
        if temp_pt is None:
            # 反方向
            opposite = 'right' if preferred_direction == 'left' else 'left'
            temp_pt = self.find_temp_avoid_point(obstacle_pt, path_heading, opposite)
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
            temp_pt2 = self.find_temp_avoid_point(obstacle_pt, path_heading, opposite)
            if temp_pt2 is not None:
                map_pt2 = self.transform_point_odom_to_map(temp_pt2[0], temp_pt2[1])
                if map_pt2 is not None:
                    temp_pose2 = self._make_temp_pose(map_pt2[0], map_pt2[1], path_heading)
                    if self.start_navigation(temp_pose2, is_replan=True):
                        self.original_goal_pose = saved_original
                        self.temp_goal_active = True
                        return True

        return False   # 所有尝试均失败
    
    def _get_cost_area_max(self, robot_x, robot_y, robot_yaw, angles_deg, distance=1.0):
        """在指定的多个角度方向、距离处采样，返回最大代价"""
        max_cost = 0
        for ang in angles_deg:
            rad = math.radians(ang)
            wx = robot_x + distance * math.cos(robot_yaw + rad)
            wy = robot_y + distance * math.sin(robot_yaw + rad)
            cost = self.get_cost_at(wx, wy)
            if cost > max_cost:
                max_cost = cost
        return max_cost
    
    def _get_max_cost_in_direction(self, x, y, angle, max_dist):
        """沿 angle 方向，距离从 0.1 到 max_dist，返回遇到的最大代价"""
        max_val = 0
        d = 0.2
        while d <= max_dist:
            wx = x + d * math.cos(angle)
            wy = y + d * math.sin(angle)
            val = self.get_cost_at(wx, wy)
            if val > max_val:
                max_val = val
            d += 0.1
        return max_val
    
    def _is_heading_towards_path(self, x, y, yaw):
        if self.current_segment_index < len(self.path_segments):
            seg = self.path_segments[self.current_segment_index]
            desired_heading = math.atan2(seg.end_point[1] - y, seg.end_point[0] - x)
            diff = desired_heading - yaw
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            return abs(math.degrees(diff)) < 30.0
        return True
    
    def _check_path_collision(self, robot_x_odom, robot_y_odom):
        if self.current_segment_index >= len(self.path_segments):
            return False, None, None

        seg = self.path_segments[self.current_segment_index]
        end_x_odom, end_y_odom = self.transform_point_map_to_odom(
            seg.end_point[0], seg.end_point[1])

        if end_x_odom is None:
            self.get_logger().warn("TF 转换失败，本次碰撞检测跳过")
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

        OBSTACLE_THRESH = 80
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
            center_cost = self.get_cost_at(px, py)

            if center_cost > OBSTACLE_THRESH:
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
        
    def costmap_callback(self, msg):
        """ 保存局部代价地图"""
        # 拒绝比已有时间戳更旧的消息
        if self.latest_costmap is not None:
            new_stamp = Time.from_msg(msg.header.stamp)
            old_stamp = Time.from_msg(self.latest_costmap.header.stamp)
            if new_stamp < old_stamp:
                return
        with self.costmap_lock:
            self.latest_costmap = msg
    
    def play_next_segment_instruction(self, current_yaw):
        if len(self.path_segments) == 0:
            self.get_logger().error("❌ path_segments 意外为空，取消当前导航")
            self.finish_navigation()
            return
        if self.current_segment_index >= len(self.path_segments):
            self.finish_navigation()
            return
        next_seg = self.path_segments[self.current_segment_index]
        diff = next_seg.heading - current_yaw
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        diff_deg = math.degrees(diff)

        if abs(diff_deg) <= ANGLE_THRESHOLD_DEG:
            self.vibrate_pattern('straight') 
            self.waiting_for_turn = False
            self.last_turn_direction = None
        else:
            new_dir = 'left' if diff_deg > 0 else 'right'
            # 滞回：只有当方向真正改变且偏差足够大时才切换振动 0727
            if (self.last_turn_direction is None) or \
                (new_dir != self.last_turn_direction and abs(diff_deg) > 15.0):
                self.last_turn_direction = new_dir
                self.vibrate_pattern(new_dir) 
            self.waiting_for_turn = True 

    def finish_navigation(self):
        self.vibrate_pattern('stop')
        self.nav_state = 'IDLE'
        self.perception_active = False
        self.temp_goal_active = False
        self.original_goal_pose = None
        self.vibrate_pattern('stop')
        self.speak("ARRIVED")
        self.is_navigating = False

    def check_if_off_track(self, user_pos, segment, current_yaw):
        if not self.path_segments:
            return False
        now = time.time()
        if now - self.last_off_track_check_time < OFF_TRACK_CHECK_INTERVAL: return False
        self.last_off_track_check_time = now
        if self.has_warned_off_track: return False

        px, py = user_pos
        x1, y1 = segment.start_point
        x2, y2 = segment.end_point
        vx, vy = x2 - x1, y2 - y1
        if vx == 0.0 and vy == 0.0: return False
        len_sq = vx * vx + vy * vy
        u = max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / len_sq))
        proj_x = x1 + u * vx
        proj_y = y1 + u * vy
        dist = math.sqrt((px - proj_x) ** 2 + (py - proj_y) ** 2)
        
        # 1. 距离是否偏离超过 0.6 米
        if dist > OFF_TRACK_LIMIT:
            return True
            
        # 2. 如果盲人没在转弯，但他面朝的方向跟路线差了超过 90 度（走反了）
        if not self.waiting_for_turn:
            diff = segment.heading - current_yaw
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi
            if abs(math.degrees(diff)) > 90.0:
                print("\n⚠️ 检测到方向彻底反向！触发秒切...")
                return True

        return False

    def handle_off_track(self):
        self.has_warned_off_track = True
        print("\n⚠️ 检测到路线偏离，正在后台秒切新路线...")
        if self.current_goal_pose:
            success = self.start_navigation(self.current_goal_pose, is_replan=True)
            if success:
                self.off_track_fail_count = 0
            else:
                self.off_track_fail_count += 1
                if self.off_track_fail_count >= self.max_off_track_retries:
                    self.speak("OFF_TRACK_STOP")
                    self.finish_navigation()

    def start_navigation(self, goal_pose, is_replan=False):
        if self._nav_start_lock:
            self.get_logger().warn("⚠️ start_navigation 重入被阻止")
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
                self.speak("PLAN_FAILED")
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
                except Exception:
                    pass
            with self.nav_lock:
                path = self.navigator.getPath(start_pose, goal_pose)
            if not path or len(path.poses) == 0:
                self.speak("PLAN_FAILED")
                # 规划失败时，重置锁，让它过几秒还能继续尝试自救
                self.has_warned_off_track = False 
                if not is_replan:
                    self.is_navigating = False
                    self.vibrate_pattern('stop')
                return False
            if self.process_path(path):
                self.current_segment_index = 0
                self.is_navigating = True   
                self.nav_state = 'NAVIGATING'
                self.perception_active = False      
                # 要拿到了新路线，立刻重置防抖锁
                self.has_warned_off_track = False         
                if not is_replan:
                    self.speak("NAV_STARTED")          
                self.play_next_segment_instruction(yaw)
                return True
            else:
                #self.speak("目标点就在附近")
                self.has_warned_off_track = False
                return False
        finally:
            self._nav_start_lock = False

# ================= 本地网络监听循环  =================
def udp_command_loop(guide_node, goal_A, goal_B):
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("127.0.0.1", 5000))
    print("📡 导航系统已开启网络监听端口 5000，等待语音助手下发指令...")
    
    while rclpy.ok():
        try:
            # 阻塞接收
            data, addr = server.recvfrom(1024)
            cmd = data.decode('utf-8')
            
            if cmd == "GO_A":
                print("📡 收到网络指令：去座位")
                guide_node.start_navigation(goal_A)
            elif cmd == "GO_B":
                print("📡 收到网络指令：去门口")
                guide_node.start_navigation(goal_B)
            elif cmd == "STOP":
                print("📡 收到网络指令：停止")
                guide_node.nav_state = 'IDLE'
                guide_node.is_navigating = False
                try: guide_node.navigator.cancelNav() 
                except: pass
                stop_all_motors()
                guide_node.vibrate_pattern('stop')
                guide_node.speak("NAV_STOPPED")
            elif cmd == "PAUSE_NAV":
                guide_node.nav_state = 'PERCEPTION_ACTIVE'
                guide_node.perception_active = True
                guide_node.vibrate_pattern('stop')
                # 清理所有临时状态，确保恢复后从干净状态开始
                guide_node.temp_goal_active = False
                guide_node.original_goal_pose = None
                guide_node.waiting_for_turn = False
                guide_node.is_waiting_for_clear = False
                guide_node.waiting_clear_count = 0
                print("[NAV] Perception active – guidance paused")
            elif cmd == "RESUME_NAV":
                guide_node.nav_state = 'NAVIGATING'
                guide_node.perception_active = False
                # 立即重新输出当前方向（振动会由 navigation_loop 下一轮自动调用 play_next_segment_instruction）
                # 但为了让用户立刻感受到恢复，可以主动获取姿态并触发一次方向提示
                x, y, yaw = guide_node.get_current_pose()
                if yaw is not None and guide_node.current_segment_index < len(guide_node.path_segments):
                    guide_node.play_next_segment_instruction(yaw)
                print("[NAV] Navigation resumed")
            elif cmd == "MUTE_NAV":
                guide_node.audio_blocked = True
            elif cmd == "UNMUTE_NAV":
                guide_node.audio_blocked = False
        except Exception as e:
            print(f"网络接收异常: {e}")

def create_pose(x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1.0
    return p

# ================= 主程序 =================
def main():
    init_motors()

    rclpy.init()
    node_name = f'blind_guide_node_{int(time.time())}'
    guide_node = BlindGuideNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(guide_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    nav_thread = threading.Thread(target=guide_node.navigation_loop, daemon=True)
    nav_thread.start()

    # 预设的两个目标点
    goal_A = create_pose(-2.944, -3.470)   # 座位
    goal_B = create_pose(5.226, 6.950)   # 门口

    # 1. 启动轻量级 UDP 监听线程 (替代原来的语音线程)
    udp_thread = threading.Thread(
        target=udp_command_loop, 
        args=(guide_node, goal_A, goal_B), 
        daemon=True
    )
    udp_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("退出程序")
    finally:
        print("🛑 正在停止硬件资源和马达...")
        stop_all_motors()
        GPIO.cleanup()
        
        guide_node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

