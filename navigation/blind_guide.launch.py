import os
import time
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    # 自动优化系统网络缓冲区，彻底解决点云造成的 DDS 丢包和 Service 挂起问题
    os.system("sudo sysctl -w net.core.rmem_max=8388608")
    os.system("sudo sysctl -w net.core.wmem_max=8388608")
    
    # 1. 自动创建独立的日志文件夹
    log_dir = '/home/nvidia/nav/logs'
    os.makedirs(log_dir, exist_ok=True)
    
    # 获取环境变量
    env_source = "source /opt/ros/humble/setup.bash && "

    print(f"\n" + "="*55)
    print(f" 🚀 盲人导航系统启动中...")
    print(f" 📁 底层引擎日志重定向至: {log_dir}")
    print(f"=======================================================\n")

    # ----- 底层引擎定义 (后台静默运行) -----

    # 1. ZED 相机 (已添加降频和禁止全局定位，防止网络风暴和坐标打架)
    zed_cmd = (
        f"{env_source} exec ros2 launch zed_wrapper zed_camera.launch.py "
        f"camera_model:=zed2i "
        f"publish_tf:=true "  #0601
        f"general.grab_resolution:=VGA "
        #f"general.grab_resolution:=HD720 " 
        f"general.grab_frame_rate:=15 "
        f"depth.point_cloud_freq:=5.0 " 
        f"depth.publish_depth:=true " 
        f"depth.depth_confidence:=50 " 
        f"depth.depth_texture_conf:=100 "        
        f"pos_tracking.publish_map_tf:=false "   # 禁止 ZED 发布地图坐标
        f"depth.point_cloud_organized:=false "
        f"publish_urdf:=true "
        f"sensors.sensors_image_sync:=true "
        f"pos_tracking.tf_pub_rate:=100.0 "            # 提高 TF 发布频率（默认 30~50 Hz）
        #f"cam_pos_z:=1.2 "
        f"> {log_dir}/zed.log 2>&1"
    )
    
    # 2. 静态 TF (已修正相机的偏航角，确保正前方对齐)
    tf_cmd = (
        f"{env_source} exec ros2 run tf2_ros static_transform_publisher "
        f"0 0 -0.8 0 0 0 zed_camera_link base_link "   
        f"> {log_dir}/tf.log 2>&1"
    )

    # 3. RTAB-Map (保持原样)
    rtabmap_args = (
        "--Mem/IncrementalMemory false "
        "--Mem/InitWMWithAllNodes true "
        "--Vis/MinInliers 12 "
        "--RGBD/OptimizeMaxError 10.0 "
        "--RGBD/LoopClosureRecheck true "
    )
    rtabmap_cmd = (
        f"{env_source} exec ros2 launch rtabmap_launch rtabmap.launch.py "
        f"rtabmap_args:='{rtabmap_args}' "
        f"database_path:='/home/nvidia/.ros/rtabmap.db' "
        f"frame_id:=base_link "
        f"rgb_topic:=/zed/zed_node/rgb/color/rect/image "
        f"depth_topic:=/zed/zed_node/depth/depth_registered "
        f"camera_info_topic:=/zed/zed_node/rgb/color/rect/camera_info "
        f"odom_topic:=/zed/zed_node/odom "
        f"approx_sync:=true "
        f"wait_for_transform:=3.0 "
        f"qos:=1 "
        f"visual_odometry:=false "
        f"rtabmap_viz:=false "
        f"> {log_dir}/rtabmap.log 2>&1"
    )

    # 4. Nav2 (已彻底踢除 AMCL，防止跳回正北)
    nav2_cmd = (
        f"{env_source} exec ros2 launch nav2_bringup bringup_launch.py "
        f"use_sim_time:=False "
        f"autostart:=True "
        f"map:=/home/nvidia/nav/rtabmap1.yaml "
        f"use_localization:=false "               
        f"use_amcl:=false "                       
        f"lifecycle_manager_localization:=False "   
        f"params_file:=/home/nvidia/nav/nav2_params.yaml "
        f"> {log_dir}/nav2.log 2>&1"
    )
    

    # ----- 业务逻辑脚本定义 (终端实时显示) -----


    # 6. 导航主控脚本 (独立进程)
    nav_script_node = ExecuteProcess(
        cmd=['python3', '-u', '/home/nvidia/nav/nav.py'],
        output='screen'
    )

    # ================= 启动编排 =================
    return LaunchDescription([
        # T=0: 相机和坐标
        ExecuteProcess(cmd=['bash', '-c', zed_cmd]),
        ExecuteProcess(cmd=['bash', '-c', tf_cmd]),     
        # rosbridge WebSocket 
        ExecuteProcess(
            cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
            output='screen',
            name='rosbridge'
        ),
        
        # T=10: 定位引擎 
        TimerAction(period=10.0, actions=[
            ExecuteProcess(cmd=['bash', '-c', rtabmap_cmd])
        ]),
        
        # T=25: 导航规划 (给 RTAB-Map 预留 15 秒，同时拉起 Nav2 与补充的 MapServer)
        TimerAction(period=25.0, actions=[
            ExecuteProcess(cmd=['bash', '-c', nav2_cmd]),
        ]),
        
        # T=40: 启动业务逻辑层
        TimerAction(period=40.0, actions=[
            nav_script_node
        ]),

        # T=45 或更晚，等待所有基础组件就绪后启动感知模块
        TimerAction(period=45.0, actions=[
            ExecuteProcess(
                cmd=['bash', '-c', 'conda run -n visual python /home/nvidia/nav/vad_mode.py'],
                output='screen'
            )
        ]),
    ])
