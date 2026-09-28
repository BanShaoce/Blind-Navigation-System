# Blind Navigation System

面向盲人用户的室内辅助导航原型。系统基于 ROS 2、Nav2、RTAB-Map 与 ZED 相机完成定位、路径规划和局部避障，通过三路震动马达给出左转、直行、右转提示；感知进程接入多模态实时模型，支持中文语音指令和环境描述。

## 功能

- 室内地图定位、Nav2 路径规划与局部代价地图避障；
- 左、前、右三路震动马达触觉引导；
- “去座位”“去门口”“停止/暂停/恢复导航”等中文语音指令；
- 基于摄像头图像的中文环境描述；
- 导航与感知进程通过仅绑定回环地址的 UDP 协议协作；
- 参数、目标点、地图、密钥与代码分离；
- 标准 ROS 2 `ament_python` 包、单元测试和持续集成。

## 系统架构

```text
麦克风/摄像头 ──> 感知进程 ──UDP 命令──> 导航进程 ──> Nav2 / RTAB-Map
                       ^                       │
                       └────UDP 事件───────────┘
                                               └──> GPIO 震动马达
```

详细职责、状态流转和扩展约定见 [架构设计](docs/架构设计.md)。

## 目录结构

```text
blind_navigation/
  common/          # 协议与配置
  navigation/      # ROS 节点、路径逻辑、命令服务、马达适配器
  perception/      # 多模态客户端、意图识别、音视频应用
config/            # Nav2 与业务参数
launch/            # ROS 2 一键启动文件
maps/              # 地图及历史地图
docs/              # 中文架构、部署、建图与运维文档
tests/             # 无硬件依赖的单元测试
```

## 运行环境

- NVIDIA Jetson Orin（项目当前硬件目标）；
- Ubuntu 22.04、ROS 2 Humble；
- ZED 2i 与对应 ZED SDK/ROS 2 Wrapper；
- Nav2、RTAB-Map、rosbridge_server；
- Python 3.10，ALSA/PulseAudio；
- 三路震动马达及匹配的驱动电路。

## 快速开始

以下命令假定仓库位于 ROS 2 工作空间的 `src/Blind-Navigation-System`。

```bash
cd ~/ros2_ws/src/Blind-Navigation-System
sudo apt update
sudo apt install -y portaudio19-dev alsa-utils \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-rosbridge-server ros-humble-rtabmap-ros

python3 -m pip install -r requirements.txt
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select blind_navigation_system
source install/setup.bash
```

配置 API Key，真实密钥不要写入文件或提交 Git：

```bash
export DASHSCOPE_API_KEY='你的密钥'
```

确保定位数据库位于 `~/.ros/rtabmap.db`，然后启动：

```bash
ros2 launch blind_navigation_system blind_guide.launch.py
```

只验证导航、不启动云端感知：

```bash
ros2 launch blind_navigation_system blind_guide.launch.py enable_perception:=false
```

完整的系统准备、网络缓冲区设置、参数说明、启动验证和故障排查见 [部署与运行说明](docs/部署与运行.md)。地图制作流程见 [建图指南](docs/建图指南.md)。

## 配置

- 目标点和引导阈值：`config/navigation.yaml`；
- Nav2 参数：`config/nav2_params.yaml`；
- 地图：`maps/rtabmap3.yaml` 与 `maps/rtabmap3.pgm`；
- 感知环境变量：参考 `.env.example`；
- RTAB-Map 数据库：启动参数 `database_path`，默认 `~/.ros/rtabmap.db`。

修改目标点后需要重新构建并重新加载工作空间；开发模式使用 `--symlink-install` 时，YAML 修改通常可直接生效。

## 开发与验证

不连接 ROS 和硬件即可运行纯逻辑测试：

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m unittest discover -s tests -v
python3 -m compileall -q blind_navigation launch tests
```

在完整 ROS 2 环境中还应执行：

```bash
colcon test --packages-select blind_navigation_system
colcon test-result --verbose
```

## 安全与密钥

- API Key 只从 `DASHSCOPE_API_KEY` 环境变量读取；
- UDP 控制端口默认只绑定 `127.0.0.1`；
- GPIO 在进程退出时统一关闭；
- 仓库早期版本曾包含明文密钥。仓库维护者应立即在服务端吊销该密钥，并在必要时清理 Git 历史；仅删除当前文件中的字符串不能使旧密钥失效。

安全问题请不要公开披露带有真实密钥、人员信息或室内地图的日志。

## 许可证

本项目采用 [MIT License](LICENSE)。第三方组件仍分别遵循其自身许可证。
