"""导航进程入口与资源生命周期管理。"""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor

from blind_navigation.common.protocol import NavigationCommand, UdpEndpoints
from blind_navigation.navigation.command_server import CommandServer
from blind_navigation.navigation.motors import MotorController
from blind_navigation.navigation.node import BlindGuideNode


def _pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    motors = MotorController()
    motors.initialize()
    endpoints = UdpEndpoints()
    guide = BlindGuideNode(motors, endpoints=endpoints)

    guide.declare_parameter("destinations.seat.x", -2.944)
    guide.declare_parameter("destinations.seat.y", -3.470)
    guide.declare_parameter("destinations.entrance.x", 5.226)
    guide.declare_parameter("destinations.entrance.y", 6.950)
    destinations = {
        NavigationCommand.GO_SEAT: _pose(
            float(guide.get_parameter("destinations.seat.x").value),
            float(guide.get_parameter("destinations.seat.y").value),
        ),
        NavigationCommand.GO_ENTRANCE: _pose(
            float(guide.get_parameter("destinations.entrance.x").value),
            float(guide.get_parameter("destinations.entrance.y").value),
        ),
    }
    command_server = CommandServer(guide, destinations, endpoints)
    executor = SingleThreadedExecutor()
    executor.add_node(guide)

    threads = [
        threading.Thread(target=executor.spin, daemon=True, name="ros-executor"),
        threading.Thread(
            target=guide.navigation_loop, daemon=True, name="navigation-loop"
        ),
        threading.Thread(
            target=command_server.serve_forever,
            daemon=True,
            name="udp-command-server",
        ),
    ]
    for thread in threads:
        thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        guide.get_logger().info("收到退出信号")
    finally:
        command_server.close()
        guide.close()
        motors.cleanup()
        executor.shutdown()
        guide.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
