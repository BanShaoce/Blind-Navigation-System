"""本地 UDP 命令服务。"""

from __future__ import annotations

import socket
import threading

import rclpy

from blind_navigation.common.protocol import (
    NavigationCommand,
    UdpEndpoints,
    parse_command,
)
from blind_navigation.navigation.node import BlindGuideNode


class CommandServer:
    def __init__(
        self,
        guide: BlindGuideNode,
        destinations: dict[NavigationCommand, object],
        endpoints: UdpEndpoints | None = None,
    ) -> None:
        self.guide = guide
        self.destinations = destinations
        self.endpoints = endpoints or UdpEndpoints()
        self._stop = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(0.5)

    def serve_forever(self) -> None:
        self._socket.bind((self.endpoints.host, self.endpoints.command_port))
        self.guide.get_logger().info(
            f"命令服务监听 {self.endpoints.host}:{self.endpoints.command_port}"
        )
        while rclpy.ok() and not self._stop.is_set():
            try:
                data, _ = self._socket.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            command = parse_command(data.decode("utf-8", errors="replace"))
            if command is None:
                self.guide.get_logger().warning("收到未知导航命令")
                continue
            self._dispatch(command)

    def _dispatch(self, command: NavigationCommand) -> None:
        if command in self.destinations:
            self.guide.start_navigation(self.destinations[command])
        elif command is NavigationCommand.STOP:
            self.guide.stop_navigation()
        elif command is NavigationCommand.PAUSE:
            self.guide.pause_navigation()
        elif command is NavigationCommand.RESUME:
            self.guide.resume_navigation()
        elif command is NavigationCommand.MUTE:
            self.guide.set_audio_blocked(True)
        elif command is NavigationCommand.UNMUTE:
            self.guide.set_audio_blocked(False)

    def close(self) -> None:
        self._stop.set()
        self._socket.close()
