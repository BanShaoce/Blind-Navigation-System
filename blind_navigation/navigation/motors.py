"""Jetson GPIO 震动马达适配器。"""

from __future__ import annotations

import logging
import threading
from typing import Any

LOGGER = logging.getLogger(__name__)


class MotorController:
    """统一管理 GPIO 生命周期；非 Jetson 环境自动降级为空实现。"""

    def __init__(self, left_pin: int = 7, front_pin: int = 29, right_pin: int = 31):
        self.left_pin = left_pin
        self.front_pin = front_pin
        self.right_pin = right_pin
        self._lock = threading.Lock()
        self._gpio: Any | None = None

    @property
    def available(self) -> bool:
        return self._gpio is not None

    def initialize(self) -> None:
        try:
            import Jetson.GPIO as gpio
        except ImportError:
            LOGGER.warning("未安装 Jetson.GPIO，震动马达以模拟模式运行")
            return
        self._gpio = gpio
        gpio.setmode(gpio.BOARD)
        gpio.setup(
            [self.left_pin, self.front_pin, self.right_pin],
            gpio.OUT,
            initial=gpio.LOW,
        )

    def activate(self, direction: str) -> None:
        with self._lock:
            self._stop_unlocked()
            if self._gpio is None or direction == "stop":
                return
            pin = {
                "left": self.left_pin,
                "straight": self.front_pin,
                "right": self.right_pin,
            }.get(direction)
            if pin is None:
                raise ValueError(f"未知震动方向: {direction}")
            self._gpio.output(pin, self._gpio.HIGH)

    def stop(self) -> None:
        with self._lock:
            self._stop_unlocked()

    def _stop_unlocked(self) -> None:
        if self._gpio is not None:
            self._gpio.output(
                [self.left_pin, self.front_pin, self.right_pin], self._gpio.LOW
            )

    def cleanup(self) -> None:
        with self._lock:
            self._stop_unlocked()
            if self._gpio is not None:
                self._gpio.cleanup()
