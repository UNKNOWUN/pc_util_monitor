# SPDX-FileCopyrightText: 2025 Toshiaki Kou
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from .util_reader import UtilMonitor


class PcUtilMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("pc_util_monitor")

        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("include_gpu", True)
        self.declare_parameter("gpu_index", 0)

        rate = float(self.get_parameter("publish_rate_hz").value)
        include_gpu = bool(self.get_parameter("include_gpu").value)
        gpu_index = int(self.get_parameter("gpu_index").value)

        self._monitor = UtilMonitor(include_gpu=include_gpu, gpu_index=gpu_index)

        self._pub_cpu = self.create_publisher(Float32, "/pc/cpu_util", 10)
        self._pub_gpu = self.create_publisher(Float32, "/pc/gpu_util", 10)
        self._pub_status = self.create_publisher(String, "/pc/monitor_status", 10)

        period = 1.0 / rate if rate > 0 else 0.5
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"pc_util_monitor started (percent): rate={rate}Hz "
            f"include_gpu={include_gpu} gpu_index={gpu_index}"
        )

    def _on_timer(self) -> None:
        sample = self._monitor.read()

        cpu_msg = Float32()
        cpu_msg.data = float(sample.cpu_util * 100.0)

        gpu_msg = Float32()
        gpu_msg.data = float(sample.gpu_util * 100.0)

        status_msg = String()
        status_msg.data = sample.status

        self._pub_cpu.publish(cpu_msg)
        self._pub_gpu.publish(gpu_msg)
        self._pub_status.publish(status_msg)


def main() -> None:
    rclpy.init()
    node = PcUtilMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
