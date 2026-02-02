"""
TeraSim Tick Driver

Continuously ticks the TeraSim simulation via HTTP API.
This node drives the simulation forward.
"""

import time
import requests
import rclpy
from rclpy.node import Node


class TeraSimTickDriver(Node):

    def __init__(self, **kwargs):
        super().__init__('terasim_tick_driver', **kwargs)

        # Declare parameters
        self.declare_parameter("http_host", "localhost")
        self.declare_parameter("http_port", 8000)
        self.declare_parameter("simulation_id", "")

        self.http_host = self.get_parameter("http_host").value
        self.http_port = self.get_parameter("http_port").value
        self.simulation_id = self.get_parameter("simulation_id").value

        self.base_url = f"http://{self.http_host}:{self.http_port}"
        self.session = requests.Session()

        # Statistics
        self.tick_count = 0
        self.last_print_time = time.time()

        # Timer for ticking (as fast as possible, limited by simulation)
        self.timer = self.create_timer(0.001, self.on_timer)

        self.get_logger().info(f"Tick driver started: {self.base_url}, sim_id={self.simulation_id}")

    def on_timer(self):
        """Send tick command and wait for completion."""
        if not self.simulation_id:
            return

        try:
            # Send tick
            self.session.post(
                f"{self.base_url}/simulation_tick/{self.simulation_id}",
                timeout=5
            )

            # Wait for tick to complete
            for _ in range(100):
                status = self.session.get(
                    f"{self.base_url}/simulation_status/{self.simulation_id}",
                    timeout=5
                ).json().get("status")

                if status in ("ticked", "finished"):
                    break
                time.sleep(0.01)

            if status == "finished":
                # Get simulation result with exit reason
                try:
                    result = self.session.get(
                        f"{self.base_url}/simulation_result/{self.simulation_id}",
                        timeout=5
                    ).json()
                    self.get_logger().info(f"Simulation finished: {result}")
                except Exception as e:
                    self.get_logger().info(f"Simulation finished (could not get result: {e})")
                # Cancel timer and raise KeyboardInterrupt to stop the executor
                self.timer.cancel()
                raise KeyboardInterrupt("Simulation finished")

            # Update statistics
            self.tick_count += 1
            now = time.time()
            if now - self.last_print_time >= 5.0:
                rate = self.tick_count / (now - self.last_print_time)
                self.get_logger().info(f"SUMO: {rate:.1f} ticks/sec")
                self.tick_count = 0
                self.last_print_time = now

        except Exception as e:
            self.get_logger().warn(f"Tick error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TeraSimTickDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
