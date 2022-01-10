import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import asyncio
from mavsdk import System


async def run(node):
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0")
    asyncio.ensure_future(print_attitude(drone, node))

async def print_attitude(drone, node):
    async for attitude in drone.telemetry.attitude_euler():
        print(f"attitude: {attitude}")

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer= self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.battery_percent = None
        self._event_loop = asyncio.get_running_loop()

    async def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


async def spin_once(minimal_publisher):
    rclpy.spin_once(minimal_publisher, timeout_sec=0)


async def run_ros2(minimal_publisher):
    while True:
        await spin_once(minimal_publisher)
        await asyncio.sleep(0.0)


async def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    task_ros2 = asyncio.create_task(run_ros2(minimal_publisher))
    task_mavsdk = asyncio.create_task(run(minimal_publisher))
    await asyncio.gather(task_ros2, task_mavsdk)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # asyncio.run(main(), debug=True)
    asyncio.run(main())
