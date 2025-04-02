import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
import asyncio
import threading

class MAVBridge(Node):
    def __init__(self):
        super().__init__('mavsdk_bridge')
        self.subscription = self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.pose = None
        self.loop = asyncio.get_event_loop()
        self.drone = System()
        self.connected = False
        threading.Thread(target=self.init_mavsdk, daemon=True).start()

    def pose_callback(self, msg):
        self.pose = msg.pose
        if self.connected:
            asyncio.run_coroutine_threadsafe(self.send_position_command(), self.loop)

    def init_mavsdk(self):
        asyncio.run(self.mavsdk_main())

    async def mavsdk_main(self):
        await self.drone.connect(system_address="udp://:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("✅ MAVSDK connected.")
                break

        await self.drone.action.arm()
        await self.drone.offboard.set_position_ned(PositionNedYaw(0, 0, -1, 0))
        await self.drone.offboard.start()
        self.connected = True

    async def send_position_command(self):
        if self.pose:
            # Convert ENU (ROS) → NED (PX4)
            x = self.pose.position.y
            y = self.pose.position.x
            z = -self.pose.position.z

            await self.drone.offboard.set_position_ned(PositionNedYaw(x, y, z, 0.0))
            self.get_logger().info(f"📡 Sent setpoint to NED: [{x}, {y}, {z}]")

def main(args=None):
    rclpy.init(args=args)
    node = MAVBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
