import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from drone_interfaces.action import DroneMission
from src.drone_code import run_drone_mission

class DroneMissionServer(Node):
    def __init__(self):
        super().__init__('drone_mission_server')
        self._action_server = ActionServer(
            self,
            DroneMission,
            'drone_mission',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received request to start mission: {goal_handle.request.start_mission}')

        feedback_msg = DroneMission.Feedback()
        
        def update_drone_coords_callback(x: int, y: int, z: int):
            feedback_msg.drone_x = x
            feedback_msg.drone_y = y
            feedback_msg.drone_z = z
            goal_handle.publish_feedback(feedback_msg)

        result = DroneMission.Result()

        pkg_x, pkg_y = run_drone_mission(update_drone_coords_callback)

        self.get_logger().info("Found the package. Sending package coordinates to client.")

        result.package_x = pkg_x
        result.package_y = pkg_y

        goal_handle.succeed()

        return result

def main(args=None):
    rclpy.init(args=args)
    node = DroneMissionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
