import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from drone_interfaces.action import DroneMission

class DroneMissionClient(Node):
    def __init__(self):
        super().__init__('drone_mission_client')
        self._client = ActionClient(self, DroneMission, 'drone_mission')

    def send_goal(self):
        goal_msg = DroneMission.Goal()
        goal_msg.start_mission = True

        self._client.wait_for_server()
        self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Package coordinates: ({result.package_x}, {result.package_y})')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Drone coordinates: ({feedback_msg.feedback.drone_x},{feedback_msg.feedback.drone_y},{feedback_msg.feedback.drone_z})')

def main(args=None):
    rclpy.init(args=args)
    node = DroneMissionClient()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
