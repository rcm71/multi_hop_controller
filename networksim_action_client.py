import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from multi_hop_interfaces.action import NetworkSimulator


class NetworkSimActionClient(Node):

    def __init__(self, name):
        super().__init__(f'networksim_client_{name}')
        self.name = name
        self._action_client = ActionClient(self, NetworkSimulator, 'networksimulator')

    def send_message(self):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Network server not available')
            return

        goal_msg = NetworkSimulator.Goal()
        goal_msg.sender_id = self.name

        self.get_logger().info(f'SENDING MESSAGE FROM {self.name}')

        self._send_goal_future = self._action_client.send_goal_asymc(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('GOAL REJECTED :(')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'RESULT: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = NetworkSimActionClient()

    action_client.send_goal('drone_1')

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
