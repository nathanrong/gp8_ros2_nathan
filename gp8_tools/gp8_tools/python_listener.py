import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('python_listener')
        self.latest_positions = {}

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.print_positions)
        self.get_logger().info('Listening to /joint_states')

    def joint_state_callback(self, msg):
        self.latest_positions = dict(zip(msg.name, msg.position))

    def print_positions(self):
        if not self.latest_positions:
            self.get_logger().info('Waiting for joint state messages...')
            return

        joint_order = ['joint_1_s', 'joint_2_l', 'joint_3_u', 
                     'joint_4_r', 'joint_5_b', 'joint_6_t']

        output = []

        for joint_name in joint_order:
            if joint_name in self.latest_positions:
                position = self.latest_positions[joint_name]
                output.append(f'{joint_name}: {position:.3f} rad')

        self.get_logger().info(' | '.join(output))

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()