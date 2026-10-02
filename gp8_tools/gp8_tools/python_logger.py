import csv
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

joint_order = ['joint_1_s', 'joint_2_l', 'joint_3_u', 
                'joint_4_r', 'joint_5_b', 'joint_6_t']

class JointStateLogger(Node):
    def __init__(self):
        super().__init__('python_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.first_timestamp = None
        output_directory = (Path.home()/'ros2_ws'/'src'/'gp8_ros2'/'data')
        output_file = output_directory / 'gp8_data_out.csv'
        self.file = open(output_file, 'w', newline='')

        self.writer = csv.writer(self.file)
        
        header = ['time (s)']
        for joint_name in joint_order:
            header.append(f'{joint_name}_position_rad')
        for joint_name in joint_order:
            header.append(f'{joint_name}_velocity_rad_s')

        header += [
            'tool0_x_m',
            'tool0_y_m',
            'tool0_z_m',
            'tool0_qx',
            'tool0_qy',
            'tool0_qz',
            'tool0_qw',
        ]

        self.writer.writerow(header)
        self.file.flush()

        self.subscription = self.create_subscription(
                    JointState,
                    '/joint_states',
                    self.joint_state_callback,
                    10
                )

        self.get_logger().info('Logging /joint_states to data/gp8_data_out.csv ...')

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))

        required_positions = all(joint_name in positions for joint_name in joint_order)
        required_velocities = all(joint_name in velocities for joint_name in joint_order)

        # Check for no data
        if not required_positions or not required_velocities:
            return

        timestamp = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        try:
            tool_transform = self.tf_buffer.lookup_transform('base_link', 'tool0',
                                                             Time.from_msg(msg.header.stamp))
        except:
            return

        if self.first_timestamp is None:
            self.first_timestamp = timestamp

        elapsed_time = timestamp - self.first_timestamp

        row = [f'{elapsed_time:.9f}']
        for joint_name in joint_order:
            row.append(f'{positions[joint_name]:.9f}')
        for joint_name in joint_order:
            row.append(f'{velocities[joint_name]:.9f}')

        translation = tool_transform.transform.translation
        rotation = tool_transform.transform.rotation

        row.extend([
            f'{translation.x:.9f}',
            f'{translation.y:.9f}',
            f'{translation.z:.9f}',
            f'{rotation.x:.9f}',
            f'{rotation.y:.9f}',
            f'{rotation.z:.9f}',
            f'{rotation.w:.9f}',
        ])

        self.writer.writerow(row)
        self.file.flush()

    def close_file(self):
        if not self.file.closed:
            self.close_file()

def main(args=None):
    rclpy.init(args=args)
    node = JointStateLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_file()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()