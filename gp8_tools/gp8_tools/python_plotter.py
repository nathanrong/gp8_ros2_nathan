import math
import time
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

joint_order = ['joint_1_s', 'joint_2_l', 'joint_3_u', 
               'joint_4_r', 'joint_5_b', 'joint_6_t']

class JointStatePlotter(Node):
    def __init__(self):
        super().__init__('python_plotter')

        self.start_time = time.monotonic()
        self.times = []
        self.history = {
            joint_name: []
            for joint_name in joint_order
        }

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Plotting /joint_state position values...')

    def joint_state_callback(self, msg):
        curr_time = time.monotonic() - self.start_time
        positions = dict(zip(msg.name, msg.position))
        self.times.append(curr_time)

        for joint_name in joint_order:
            position = positions.get(joint_name, float('nan'))
            self.history[joint_name].append(position)

def main(args = None):
    rclpy.init(args = args)
    node = JointStatePlotter()

    plt.ion()
    figure, axis = plt.subplots(figsize=[10, 6])
    lines = {}
    for joint_name in joint_order:
        line, = axis.plot([], [], label = joint_name)
        lines[joint_name] = line

    axis.set_title('Joint Positions')
    axis.set_xlabel('Time (s)')
    axis.set_ylabel('Joint Position (deg)')
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    plt.show(block=False)

    try:
        while rclpy.ok() and plt.fignum_exists(figure.number):
            rclpy.spin_once(node, timeout_sec = 0.01)

            if node.times:
                for joint_name in joint_order:
                    pos_degree = [
                        math.degrees(position)
                        for position in node.history[joint_name]
                    ]

                    lines[joint_name].set_data(node.times, pos_degree)
                    latest_time = node.times[-1]

                    axis.set_xlim(
                        max(0.0, latest_time - 10.0),
                        max(10.0, latest_time)
                    )

                    axis.relim()
                    axis.autoscale_view(scalex = False, scaley = True)
                    figure.canvas.draw_idle()
                    figure.canvas.flush_events()

                plt.pause(0.01)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    plt.close(figure)

if __name__ == '__main__':
    main()