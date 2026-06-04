#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import numpy as np


class JointStateStepCLI(Node):
    def __init__(self):
        super().__init__('joint_state_step_cli')

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint']
        self.positions = [0.0] * len(self.joint_names)

        self.get_logger().info("CLI JointState publisher (INPUT W STOPNIACH)")

        threading.Thread(target=self.input_loop, daemon=True).start()
        self.create_timer(0.1, self.publish)

    def input_loop(self):
        while True:
            print("\nPodaj kąty w STOPNIACH")

            new_positions = []

            for i, name in enumerate(self.joint_names):
                while True:
                    try:
                        deg = float(input(f"Oś {i+1} ({name}) [deg] = "))
                        rad = np.deg2rad(deg)
                        new_positions.append(rad)
                        break
                    except ValueError:
                        print("Błąd: wpisz liczbę")

            self.positions = new_positions
            print(f"Ustawiono (rad): {self.positions}")

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateStepCLI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
