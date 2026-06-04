#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def T(x, y, z):
    M = np.eye(4)
    M[0, 3] = x
    M[1, 3] = y
    M[2, 3] = z
    return M

def Rx(a):
    c, s = math.cos(a), math.sin(a)
    R = np.eye(4)
    R[1, 1] = c
    R[1, 2] = -s
    R[2, 1] = s
    R[2, 2] = c
    return R

def Ry(a):
    c, s = math.cos(a), math.sin(a)
    R = np.eye(4)
    R[0, 0] = c
    R[0, 2] = s
    R[2, 0] = -s
    R[2, 2] = c
    return R

def Rz(a):
    c, s = math.cos(a), math.sin(a)
    R = np.eye(4)
    R[0, 0] = c
    R[0, 1] = -s
    R[1, 0] = s
    R[1, 1] = c
    return R

def deg(a):
    return a * math.pi / 180.0

BASE_HEIGHT = 0.70

JOINT_RADIUS = 0.085
JOINT_LENGTH = 0.34
ROD_LENGTH = 0.30

ARM_1_Z = BASE_HEIGHT

JOINT_1_VISUAL_Z = JOINT_LENGTH / 2.0

ROD_1_END_X = JOINT_RADIUS + ROD_LENGTH
ROD_2_END_Z = JOINT_1_VISUAL_Z + ROD_LENGTH
ROD_3_END_X = ROD_1_END_X + ROD_LENGTH
ROD_3_Z = ROD_2_END_Z

ARM_2_X = ROD_3_END_X + JOINT_LENGTH / 2.0
ARM_2_Z = ROD_3_Z

ROD_4_END_Y = JOINT_RADIUS + ROD_LENGTH
ARM_3_Y = ROD_4_END_Y + JOINT_LENGTH / 2.0

EFF_FIXED_X = JOINT_RADIUS

EFFECTOR_REACH = 0.47
DOWN_FROM_JOINT1 = 2.0 * ROD_LENGTH + JOINT_LENGTH + EFFECTOR_REACH
LOWER_Z = max(0.0, ARM_1_Z - DOWN_FROM_JOINT1)

def line_x(x0, x1, y, z, n):
    return [(float(x), y, z) for x in np.linspace(x0, x1, n)]

def line_z(x, y, z0, z1, n):
    return [(x, y, float(z)) for z in np.linspace(z0, z1, n)]

def sphere_points(cx, cy, cz, r, rings=True):
    pts = [
        (cx, cy, cz),
        (cx + r, cy, cz),
        (cx - r, cy, cz),
        (cx, cy + r, cz),
        (cx, cy - r, cz),
        (cx, cy, cz + r),
        (cx, cy, cz - r),
    ]

    if rings:
        for a in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            pts.append((cx, cy + r * math.cos(a), cz + r * math.sin(a)))
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), cz))
            pts.append((cx + r * math.cos(a), cy, cz + r * math.sin(a)))

    return pts

def build_effector_points_sparse():
    pts = []
    pts += line_x(0.00, 0.24, 0.0, 0.06, 4)
    pts += line_x(0.00, 0.24, 0.0, -0.06, 4)
    pts += line_z(0.24, 0.0, -0.06, 0.06, 4)
    pts += line_x(0.24, 0.42, 0.0, 0.0, 4)
    pts += sphere_points(0.44, 0.0, 0.0, 0.03, rings=False)
    return pts

def build_effector_points_dense():
    pts = []
    pts += line_x(0.00, 0.24, 0.0, 0.06, 18)
    pts += line_x(0.00, 0.24, 0.0, -0.06, 18)
    pts += line_z(0.24, 0.0, -0.06, 0.06, 18)
    pts += line_x(0.24, 0.42, 0.0, 0.0, 18)
    pts += sphere_points(0.44, 0.0, 0.0, 0.03, rings=True)
    return pts

EFFECTOR_POINTS_SPARSE = build_effector_points_sparse()
EFFECTOR_POINTS_DENSE = build_effector_points_dense()

class WorkspaceNode(Node):

    def __init__(self):
        super().__init__("workspace_node")

        self.pub = self.create_publisher(Marker, "/workspace_marker", 10)

        self.points = None
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info("Workspace node started: effector + smooth downward clearance")

    def fk_to_effector_frame(self, q1, q2, q3):
        Tm = np.eye(4)

        Tm = Tm @ T(0, 0, ARM_1_Z)
        Tm = Tm @ Rz(q1)

        Tm = Tm @ T(ARM_2_X, 0, ARM_2_Z)
        Tm = Tm @ Rx(math.pi)
        Tm = Tm @ Rx(q2)

        Tm = Tm @ T(0, ARM_3_Y, 0)
        Tm = Tm @ Ry(math.pi)
        Tm = Tm @ Ry(q3)

        Tm = Tm @ T(EFF_FIXED_X, 0, 0)
        Tm = Tm @ Rx(math.pi / 2.0)

        return Tm

    def add_point(self, pts, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        pts.append(p)

    def add_effector_geometry(self, pts, q1, q2, q3, dense=False):
        T_eff = self.fk_to_effector_frame(q1, q2, q3)
        eff_points = EFFECTOR_POINTS_DENSE if dense else EFFECTOR_POINTS_SPARSE

        for ex, ey, ez in eff_points:
            p4 = T_eff @ np.array([ex, ey, ez, 1.0])
            self.add_point(pts, p4[0], p4[1], p4[2])

    def add_downward_clearance_from_joint1_level(self, pts):

        max_radius = 1.40

        lower_samples = 14000

        for _ in range(lower_samples):
            r = max_radius * math.sqrt(np.random.uniform(0.0, 1.0))
            a = np.random.uniform(-math.pi, math.pi)

            x = r * math.cos(a)
            y = r * math.sin(a)

            z = np.random.uniform(LOWER_Z, ARM_1_Z)

            self.add_point(pts, x, y, z)

        ground_samples = 2500
        for _ in range(ground_samples):
            r = max_radius * math.sqrt(np.random.uniform(0.0, 1.0))
            a = np.random.uniform(-math.pi, math.pi)
            x = r * math.cos(a)
            y = r * math.sin(a)
            z = LOWER_Z + np.random.uniform(0.0, 0.035)
            self.add_point(pts, x, y, z)

    def generate(self):
        pts = []

        random_samples = 1500

        for _ in range(random_samples):
            q1 = np.random.uniform(-math.pi, math.pi)
            q2 = np.random.uniform(-math.pi, math.pi)
            q3 = np.random.uniform(-math.pi, math.pi)
            self.add_effector_geometry(pts, q1, q2, q3, dense=False)

        mandatory_configs_deg = [
            (0, 0, 0),
            (90, 90, 90),
            (180, 180, 180),
            (-90, -90, -90),
            (-180, -180, -180),
            (90, 0, 0),
            (0, 90, 0),
            (0, 0, 90),
            (180, 0, 0),
            (0, 180, 0),
            (0, 0, 180),
        ]

        for a1, a2, a3 in mandatory_configs_deg:
            self.add_effector_geometry(pts, deg(a1), deg(a2), deg(a3), dense=True)

        sweep = [deg(a) for a in range(-180, 181, 10)]

        for q1 in sweep:
            self.add_effector_geometry(pts, q1, deg(90), deg(90), dense=True)
        for q2 in sweep:
            self.add_effector_geometry(pts, deg(90), q2, deg(90), dense=True)
        for q3 in sweep:
            self.add_effector_geometry(pts, deg(90), deg(90), q3, dense=True)

        for q1 in sweep:
            self.add_effector_geometry(pts, q1, deg(180), deg(180), dense=True)
        for q2 in sweep:
            self.add_effector_geometry(pts, deg(180), q2, deg(180), dense=True)
        for q3 in sweep:
            self.add_effector_geometry(pts, deg(180), deg(180), q3, dense=True)

        self.add_downward_clearance_from_joint1_level(pts)

        return pts

    def tick(self):

        if self.points is None:
            self.get_logger().info("Generating workspace...")
            self.points = self.generate()
            self.get_logger().info(f"Done: {len(self.points)} points")

        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()

        m.ns = "workspace"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD

        m.scale.x = 0.018
        m.scale.y = 0.018

        m.color.r = 0.1
        m.color.g = 0.4
        m.color.b = 1.0
        m.color.a = 0.58

        m.points = self.points
        self.pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
