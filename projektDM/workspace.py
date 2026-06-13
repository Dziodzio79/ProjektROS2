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


# =========================================================
# Parametry geometryczne manipulatora
# =========================================================

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

# Punkt końcówki efektora w lokalnym układzie efektora.
# To jest JEDEN punkt roboczy, a nie cała geometria efektora.
TOOL_TIP_LOCAL = np.array([0.44, 0.0, 0.0, 1.0])

# =========================================================
# Parametry generowania workspace
# =========================================================

# Liczba losowych konfiguracji. Większa liczba = gładszy obrys.
NUM_SAMPLES = 70000

# Podział kierunków. Końcowo liczba punktów będzie maksymalnie:
# AZ_BINS * EL_BINS, czyli tu około 7200 punktów, a nie setki tysięcy.
AZ_BINS = 120
EL_BINS = 60

# Środek, względem którego szukamy zewnętrznej powłoki.
# Dla tej wizualizacji dobrze brać poziom pierwszego przegubu.
SHELL_CENTER = np.array([0.0, 0.0, ARM_1_Z])

# Zakresy kątów przegubów
Q_MIN = -math.pi
Q_MAX = math.pi


class WorkspaceNode(Node):

    def __init__(self):
        super().__init__("workspace_node")

        self.pub = self.create_publisher(Marker, "/workspace_marker", 10)

        self.points = None
        self.timer = self.create_timer(1.0, self.tick)

        self.get_logger().info("Workspace node started: outer shell only")

    def fk_to_effector_frame(self, q1, q2, q3):
        """
        Kinematyka prosta do układu efektora.
        """

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

    def tool_tip_position(self, q1, q2, q3):
        """
        Zwraca pozycję jednego punktu końcówki efektora.
        Nie dodajemy całej geometrii efektora, bo to sztucznie pogrubia
        i wypełnia workspace.
        """

        T_eff = self.fk_to_effector_frame(q1, q2, q3)
        p = T_eff @ TOOL_TIP_LOCAL
        return p[:3]

    def add_point(self, pts, x, y, z):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        pts.append(p)

    def add_shell_candidate(self, shell, p):
        """
        Zostawia tylko najbardziej zewnętrzny punkt w danym kierunku.

        Dzielimy przestrzeń na kierunki azymut/elewacja.
        Dla każdego kierunku zapamiętujemy punkt o największej odległości
        od SHELL_CENTER. Dzięki temu środek workspace nie jest wypełniany.
        """

        v = p - SHELL_CENTER
        r = float(np.linalg.norm(v))

        if r < 1e-9:
            return

        az = math.atan2(v[1], v[0])
        el = math.asin(max(-1.0, min(1.0, v[2] / r)))

        az_idx = int((az + math.pi) / (2.0 * math.pi) * AZ_BINS)
        el_idx = int((el + math.pi / 2.0) / math.pi * EL_BINS)

        az_idx = max(0, min(AZ_BINS - 1, az_idx))
        el_idx = max(0, min(EL_BINS - 1, el_idx))

        key = (az_idx, el_idx)

        if key not in shell or r > shell[key][0]:
            shell[key] = (r, p)

    def generate(self):
        """
        Generuje sam zewnętrzny obrys workspace.

        Poprzednia wersja wypełniała środek, bo dodawała:
        - losowe punkty wewnątrz objętości,
        - całą geometrię efektora dla każdej konfiguracji.

        Ta wersja:
        - liczy tylko punkt końcówki efektora,
        - losuje dużo konfiguracji,
        - zostawia wyłącznie najbardziej zewnętrzne punkty w danych kierunkach.
        """

        shell = {}

        rng = np.random.default_rng()

        # Losowe konfiguracje
        for _ in range(NUM_SAMPLES):
            q1 = rng.uniform(Q_MIN, Q_MAX)
            q2 = rng.uniform(Q_MIN, Q_MAX)
            q3 = rng.uniform(Q_MIN, Q_MAX)

            p = self.tool_tip_position(q1, q2, q3)
            self.add_shell_candidate(shell, p)

        # Dodatkowe konfiguracje regularne, żeby złapać skraje dokładniej.
        sweep = np.linspace(Q_MIN, Q_MAX, 73)

        for q1 in sweep:
            for q2 in sweep[::3]:
                for q3 in sweep[::3]:
                    p = self.tool_tip_position(q1, q2, q3)
                    self.add_shell_candidate(shell, p)

        for q2 in sweep:
            for q1 in sweep[::3]:
                for q3 in sweep[::3]:
                    p = self.tool_tip_position(q1, q2, q3)
                    self.add_shell_candidate(shell, p)

        for q3 in sweep:
            for q1 in sweep[::3]:
                for q2 in sweep[::3]:
                    p = self.tool_tip_position(q1, q2, q3)
                    self.add_shell_candidate(shell, p)

        pts = []

        for _, p in shell.values():
            self.add_point(pts, p[0], p[1], p[2])

        return pts

    def tick(self):

        if self.points is None:
            self.get_logger().info("Generating workspace outer shell...")
            self.points = self.generate()
            self.get_logger().info(f"Done: {len(self.points)} shell points")

        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()

        m.ns = "workspace"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD

        # Mniejsze punkty, żeby nie zalewały wizualizacji.
        m.scale.x = 0.012
        m.scale.y = 0.012

        m.color.r = 0.1
        m.color.g = 0.4
        m.color.b = 1.0
        m.color.a = 0.75

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
