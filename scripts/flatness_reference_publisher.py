#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool

from px4adrc.msg import FlatTrajectoryReference


Array3 = np.ndarray
TrajectorySample = Tuple[Array3, Array3, Array3]


def normalize(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm < 1.0e-9:
        return fallback.copy()
    return vector / norm


def quaternion_normalize(q_wxyz: np.ndarray) -> np.ndarray:
    return normalize(
        np.asarray(q_wxyz, dtype=float),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
    )


def align_quaternion_sign(q_wxyz: np.ndarray, anchor_q_wxyz: np.ndarray) -> np.ndarray:
    q = quaternion_normalize(q_wxyz)
    anchor_q = quaternion_normalize(anchor_q_wxyz)
    if float(np.dot(q, anchor_q)) < 0.0:
        q = -q
    return q


def quaternion_conjugate(q_wxyz: np.ndarray) -> np.ndarray:
    q = quaternion_normalize(q_wxyz)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quaternion_multiply(q1_wxyz: np.ndarray, q2_wxyz: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = quaternion_normalize(q1_wxyz)
    w2, x2, y2, z2 = quaternion_normalize(q2_wxyz)
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    trace = np.trace(matrix)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return quaternion_normalize(
            np.array(
                [
                    0.25 * scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ],
                dtype=float,
            )
        )

    diag = np.diag(matrix)
    index = int(np.argmax(diag))
    if index == 0:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ],
            dtype=float,
        )
    elif index == 1:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        quaternion = np.array(
            [
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ],
            dtype=float,
        )
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        quaternion = np.array(
            [
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ],
            dtype=float,
        )
    return quaternion_normalize(quaternion)


def quaternion_from_accel_and_yaw_ned(
    acceleration_ned: np.ndarray,
    yaw: float,
    gravity: float,
) -> np.ndarray:
    gravity_ned = np.array([0.0, 0.0, gravity], dtype=float)
    z_body_ned = normalize(gravity_ned - np.asarray(acceleration_ned, dtype=float), gravity_ned)
    x_course_ned = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=float)
    y_body_ned = np.cross(z_body_ned, x_course_ned)
    if np.linalg.norm(y_body_ned) < 1.0e-9:
        y_body_ned = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=float)
    y_body_ned = normalize(y_body_ned, np.array([0.0, 1.0, 0.0], dtype=float))
    x_body_ned = normalize(np.cross(y_body_ned, z_body_ned), np.array([1.0, 0.0, 0.0], dtype=float))
    return quaternion_from_rotation_matrix(np.column_stack((x_body_ned, y_body_ned, z_body_ned)))


def body_rates_from_quaternion_samples(
    q_now_wxyz: np.ndarray,
    q_next_wxyz: np.ndarray,
    dt: float,
) -> np.ndarray:
    if dt <= 1.0e-9:
        return np.zeros(3, dtype=float)

    q_now = quaternion_normalize(q_now_wxyz)
    q_next = align_quaternion_sign(q_next_wxyz, q_now)
    q_delta = quaternion_multiply(quaternion_conjugate(q_now), q_next)
    q_delta = quaternion_normalize(q_delta)
    angle = 2.0 * math.atan2(np.linalg.norm(q_delta[1:]), max(q_delta[0], 1.0e-9))
    if angle < 1.0e-9:
        return np.zeros(3, dtype=float)

    axis = normalize(q_delta[1:], np.array([1.0, 0.0, 0.0], dtype=float))
    return axis * (angle / dt)


def body_torque_from_rates(
    body_rates_now: np.ndarray,
    body_rates_next: np.ndarray,
    dt: float,
    inertia_diag: np.ndarray,
) -> np.ndarray:
    omega_now = np.asarray(body_rates_now, dtype=float)
    omega_next = np.asarray(body_rates_next, dtype=float)
    inertia = np.diag(np.asarray(inertia_diag, dtype=float))
    omega_dot = np.zeros(3, dtype=float) if dt <= 1.0e-9 else (omega_next - omega_now) / dt
    return inertia @ omega_dot + np.cross(omega_now, inertia @ omega_now)


def yaw_from_velocity_ned(velocity_ned: np.ndarray, fallback_yaw: float) -> float:
    velocity = np.asarray(velocity_ned, dtype=float)
    if np.linalg.norm(velocity[:2]) < 1.0e-6:
        return float(fallback_yaw)
    return math.atan2(velocity[1], velocity[0])


def omega_from_speed(radius: float, speed: float) -> float:
    return float(speed) / max(float(radius), 1.0e-6)


def omega_from_period(period_s: float) -> float:
    return 2.0 * math.pi / max(float(period_s), 1.0e-6)


def rotation_x(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def apply_tilt(
    position_local: Array3,
    velocity_local: Array3,
    acceleration_local: Array3,
    center_ned: Array3,
    tilt_angle_rad: float,
) -> TrajectorySample:
    rotation = rotation_x(tilt_angle_rad)
    return (
        center_ned + rotation @ position_local,
        rotation @ velocity_local,
        rotation @ acceleration_local,
    )


def circle_ned(
    t: float,
    radius: float,
    speed: float,
    center_ned: Array3,
    z_ned: float,
) -> TrajectorySample:
    omega = omega_from_speed(radius, speed)
    x = center_ned[0] + radius * math.cos(omega * t)
    y = center_ned[1] + radius * math.sin(omega * t)
    vx = -radius * omega * math.sin(omega * t)
    vy = radius * omega * math.cos(omega * t)
    ax = -radius * omega * omega * math.cos(omega * t)
    ay = -radius * omega * omega * math.sin(omega * t)
    return (
        np.array([x, y, z_ned], dtype=float),
        np.array([vx, vy, 0.0], dtype=float),
        np.array([ax, ay, 0.0], dtype=float),
    )


def figure_eight_ned(
    t: float,
    x_amplitude: float,
    y_amplitude: float,
    period_s: float,
    center_ned: Array3,
    z_ned: float,
) -> TrajectorySample:
    omega = omega_from_period(period_s)
    x = center_ned[0] + x_amplitude * math.sin(omega * t)
    y = center_ned[1] + y_amplitude * math.sin(2.0 * omega * t)
    vx = x_amplitude * omega * math.cos(omega * t)
    vy = 2.0 * y_amplitude * omega * math.cos(2.0 * omega * t)
    ax = -x_amplitude * omega * omega * math.sin(omega * t)
    ay = -4.0 * y_amplitude * omega * omega * math.sin(2.0 * omega * t)
    return (
        np.array([x, y, z_ned], dtype=float),
        np.array([vx, vy, 0.0], dtype=float),
        np.array([ax, ay, 0.0], dtype=float),
    )


def tilted_circle_ned(
    t: float,
    radius: float,
    speed: float,
    center_ned: Array3,
    tilt_angle_rad: float,
) -> TrajectorySample:
    omega = omega_from_speed(radius, speed)
    position_local = np.array(
        [radius * math.cos(omega * t), radius * math.sin(omega * t), 0.0], dtype=float
    )
    velocity_local = np.array(
        [-radius * omega * math.sin(omega * t), radius * omega * math.cos(omega * t), 0.0],
        dtype=float,
    )
    acceleration_local = np.array(
        [
            -radius * omega * omega * math.cos(omega * t),
            -radius * omega * omega * math.sin(omega * t),
            0.0,
        ],
        dtype=float,
    )
    return apply_tilt(position_local, velocity_local, acceleration_local, center_ned, tilt_angle_rad)


def tilted_figure_eight_ned(
    t: float,
    x_amplitude: float,
    y_amplitude: float,
    period_s: float,
    center_ned: Array3,
    tilt_angle_rad: float,
) -> TrajectorySample:
    omega = omega_from_period(period_s)
    position_local = np.array(
        [x_amplitude * math.sin(omega * t), y_amplitude * math.sin(2.0 * omega * t), 0.0],
        dtype=float,
    )
    velocity_local = np.array(
        [
            x_amplitude * omega * math.cos(omega * t),
            2.0 * y_amplitude * omega * math.cos(2.0 * omega * t),
            0.0,
        ],
        dtype=float,
    )
    acceleration_local = np.array(
        [
            -x_amplitude * omega * omega * math.sin(omega * t),
            -4.0 * y_amplitude * omega * omega * math.sin(2.0 * omega * t),
            0.0,
        ],
        dtype=float,
    )
    return apply_tilt(position_local, velocity_local, acceleration_local, center_ned, tilt_angle_rad)


class FlatnessReferencePublisher(Node):
    def __init__(self) -> None:
        super().__init__("px4adrc_reference")

        self.declare_parameter("topics.reference", "/px4adrc/reference")
        self.declare_parameter("topics.start_tracking", "/mission/start_tracking")
        self.declare_parameter("planner.publish_rate_hz", 100.0)
        self.declare_parameter("planner.trajectory_type", "figure_eight")
        self.declare_parameter("planner.radius", 1.5)
        self.declare_parameter("planner.speed", 1.0)
        self.declare_parameter("planner.period_s", 8.0)
        self.declare_parameter("planner.figure8_x_amplitude_m", 1.5)
        self.declare_parameter("planner.figure8_y_amplitude_m", 0.75)
        self.declare_parameter("planner.height_offset_ned", -2.0)
        self.declare_parameter("planner.tilt_angle_deg", 15.0)
        self.declare_parameter("planner.yaw_mode", "track_velocity")
        self.declare_parameter("planner.speed_ramp_time_s", 5.0)
        self.declare_parameter("vehicle.gravity", 9.81)
        self.declare_parameter("vehicle.inertia_diag", [0.02384669, 0.02394962, 0.04399995])

        self.reference_topic = str(self.get_parameter("topics.reference").value)
        self.start_tracking_topic = str(self.get_parameter("topics.start_tracking").value)
        self.publish_rate_hz = float(self.get_parameter("planner.publish_rate_hz").value)
        self.trajectory_type = str(self.get_parameter("planner.trajectory_type").value)
        self.radius = float(self.get_parameter("planner.radius").value)
        self.speed = float(self.get_parameter("planner.speed").value)
        self.period_s = float(self.get_parameter("planner.period_s").value)
        self.figure8_x_amplitude_m = float(
            self.get_parameter("planner.figure8_x_amplitude_m").value
        )
        self.figure8_y_amplitude_m = float(
            self.get_parameter("planner.figure8_y_amplitude_m").value
        )
        self.height_offset_ned = float(self.get_parameter("planner.height_offset_ned").value)
        self.tilt_angle_rad = math.radians(
            float(self.get_parameter("planner.tilt_angle_deg").value)
        )
        self.yaw_mode = str(self.get_parameter("planner.yaw_mode").value)
        self.speed_ramp_time_s = float(self.get_parameter("planner.speed_ramp_time_s").value)
        self.gravity = float(self.get_parameter("vehicle.gravity").value)
        self.inertia_diag = np.asarray(
            self.get_parameter("vehicle.inertia_diag").value,
            dtype=float,
        )
        self.derivative_dt = max(1.0e-3, min(0.02, 1.0 / max(self.publish_rate_hz, 1.0)))

        self.enabled = False
        self.start_time_s = 0.0
        self.latest_position_ned: Optional[np.ndarray] = None
        self.latest_yaw_rad: Optional[float] = None
        self.active_center_ned: Optional[np.ndarray] = None
        self.active_base_height_ned = 0.0
        self.locked_yaw_rad = 0.0
        self.last_pose_frame_warning_ns = 0

        self.reference_pub = self.create_publisher(FlatTrajectoryReference, self.reference_topic, 10)
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos_profile_sensor_data,
        )
        start_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_sub = self.create_subscription(
            Bool,
            self.start_tracking_topic,
            self.start_callback,
            start_qos,
        )
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self.timer_callback,
        )

    def odometry_callback(self, msg: VehicleOdometry) -> None:
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_pose_frame_warning_ns > 5_000_000_000:
                self.get_logger().warning(
                    f"vehicle_odometry pose_frame is not NED: {msg.pose_frame}"
                )
                self.last_pose_frame_warning_ns = now_ns
            return

        self.latest_position_ned = np.asarray(msg.position, dtype=float)
        q_wxyz = quaternion_normalize(np.asarray(msg.q, dtype=float))
        w, x, y, z = q_wxyz
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.latest_yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    def start_callback(self, msg: Bool) -> None:
        if msg.data:
            if self.enabled:
                return
            if self.latest_position_ned is None:
                self.get_logger().warning("Start requested before odometry was received")
                return
            if self.yaw_mode == "fixed":
                self.locked_yaw_rad = (
                    float(self.latest_yaw_rad) if self.latest_yaw_rad is not None else 0.0
                )
            self.active_center_ned = self.latest_position_ned.copy()
            self.active_center_ned -= self.trajectory_start_offset_ned()
            self.active_base_height_ned = float(self.latest_position_ned[2]) + self.height_offset_ned
            self.enabled = True
            self.start_time_s = self.get_clock().now().nanoseconds / 1.0e9
            self.get_logger().info("Flatness trajectory publisher enabled")
            return

        if self.enabled:
            self.enabled = False
            self.get_logger().info("Flatness trajectory publisher disabled")

    def trajectory_start_offset_ned(self) -> np.ndarray:
        if self.trajectory_type in ("circle", "tilted_circle"):
            return np.array([self.radius, 0.0, 0.0], dtype=float)
        return np.zeros(3, dtype=float)

    def reparameterized_kinematics(self, elapsed_s: float) -> Tuple[float, float, float]:
        elapsed = max(float(elapsed_s), 0.0)
        ramp = float(self.speed_ramp_time_s)
        if ramp <= 0.0:
            return elapsed, 1.0, 0.0
        if elapsed >= ramp:
            return elapsed - 0.5 * ramp, 1.0, 0.0

        u = elapsed / ramp
        path_time = ramp * (u ** 3 - 0.5 * u ** 4)
        path_time_dot = 3.0 * u ** 2 - 2.0 * u ** 3
        path_time_ddot = (6.0 * u - 6.0 * u ** 2) / ramp
        return path_time, path_time_dot, path_time_ddot

    def sample_path(self, path_time_s: float) -> TrajectorySample:
        if self.active_center_ned is None:
            raise RuntimeError("Trajectory anchor missing")

        if self.trajectory_type == "circle":
            return circle_ned(
                path_time_s,
                self.radius,
                self.speed,
                self.active_center_ned,
                self.active_base_height_ned,
            )
        if self.trajectory_type == "figure_eight":
            return figure_eight_ned(
                path_time_s,
                self.figure8_x_amplitude_m,
                self.figure8_y_amplitude_m,
                self.period_s,
                self.active_center_ned,
                self.active_base_height_ned,
            )
        if self.trajectory_type == "tilted_circle":
            return tilted_circle_ned(
                path_time_s,
                self.radius,
                self.speed,
                self.active_center_ned,
                self.tilt_angle_rad,
            )
        if self.trajectory_type == "tilted_figure_eight":
            return tilted_figure_eight_ned(
                path_time_s,
                self.figure8_x_amplitude_m,
                self.figure8_y_amplitude_m,
                self.period_s,
                self.active_center_ned,
                self.tilt_angle_rad,
            )
        raise ValueError(f"Unsupported trajectory type: {self.trajectory_type}")

    def yaw_reference(self, velocity_ned: np.ndarray, fallback_yaw: float) -> float:
        if self.yaw_mode == "fixed":
            return self.locked_yaw_rad
        return yaw_from_velocity_ned(velocity_ned, fallback_yaw)

    def sample_reference_state(self, elapsed_s: float) -> TrajectorySample:
        path_time, path_time_dot, path_time_ddot = self.reparameterized_kinematics(elapsed_s)
        position, path_velocity, path_acceleration = self.sample_path(path_time)
        velocity = path_velocity * path_time_dot
        acceleration = path_acceleration * (path_time_dot ** 2) + path_velocity * path_time_ddot
        return position, velocity, acceleration

    def sample_flat_reference(self, elapsed_s: float) -> dict:
        dt = self.derivative_dt
        position, velocity, acceleration = self.sample_reference_state(elapsed_s)
        _, velocity_p1, acceleration_p1 = self.sample_reference_state(elapsed_s + dt)
        _, velocity_p2, acceleration_p2 = self.sample_reference_state(elapsed_s + 2.0 * dt)

        fallback_yaw = self.locked_yaw_rad if self.yaw_mode == "fixed" else 0.0
        yaw = self.yaw_reference(velocity, fallback_yaw)
        yaw_p1 = self.yaw_reference(velocity_p1, yaw)
        yaw_p2 = self.yaw_reference(velocity_p2, yaw_p1)

        q_now = quaternion_from_accel_and_yaw_ned(acceleration, yaw, self.gravity)
        q_next = quaternion_from_accel_and_yaw_ned(acceleration_p1, yaw_p1, self.gravity)
        q_next = align_quaternion_sign(q_next, q_now)
        q_next2 = quaternion_from_accel_and_yaw_ned(acceleration_p2, yaw_p2, self.gravity)
        q_next2 = align_quaternion_sign(q_next2, q_next)

        body_rates = body_rates_from_quaternion_samples(q_now, q_next, dt)
        body_rates_next = body_rates_from_quaternion_samples(q_next, q_next2, dt)
        body_torque = body_torque_from_rates(body_rates, body_rates_next, dt, self.inertia_diag)

        return {
            "position": position,
            "velocity": velocity,
            "acceleration": acceleration,
            "yaw": yaw,
            "body_rates": body_rates,
            "body_torque": body_torque,
        }

    def timer_callback(self) -> None:
        if not self.enabled:
            return

        now = self.get_clock().now()
        elapsed_s = now.nanoseconds / 1.0e9 - self.start_time_s
        reference = self.sample_flat_reference(elapsed_s)

        msg = FlatTrajectoryReference()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "ned"
        msg.position_ned.x = float(reference["position"][0])
        msg.position_ned.y = float(reference["position"][1])
        msg.position_ned.z = float(reference["position"][2])
        msg.velocity_ned.x = float(reference["velocity"][0])
        msg.velocity_ned.y = float(reference["velocity"][1])
        msg.velocity_ned.z = float(reference["velocity"][2])
        msg.acceleration_ned.x = float(reference["acceleration"][0])
        msg.acceleration_ned.y = float(reference["acceleration"][1])
        msg.acceleration_ned.z = float(reference["acceleration"][2])
        msg.body_rates_frd.x = float(reference["body_rates"][0])
        msg.body_rates_frd.y = float(reference["body_rates"][1])
        msg.body_rates_frd.z = float(reference["body_rates"][2])
        msg.body_torque_frd.x = float(reference["body_torque"][0])
        msg.body_torque_frd.y = float(reference["body_torque"][1])
        msg.body_torque_frd.z = float(reference["body_torque"][2])
        msg.yaw = float(reference["yaw"])
        self.reference_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlatnessReferencePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
