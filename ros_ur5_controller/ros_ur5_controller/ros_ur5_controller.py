import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState

from .ur5_robot import UR5Robot, rotvec_to_quaternion

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class RosUR5Controller(Node):
    def __init__(self):
        super().__init__("ros_ur5_controller")

        self._desired_velocity = None
        self._measured_pose = None
        self._measured_joints = None

        self._declare_parameters()
        self._setup_communication()
        self._setup_publishers_subscribers()

        control_period = 1.0 / self.control_rate

        self.timer = self.create_timer(control_period, self._control_and_state_callback)

    def _declare_parameters(self):
        self.host = self.declare_parameter("host", "192.168.137.1").get_parameter_value().string_value
        self.port = self.declare_parameter("port", 30010).get_parameter_value().integer_value
        self.tcp = self.declare_parameter("tcp", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.payload = self.declare_parameter("payload", 0.0).get_parameter_value().double_value
        self.acc = self.declare_parameter("acc", 0.5).get_parameter_value().double_value
        self.control_rate = self.declare_parameter("control_rate", 50.0).get_parameter_value().double_value

        self.desired_velocity_topic = (
            self.declare_parameter("desired_velocity_topic", "/desired_velocity").get_parameter_value().string_value
        )
        self.measured_pose_topic = self.declare_parameter("measured_pose_topic", "/measured_pose").get_parameter_value().string_value
        self.measured_joint_states_topic = (
            self.declare_parameter("measured_joint_states_topic", "/measured_joint_states").get_parameter_value().string_value
        )
        self.measured_wrench_topic = self.declare_parameter("measured_wrench_topic", "/measured_wrench").get_parameter_value().string_value
        self.measured_velocity_topic = (
            self.declare_parameter("measured_velocity_topic", "/measured_velocity").get_parameter_value().string_value
        )

    def _setup_communication(self):
        self.get_logger().info(f"Waiting for UR5 to connect on {self.host}:{self.port} ...")
        self._robot = UR5Robot(
            host=self.host,
            port=self.port,
            tcp=list(self.tcp),
            payload=self.payload,
            logger=self.get_logger(),
        )
        self.get_logger().info("Connected to UR5.")

    def _setup_publishers_subscribers(self):
        self.desired_velocity_subscription = self.create_subscription(
            Twist, self.desired_velocity_topic, self._desired_velocity_callback, 10
        )

        self.measured_pose_publisher = self.create_publisher(PoseStamped, self.measured_pose_topic, 10)
        self.measured_joint_states_publisher = self.create_publisher(JointState, self.measured_joint_states_topic, 10)
        self.measured_wrench_publisher = self.create_publisher(WrenchStamped, self.measured_wrench_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(TwistStamped, self.measured_velocity_topic, 10)

    def _desired_velocity_callback(self, msg: Twist):
        self._desired_velocity = msg

    def _control_and_state_callback(self):
        stamp = self.get_clock().now().to_msg()

        pose = self._robot.getl()
        if len(pose) >= 6:
            self._measured_pose = pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = "ur5_base_link"
            pose_msg.pose.position.x = pose[0]
            pose_msg.pose.position.y = pose[1]
            pose_msg.pose.position.z = pose[2]
            qx, qy, qz, qw = rotvec_to_quaternion(pose[3], pose[4], pose[5])
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            self.measured_pose_publisher.publish(pose_msg)

        if self._desired_velocity is None:
            return

        twist = self._desired_velocity
        vx = twist.linear.x
        vy = twist.linear.y
        vz = twist.linear.z
        wx = twist.angular.x
        wy = twist.angular.y
        wz = twist.angular.z

        velocity = [vx, vy, vz, wx, wy, wz]
        self._robot.speedl_no_block(velocity, acc=self.acc)

        # joints = self._robot.getj()
        # if len(joints) >= 6:
        #     self._measured_joints = joints
        #     joint_msg = JointState()
        #     joint_msg.header.stamp = stamp
        #     joint_msg.name = JOINT_NAMES
        #     joint_msg.position = joints[:6]
        #     self.measured_joint_states_publisher.publish(joint_msg)

        # wrench = self._robot.get_forces()
        # if len(wrench) >= 6:
        #     wrench_msg = WrenchStamped()
        #     wrench_msg.header.stamp = stamp
        #     wrench_msg.header.frame_id = "ur5_tcp"
        #     wrench_msg.wrench.force.x = wrench[0]
        #     wrench_msg.wrench.force.y = wrench[1]
        #     wrench_msg.wrench.force.z = wrench[2]
        #     wrench_msg.wrench.torque.x = wrench[3]
        #     wrench_msg.wrench.torque.y = wrench[4]
        #     wrench_msg.wrench.torque.z = wrench[5]
        #     self.measured_wrench_publisher.publish(wrench_msg)

        # velocity = self._robot.getlv()
        # if len(velocity) >= 6:
        #     twist_msg = TwistStamped()
        #     twist_msg.header.stamp = stamp
        #     twist_msg.header.frame_id = "ur5_tcp"
        #     twist_msg.twist.linear.x = velocity[0]
        #     twist_msg.twist.linear.y = velocity[1]
        #     twist_msg.twist.linear.z = velocity[2]
        #     twist_msg.twist.angular.x = velocity[3]
        #     twist_msg.twist.angular.y = velocity[4]
        #     twist_msg.twist.angular.z = velocity[5]
        #     self.measured_velocity_publisher.publish(twist_msg)

    def destroy_node(self):
        self.get_logger().info("Closing UR5 connection...")
        if hasattr(self, "_robot") and self._robot is not None:
            self._robot.close()
            self._robot = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = RosUR5Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
