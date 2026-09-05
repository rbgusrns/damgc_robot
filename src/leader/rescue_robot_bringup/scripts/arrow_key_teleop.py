#!/usr/bin/env python3

"""Keyboard teleoperation for the leader robot."""

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ARROW_KEYS = {
    "\x1b[A": "forward",
    "\x1b[B": "reverse",
    "\x1b[C": "right",
    "\x1b[D": "left",
}


COMBINATION_KEYS = {
    "q": "forward_left",
    "w": "forward_right",
    "a": "reverse_left",
    "s": "reverse_right",
}


GRIPPER_KEYS = {
    "z": "GRIPPER_OPEN",
    "x": "GRIPPER_CLOSE",
    "c": "LIFT_UP",
    "v": "LIFT_DOWN",
}


class ArrowKeyTeleop(Node):

    def __init__(self):
        super().__init__("arrow_key_teleop")

        self.declare_parameter(
            "command_topic",
            "/leader/cmd_vel",
        )
        self.declare_parameter(
            "gripper_command_topic",
            "/leader/gripper/manual_command",
        )
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("key_timeout", 0.25)
        self.declare_parameter("publish_rate", 20.0)

        if not sys.stdin.isatty():
            raise RuntimeError(
                "arrow_key_teleop must run in an interactive terminal"
            )

        command_topic = str(
            self.get_parameter("command_topic").value
        )
        gripper_topic = str(
            self.get_parameter("gripper_command_topic").value
        )
        publish_rate = float(
            self.get_parameter("publish_rate").value
        )

        if publish_rate <= 0.0:
            raise ValueError(
                "publish_rate must be greater than zero"
            )

        self._velocity_publisher = self.create_publisher(
            Twist,
            command_topic,
            10,
        )

        self._gripper_publisher = self.create_publisher(
            String,
            gripper_topic,
            10,
        )

        self._stdin_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(
            self._stdin_fd
        )

        self._input_buffer = ""
        self._motion = None
        self._last_motion_key_time = 0.0
        self._terminal_configured = False

        tty.setcbreak(self._stdin_fd)
        self._terminal_configured = True

        self.create_timer(
            1.0 / publish_rate,
            self._update,
        )

        self.get_logger().info(
            f"Velocity topic: {command_topic}"
        )
        self.get_logger().info(
            f"Gripper topic: {gripper_topic}"
        )

        self._print_controls()

    @staticmethod
    def _print_controls():
        print(
            "\n"
            "============= LEADER CONTROL =============\n"
            " UP              : forward\n"
            " DOWN            : reverse\n"
            " LEFT            : rotate left\n"
            " RIGHT           : rotate right\n"
            "\n"
            " Q               : forward + left\n"
            " W               : forward + right\n"
            " A               : reverse + left\n"
            " S               : reverse + right\n"
            "\n"
            " Z               : open gripper\n"
            " X               : close gripper\n"
            " C               : lift up\n"
            " V               : lift down\n"
            "\n"
            " SPACE           : stop everything\n"
            " Ctrl-C          : exit\n"
            "==========================================\n",
            flush=True,
        )

    def _update(self):
        self._read_available_input()

        velocity, gripper_command = (
            self._commands_for_current_state()
        )

        self._velocity_publisher.publish(velocity)
        self._gripper_publisher.publish(
            String(data=gripper_command)
        )

    def _set_motion(self, motion):
        self._motion = motion
        self._last_motion_key_time = time.monotonic()

    def _read_available_input(self):
        while select.select(
            [self._stdin_fd],
            [],
            [],
            0.0,
        )[0]:
            data = os.read(self._stdin_fd, 32)

            if not data:
                break

            self._input_buffer += data.decode("latin1")

        while self._input_buffer:
            if self._input_buffer.startswith("\x1b"):
                if len(self._input_buffer) < 3:
                    return

                sequence = self._input_buffer[:3]
                self._input_buffer = self._input_buffer[3:]

                motion = ARROW_KEYS.get(sequence)

                if motion is not None:
                    self._set_motion(motion)

                continue

            key = self._input_buffer[0].lower()
            self._input_buffer = self._input_buffer[1:]

            if key == " ":
                self._motion = None
                self._last_motion_key_time = 0.0
                continue

            motion = COMBINATION_KEYS.get(key)

            if motion is None:
                motion = GRIPPER_KEYS.get(key)

            if motion is not None:
                self._set_motion(motion)

    def _commands_for_current_state(self):
        velocity = Twist()
        gripper_command = "STOP"

        timeout = float(
            self.get_parameter("key_timeout").value
        )

        command_expired = (
            self._motion is None
            or time.monotonic()
            - self._last_motion_key_time
            > timeout
        )

        if command_expired:
            self._motion = None
            return velocity, gripper_command

        linear_speed = float(
            self.get_parameter("linear_speed").value
        )
        angular_speed = float(
            self.get_parameter("angular_speed").value
        )

        if self._motion == "forward":
            velocity.linear.x = linear_speed

        elif self._motion == "reverse":
            velocity.linear.x = -linear_speed

        elif self._motion == "left":
            velocity.angular.z = angular_speed

        elif self._motion == "right":
            velocity.angular.z = -angular_speed

        elif self._motion == "forward_left":
            velocity.linear.x = linear_speed
            velocity.angular.z = angular_speed

        elif self._motion == "forward_right":
            velocity.linear.x = linear_speed
            velocity.angular.z = -angular_speed

        elif self._motion == "reverse_left":
            velocity.linear.x = -linear_speed
            velocity.angular.z = angular_speed

        elif self._motion == "reverse_right":
            velocity.linear.x = -linear_speed
            velocity.angular.z = -angular_speed

        elif self._motion in GRIPPER_KEYS.values():
            gripper_command = self._motion

        return velocity, gripper_command

    def stop(self):
        self._motion = None
        self._velocity_publisher.publish(Twist())
        self._gripper_publisher.publish(
            String(data="STOP")
        )

    def restore_terminal(self):
        if self._terminal_configured:
            termios.tcsetattr(
                self._stdin_fd,
                termios.TCSADRAIN,
                self._terminal_settings,
            )
            self._terminal_configured = False

    def destroy_node(self):
        self.restore_terminal()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ArrowKeyTeleop()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
                time.sleep(0.05)

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
