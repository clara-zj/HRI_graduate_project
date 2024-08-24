#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from motion_msgs.msg import MotionCtrl
import time
from rclpy.qos import qos_profile_sensor_data

from cabo_bot.cabo_constants import *
from cabo_bot.cabo_motion_diablo import CaboMotionDiablo

import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'


class CaboMotionNode(Node):
    def __init__(self):
        super().__init__('cabo_motion_node')
        self.command_publisher = self.create_publisher(String, 'motion_command', 10)
        self.teleop_cmd = self.create_publisher(MotionCtrl, 'diablo/MotionCmd', 3)
        self.cmd = CaboMotionDiablo(self.teleop_cmd,
                                    self.get_logger().info)

        self.state_subscriber = self.create_subscription(
            String,
            'cabo_command',
            self.state_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("CaboMotionNode has been started.")

    def report_finish(self):
        msg = String()
        msg.data = CMD_MOTION_2_STATE_SHORT_FINISHED
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published command: {CMD_MOTION_2_STATE_SHORT_FINISHED}")

    def state_callback(self, msg):
        # stand by
        if msg.data == CMD_STATE_2_MOTION_STANDBY:
            self.cmd.on_standby()
        elif msg.data == CMD_STATE_2_MOTION_SIT:
            self.cmd.on_sit()
        # short move
        elif msg.data in [CMD_STATE_2_MOTION_FORWARD, 
                        CMD_STATE_2_MOTION_BACKWARD,
                        CMD_STATE_2_MOTION_DANCE,
                        CMD_STATE_2_MOTION_TURN_LEFT,
                        CMD_STATE_2_MOTION_TURN_RIGHT]:
            self.get_logger().info(f"Received state: {msg.data}")
            
            if msg.data == CMD_STATE_2_MOTION_FORWARD:
                self.cmd.on_forward()
            elif msg.data == CMD_STATE_2_MOTION_BACKWARD:
                self.cmd.on_backward()  
            elif msg.data == CMD_STATE_2_MOTION_DANCE:
                self.cmd.on_dance()  
            elif msg.data == CMD_STATE_2_MOTION_TURN_LEFT:
                self.cmd.on_turn_left()
            elif msg.data == CMD_STATE_2_MOTION_TURN_RIGHT:
                self.cmd.on_turn_right() 
                
            self.report_finish()
        # long move
        elif msg.data == CMD_STATE_2_MOTION_KEEP_FORWARD:
            self.cmd.on_long_forward()
        else:
            # track mode
            texts = msg.data.split(',')
            if len(texts) == 1:
                texts = texts[0]
            if len(texts) == 3 and texts[0] == 'FOLLOW':
                print('Track Mode')
                turn_left, forward = texts[1:]
                self.cmd.on_follow(int(turn_left), int(forward))
            else:
                print(f'incorrect msg: [{texts}] !!!!!!!!')

        self.get_logger().info(f"Received state: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = CaboMotionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()