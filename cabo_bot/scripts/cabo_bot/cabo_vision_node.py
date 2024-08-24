#!/usr/bin/env python3

import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
from std_msgs.msg import String
import numpy as np
from datetime import datetime
from rclpy.qos import qos_profile_sensor_data

from cabo_bot.cabo_constants import *
from cabo_bot.cabo_vision_person_detector import PersonDetectionSystem
from cabo_bot.cabo_vision_source import VisionSource
import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'


class CaboVisionNode(Node):
    def __init__(self, data_source, person_detection_system):
        super().__init__('cabo_vision_node')
        self.data_source = data_source
        self.command_publisher = self.create_publisher(String, 'vision_command', qos_profile_sensor_data)
        self.image_publisher = self.create_publisher(Image, 'vision_display', qos_profile_sensor_data)
        self.bridge = CvBridge()  # 创建CvBridge实例

        self.person_detection_system = person_detection_system
        self.binded = False

        # notify node is running
        # self.__push_command(CMD_VISION_2_STATE_SPINNING)
        # self.__push_command(CMD_VISION_2_STATE_SPINNING)

        self.state_subscriber = self.create_subscription(String, 'cabo_command',
                                                         self.state_callback, 10)
        
    def state_callback(self, msg):
        if msg.data == CMD_STATE_2_VISION_BIND:
            self.get_logger().info(f"Received cmd: {msg.data}")
            self.binded = False

            if self.person_detection_system.init_vision_system():
                self.binded = True
                self.person_detection_system.on_target_detected = self.on_target_detected
                self.__push_command(CMD_VISION_2_STATE_TRAIN_SUCCESS)
            else:
                self.__push_command(CMD_VISION_2_STATE_TRAIN_FAIL)
            pass
        
        elif msg.data == CMD_STATE_2_VISION_TRACK:
            self.get_logger().info(f"Received cmd: {msg.data}")
            self.person_detection_system.start()
        
        elif msg.data == CMD_STATE_2_VISION_STOP:
            self.get_logger().info(f"Received cmd: {msg.data}")
            self.person_detection_system.stop()
            frame = np.zeros((2, 2, 3), dtype=np.uint8)
            self.show_frame(frame, '0')
        
        else:
            return

    def show_frame(self, frame, mark='1'):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.frame_id = mark
            self.image_publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Failed to convert and publish image: {str(e)}')
        
    def __push_command(self, command):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")

    def on_target_detected(self, turn_left, forwarding):
        current_time = datetime.now()
        formatted_time = current_time.strftime('%Y-%m-%d %H:%M:%S')
        print(f"Timestamp: {formatted_time}, turn_left: {turn_left}, Forwarding: {forwarding}")

        msg = String()
        msg.data = f"TRACK,{turn_left},{forwarding}"
        self.command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    data_source = VisionSource()
    

    person_detection_system = PersonDetectionSystem(data_source)
    cabo_vision_node = CaboVisionNode(data_source, person_detection_system)  
    executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(data_source)
    executor.add_node(cabo_vision_node)
    person_detection_system.show_frame = cabo_vision_node.show_frame


    executor.spin()

    # data_source.start()

    person_detection_system.stop()
    cabo_vision_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
