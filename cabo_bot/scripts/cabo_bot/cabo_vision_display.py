#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data

import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'


class CaboVisionDisplayNode(Node):
    def __init__(self):
        super().__init__('vision_display_node')
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/vision_display',
        #     self.image_callback,
        #     10)
        # self.subscription 

        self.subscription = self.create_subscription(
            Image,
            '/vision_display',
            self.image_callback,
            qos_profile_sensor_data)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        frame_id = msg.header.frame_id  # 读取 frame_id
        # print(frame_id)
        if frame_id == '1':
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Vision Output", cv_image)
            # cv2.resizeWindow('Vision Output', 640, 480)
            cv2.waitKey(1)
        else:
            print('distroy window')
            cv2.destroyWindow("Vision Output")
            cv2.destroyAllWindows()
            # cv2.resizeWindow('Vision Output', 1, 1)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CaboVisionDisplayNode()
    rclpy.spin(node)

    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
