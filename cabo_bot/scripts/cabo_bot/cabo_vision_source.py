#!/usr/bin/env python3

import pyrealsense2 as rs
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
from cabo_bot.cabo_constants import FRAME_WIDTH, FRAME_HEIGHT
from threading import Lock
from datetime import datetime

import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'


class DataSource:
    def get_frame(self):
        raise NotImplementedError("get_frame() must be implemented by subclass")
    
    def start(self):
        pass

    def stop(self):
        pass


class RealSensePipelineSource(DataSource):
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, 30)
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data())


class WebcamSource(DataSource):
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise Exception("Error: Could not open video stream.")

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None
        return cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT)), None


class RealSenseROSTopicSource(Node, DataSource):
    def __init__(self):
        super().__init__('realsense_ros_topic_source')
        self.rgb_topic = '/camera/camera/color/image_raw'
        self.depth_topic = '/camera/camera/depth/image_rect_raw'
        self.rgb_frame = None
        self.depth_frame = None
        self.br = CvBridge()
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.lock = Lock()

    def rgb_callback(self, data):
        with self.lock:
            rgb_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
            self.rgb_frame = cv2.resize(rgb_frame, (FRAME_WIDTH, FRAME_HEIGHT))

    def depth_callback(self, data):
        with self.lock:
            depth_frame = self.br.imgmsg_to_cv2(data, '16UC1')
            self.depth_frame = cv2.resize(depth_frame, (FRAME_WIDTH, FRAME_HEIGHT))

    def get_frame(self):
        with self.lock:
            rgb, depth = self.rgb_frame, self.depth_frame
            self.rgb_frame, self.depth_frame = None, None
            return rgb, depth


# =================================================================================
# Test
def test_pipeline_source():
    source = RealSensePipelineSource()
    try:
        while True:
            rgb_frame, depth_frame = source.get_frame()
            if rgb_frame is not None:
                cv2.imshow('RGB Frame', rgb_frame)
            if depth_frame is not None:
                # Normalize depth image for better visualization
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow('Depth Frame', depth_colormap)
            
            key = cv2.waitKey(1)
            if key == 27:  # Press 'Esc' to exit
                break
    finally:
        source.stop()
        cv2.destroyAllWindows()


def test_ros2_source():
    rclpy.init(args=None)
    source = RealSenseROSTopicSource()
    try:
        source.start()
        while rclpy.ok():
            rgb_frame, depth_frame = source.get_frame()
            if rgb_frame is not None:
                cv2.imshow('RGB Frame', rgb_frame)
            if depth_frame is not None:
                # Normalize depth image for better visualization
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow('Depth Frame', depth_colormap)

            key = cv2.waitKey(1)
            if key == 27:  # Press 'Esc' to exit
                break
    finally:
        source.stop()
        cv2.destroyAllWindows()


def test_multi_threading():
    rclpy.init(args=None)
    node = RealSenseROSTopicSource()
    node2 = rclpy.create_node('simpler_node')
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node2)
    executor.spin()


def test_webcam_source():
    source = WebcamSource()
    try:
        while True:
            rgb_frame, _ = source.get_frame()
            if rgb_frame is not None:
                cv2.imshow('RGB Frame', rgb_frame)
          
            key = cv2.waitKey(1)
            if key == 27:  # Press 'Esc' to exit
                break
    finally:
        source.stop()
        cv2.destroyAllWindows()




if __name__ == '__main__':
    test_pipeline_source()
    # test_ros2_source()
    # test_webcam_source()
    # test_multi_threading()


VisionSource = RealSensePipelineSource
# VisionSource = RealSenseROSTopicSource
# VisionSource = WebcamSource


