#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pygame
from std_msgs.msg import String

from cabo_bot.cabo_voice_segmenter import AudioSegmenter
from cabo_bot.cabo_constants import *
import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

import warnings
warnings.filterwarnings("ignore", message="FP16 is not supported on CPU; using FP32 instead")



class CaboVoiceNode(Node, AudioSegmenter):
    def __init__(self):
        Node.__init__(self, 'cabo_voice_node')
        AudioSegmenter.__init__(self)
        
        self.command_publisher = self.create_publisher(String, 'voice_command', 10)
        self.init_audio()

       
    def init_audio(self):
        # pygame.mixer.init()
        # pygame.mixer.music.load(VOICE_4_VOICE_INITIALIZING)
        # pygame.mixer.music.play()
        self.initialize_stream()

        msg = String()
        msg.data = CMD_VOICE_2_STATE_INITED
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published command: {CMD_VOICE_2_STATE_INITED}")
        self.start()  # separate thread
        # pygame.mixer.music.load(VOICE_4_VOICE_INITED)
        # pygame.mixer.music.play()

    def on_text_recognized(self, text):
        self.get_logger().error('------------------------------------')
        text = text.lower()
        self.get_logger().error(text)
        self.get_logger().error('------------------------------------')
        for key, value in COMMANDS.items():
            if key in text:
                msg = String()
                msg.data = value
                self.command_publisher.publish(msg)
                self.get_logger().info(f"Published command: {value}")
                return


def main(args=None):
    rclpy.init(args=args)
    node = CaboVoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
