#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import pygame

from cabo_bot.cabo_state_machine import StateController
from cabo_bot.cabo_constants import *
import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

################################################################
class CaboStateMachineNode(Node):
    def __init__(self):
        super().__init__('cabo_state_node')
        self.get_logger().info('cabo_state_node is running')

        self.vision_node_spinning = False
        self.state_controller = StateController(self)
        self.command_publisher = self.create_publisher(String, 'cabo_command', 10)

        self.voice_cmd_subscriber = self.create_subscription(
            String,
            'voice_command',
            self.voice_cmd_callback,
            10
        )

        self.vision_cmd_subscriber = self.create_subscription(
            String,
            'vision_command',
            self.vision_cmd_callback,
            qos_profile_sensor_data
        )

        self.motion_cmd_subscriber = self.create_subscription(
            String,
            'motion_command',
            self.motion_cmd_callback,
            10
        )

        pygame.mixer.init()

    def play_audio(self, audio_file):
        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()

    def voice_cmd_callback(self, msg):
        self.get_logger().info(f"Received voice command: {msg.data}")
        rt = None
        if msg.data == CMD_VOICE_2_STATE_READY:
            rt = self.state_controller.handle_started()
        elif msg.data == CMD_VOICE_2_STATE_FORWARD:
            rt = self.state_controller.handle_forward()
        elif msg.data == CMD_VOICE_2_STATE_BACKWARD:
            rt = self.state_controller.handle_backward()
        elif msg.data == CMD_VOICE_2_STATE_DANCE:
            rt = self.state_controller.handle_dance()
        
            # if self.vision_node_spinning:
            #     self.state_controller.handle_bind()
            # else:
            #     self.get_logger().error("Vision node is not spinning")
        elif msg.data == CMD_VOICE_2_STATE_STOP:
            rt = self.state_controller.handle_stop()
        elif msg.data == CMD_VOICE_2_STATE_KEEP_FORWARD:
            rt = self.state_controller.handle_long_forward()          
        elif msg.data == CMD_VOICE_2_STATE_FOLLOW:
            rt = self.state_controller.handle_follow()       
        elif msg.data == CMD_VOICE_2_STATE_TURN_LEFT:
            rt = self.state_controller.handle_left()
        elif msg.data == CMD_VOICE_2_STATE_TURN_RIGHT:
            rt = self.state_controller.handle_right()
        elif msg.data == CMD_VOICE_2_STATE_SIT:
            rt = self.state_controller.handle_sit()
        elif msg.data == CMD_VOICE_2_STATE_BIND:
            rt = self.state_controller.handle_bind()
        
        if rt is True:
            if msg.data == CMD_VOICE_2_STATE_BIND:
                self.play_audio(VOICE_4_VOICE_SURE)
            elif msg.data == CMD_VOICE_2_STATE_DANCE:
                self.play_audio(VOICE_4_VOICE_OF_COURSE)
            else:   
                self.play_audio(VOICE_4_VOICE_OK)
            return
        elif rt is False:
            self.play_audio(VOICE_4_VOICE_INCORRECT)
            return

        if msg.data == CMD_VOICE_2_STATE_INITED:
            if self.state_controller.handle_started():
                self.play_audio(VOICE_4_VOICE_INITED)


    def vision_cmd_callback(self, msg):
        self.get_logger().info(f"Received vision command: {msg.data}")
        if msg.data == CMD_VISION_2_STATE_SPINNING:
            self.vision_node_spinning = True

        elif msg.data == CMD_VISION_2_STATE_TRAIN_SUCCESS:
            self.state_controller.handle_binded()
            self.play_audio(VOICE_4_VOICE_VISION)

        elif msg.data == CMD_VISION_2_STATE_TRAIN_FAIL:
            self.state_controller.handle_bind_faild()
            self.play_audio(VOICE_4_VOICE_VISION_FAILED)

        elif self.state_controller.is_following():
            texts = msg.data.split(',')
            if len(texts) == 3 and texts[0] == 'TRACK':
                turn_left, forward = texts[1:]
                self.publish_cmd(f"FOLLOW,{turn_left},{forward}")

    def motion_cmd_callback(self, msg):
        self.get_logger().info(f"Received motion command: {msg.data}")
        if msg.data == CMD_MOTION_2_STATE_SHORT_FINISHED:
            self.state_controller.handle_short_finished()

    def publish_cmd(self, text):
        msg = String()
        msg.data = text
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Published command: {text}")

    def move_standby(self):
        self.publish_cmd(CMD_STATE_2_MOTION_STANDBY)

    def move_forward(self):
        self.publish_cmd(CMD_STATE_2_MOTION_FORWARD)

    def move_backward(self):
        self.publish_cmd(CMD_STATE_2_MOTION_BACKWARD)

    def move_dance(self):
        self.publish_cmd(CMD_STATE_2_MOTION_DANCE)

    def move_long_forward(self):
        self.publish_cmd(CMD_STATE_2_MOTION_KEEP_FORWARD)

    def move_stop(self):
        self.publish_cmd(CMD_STATE_2_MOTION_STOP)

    def move_bind(self):
        self.publish_cmd(CMD_STATE_2_VISION_BIND)

    def move_follow(self):
        self.publish_cmd(CMD_STATE_2_VISION_TRACK)
    
    def move_stop_vision(self):
        self.publish_cmd(CMD_STATE_2_VISION_STOP)

    def move_left(self):
        self.publish_cmd(CMD_STATE_2_MOTION_TURN_LEFT)
    
    def move_right(self):
        self.publish_cmd(CMD_STATE_2_MOTION_TURN_RIGHT)

    def move_sit(self):
        self.publish_cmd(CMD_STATE_2_MOTION_SIT)


################################################################
def main(args=None):
    rclpy.init(args=args)
    node = CaboStateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

