#!/usr/bin/env python3

from motion_msgs.msg import MotionCtrl
from rclpy.node import Node
import rclpy
import time

import os
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'

# CMD_REPEAT_COUNT = 10
CMD_SLEEP_GAP = 0.05
WALKING_SPEED = 0.6
SHORT_MOVE_GAP = 1.5
FOLLOW_GAP = 0.1
DANCE_GAP = 0.5

class CaboMotionLimo:
    def __init__(self, publisher, logger):
        self.ctrlMsgs = MotionCtrl()
        self.ctrlMsgs.value.forward = 0.0
        self.publisher = publisher
        self.log = logger

    def send_msg(self, forward_speed = 0.0, 
                 left_speed = 0.0, 
                 mode_mark = False,
                 pitch = 0.0):
        self.ctrlMsgs.value.forward = forward_speed
        self.ctrlMsgs.value.left = left_speed
        self.ctrlMsgs.mode_mark = mode_mark
        # self.ctrlMsgs.value.pitch = pitch
        
        self.publisher.publish(self.ctrlMsgs)
        time.sleep(CMD_SLEEP_GAP)
        pass

    def on_standby(self):
        self.ctrlMsgs.mode.stand_mode = True
        self.send_msg(mode_mark=True)
        self.ctrlMsgs.value.up = 0.5
        self.ctrlMsgs.value.pitch = -0.5
        self.send_msg()
        self.log('Stand up mode.')

    def on_sit(self):
        self.ctrlMsgs.value.up = 0.0
        self.ctrlMsgs.value.pitch = 0.0
        self.send_msg()
        self.ctrlMsgs.mode.stand_mode = False
        self.send_msg(mode_mark=True)
        self.log('Sit up mode.')


    def on_forward(self):
        self.send_msg(forward_speed=WALKING_SPEED)
        self.log(f'Forward.')
        time.sleep(SHORT_MOVE_GAP)

        self.send_msg()
        self.log(f'Stop.')

    def on_backward(self):
        self.send_msg(forward_speed=-WALKING_SPEED)
        self.log(f'Backward.')
        time.sleep(SHORT_MOVE_GAP)

        self.send_msg()
        self.log(f'Stop.')

    def on_turn_left(self):
        self.send_msg(left_speed=0.5)
        self.log(f'Turn left.')
        time.sleep(SHORT_MOVE_GAP)

        self.send_msg()
        self.log(f'Stop.')

    def on_turn_right(self):
        self.send_msg(left_speed=-0.5)
        self.log(f'Turn right.')
        time.sleep(SHORT_MOVE_GAP)

        self.send_msg()
        self.log(f'Stop.')

    def on_follow(self, turn_left, forward):
        if turn_left != 0:
            self.send_msg(left_speed=turn_left)
            time.sleep(FOLLOW_GAP)
        if forward!= 0:
            self.send_msg(orward_speed=forward)
            time.sleep(FOLLOW_GAP)
        
        self.log(f'Follow mode, turn left: {turn_left}, forward: {forward}')
      
    def on_long_forward(self):
        self.send_msg(forward_speed=WALKING_SPEED)
        self.log(f'Long Forward.')

    def on_dance(self):
        self.log(f'Start to dance...')

        #stand up
        self.ctrlMsgs.value.up = 1.0
        self.send_msg()

        # look down
        self.send_msg(pitch = -0.5)

        # forward
        self.send_msg(pitch = -0.5, forward_speed = 1.0)
        time.sleep(DANCE_GAP)

        # look front
        self.send_msg()

        # swing head
        self.ctrlMsgs.value.roll = -1.0
        self.send_msg()
        time.sleep(DANCE_GAP)
        self.ctrlMsgs.value.roll = 1.0
        self.send_msg()
        time.sleep(DANCE_GAP)
        self.ctrlMsgs.value.roll = 0.0
        self.send_msg()
        time.sleep(DANCE_GAP)

        # look up
        self.send_msg(pitch = 0.5)

        # backward
        self.send_msg(pitch = 0.5, forward_speed = -1.0)
        time.sleep(DANCE_GAP)

        # stop
        self.send_msg()



class TestNde(Node):
    def __init__(self):
        super().__init__('cabo_motion_diablo')
        self.teleop_cmd = self.create_publisher(MotionCtrl, 'diablo/MotionCmd', 3)
        self.diablo = CaboMotionDiablo(self.teleop_cmd,
                                       self.get_logger().info)

    def test(self):
        self.diablo.on_standby()
        self.diablo.on_forward()
        self.diablo.on_forward()
        self.diablo.on_backward()
        self.diablo.on_dance()


def main(args=None):
    rclpy.init(args=args)
    node = TestNde()
    
    try:
        node.test()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
