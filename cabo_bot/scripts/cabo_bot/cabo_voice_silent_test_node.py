#!/usr/bin/env python3
from http.client import OK
import rclpy
import time
import sys
import tty
import termios
import threading
from rclpy.node import Node
from cabo_bot.cabo_constants import *
from std_msgs.msg import String
import time

print("Silent test start now!")
print("Press 'q' to exit!")

keyQueue = []
old_setting = termios.tcgetattr(sys.stdin)

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def getKeyBoard():
    global keyQueue
    while True:
        c = readchar()
        keyQueue.append(c)


t1 =threading.Thread(target=getKeyBoard)
t1.setDaemon(True)
t1.start()



def main(args=None):
    global ctrlMsgs
    rclpy.init(args=args) 
    node = Node("voice_silent_test_node")  

    command_publisher = node.create_publisher(String,"voice_command",10)

    while True:
        msg = String()
        msg.data = CMD_VOICE_2_STATE_INITED
        if len(keyQueue) > 0:
            key = keyQueue.pop(0)
            print(f'key: [{key}]\r\n')
            if key == '0':
                msg.data = CMD_VOICE_2_STATE_INITED
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)

            elif key == '1':
                msg.data = CMD_VOICE_2_STATE_READY
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)
                
            elif key == '8':
                msg.data = CMD_VOICE_2_STATE_FORWARD
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)
                            
            elif key == '2':
                msg.data = CMD_VOICE_2_STATE_BACKWARD
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)

            elif key == '9':
                msg.data = CMD_VOICE_2_STATE_KEEP_FORWARD
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)

            elif key == '5':
                msg.data = CMD_VOICE_2_STATE_STOP
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)

            elif key == '4':
                msg.data = CMD_VOICE_2_STATE_TURN_LEFT
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)

            elif key == '6':
                msg.data = CMD_VOICE_2_STATE_TURN_RIGHT
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)

            elif key == 'b':
                msg.data = CMD_VOICE_2_STATE_BIND
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)
              
            elif key == 'f':
                msg.data = CMD_VOICE_2_STATE_FOLLOW
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)  
              
            elif key == 'd':
                msg.data = CMD_VOICE_2_STATE_DANCE
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)  

            elif key == 's':
                msg.data = CMD_VOICE_2_STATE_SIT
                print(f'CMD: [{msg.data}]\r\n')
                command_publisher.publish(msg)

            elif key == 'q':
                break

        else:
            pass
            
        time.sleep(0.08)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
    print('exit!')
    rclpy.shutdown() 


if __name__ == '__main__':
    main()