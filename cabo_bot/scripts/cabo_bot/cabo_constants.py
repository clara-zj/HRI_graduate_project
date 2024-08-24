

# Frame Size
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


# Command Constants: From [voice node] to [state node]
CMD_VOICE_2_STATE_INITED = 'voice inited'
CMD_VOICE_2_STATE_READY = 'get ready'

CMD_VOICE_2_STATE_FORWARD = 'forward'
CMD_VOICE_2_STATE_BACKWARD = 'backward'
CMD_VOICE_2_STATE_KEEP_FORWARD = 'keep forward'

CMD_VOICE_2_STATE_TURN_LEFT = 'left'
CMD_VOICE_2_STATE_TURN_RIGHT = 'right'

CMD_VOICE_2_STATE_DANCE = 'dance'

CMD_VOICE_2_STATE_BIND = 'bind'
CMD_VOICE_2_STATE_FOLLOW = 'follow'
CMD_VOICE_2_STATE_STOP = 'stop'
CMD_VOICE_2_STATE_SIT = 'sit'


# audio to voice command
COMMANDS = {
    "hello robot": CMD_VOICE_2_STATE_BIND,
    "follow me": CMD_VOICE_2_STATE_FOLLOW,

    "can you dance": CMD_VOICE_2_STATE_DANCE,
    "back": CMD_VOICE_2_STATE_BACKWARD,
    "forward": CMD_VOICE_2_STATE_FORWARD,
    "keep going": CMD_VOICE_2_STATE_KEEP_FORWARD,
    "stop": CMD_VOICE_2_STATE_STOP,
    "turn left": CMD_VOICE_2_STATE_TURN_LEFT,
    "turn right": CMD_VOICE_2_STATE_TURN_RIGHT,
    "sit down": CMD_VOICE_2_STATE_SIT,
    "get ready": CMD_VOICE_2_STATE_READY,
}


# Command Constants: From [state node] to [motion node]
CMD_STATE_2_MOTION_STANDBY = 'standby'
CMD_STATE_2_MOTION_FORWARD = 'forward'
CMD_STATE_2_MOTION_KEEP_FORWARD = 'keep forward'
CMD_STATE_2_MOTION_BACKWARD = 'backward'
CMD_STATE_2_MOTION_TURN_LEFT = 'left'
CMD_STATE_2_MOTION_TURN_RIGHT = 'right'
CMD_STATE_2_MOTION_DANCE = 'dance'
CMD_STATE_2_MOTION_STOP = 'stop'
CMD_STATE_2_MOTION_SIT = 'sit'


# Command Constants: From [motion node] to [state node]
CMD_MOTION_2_STATE_SHORT_FINISHED = 'short finished'
CMD_MOTION_2_STATE_STOP_SENT = 'stop sent'


# user name
USER_NAME = 'clara'
# USER_NAME = 'diablo'


# Audio filenames
VOICE_4_VOICE_INITED = '/home/'+USER_NAME+'/cabo_ros2/src/cabo_bot/resource/initiated.mp3'
VOICE_4_VOICE_OK = '/home/'+USER_NAME+'/cabo_ros2/src/cabo_bot/resource/affirmative.mp3'
VOICE_4_VOICE_INCORRECT = '/home/'+USER_NAME+'/cabo_ros2/src/cabo_bot/resource/incorrect.mp3'
VOICE_4_VOICE_VISION = '/home/'+USER_NAME+'/cabo_ros2/src/cabo_bot/resource/vision.mp3'
VOICE_4_VOICE_VISION_FAILED = '/home/'+USER_NAME+'/cabo_ros2/src/cabo_bot/resource/vision_failed.mp3'
VOICE_4_VOICE_SURE = '/home/'+USER_NAME+'/cabo_ros2/src/cabo_bot/resource/ok_sure.mp3'
VOICE_4_VOICE_OF_COURSE = '/home/'+USER_NAME+'/cabo_ros2/src/cabo_bot/resource/of_course.mp3'


# Command Constants: From [state node] to [vision node]
CMD_STATE_2_VISION_BIND = 'bind'
CMD_STATE_2_VISION_TRACK = 'track'
CMD_STATE_2_VISION_STOP = 'stop'


# Command Constants: From [vision node] to [state node]
CMD_VISION_2_STATE_SPINNING = 'spinning'
CMD_VISION_2_STATE_TRAIN_SUCCESS = 'train success'
CMD_VISION_2_STATE_TRAIN_FAIL = 'train fail'
# CMD_VISION_2_STATE_LOCATION = 'location_xxx_xxx'





# Constants for vision commands
