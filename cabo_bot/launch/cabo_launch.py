"""
Launch file
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    node1 = Node(
        package='cabo_bot',
        executable='cabo_state_node.py',
        name='cabo_state_node',
        output='screen'
    )
    
    node2 = Node(
        package='cabo_bot',
        executable='cabo_voice_node.py',
        name='cabo_voice_node',
        output='screen'
    )

    # node3 = Node(
    #     package='cabo_bot',
    #     executable='cabo_vision_node.py',
    #     name='cabo_vision_node',
    #     output='screen'
    # )

    node4 = Node(
        package='cabo_bot',
        executable='cabo_motion_node.py',
        name='cabo_motion_node',
        output='screen'
    )
    
    # Wait 2 seconds before starting node2, node3, and node4
    # timer_action = TimerAction(
    #     period=1.0,
    #     # actions=[node2, node3, node4]
    #     actions=[node2, node4]
    # )
    
    return LaunchDescription([
        # node1,
        node2,
        # node3,
        node4
    ])