from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    mobile_node = Node(
        package="localization",
        executable="mobile_node"
    )
    
    state_node = Node(
        package="planing",
        executable="state_node"
    )
    
    vision_ball = Node(
        package="vision",
        executable="vision_ball"
    )
    
    ld.add_action(mobile_node)
    ld.add_action(state_node)
    ld.add_action(vision_ball)
    
    return ld