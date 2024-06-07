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
    
    vision_ball_node = Node(
        package="vision",
        executable="vision_ball"
    )
    
    vision_silo_node = Node(
        package="vision",
        executable="vision_silo"
    )
    
    ld.add_action(mobile_node)
    ld.add_action(state_node)
    ld.add_action(vision_ball_node)
    ld.add_action(vision_silo_node)
    
    return ld