
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='decision_maker',
            executable='decision_maker_node',
            name='decision_maker_node'
        ),
    ])