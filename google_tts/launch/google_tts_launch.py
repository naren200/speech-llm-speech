
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='google_tts',
            executable='google_tts',
            name='google_tts_node'
        ),
    ])