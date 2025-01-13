
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='whisper_asr',
            executable='whisper_asr_node',
            name='whisper_asr_node',
        ),
    ])