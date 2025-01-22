import os
import asyncio
import multiprocessing

import json
from flask_cors import CORS
from flask import Flask, request, Response

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


#  flask --app /root/ros2_ws/src/speech-llm-speech/decision_asr/ros_process_control.py run --host=0.0.0.0 --port=5050

if __name__ == '__main__':
    print()
    print('    ERROR !!! This needs to be run with flask: !!!')
    print()
    print('    flask --app /root/ros2_ws/src/speech-llm-speech/decision_asr/ros_process_control.py run --host=0.0.0.0 --port=5000')
    print()
    exit(1)

def generate_launch_description(path, file):
    """Launch the example.launch.py launch file."""
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([path, file]),
            launch_arguments={'node_prefix': 'FOO'}.items(),
        ),
    ])


class Ros2LaunchParent:
    """
    source: # https://github.com/ros2/launch/issues/724
    """

    def __init__(self, launch_description: LaunchDescription):

        self.launch_description = launch_description
        self._stop_event = multiprocessing.Event()
        self._process = None

    def start(self):
        print("Starting ROS")
        self._stop_event = multiprocessing.Event()
        self._process = multiprocessing.Process(target=self._run_process,
                                                args=(self._stop_event, self.launch_description), daemon=True)
        self._process.start()

    def shutdown(self):
        print("Shutting down ROS")
        self._stop_event.set()
        self._process.join()

    def status(self):
        return self._process.is_alive()

    def _run_process(self, stop_event, launch_description):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())
        loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))
        if not launch_task.done():
            asyncio.ensure_future(launch_service.shutdown(), loop=loop)
            loop.run_until_complete(launch_task)


app = Flask(__name__)
CORS(app)
path = '/root/ros2_ws/src/speech-llm-speech/decision_maker/launch/'
file = os.getenv('LAUNCH_FILE', 'decision_maker_launch.py')
ld = generate_launch_description(path, file)
manager = Ros2LaunchParent(ld)
manager.start()


@app.route('/restart_ros', methods=['POST'])
def restart_ros():
    manager.shutdown()
    if not manager.status():
        manager.start()
    success = manager.status()
    return {'success': success, 'msg': ''}, (200 if success else 400)


@app.route('/shutdown_ros', methods=['POST'])
def shutdown_ros():
    manager.shutdown()
    success = not manager.status()
    return {'success': success, 'msg': ''}, (200 if success else 400)
