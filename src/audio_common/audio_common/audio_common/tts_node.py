#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import os
import wave
import pyaudio
import tempfile
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from audio_common_msgs.msg import AudioStamped
from audio_common_msgs.action import TTS
from audio_common.utils import data_to_msg
from audio_common.utils import get_msg_chunk


class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("tts_node")

        self.declare_parameters("", [
            ("chunk", 4096),
            ("frame_id", ""),
        ])

        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        self.espeak_cmd = "espeak -v{} -s{} -a{} -w {} '{}'"

        self.player_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile_sensor_data)

        # action server
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            TTS,
            "say",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("TTS node started")

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request: ServerGoalHandle) -> int:
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> None:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> TTS.Result:

        request: TTS.Goal = goal_handle.request

        text = request.text
        language = request.language
        rate = request.rate * 350
        volume = request.volume * 200

        # create audio file
        audio_file = tempfile.NamedTemporaryFile(mode="w+")
        os.system(self.espeak_cmd.format(
            language, rate, volume, audio_file.name, text))

        # pub audio
        audio_file.seek(0)
        wf = wave.open(audio_file.name, "rb")
        audio_file.close()
        audio_format = pyaudio.get_format_from_width(wf.getsampwidth())

        frequency = wf.getframerate() / self.chunk
        pub_rate = self.create_rate(frequency)

        # send audio data
        data = wf.readframes(self.chunk)
        while data:
            if not goal_handle.is_active:
                return TTS.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return TTS.Result()

            audio_msg = data_to_msg(data, audio_format)
            if audio_msg is None:
                self.get_logger().error(f"Format {audio_format} unknown")
                self._goal_handle.abort()
                return TTS.Result()

            msg = AudioStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.audio = audio_msg
            msg.audio.info.channels = wf.getnchannels()
            msg.audio.info.chunk = get_msg_chunk(audio_msg)
            msg.audio.info.rate = wf.getframerate()

            self.player_pub.publish(msg)
            pub_rate.sleep()

            data = wf.readframes(self.chunk)

        result = TTS.Result()
        result.text = text
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
