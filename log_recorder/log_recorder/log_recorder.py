import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from datetime import datetime
import os
import subprocess
import signal


class LogRecorder(Node):
    def __init__(self):
        super().__init__('log_recorder')

        # Declare parameters
        self.declare_parameter('topic_list', [""], 
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="A list of topics to record",
            )
        )

        # Initialize services
        self.start_service = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
        self.stop_service = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)

        # Declare members
        self.is_recording = False
        self.process = None

        self.get_logger().info(f"Log Recorder Node is initialized.")

    def start_recording_callback(self, request, response):
        if self.is_recording:
            response.success = False
            response.message = "Recording is already in progress."
            return response

        # Generate unique filename based on date and time
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"log_recording_{timestamp}"
        fileDir = "./"
        self.get_logger().info(f"Generated Filename: {fileDir + filename}")

        # Get the topics list from parameters
        topic_list = self.get_parameter('topic_list').get_parameter_value().string_array_value
        if not topic_list:
            response.success = False
            response.message = "No topics specified for recording."
            return response
        
        # Prepare the command to start rosbag recording
        topics = ' '.join(topic_list)  # Join the topics into a single string
        command = f"ros2 bag record {topics} -o {fileDir + filename}"
        
        # The os.setsid() is passed in the argument preexec_fn so it's run after the fork() and before  exec() to run the shell.
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

        self.is_recording = True
        response.success = True
        response.message = f"Started recording to: {fileDir + filename}."
        return response

    def stop_recording_callback(self, request, response):
        if not self.is_recording:
            response.success = False
            response.message = "No recording in progress."
            return response

        self.get_logger().info("Preparing to stop....")
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        
        self.is_recording = False

        response.success = True
        response.message = "Stopped recording."

        return response

def main(args=None):
    rclpy.init(args=args)
    node = LogRecorder()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
