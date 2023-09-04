#
#    @@@@@@@@@@@@@@@@@@@@
#    @@@@@@@@@&@@@&&@@@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@
#    @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
#    @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
#    @@@@@ @@  @@    @@@@ Author: Martiño Crespo <martinho@accelerationrobotics.com>
#    @@@@@@@@@&@@@@@@@@@@
#    @@@@@@@@@@@@@@@@@@@@
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from benchmark_utilities.analysis import BenchmarkAnalyzer
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from ros2_benchmark.data_uploader import DataUploader
from ros2_benchmark.device_info import DeviceInfo
from ros2_benchmark.ros2_benchmark_config import ROS2BenchmarkConfig


import sys
import argparse
import json

rosbag = os.environ.get('ROSBAG') # Example: 'perception/r2b_cafe'
package = os.environ.get('PACKAGE') # Example: 'a7_pointcloud_to_laserscan'
type = os.environ.get('TYPE') # Example: 'grey'
metric = os.environ.get('METRIC') # Example: 'latency'

def main(argv):

    # Parse the command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware_device_type', type=str, help='Hardware Device Type (e.g. cpu or fpga)', default ='cpu')
    parser.add_argument('--trace_path', type=str, help='Path to trace files (e.g. /tmp/analysis/trace)', default = '/tmp/analysis/trace')
    parser.add_argument('--metrics', type=str, help='List of metrics to be analyzed (e.g. latency and/or throughput)', default = ['latency'])
    parser.add_argument('--upload_data', type=str, help='Set to "true" to upload data', default='false')
    args = parser.parse_args(argv)

    # Get the values of the arguments
    hardware_device_type = args.hardware_device_type
    trace_path = args.trace_path
    metrics_string = args.metrics
    upload_data = True if args.upload_data.lower() == 'true' else False
    metrics_elements = [element.strip() for element in metrics_string.strip("[]").split(",")]
    metrics = json.loads(json.dumps(metrics_elements))
 
    # Instantiate the class
    ba = BenchmarkAnalyzer('a7_pointcloud_to_laserscan', hardware_device_type)

    if hardware_device_type == 'cpu':
        target_chain = [
        # "ros2:callback_start",
        'robotperf_benchmarks:robotperf_pointcloud_input_cb_init',
        'robotperf_benchmarks:robotperf_pointcloud_input_cb_fini',
        # 'ros2:callback_end',
        # "ros2:callback_start",
        'robotperf_benchmarks:robotperf_laserscan_output_cb_init',
        'robotperf_benchmarks:robotperf_laserscan_output_cb_fini',
        # "ros2:callback_end",
        ]

        # add parameters for analyzing the traces
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_pointcloud_input_cb_init",
                "name_disambiguous": "robotperf_benchmarks:robotperf_pointcloud_input_cb_init",
                "colors_fg": "blue",
                "colors_fg_bokeh": "silver",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_pointcloud_input_cb_fini",
                "name_disambiguous": "robotperf_benchmarks:robotperf_pointcloud_input_cb_fini",
                "colors_fg": "blue",
                "colors_fg_bokeh": "darkgray",
                "layer": "benchmark",
                "label_layer": 5,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_laserscan_output_cb_init",
                "name_disambiguous": "robotperf_benchmarks:robotperf_laserscan_output_cb_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "red",
                "layer": "benchmark",
                "label_layer": 5,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_laserscan_output_cb_fini",
                "name_disambiguous": "robotperf_benchmarks:robotperf_laserscan_output_cb_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "lavender",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
    else:
        print('The hardware device type ' + hardware_device_type + ' is not yet implemented\n')

    num_metrics = 0 # initialize the metric count
    add_power = False # initialize the boolean
    for metric in metrics:
        if metric == 'power':
            add_power = True
            ba.add_power(
            {
                "name": "robotcore_power:robotcore_power_output_cb_fini",
                "name_disambiguous": "robotcore_power:robotcore_power_output_cb_fini",
                "colors_fg": "blue",
                "colors_fg_bokeh": "silver",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
            )
        else:
            num_metrics += 1 # it will be larger than 0 if other metrics besides power are desired
    
    cpu_profiler_raw_data = None
    latency_raw_data = None
    power_raw_data = None
    for metric in metrics:
        if metric == 'latency':
            ba.analyze_latency(trace_path, add_power)
            if upload_data:
                latency_raw_data = ba.get_raw_latency_data()
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power)
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')
    
    if upload_data:
        device_info = DeviceInfo()
        config = ROS2BenchmarkConfig()
        uploader = DataUploader(config.auth_json_path)
        uploader.upload_data(
            google_sheet_url=config.google_sheet_url,
            benchmark_name=package,
            method_type='grey', # Always grey if using tracepoints
            hardware=device_info.get_device_name(),
            fps='30',
            rosbag_path=rosbag,
            latency_data=latency_raw_data,
            cpu_usage_data=cpu_profiler_raw_data, # Will get uploaded during the ros2_benchmark_test
            power_data=power_raw_data,
        )
  
def generate_launch_description():
    # Declare the launch arguments
    hardware_device_type_arg = DeclareLaunchArgument(
        'hardware_device_type',
        default_value='cpu',
        description='Hardware Device Type (e.g. cpu or fpga)'
    )

    trace_path_arg = DeclareLaunchArgument(
        'trace_path',
        default_value='/tmp/analysis/trace',
        description='Path to trace files (e.g. /tmp/analysis/trace)'
    )

    metrics_arg = DeclareLaunchArgument(
        'metrics',
        default_value=['latency'],
        description='List of metrics to be analyzed (e.g. latency and/or throughput)'
    )

    upload_data_arg = DeclareLaunchArgument(
        'upload_data',
        default_value='false',
        description='Set to "true" to upload data'
    )

    
    # Create the launch description
    ld = LaunchDescription()
    
    # Define the ExecuteProcess action to run the Python script
    analyzer = ExecuteProcess(
        cmd=[
            'python3', "src/benchmarks/benchmarks/perception/a7_pointcloud_to_laserscan/launch/analyze_a7_pointcloud_to_laserscan.launch.py",
            '--hardware_device_type', LaunchConfiguration('hardware_device_type'),
            '--trace_path', LaunchConfiguration('trace_path'),
            '--metrics', LaunchConfiguration('metrics'),
            '--upload_data', LaunchConfiguration('upload_data')],
        output='screen'
    )

    # Add the declared launch arguments to the launch description
    ld.add_action(hardware_device_type_arg)
    ld.add_action(trace_path_arg)
    ld.add_action(metrics_arg)
    ld.add_action(upload_data_arg)
    
    # Add the ExecuteProcess action to the launch description
    ld.add_action(analyzer)

    return ld

if __name__ == '__main__':
    main(sys.argv[1:])
    