import os
from benchmark_utilities.analysis import BenchmarkAnalyzer
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import sys
import argparse
import json

def main(argv):
    
    # Parse the command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware_device_type', type=str, help='Hardware Device Type (e.g. cpu or fpga)', default ='cpu')
    parser.add_argument('--trace_path', type=str, help='Path to trace files (e.g. /tmp/analysis/trace)', default = '/root/.ros/tracing/end_to_end_perception')
    parser.add_argument('--metrics', type=str, help='List of metrics to be analyzed (e.g. latency and/or throughput)', default = ['latency'])
    parser.add_argument('--integrated', type=str, help='Integrated or separated version of the Resize and Rectify nodes (only for fpga now)', default='false') 
    args = parser.parse_args(argv)

    # Get the values of the arguments
    hardware_device_type = args.hardware_device_type
    trace_path = args.trace_path
    metrics_string = args.metrics
    metrics_elements = [element.strip() for element in metrics_string.strip("[]").split(",")]
    metrics = json.loads(json.dumps(metrics_elements))
    integrated = args.integrated

    # Instantiate the class
    ba = BenchmarkAnalyzer('e1_quadruped_control', hardware_device_type)

    if hardware_device_type == 'cpu':
        # add parameters for analyzing the traces
        ## using message header id
        target_chain = [
            "robotperf_benchmarks:robotperf_twist_input_cb_init",
            "robotperf_benchmarks:robotperf_twist_input_cb_fini",
            "robotperf_benchmarks:robotperf_msg_received_1", 
            "robotperf_benchmarks:robotperf_msg_published_1",
            "robotperf_benchmarks:robotperf_joint_trajectory_input_cb_init",
            "robotperf_benchmarks:robotperf_joint_trajectory_input_cb_fini"
        ]

        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_twist_input_cb_init",
                "name_disambiguous": "robotperf_benchmarks:robotperf_twist_input_cb_init",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "salmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_twist_input_cb_fini",
                "name_disambiguous": "robotperf_benchmarks:robotperf_twist_input_cb_fini",
                "colors_fg": "red",
                "colors_fg_bokeh": "darksalmon",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )

        #  -----
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_msg_received_1",
                "name_disambiguous": "robotperf_benchmarks:robotperf_msg_received_1",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )

        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_msg_published_1",
                "name_disambiguous": "robotperf_benchmarks:robotperf_msg_published_1",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        # ----


        # ba.add_target(
        #     {
        #         "name": "robotperf_benchmarks:robotperf_joint_trajectory_output_cb_init",
        #         "name_disambiguous": "robotperf_benchmarks:robotperf_joint_trajectory_output_cb_init",
        #         "colors_fg": "red",
        #         "colors_fg_bokeh": "lightcoral",
        #         "layer": "userland",
        #         "label_layer": 4,
        #         "marker": "plus",
        #     }
        # )


        # ba.add_target(
        #     {
        #         "name": "robotperf_benchmarks:robotperf_joint_trajectory_output_cb_fini",
        #         "name_disambiguous": "robotperf_benchmarks:robotperf_joint_trajectory_output_cb_fini",
        #         "colors_fg": "yellow",
        #         "colors_fg_bokeh": "darkred",
        #         "layer": "userland",
        #         "label_layer": 4,
        #         "marker": "plus",
        #     }
        # )

        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_joint_trajectory_input_cb_init",
                "name_disambiguous": "robotperf_benchmarks:robotperf_joint_trajectory_input_cb_init",
                "colors_fg": "red",
                "colors_fg_bokeh": "lightcoral",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        ba.add_target(
            {
                "name": "robotperf_benchmarks:robotperf_joint_trajectory_input_cb_fini",
                "name_disambiguous": "robotperf_benchmarks:robotperf_joint_trajectory_input_cb_fini",
                "colors_fg": "yellow",
                "colors_fg_bokeh": "darkred",
                "layer": "userland",
                "label_layer": 4,
                "marker": "plus",
            }
        )
        
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
    
    for metric in metrics:
        if metric == 'latency':
            ba.analyze_latency(trace_path, add_power, filter_type='name') #name filter if no header keys
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power, filter_type='name')
        elif metric == 'power': 
            if num_metrics == 0: # launch independently iff no other metric is requested
                total_consumption = ba.analyze_power(trace_path)
                print("The average consumption is {} W".format(total_consumption))
        else:
            print('The metric ' + metric + ' is not yet implemented\n')
    

def generate_launch_description():
    # Declare the launch arguments
    hardware_device_type_arg = DeclareLaunchArgument(
        'hardware_device_type',
        default_value='cpu',
        description='Hardware Device Type (e.g. cpu or fpga)'
    )

    trace_path_arg = DeclareLaunchArgument(
        'trace_path',
        default_value='/root/.ros/tracing/end_to_end_perception',
        description='Path to trace files (e.g. /tmp/analysis/trace)'
    )

    metrics_arg = DeclareLaunchArgument(
        'metrics',
        default_value=['latency'],
        description='List of metrics to be analyzed (e.g. latency and/or throughput)'
    )
    
    integrated_arg = DeclareLaunchArgument(
        'integrated',
        default_value="false",
        description='Integrated or separated version of the Resize and Rectify nodes (only for fpga now)'
    )

    # Create the launch description
    ld = LaunchDescription()
    
    # Define the ExecuteProcess action to run the Python script
    analyzer = ExecuteProcess(
        cmd=[
            'python3', "src/benchmarks/benchmarks/end_to_end/e1_autonomous_quadruped/launch/analyze_quadruped_control.launch.py",
            '--hardware_device_type', LaunchConfiguration('hardware_device_type'),
            '--trace_path', LaunchConfiguration('trace_path'),
            '--metrics', LaunchConfiguration('metrics'),
            '--integrated', LaunchConfiguration('integrated')],
        output='screen'
    )

    # Add the declared launch arguments to the launch description
    ld.add_action(hardware_device_type_arg)
    ld.add_action(trace_path_arg)
    ld.add_action(metrics_arg)
    ld.add_action(integrated_arg)
    
    # Add the ExecuteProcess action to the launch description
    ld.add_action(analyzer)

    return ld

if __name__ == '__main__':
    main(sys.argv[1:])
    