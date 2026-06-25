"""
Analyze: Image -> DisparityImage  (stereo_disparity)
Session : e4_stereo_disparity

6-tracepoint chain (kernel has msg_received_1 / msg_published_1):
  T_in      = image_input_cb_init -> msg_received_1
  T_kernel  = msg_received_1 -> msg_published_1
  T_out     = msg_published_1 -> disparity_output_cb_fini
"""
import sys
import argparse
import json
import numpy as np
from benchmark_utilities.analysis import BenchmarkAnalyzer
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def print_latentros_summary(barchart_data, benchmark_name, target_chain):
    """Print LatentROS summary with full chain detail.

    barchart_data is a list of lists, each with 6 values (ms):
      [0] reference (always 0)
      [1] input_cb_init -> input_cb_fini       (T_in component)
      [2] input_cb_fini -> msg_received_1       (DDS transport in)
      [3] msg_received_1 -> msg_published_1     (T_kernel)
      [4] msg_published_1 -> output_cb_init     (DDS transport out)
      [5] output_cb_init -> output_cb_fini      (T_out component)
    """
    data = np.array(barchart_data)
    n = len(data)

    # ---- Detailed 5-interval chain breakdown ----
    labels = [
        'T_in (input component)',
        'DDS_in (input -> kernel)',
        'T_kernel (computation)',
        'DDS_out (kernel -> output)',
        'T_out (output component)',
    ]
    chain_names = [tp.split(':')[-1] for tp in target_chain]

    print(f'\n{"="*80}')
    print(f'  LatentROS Benchmark: {benchmark_name}')
    print(f'  Matched samples: {n}')
    print(f'{"="*80}')

    print(f'\n  6-Tracepoint Chain:')
    for i, name in enumerate(chain_names):
        print(f'    [{i}] {name}')

    interval_labels = [
        '[0]->[1]  T_in (input component)',
        '[1]->[2]  DDS_in (input -> kernel)',
        '[2]->[3]  T_kernel (computation)',
        '[3]->[4]  DDS_out (kernel -> output)',
        '[4]->[5]  T_out (output component)',
    ]

    print(f'\n  Per-Interval Breakdown (5 intervals):')
    print(f'  {"Interval":<38} {"Mean (ms)":>10} {"Median (ms)":>12} {"Std (ms)":>10} {"Mean %":>8}')
    print(f'  {"-"*38} {"-"*10} {"-"*12} {"-"*10} {"-"*8}')

    totals = np.sum(data[:, 1:], axis=1)
    mean_tot = np.mean(totals)

    for i in range(5):
        col = data[:, i + 1]
        m, md, s = np.mean(col), np.median(col), np.std(col)
        pct = (m / mean_tot * 100) if mean_tot > 0 else 0
        print(f'  {interval_labels[i]:<38} {m:10.4f} {md:12.4f} {s:10.4f} {pct:7.1f}%')

    print(f'  {"-"*38} {"-"*10} {"-"*12} {"-"*10} {"-"*8}')
    print(f'  {"Total":<38} {mean_tot:10.4f} {np.median(totals):12.4f} {np.std(totals):10.4f} {"100.0":>7}%')

    # ---- Grouped summary (ROS2 overhead vs kernel) ----
    ros2_in   = data[:, 1] + data[:, 2]
    t_kernel  = data[:, 3]
    ros2_out  = data[:, 4] + data[:, 5]
    total     = ros2_in + t_kernel + ros2_out

    mean_in, med_in   = np.mean(ros2_in),  np.median(ros2_in)
    mean_k,  med_k    = np.mean(t_kernel), np.median(t_kernel)
    mean_out, med_out = np.mean(ros2_out), np.median(ros2_out)
    mean_t,  med_t    = np.mean(total),    np.median(total)

    pct_in  = mean_in  / mean_t * 100 if mean_t > 0 else 0
    pct_k   = mean_k   / mean_t * 100 if mean_t > 0 else 0
    pct_out = mean_out / mean_t * 100 if mean_t > 0 else 0

    print(f'\n  Grouped Summary:')
    print(f'  {"Component":<25} {"Mean (ms)":>10} {"Median (ms)":>12} {"Mean %":>8}')
    print(f'  {"-"*25} {"-"*10} {"-"*12} {"-"*8}')
    print(f'  {"ROS2 Overhead In":<25} {mean_in:10.4f} {med_in:12.4f} {pct_in:7.1f}%')
    print(f'  {"T_kernel":<25} {mean_k:10.4f} {med_k:12.4f} {pct_k:7.1f}%')
    print(f'  {"ROS2 Overhead Out":<25} {mean_out:10.4f} {med_out:12.4f} {pct_out:7.1f}%')
    print(f'  {"-"*25} {"-"*10} {"-"*12} {"-"*8}')
    print(f'  {"Total":<25} {mean_t:10.4f} {med_t:12.4f} {"100.0":>7}%')
    print(f'')
    print(f'  Comm% (mean):   {pct_in + pct_out:.1f}%')
    print(f'  Comm% (median): {(med_in + med_out) / med_t * 100 if med_t > 0 else 0:.1f}%')
    print(f'{"="*80}\n')


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--hardware_device_type', type=str, default='cpu')
    parser.add_argument('--trace_path', type=str,
                        default='/root/.ros/tracing/e4_stereo_disparity')
    parser.add_argument('--metrics', type=str, default=['latency'])
    parser.add_argument('--integrated', type=str, default='false')
    parser.add_argument('--warmup', type=int, default=100,
                        help='Number of initial samples to discard for steady-state')
    args = parser.parse_args(argv)

    hardware_device_type = args.hardware_device_type
    trace_path = args.trace_path
    warmup = args.warmup
    metrics_raw = args.metrics
    if isinstance(metrics_raw, list):
        metrics = metrics_raw
    else:
        metrics = [e.strip() for e in metrics_raw.strip('[]').split(',')]

    ba = BenchmarkAnalyzer('e4_stereo_disparity', hardware_device_type)

    target_chain = [
        'robotperf_benchmarks:robotperf_image_input_cb_init',
        'robotperf_benchmarks:robotperf_image_input_cb_fini',
        'robotperf_benchmarks:robotperf_msg_received_1',
        'robotperf_benchmarks:robotperf_msg_published_1',
        'robotperf_benchmarks:robotperf_disparity_output_cb_init',
        'robotperf_benchmarks:robotperf_disparity_output_cb_fini',
    ]

    if hardware_device_type == 'cpu':
        colors = ['yellow', 'red', 'red', 'red', 'red', 'yellow']
        bokeh_colors = ['salmon', 'darksalmon', 'lightcoral', 'lightcoral', 'lightcoral', 'darkred']
        for tp, color, bokeh_color in zip(target_chain, colors, bokeh_colors):
            ba.add_target({
                'name': tp,
                'name_disambiguous': tp,
                'colors_fg': color,
                'colors_fg_bokeh': bokeh_color,
                'layer': 'userland',
                'label_layer': 4,
                'marker': 'plus',
            })

    add_power = False
    num_metrics = 0
    for metric in metrics:
        if metric == 'power':
            add_power = True
            ba.add_power({
                'name': 'robotcore_power:robotcore_power_output_cb_fini',
                'name_disambiguous': 'robotcore_power:robotcore_power_output_cb_fini',
                'colors_fg': 'blue', 'colors_fg_bokeh': 'silver',
                'layer': 'userland', 'label_layer': 4, 'marker': 'plus',
            })
        else:
            num_metrics += 1

    for metric in metrics:
        if metric == 'latency':
            # Manual flow so we can skip warmup samples
            ba.set_trace_sets_filter_type(filter_type='name')
            ba.get_target_chain_traces(trace_path, debug=True)
            total = len(ba.image_pipeline_msg_sets)
            ba.image_pipeline_msg_sets = ba.image_pipeline_msg_sets[warmup:]
            print(f'\n>>> Discarded first {warmup} of {total} samples '
                  f'({len(ba.image_pipeline_msg_sets)} remaining)\n')
            ba.bar_charts_latency()
            print_latentros_summary(
                ba.image_pipeline_msg_sets_barchart,
                'e4_stereo_disparity', target_chain)
        elif metric == 'throughput':
            ba.analyze_throughput(trace_path, add_power, filter_type='name')
        elif metric == 'power' and num_metrics == 0:
            print('The average consumption is {} W'.format(
                ba.analyze_power(trace_path)))
        else:
            print(f'The metric {metric} is not yet implemented\n')


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('hardware_device_type', default_value='cpu'))
    ld.add_action(DeclareLaunchArgument(
        'trace_path', default_value='/root/.ros/tracing/e4_stereo_disparity'))
    ld.add_action(DeclareLaunchArgument('metrics', default_value=['latency']))
    ld.add_action(DeclareLaunchArgument('integrated', default_value='false'))
    ld.add_action(ExecuteProcess(
        cmd=[
            'python3',
            'src/benchmarks/benchmarks/end_to_end/e4_latentros/launch/analyze_stereo_disparity.launch.py',
            '--hardware_device_type', LaunchConfiguration('hardware_device_type'),
            '--trace_path', LaunchConfiguration('trace_path'),
            '--metrics', LaunchConfiguration('metrics'),
            '--integrated', LaunchConfiguration('integrated'),
        ],
        output='screen',
    ))
    return ld


if __name__ == '__main__':
    main(sys.argv[1:])
