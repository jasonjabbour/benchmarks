#!/usr/bin/env python3
"""
ROS 2 Message Transport Latency Ablation Study
Measures DDS transport latency (publish → subscribe) across message types and sizes.
Outputs CSV: message_type, message_size_elements, serialized_bytes, latency_ns
"""
import subprocess
import time
import os
import sys
import signal
import csv

TRACE_DIR = "/root/.ros/tracing/message_benchmark_test"
E2_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_CSV = os.path.join(E2_DIR, "transport_ablation_results.csv")
NUM_MESSAGES = 600  # publish 600, discard first 100 = 500 steady-state
PUBLISH_HZ = 10     # 10 Hz → 60 seconds per run

# Message types and their size configurations
# For dynamic-size messages: list of element counts that produce sizes from ~10KB to ~1GB
# For fixed-size messages: just one entry (size param is ignored)
EXPERIMENTS = {
    # === Fixed-size messages ===
    "amcl_pose":      [1],         # ~360 bytes, fixed
    "twist_stamped":  [1],         # ~76 bytes, fixed
    "imu":            [1],         # ~360 bytes, fixed

    # === Dynamic-size messages (element counts) ===
    # PointCloud2: each element = 12 bytes (3x float32 for x,y,z)
    "pointcloud": [
        833,          # ~10 KB
        4166,         # ~50 KB
        8333,         # ~100 KB
        41666,        # ~500 KB
        83333,        # ~1 MB
        416666,       # ~5 MB
        833333,       # ~10 MB
        2500000,      # ~30 MB
        4166666,      # ~50 MB
        8333333,      # ~100 MB
    ],

    # LaserScan: each element = 8 bytes (float32 range + float32 intensity)
    "laserscan": [
        1250,         # ~10 KB
        6250,         # ~50 KB
        12500,        # ~100 KB
        62500,        # ~500 KB
        125000,       # ~1 MB
        625000,       # ~5 MB
        1250000,      # ~10 MB
        3750000,      # ~30 MB
        6250000,      # ~50 MB
        12500000,     # ~100 MB
    ],

    # OccupancyGrid: each element = 1 byte (int8 cell)
    "occupancy_grid": [
        10000,        # ~10 KB
        50000,        # ~50 KB
        100000,       # ~100 KB
        500000,       # ~500 KB
        1000000,      # ~1 MB
        5000000,      # ~5 MB
        10000000,     # ~10 MB
        30000000,     # ~30 MB
        50000000,     # ~50 MB
        100000000,    # ~100 MB
    ],

    # Image: publisher uses height=message_size, width=1, step=1, 1 byte/pixel
    "image": [
        10000,        # ~10 KB
        50000,        # ~50 KB
        100000,       # ~100 KB
        500000,       # ~500 KB
        1000000,      # ~1 MB
        5000000,      # ~5 MB
        10000000,     # ~10 MB
        30000000,     # ~30 MB
        50000000,     # ~50 MB
        100000000,    # ~100 MB
    ],
}


def kill_stale():
    """Kill any stale ROS processes."""
    subprocess.run(
        "pkill -9 -f 'message_publisher\\|message_subscriber\\|component_container' 2>/dev/null",
        shell=True, capture_output=True)
    time.sleep(1)


def destroy_trace():
    """Destroy any existing LTTng session."""
    subprocess.run(
        "lttng destroy --all 2>/dev/null",
        shell=True, capture_output=True)


def run_experiment(msg_type, msg_size, results_writer):
    """Run one experiment: launch pub+sub, collect trace, extract latencies."""
    print(f"\n{'='*60}")
    print(f"  {msg_type}  size={msg_size}")
    print(f"{'='*60}")

    # Clean up
    kill_stale()
    destroy_trace()
    if os.path.exists(TRACE_DIR):
        subprocess.run(f"rm -rf {TRACE_DIR}", shell=True)

    # Source and launch
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["CYCLONEDDS_URI"] = (
        "<CycloneDDS><Domain>"
        "<SharedMemory><Enable>false</Enable></SharedMemory>"
        "</Domain></CycloneDDS>"
    )

    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source /tmp/benchmark_ws/install/setup.bash && "
        f"ros2 launch e2_navigation_messages robotperf_message_types.launch.py "
        f"message_type:={msg_type} message_size:={msg_size}"
    )

    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid)

    # Wait for messages (NUM_MESSAGES at PUBLISH_HZ)
    wait_time = NUM_MESSAGES / PUBLISH_HZ + 5  # extra buffer
    # Cap wait time for very large messages that might be slow
    wait_time = min(wait_time, 120)
    print(f"  Waiting {wait_time:.0f}s for {NUM_MESSAGES} messages...")
    time.sleep(wait_time)

    # Stop trace
    subprocess.run(
        "lttng stop message_benchmark_test 2>/dev/null",
        shell=True, capture_output=True)
    time.sleep(1)

    # Kill processes
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass
    time.sleep(1)

    # Extract latencies from trace
    if not os.path.exists(TRACE_DIR):
        print(f"  ERROR: No trace directory found")
        return

    result = subprocess.run(
        f"babeltrace2 {TRACE_DIR} 2>/dev/null | grep robotperf",
        shell=True, capture_output=True, text=True)

    lines = result.stdout.strip().split('\n')
    if not lines or lines == ['']:
        print(f"  ERROR: No trace events found")
        return

    # Parse events: match published_size_1 and received_size_1 by key
    published = {}  # key → (timestamp_ns, size)
    received = {}   # key → (timestamp_ns, size)

    for line in lines:
        try:
            # Extract timestamp from babeltrace2 output
            # Format: [HH:MM:SS.NNNNNNNNN] ...
            ts_str = line.split(']')[0].split('[')[1].strip()
            parts = ts_str.split('.')
            time_parts = parts[0].split(':')
            h, m, s = int(time_parts[0]), int(time_parts[1]), int(time_parts[2])
            ns = int(parts[1])
            timestamp_ns = ((h * 3600 + m * 60 + s) * 1_000_000_000) + ns

            if 'msg_published_size_1' in line:
                key_match = line.split('key = ')[1].split(',')[0].strip()
                size_match = line.split('msg_size = ')[1].split(',')[0].strip().rstrip('}')
                published[int(key_match)] = (timestamp_ns, int(size_match))
            elif 'msg_received_size_1' in line:
                key_match = line.split('key = ')[1].split(',')[0].strip()
                size_match = line.split('msg_size = ')[1].split(',')[0].strip().rstrip('}')
                received[int(key_match)] = (timestamp_ns, int(size_match))
        except (IndexError, ValueError):
            continue

    # Match keys and compute latencies
    matched = 0
    skipped_warmup = 0
    warmup = 100

    sorted_keys = sorted(set(published.keys()) & set(received.keys()))
    for key in sorted_keys:
        if skipped_warmup < warmup:
            skipped_warmup += 1
            continue

        pub_ts, pub_size = published[key]
        recv_ts, recv_size = received[key]
        latency_ns = recv_ts - pub_ts

        if latency_ns > 0:
            latency_ms = latency_ns / 1_000_000.0
            results_writer.writerow([
                msg_type, msg_size, pub_size, f"{latency_ms:.4f}"
            ])
            matched += 1

    print(f"  Published: {len(published)}, Received: {len(received)}, "
          f"Matched (after warmup): {matched}")
    if matched > 0 and sorted_keys:
        sample_key = sorted_keys[warmup] if len(sorted_keys) > warmup else sorted_keys[0]
        sample_size = published[sample_key][1]
        print(f"  Serialized size: {sample_size:,} bytes ({sample_size/1024/1024:.2f} MB)")


def main():
    print("=" * 60)
    print("  ROS 2 Message Transport Latency Ablation")
    print("=" * 60)

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["message_type", "element_count", "serialized_bytes", "latency_ms"])

        for msg_type, sizes in EXPERIMENTS.items():
            for size in sizes:
                try:
                    run_experiment(msg_type, size, writer)
                    f.flush()
                except Exception as e:
                    print(f"  EXCEPTION: {e}")
                    continue

    print(f"\n{'='*60}")
    print(f"  Results saved to {OUTPUT_CSV}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
