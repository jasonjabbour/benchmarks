#!/usr/bin/env python3
"""Re-run just the Image experiments after fixing the publisher."""
import subprocess, time, os, signal, csv

TRACE_DIR = "/root/.ros/tracing/message_benchmark_test"
E2_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT_CSV = os.path.join(E2_DIR, "transport_ablation_image.csv")
NUM_MESSAGES = 600
PUBLISH_HZ = 10

SIZES = [10000, 50000, 100000, 500000, 1000000, 5000000, 10000000, 30000000, 50000000, 100000000]

def run(msg_size, writer):
    subprocess.run("pkill -9 -f 'message_publisher\\|message_subscriber' 2>/dev/null", shell=True, capture_output=True)
    time.sleep(1)
    subprocess.run("lttng destroy --all 2>/dev/null", shell=True, capture_output=True)
    subprocess.run(f"rm -rf {TRACE_DIR}", shell=True)

    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["CYCLONEDDS_URI"] = "<CycloneDDS><Domain><SharedMemory><Enable>false</Enable></SharedMemory></Domain></CycloneDDS>"

    cmd = (f"source /opt/ros/humble/setup.bash && source /tmp/benchmark_ws/install/setup.bash && "
           f"ros2 launch e2_navigation_messages robotperf_message_types.launch.py "
           f"message_type:=image message_size:={msg_size}")

    proc = subprocess.Popen(["bash", "-c", cmd], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
    wait = min(NUM_MESSAGES / PUBLISH_HZ + 5, 120)
    print(f"  image size={msg_size} waiting {wait:.0f}s...")
    time.sleep(wait)

    subprocess.run("lttng stop message_benchmark_test 2>/dev/null", shell=True, capture_output=True)
    time.sleep(1)
    try: os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except: pass
    time.sleep(1)

    result = subprocess.run(f"babeltrace2 {TRACE_DIR} 2>/dev/null | grep robotperf", shell=True, capture_output=True, text=True)
    lines = result.stdout.strip().split('\n')
    published, received = {}, {}
    for line in lines:
        try:
            ts_str = line.split(']')[0].split('[')[1].strip()
            parts = ts_str.split('.')
            tp = parts[0].split(':')
            ns = ((int(tp[0])*3600+int(tp[1])*60+int(tp[2]))*1_000_000_000)+int(parts[1])
            if 'msg_published_size_1' in line:
                k=int(line.split('key = ')[1].split(',')[0]); s=int(line.split('msg_size = ')[1].split(',')[0].rstrip('}'))
                published[k]=(ns,s)
            elif 'msg_received_size_1' in line:
                k=int(line.split('key = ')[1].split(',')[0]); s=int(line.split('msg_size = ')[1].split(',')[0].rstrip('}'))
                received[k]=(ns,s)
        except: continue

    matched=0; skip=0
    for key in sorted(set(published)&set(received)):
        if skip<100: skip+=1; continue
        lat=received[key][0]-published[key][0]
        if lat>0:
            writer.writerow(["image",msg_size,published[key][1],f"{lat/1e6:.4f}"])
            matched+=1
    print(f"  pub={len(published)} recv={len(received)} matched={matched}")
    if matched>0:
        sk=sorted(set(published)&set(received))[100] if len(set(published)&set(received))>100 else list(published.keys())[0]
        print(f"  serialized: {published[sk][1]:,} bytes")

with open(OUTPUT_CSV,'w',newline='') as f:
    w=csv.writer(f); w.writerow(["message_type","element_count","serialized_bytes","latency_ms"])
    for sz in SIZES:
        try: run(sz,w); f.flush()
        except Exception as e: print(f"  ERR: {e}")
print(f"\nDone → {OUTPUT_CSV}")
