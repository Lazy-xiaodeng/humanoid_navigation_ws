#!/bin/bash
# NDT bag replay: manual lifecycle control, no lifecycle manager
set -e

BAG="/home/ubuntu/nav_drift_test4/nav_drift_test4_0.mcap"
OUTPUT="/tmp/ndt_replay_planb_$(date +%Y%m%d_%H%M%S).jsonl"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_FILE="$SCRIPT_DIR/test_ndt_replay.launch.py"

cleanup() {
  echo "Cleaning up..."
  kill %1 2>/dev/null || true
  kill %2 2>/dev/null || true
}
trap cleanup EXIT

# Step 1: Launch NDT node (no lifecycle manager)
echo "[1/4] Launching NDT node..."
ros2 launch "$LAUNCH_FILE" > /tmp/ndt_replay_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 4

# Step 2: Manually configure and activate with proper waits
echo "[2/4] Configuring and activating..."

# configure
STATE=$(ros2 lifecycle get /lidar_localization 2>&1 | head -1)
echo "  Initial: $STATE"
ros2 lifecycle set /lidar_localization configure 2>&1 || true

# Wait for inactive
for i in $(seq 1 15); do
  sleep 2
  STATE=$(ros2 lifecycle get /lidar_localization 2>&1 | head -1)
  echo "  t=$((i*2))s: $STATE"
  if echo "$STATE" | grep -qE "inactive|active"; then
    break
  fi
done

# Now activate (map loading happens here, takes ~10s for 125MB PCD)
if echo "$STATE" | grep -q "inactive"; then
  echo "  Activating (loading 125MB PCD map, this takes ~10s)..."
  ros2 lifecycle set /lidar_localization activate 2>&1 || true

  # Wait for active - with long timeout for map loading
  for i in $(seq 1 20); do
    sleep 2
    STATE=$(ros2 lifecycle get /lidar_localization 2>&1 | head -1)
    echo "  t=$((i*2+30))s: $STATE"
    if echo "$STATE" | grep -q "active"; then
      echo "  Node ACTIVE!"
      break
    fi
  done
fi

FINAL_STATE=$(ros2 lifecycle get /lidar_localization 2>&1 | head -1)
echo "  Final state: $FINAL_STATE"

if ! echo "$FINAL_STATE" | grep -q "active"; then
  echo "ERROR: Node not active. Launch log:"
  tail -30 /tmp/ndt_replay_launch.log
  exit 1
fi

# Step 3: Start recording
echo "[3/4] Starting NDT status recorder..."
python3 -c "
import rclpy, json
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')
        self.out = open('$OUTPUT', 'w')
        self.count = 0
        self.sub = self.create_subscription(String, '/localization/ndt_status', self.cb, 1000)
    def cb(self, msg):
        self.out.write(msg.data + '\n')
        self.count += 1
        if self.count % 500 == 0:
            print(f'  Recorded {self.count} messages', flush=True)
    def close(self):
        self.out.close()
        print(f'Total recorded: {self.count} messages', flush=True)

rec = Recorder()
try:
    rclpy.spin(rec)
except KeyboardInterrupt:
    pass
finally:
    rec.close()
    rclpy.shutdown()
" &
RECORD_PID=$!
sleep 2

# Step 4: Play bag
echo "[4/4] Playing bag (8 min, please wait)..."
START_TS=$(date +%s)
ros2 bag play "$BAG" --clock 100 \
  --topics /fast_lio/cloud_registered /tf /tf_static /initialpose 2>&1 | tail -5
END_TS=$(date +%s)
echo "  Bag playback took $((END_TS - START_TS))s"

sleep 10
kill $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true
sleep 2

echo ""
echo "============================================================"
echo "  RESULTS: Plan B NDT Replay vs Bag Recorded NDT"
echo "============================================================"

python3 << PYEOF
import json

with open("$OUTPUT") as f:
    lines = [json.loads(l) for l in f if l.strip()]

total = len(lines)
acc = [l for l in lines if l.get('state') == 'accepted']
rej = [l for l in lines if l.get('state') == 'rejected']
conf = [l for l in lines if l.get('state') == 'confirming']

print(f"  Total frames: {total}")
print(f"  Accepted: {len(acc)} ({len(acc)/total*100:.1f}%)" if total else "  No data!")
print(f"  Rejected: {len(rej)} ({len(rej)/total*100:.1f}%)" if total else "")
print(f"  Confirming: {len(conf)}" if total else "")

if acc:
    corrs = sorted([l['correction_translation'] for l in acc if 'correction_translation' in l])
    fits = sorted([l['fitness_score'] for l in acc if 'fitness_score' in l and l['fitness_score'] >= 0])
    da = sum(1 for l in lines if l.get('fastlio_delta_applied'))
    n = len(corrs)
    print(f"")
    print(f"  Accepted correction (m):")
    print(f"    p50={corrs[n//2]:.4f}  p90={corrs[int(n*0.9)]:.4f}  p99={corrs[int(n*0.99)]:.4f}  max={max(corrs):.4f}")
    if fits:
        fn = len(fits)
        print(f"    Fitness: p50={fits[fn//2]:.4f}  p90={fits[int(fn*0.9)]:.4f}")
    print(f"")
    print(f"  Fast-LIO delta stats:")
    print(f"    Delta applied: {da} frames ({da/total*100:.1f}%)")
    for reason in ['tf_lookup_failed', 'tf_stamp_mismatch', 'tf_clock_mismatch',
                    'no_prev_pose', 'dead_reckon_timeout', 'max_delta_exceeded']:
        cnt = sum(1 for l in lines if l.get('fastlio_delta_reject_reason') == reason)
        if cnt > 0:
            print(f"    {reason}: {cnt} frames ({cnt/total*100:.1f}%)")

    # Show timeline: accept rate over time in windows
    window = 50
    print(f"\n  Accept rate timeline (window={window}):")
    for i in range(0, len(lines), window):
        chunk = lines[i:i+window]
        a = sum(1 for l in chunk if l.get('state') == 'accepted')
        r = sum(1 for l in chunk if l.get('state') == 'rejected')
        rate = a/len(chunk)*100
        t_rel = chunk[0].get('stamp_sec', 0) - lines[0].get('stamp_sec', 0) if lines and lines[0] else 0
        bar = '#' * int(rate / 100 * 30)
        if i % (window * 5) == 0 or rate < 50:
            print(f"    t={t_rel:6.0f}s  {a:3d}/{len(chunk)} ({rate:5.1f}%)  rej={r:3d}  {bar}")

elif total == 0:
    print("")
    print("  ERROR: No data recorded. Launch log:")
    import subprocess
    subprocess.run(['tail', '-30', '/tmp/ndt_replay_launch.log'])
PYEOF
