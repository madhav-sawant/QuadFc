#!/usr/bin/env python3
import csv, math, sys

FILE = "blackbox (1).csv"

rows = []
with open(FILE) as f:
    for r in csv.DictReader(f):
        rows.append({k: float(v) for k, v in r.items()})

N = len(rows)
print(f"Total samples: {N}")
print(f"Duration: ~{N/83:.1f} sec (at 83Hz log rate)\n")

# Helper
def rms(v): return math.sqrt(sum(x*x for x in v)/len(v)) if v else 0
def avg(v): return sum(v)/len(v) if v else 0

# --- 1. ACCEL Z ---
az = [r['accel_z_raw'] for r in rows]
spikes = sum(1 for a in az if abs(a - 1.0) > 0.1)
print("=== 1. ACCEL Z (should be ~1.0g level) ===")
print(f"  Mean={avg(az):.3f}g  Min={min(az):.3f}g  Max={max(az):.3f}g")
print(f"  Spikes (>0.1g off): {spikes}/{N} ({100*spikes/N:.1f}%)\n")

# --- 2. GYRO BIAS (ALL samples, grouped by throttle) ---
print("=== 2. GYRO MEAN/RMS BY THROTTLE BAND ===")
bands = [(1000,1100,'idle'),(1100,1200,'low'),(1200,1300,'mid'),(1300,1500,'high')]
for lo,hi,label in bands:
    b = [r for r in rows if lo <= r['rc_thr'] < hi]
    if not b: continue
    gx=[r['gyro_x'] for r in b]; gy=[r['gyro_y'] for r in b]; gz=[r['gyro_z'] for r in b]
    print(f"  {label:5s} (n={len(b):4d}): gx_avg={avg(gx):+.2f} gy_avg={avg(gy):+.2f} gz_avg={avg(gz):+.2f}  |  gx_rms={rms(gx):.2f} gy_rms={rms(gy):.2f} gz_rms={rms(gz):.2f}")
print()

# --- 3. ANGLE DRIFT (ALL samples, start→end) ---
print("=== 3. ANGLE DRIFT (full log, start→end) ===")
r0=rows[0]; r1=rows[-1]
print(f"  Start: roll={r0['angle_roll']:+.2f}  pitch={r0['angle_pitch']:+.2f}")
print(f"  End:   roll={r1['angle_roll']:+.2f}  pitch={r1['angle_pitch']:+.2f}")
print(f"  Net drift: roll={r1['angle_roll']-r0['angle_roll']:+.3f}  pitch={r1['angle_pitch']-r0['angle_pitch']:+.3f} deg\n")

# --- 4. ANGLE RANGE by throttle ---
print("=== 4. ANGLE RANGE BY THROTTLE ===")
for lo,hi,label in bands:
    b = [r for r in rows if lo <= r['rc_thr'] < hi]
    if not b: continue
    ar=[r['angle_roll'] for r in b]; ap=[r['angle_pitch'] for r in b]
    print(f"  {label:5s}: roll=[{min(ar):+.2f},{max(ar):+.2f}] avg={avg(ar):+.2f}  pitch=[{min(ap):+.2f},{max(ap):+.2f}] avg={avg(ap):+.2f}")
print()

# --- 5. PID OUTPUT STATS ---
print("=== 5. PID OUTPUT (motors on, thr>=1100) ===")
on = [r for r in rows if r['rc_thr'] >= 1100]
if on:
    for axis,key in [('roll','pid_roll'),('pitch','pid_pitch'),('yaw','pid_yaw')]:
        v=[r[key] for r in on]
        print(f"  {axis:5s}: avg={avg(v):+.2f}  rms={rms(v):.2f}  max={max(abs(x) for x in v):.2f}")
print()

# --- 6. MOTOR SATURATION ---
print("=== 6. MOTOR VALUES (motors on) ===")
if on:
    for mk in ['m1','m2','m3','m4']:
        v=[r[mk] for r in on]
        lo_sat=sum(1 for x in v if x<=1000)
        hi_sat=sum(1 for x in v if x>=1800)
        print(f"  {mk}: min={min(v):.0f}  max={max(v):.0f}  avg={avg(v):.0f}  sat_low={lo_sat}  sat_high={hi_sat}")
print()

# --- 7. YAW GYRO DRIFT ---
print("=== 7. YAW GYRO (gyro_z) DETAIL ===")
gz_all = [r['gyro_z'] for r in rows]
nonzero = [(i,r['gyro_z'],r['rc_thr']) for i,r in enumerate(rows) if abs(r['gyro_z']) > 5]
print(f"  Overall: mean={avg(gz_all):+.3f}  rms={rms(gz_all):.3f}  peak={max(abs(g) for g in gz_all):.2f} dps")
if nonzero:
    print(f"  Samples with |gz|>5dps: {len(nonzero)} (first at sample {nonzero[0][0]}, thr={nonzero[0][2]:.0f})")
    peak=max(nonzero, key=lambda x:abs(x[1]))
    print(f"  Peak: gz={peak[1]:+.1f}dps at sample {peak[0]}, thr={peak[2]:.0f}")
else:
    print("  No samples with |gz|>5dps")
print()

# --- 8. ACCEL NOISE vs THROTTLE ---
print("=== 8. ACCEL Z NOISE BY THROTTLE ===")
for lo,hi,label in bands:
    b = [r['accel_z_raw'] for r in rows if lo <= r['rc_thr'] < hi]
    if not b: continue
    print(f"  {label:5s}: mean={avg(b):.3f}g  range=[{min(b):.3f},{max(b):.3f}]  spread={max(b)-min(b):.3f}g  rms_dev={rms([x-avg(b) for x in b]):.4f}g")
print()

# --- 9. I2C ERRORS ---
i2c = [r['i2c_errors'] for r in rows]
print(f"=== 9. I2C ERRORS: start={i2c[0]:.0f}  end={i2c[-1]:.0f}  new_errors={i2c[-1]-i2c[0]:.0f} ===\n")

# --- 10. BATTERY ---
bv = [r['battery_mv'] for r in rows]
print(f"=== 10. BATTERY: start={bv[0]:.0f}mV  end={bv[-1]:.0f}mV  drop={bv[0]-bv[-1]:.0f}mV ===")
