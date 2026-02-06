import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def analyze_flight_logic(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        print("Usage: python log_analyzer.py <path_to_blackbox.csv>")
        return

    # 1. Load Data
    try:
        df = pd.read_csv(file_path)
        print("Data loaded successfully.")
    except Exception as e:
        print(f"Error loading file: {e}")
        return

    # Create a time index (assuming sequential logging)
    time = df.index

    # --- PLOT 1: The Flight Controller's "Thought Process" ---
    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)
    plt.subplots_adjust(hspace=0.2)

    # Subplot 1: Goal vs Reality (Roll Axis Example)
    # Compares what you asked for (RC) vs what happened (Gyro)
    ax1 = axes[0]
    ax1.set_title("1. THE GOAL vs REALITY (Roll Axis)", fontsize=14, fontweight='bold')
    
    # Plot Gyro (Reality)
    color_gyro = 'tab:blue'
    ax1.set_ylabel('Reality: Gyro Rate (deg/s)', color=color_gyro, fontsize=12)
    line1 = ax1.plot(time, df['gyro_x'], color=color_gyro, label='Gyro Roll (Actual Motion)', linewidth=1.5)
    ax1.tick_params(axis='y', labelcolor=color_gyro)
    ax1.grid(True, linestyle='--', alpha=0.5)

    # Plot RC (Goal) on secondary axis
    ax1_twin = ax1.twinx()
    color_rc = 'tab:orange'
    ax1_twin.set_ylabel('Goal: RC Command (PWM)', color=color_rc, fontsize=12)
    line2 = ax1_twin.plot(time, df['rc_roll'], color=color_rc, linestyle='--', label='RC Stick Input', linewidth=2)
    ax1_twin.tick_params(axis='y', labelcolor=color_rc)
    
    # Legend
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left')

    # Subplot 2: The Effort (PID Output)
    # How hard the FC is "thinking" to correct errors
    ax2 = axes[1]
    ax2.set_title("2. THE EFFORT (PID Corrections)", fontsize=14, fontweight='bold')
    ax2.plot(time, df['pid_roll'], label='PID Roll', color='purple', alpha=0.9)
    ax2.plot(time, df['pid_pitch'], label='PID Pitch', color='green', alpha=0.6)
    ax2.plot(time, df['pid_yaw'], label='PID Yaw', color='red', alpha=0.6)
    ax2.set_ylabel("PID Correction Strength")
    ax2.legend(loc='upper left')
    ax2.grid(True, linestyle='--', alpha=0.5)

    # Subplot 3: The Execution (Motors)
    # The physical result of the thinking
    ax3 = axes[2]
    ax3.set_title("3. THE EXECUTION (Motor Outputs)", fontsize=14, fontweight='bold')
    ax3.plot(time, df['m1'], label='Motor 1 (Rear Right)', alpha=0.8)
    ax3.plot(time, df['m2'], label='Motor 2 (Front Right)', alpha=0.8)
    ax3.plot(time, df['m3'], label='Motor 3 (Rear Left)', alpha=0.8)
    ax3.plot(time, df['m4'], label='Motor 4 (Front Left)', alpha=0.8)
    
    # Overlay throttle to see if motors are tracking the base power
    ax3.plot(time, df['rc_thr'], label='Base Throttle', color='black', linestyle='--', linewidth=2, alpha=0.5)
    
    ax3.set_ylabel("Motor Signal (PWM)")
    ax3.set_xlabel("Log Sample Time", fontsize=12)
    ax3.legend(loc='upper left', ncol=5)
    ax3.grid(True, linestyle='--', alpha=0.5)

    print("Saving 'flight_logic_overview.png'...")
    plt.savefig('flight_logic_overview.png')
    
    # Show plot
    plt.show()

    # --- AUTOMATED DIAGNOSIS ---
    print("\n" + "="*50)
    print("       FLIGHT DATA AUTOMATED DIAGNOSIS       ")
    print("="*50)

    # 1. Vibration Check
    # High noise in gyro_z is a classic sign of bad props or loose frame
    gyro_noise = df['gyro_z'].std()
    print(f"[-] Vibration Level (Gyro Z StdDev): {gyro_noise:.2f} deg/s")
    if gyro_noise > 10.0:
        print("    ⚠️  WARNING: High vibration detected! Check prop balance and frame rigidity.")
    else:
        print("    ✅  Vibration levels look acceptable.")

    # 2. Control Authority Check (Saturation)
    # Check if any motor hit 2000 (max throttle) meaning the drone ran out of power to correct
    max_motor_out = df[['m1', 'm2', 'm3', 'm4']].max().max()
    print(f"[-] Max Motor Output: {max_motor_out} us")
    if max_motor_out >= 1950:
        print("    ⚠️  CRITICAL: Motors saturated (hit max power). Drone may have tumbled.")
        print("           This usually means specific PID gains are too high or CG is off.")
    else:
        print("    ✅  Motors stayed within control limits.")

    # 3. PID Oscillation Check
    # 3. PID Oscillation Check (Per Axis)
    # Rapid sign changes in PID output usually mean P-gain is too high
    avg_throttle = df['rc_thr'].mean()
    print(f"[-] PID Oscillation Analysis (Throttle > 1200):")
    
    if avg_throttle > 1200:
        for axis in ['roll', 'pitch', 'yaw']:
            # Count zero crossings
            pid_col = f'pid_{axis}'
            crossings = ((df[pid_col] > 0) != (df[pid_col].shift(1) > 0)).sum()
            rate = crossings / len(df)
            
            print(f"    - {axis.upper()} Oscillation Index: {rate:.2f}")
            
            if rate > 0.3:
                print(f"      ⚠️  WARNING: {axis.upper()} axis is oscillating! (Index: {rate:.2f})")
                print(f"           -> Reduce {axis.upper()} P-gain or increase D-gain.")
    else:
        print("    ℹ️  Skipped (Avg throttle < 1200, system mostly idle)")
    
    # 4. I2C Error Check
    total_i2c_errs = df['i2c_errors'].max()
    print(f"[-] Total I2C Errors: {total_i2c_errs}")
    if total_i2c_errs > 0:
         print("    ⚠️  WARNING: I2C errors detected! Check wiring length and pull-up resistors.")

    print("="*50 + "\n")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please provide the CSV filename as an argument.")
        print("Example: python log_analyzer.py blackbox.csv")
    else:
        analyze_flight_logic(sys.argv[1])

