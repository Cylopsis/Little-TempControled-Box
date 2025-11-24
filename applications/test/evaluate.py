import serial
import time
from skopt import gp_minimize
from skopt.space import Real
from skopt.plots import plot_convergence
import json
import os
import re
import matplotlib.pyplot as plt
from tqdm import tqdm  # 进度条

# ==============================================================================
# --- 配置区 ---
# ==============================================================================

# 串口配置
SERIAL_PORT = 'COM3'   # ← 按需修改
BAUD_RATE = 115200

# 优化过程配置
EVAL_DURATION_MS = 180_000          # 每次评估时长（毫秒）
TOTAL_CALLS_PER_PROFILE = 50        # 每个目标温度的评估次数
RESULTS_FILE = "./ptc_pid_results.json"

# 主动冷却配置
COOL_DOWN_THRESHOLD_C = 5.0         # PTC温度距离箱内温度小于该差值即视为“冷却到常温”
COOL_DOWN_TIMEOUT_S = 180           # 主动冷却最长等待秒数，防止死循环

# 增益调度目标配置
TARGET_PROFILES = [
    {
        "name": "Temp_48C",
        "target_temp": 48.0,
        "space": [
            Real(0.01, 1.0, name='kp'),
            Real(0.001, 1.0, name='ki'),
            Real(0.0, 1.0, name='kd')
        ],
        "initial_params": [0.25, 0.03, 0.10],
    },
    {
        "name": "Temp_60C",
        "target_temp": 60.0,
        "space": [
            Real(0.01, 1.0, name='kp'),
            Real(0.001, 1.0, name='ki'),
            Real(0.0, 1.0, name='kd')
        ],
        "initial_params": [0.30, 0.05, 0.15],
    },
]

# 全局变量
ser = None
current_target_temp = 0.0
pbar_inner = None  # 内层进度条句柄

# ==============================================================================
# 加载历史最优结果
# ==============================================================================

def load_initial_points_from_results(profiles, filepath):
    if not os.path.exists(filepath):
        print(f"Info: '{filepath}' not found. Using default initial parameters from the script.")
        return profiles

    try:
        with open(filepath, 'r') as f:
            previous_results = json.load(f)
        print(f"Successfully loaded previous results from '{filepath}'.")
    except (json.JSONDecodeError, IOError) as e:
        print(f"Warning: Could not read or parse '{filepath}': {e}. Using default initial parameters.")
        return profiles

    for profile in profiles:
        profile_name = profile["name"]
        if profile_name in previous_results and "best_params" in previous_results[profile_name]:
            best_params = previous_results[profile_name]["best_params"]
            new_initial_params = [
                best_params.get('Kp', 0.0),
                best_params.get('Ki', 0.0),
                best_params.get('Kd', 0.0),
            ]
            space = profile["space"]
            is_valid = all(space[i].low <= new_initial_params[i] <= space[i].high for i in range(3))
            if is_valid:
                profile["initial_params"] = new_initial_params
                print(f"  - Updated initial point for '{profile_name}' from previous results.")
            else:
                print(f"  - Warning: Previous best for '{profile_name}' is outside the current search space. Using default.")
        else:
            print(f"  - No previous result found for '{profile_name}'. Using default initial parameters.")

    return profiles

# ==============================================================================
# 串口命令封装
# ==============================================================================

def send_cmd(cmd_to_send, wait_time=0.1):
    """向 RT-Thread 发送命令，不主动读取输出。"""
    if ser and ser.is_open:
        ser.reset_input_buffer()
        ser.write((cmd_to_send + '\n').encode('ascii'))
        time.sleep(wait_time)
    else:
        print("Error: Serial port is not open.")


def send_cmd_and_read(cmd_to_send, timeout=1.0):
    """
    发送命令并读取返回（用于 get_status 之类需要解析输出的命令）.
    读到 'msh >' 或超时为止。
    """
    if not (ser and ser.is_open):
        print("Error: Serial port is not open.")
        return ""

    ser.reset_input_buffer()
    ser.write((cmd_to_send + '\n').encode('ascii'))

    lines = []
    end_time = time.time() + timeout
    while time.time() < end_time:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue
        if "msh >" in line:
            break
        lines.append(line)

    return "\n".join(lines)

# ==============================================================================
# 状态/温度读取 & 主动冷却
# ==============================================================================

def get_current_temps():
    """
    调用 get_status，并解析 PTC 和箱内温度。
    假设固件 get_status 输出中有比如：
      PTC Temp: xx.x C
      Box Temp: yy.y C
    """
    out = send_cmd_and_read("get_status", timeout=1.0)
    if not out:
        return None, None

    ptc_temp = None
    box_temp = None

    m1 = re.search(r"PTC\s*Temp:\s*([-\d\.]+)", out)
    if m1:
        try:
            ptc_temp = float(m1.group(1))
        except ValueError:
            ptc_temp = None

    m2 = re.search(r"Box\s*Temp:\s*([-\d\.]+)", out)
    if m2:
        try:
            box_temp = float(m2.group(1))
        except ValueError:
            box_temp = None

    return ptc_temp, box_temp


def reset_system():
    """
    每次评估前复位系统：
    1) 关掉 heat PID 增益
    2) 强制进入 COOLING 状态
    3) 循环读取温度，直到 PTC 接近箱内温度，或超时
    """
    print("\n--- Resetting system: force COOLING until PTC ~= box temp ---")

    # 关掉 heat PID 的积分/输出，避免乱加热
    send_cmd("tune heat kp 0", wait_time=0.05)
    send_cmd("tune heat ki 0", wait_time=0.05)
    send_cmd("tune heat kd 0", wait_time=0.05)
    send_cmd(f"tune target 20", wait_time=0.05)

    # 强制切换到冷却状态（需要你在固件里实现 force_state 命令）
    send_cmd("force_state cooling", wait_time=0.1)

    start = time.time()
    last_log = 0.0
    while True:
        ptc_temp, box_temp = get_current_temps()
        now = time.time()

        if ptc_temp is None or box_temp is None:
            # 读不到就简单 wait 一下
            if now - start > COOL_DOWN_TIMEOUT_S:
                print("Warning: get_status failed too long, giving up cooling wait.")
                break
            time.sleep(1.0)
            continue

        diff = abs(ptc_temp - 21)
        if now - last_log > 3:
            print(f"Cooling... PTC={ptc_temp:.2f}C, Box={box_temp:.2f}C, |Δ|={diff:.2f}C")
            last_log = now

        if diff <= COOL_DOWN_THRESHOLD_C:
            print("PTC cooled close to box temperature, reset complete.")
            break

        if now - start > COOL_DOWN_TIMEOUT_S:
            print(f"Cooling timeout ({COOL_DOWN_TIMEOUT_S}s). Continue anyway.")
            break

        time.sleep(1.0)

    # 冷却完后，切回 warming（避免一直吹冷风影响下次升温）
    send_cmd("force_state warming", wait_time=0.1)

# ==============================================================================
# 目标函数
# ==============================================================================

def objective_function(params):
    """
    目标函数：给定 (kp, ki, kd)，
    1) 重置系统（冷却到近似常温）
    2) 设置 PID 参数
    3) 调用固件 eval_ptc target_temp duration
    4) 等待并解析 EVAL_RESULT:x.x
    返回 score（越小越好）
    """
    global current_target_temp, pbar_inner

    kp, ki, kd = params
    kp_str, ki_str, kd_str = f"{kp:.6f}", f"{ki:.6f}", f"{kd:.6f}"

    if pbar_inner is not None:
        pbar_inner.set_description(f"Kp={kp_str}, Ki={ki_str}, Kd={kd_str}")

    # 1. 冷却
    reset_system()

    # 2. 设置 PID 参数
    send_cmd(f"tune heat kp {kp_str}", wait_time=0.05)
    send_cmd(f"tune heat ki {ki_str}", wait_time=0.05)
    send_cmd(f"tune heat kd {kd_str}", wait_time=0.05)

    # 3. 启动评估
    send_cmd(f"eval_ptc {current_target_temp} {EVAL_DURATION_MS}", wait_time=0)

    # 4. 等结果
    timeout = time.time() + (EVAL_DURATION_MS / 1000.0) + 10.0
    score = 1e6

    while time.time() < timeout:
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
        except (serial.SerialException, TypeError) as e:
            if pbar_inner is not None:
                pbar_inner.write(f"Serial read error: {e}.")
            score = 1e6
            break

        if not line:
            continue

        if line.startswith("EVAL_RESULT:"):
            try:
                val = float(line.split(':')[1])
                score = val if val > 0 else 1e6
            except (ValueError, IndexError):
                score = 1e6
            break

    if time.time() >= timeout and score >= 1e6:
        if pbar_inner is not None:
            pbar_inner.write("Timeout: no EVAL_RESULT received.")

    if pbar_inner is not None:
        pbar_inner.write(f"Score={score:.4f} for Kp={kp_str}, Ki={ki_str}, Kd={kd_str}")
        pbar_inner.update(1)

    return score

# ==============================================================================
# MAIN
# ==============================================================================

if __name__ == '__main__':
    all_results = {}

    print("\n" + "=" * 60)
    print("           PTC PID Auto-Tuning Workflow Started           ")
    print("=" * 60)

    # 先加载已有结果（用于断点续跑）
    if os.path.exists(RESULTS_FILE):
        try:
            with open(RESULTS_FILE, 'r') as f:
                all_results = json.load(f)
            print(f"Loaded {len(all_results)} existing results from '{RESULTS_FILE}'.")
        except Exception as e:
            print(f"Warning: failed to load existing results: {e}")

    # 用历史最优点更新初始猜测
    TARGET_PROFILES = load_initial_points_from_results(TARGET_PROFILES, RESULTS_FILE)
    print("=" * 60 + "\n")

    try:
        
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        print(f"Successfully connected to serial port {SERIAL_PORT}.")
        time.sleep(2)
        ser.reset_input_buffer()

        pbar_outer = tqdm(TARGET_PROFILES, desc="Overall Progress", unit="profile")
        for profile in pbar_outer:
            profile_name = profile["name"]

            # 如果该 profile 已有结果，跳过（断点续跑）
            if profile_name in all_results:
                pbar_outer.write(f"Skip '{profile_name}' (already in {RESULTS_FILE}).")
                continue

            current_target_temp = profile["target_temp"]
            space = profile["space"]
            initial_params = profile["initial_params"]

            pbar_outer.set_description(f"Optimizing {profile_name}")
            pbar_inner = tqdm(total=TOTAL_CALLS_PER_PROFILE,
                              desc=f"Optimizing {profile_name}",
                              unit="call")

            # 贝叶斯优化
            result = gp_minimize(
                func=objective_function,
                dimensions=space,
                n_calls=TOTAL_CALLS_PER_PROFILE,
                n_initial_points=1,
                x0=initial_params,
                random_state=123,
            )

            pbar_inner.close()
            pbar_inner = None

            best_params = {
                "Kp": result.x[0],
                "Ki": result.x[1],
                "Kd": result.x[2],
            }
            all_results[profile_name] = {
                "target_temp": current_target_temp,
                "best_score": float(result.fun),
                "best_params": best_params,
            }

            # checkpoint：每个 profile 完成就保存一次
            with open(RESULTS_FILE, "w") as f:
                json.dump(all_results, f, indent=4)
            pbar_outer.write(f"Checkpoint saved for '{profile_name}'.")

            # 收敛图
            fig, ax = plt.subplots()
            plot_convergence(result, ax=ax)
            ax.set_title(f"Convergence for {profile_name} (Target={current_target_temp}°C)")
            ax.set_ylabel("Best Score (Avg. Error)")
            plot_filename = f"./convergence_{profile_name}.png"
            fig.savefig(plot_filename)
            plt.close(fig)
            pbar_outer.write(f"Convergence plot saved to '{plot_filename}'.")

        # 全部完成后输出 C 代码
        print("\n" + "=" * 60)
        print("           ALL OPTIMIZATIONS COMPLETE           ")
        print("=" * 60)
        print("Final Gain Scheduling Parameters (C-Code):\n")

        formatted_c_code = (
            "typedef struct { float target_temp; float kp; float ki; float kd; } ptc_pid_profile_t;\n\n"
            "const ptc_pid_profile_t ptc_pid_table[] = {\n"
        )
        sorted_results = sorted(all_results.items(), key=lambda it: it[1]["target_temp"])
        for name, data in sorted_results:
            t = data["target_temp"]
            kp = data["best_params"]["Kp"]
            ki = data["best_params"]["Ki"]
            kd = data["best_params"]["Kd"]
            formatted_c_code += (
                f"    {{{t:.1f}f, {kp:.6f}f, {ki:.6f}f, {kd:.6f}f}}, "
                f"// Score: {data['best_score']:.4f}\n"
            )
        formatted_c_code += "};"
        print(formatted_c_code)

        with open(RESULTS_FILE, "w") as f:
            json.dump(all_results, f, indent=4)
        print(f"\nFinal results saved to '{RESULTS_FILE}'.")

    except serial.SerialException as e:
        print(f"\nFatal Error: serial '{SERIAL_PORT}' failed. Details: {e}")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        if ser and ser.is_open:
            # 结束前切回 warming 更安全
            send_cmd("force_state warming", wait_time=0.1)
            ser.close()
            print("\nSerial port closed.")
