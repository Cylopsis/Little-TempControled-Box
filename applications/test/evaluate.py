import serial
import time
from skopt import gp_minimize
from skopt.space import Real
import json
import os # 引入os模块来检查文件是否存在

# ==============================================================================
# --- 配置区 ---
# ==============================================================================

# --- 串口配置 ---
SERIAL_PORT = 'COM3'  
BAUD_RATE = 115200

# --- 优化过程配置 ---
EVAL_DURATION_MS = 30000  # 每次评估的持续时间（毫秒）
TOTAL_CALLS_PER_PROFILE = 60 # 每个高度配置的总优化迭代次数
RESET_DURATION_S = 3      # 两次测试之间，风扇停机的复位时间（秒）
RESULTS_FILE = "./gain_scheduling_results.json" # 上次优化结果

# --- 增益调度目标配置 (Gain Scheduling Profiles) ---
# 这是核心配置。脚本会为列表中的每个字典（代表一个高度）运行一次完整的优化。
# "initial_params" 现在将作为找不到历史结果时的后备默认值。
TARGET_PROFILES = [
    # {
    #     "name": "Height_50",
    #     "target_height": 50.0,
    #     "space": [Real(0.0, 0.05, name='kp'), Real(0.0, 0.01, name='ki'), Real(0.0, 0.02, name='kd')],
    #     "initial_params": [0.0015, 0.0002, 0.0076] 
    # },
    # {
    #     "name": "Height_100",
    #     "target_height": 100.0,
    #     "space": [Real(0.0, 0.05, name='kp'), Real(0.0, 0.01, name='ki'), Real(0.0, 0.02, name='kd')],
    #     "initial_params": [0.0023, 0.00018, 0.0175]
    # },
    # {
    #     "name": "Height_150",
    #     "target_height": 150.0,
    #     "space": [Real(0.0, 0.05, name='kp'), Real(0.0, 0.01, name='ki'), Real(0.0, 0.02, name='kd')],
    #     "initial_params": [0.0015, 0.00013, 0.0142]
    # },
    {
        "name": "Height_200",
        "target_height": 200.0,
        "space": [Real(0.000001, 1, name='kp'), Real(0.000001, 1, name='ki'), Real(0.000001, 1, name='kd')],
        "initial_params": [0.0027, 0.00005, 0.0138]
    },
    {
        "name": "Height_250",
        "target_height": 250.0,
        "space": [Real(0.000001, 1, name='kp'), Real(0.000001, 1, name='ki'), Real(0.000001, 1, name='kd')],
        "initial_params": [0.0021, 0.0001, 0.0085]
    },
    # {
    #     "name": "Height_300",
    #     "target_height": 300.0,
    #     "space": [Real(0.0, 0.05, name='kp'), Real(0.0, 0.01, name='ki'), Real(0.0, 0.02, name='kd')],
    #     "initial_params": [0.0024, 0.00008, 0.0120]
    # },
    # {
    #     "name": "Height_350",
    #     "target_height": 350.0,
    #     "space": [Real(0.0, 0.05, name='kp'), Real(0.0, 0.01, name='ki'), Real(0.0, 0.02, name='kd')],
    #     "initial_params": [0.0025, 0.00026, 0.0164]
    # },
    # {
    #     "name": "Height_400",
    #     "target_height": 400.0,
    #     "space": [Real(0.0, 0.04, name='kp'), Real(0.0, 0.008, name='ki'), Real(0.0, 0.015, name='kd')],
    #     "initial_params": [0.0030, 0.00019, 0.0150]
    # },
    # {
    #     "name": "Height_450",
    #     "target_height": 450.0,
    #     "space": [Real(0.0, 0.05, name='kp'), Real(0.0, 0.01, name='ki'), Real(0.0, 0.02, name='kd')],
    #     "initial_params": [0.0027, 0.00019, 0.0114]
    # },
]

# 全局变量
current_target_height = 0
ser = None

# ==============================================================================
# 加载历史最优结果 ---
# ==============================================================================

def load_initial_points_from_results(profiles, filepath):
    """
    尝试从JSON文件中加载上次的最佳参数，并更新profiles列表。
    
    Args:
        profiles (list): 包含所有目标配置的列表。
        filepath (str): 结果文件的路径。
        
    Returns:
        list: 更新了 initial_params 的 profiles 列表。
    """
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
            # 从JSON文件中提取参数
            best_params = previous_results[profile_name]["best_params"]
            # 确保参数顺序正确 (Kp, Ki, Kd)
            new_initial_params = [best_params.get('Kp', 0.0), best_params.get('Ki', 0.0), best_params.get('Kd', 0.0)]
            
            # 检查参数是否在定义的空间内，防止因修改space导致起点无效
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
# --- 主逻辑区 ---
# ==============================================================================

def send_cmd(cmd_to_send, wait_time=0.1):
    """向RT-Thread发送命令, 并处理回显。"""
    print(f"--> {cmd_to_send}")
    if ser and ser.is_open:
        ser.reset_input_buffer()
        ser.write((cmd_to_send + '\n').encode('ascii'))
        time.sleep(wait_time)
    else:
        print("Error: Serial port is not open.")

def reset_system():
    """
    在每次评估前复位系统状态。
    让风扇停止，等待小球落下。
    """
    print(f"\n--- Resetting system: stopping fan for {RESET_DURATION_S} seconds ---")
    send_cmd("pid_tune -p 0 -i 0 -d 0", wait_time=0.2)
    time.sleep(RESET_DURATION_S)
    print("--- Reset complete, ready for next test ---")

def objective_function(params):
    global current_target_height
    kp, ki, kd = params
    kp_str, ki_str, kd_str = f"{kp:.8f}", f"{ki:.8f}", f"{kd:.8f}"

    print(f"\nTesting params: Kp={kp_str}, Ki={ki_str}, Kd={kd_str} for target_height={current_target_height}mm")

    reset_system()

    pid_cmd = f"pid_tune -p {kp_str} -i {ki_str} -d {kd_str}"
    send_cmd(pid_cmd)

    send_cmd(f"pid_tune -t {current_target_height}")

    eval_cmd = f"pid_eval {EVAL_DURATION_MS}"
    send_cmd(eval_cmd, wait_time=0)
    
    timeout = time.time() + (EVAL_DURATION_MS / 1000.0) + 5.0 
    while time.time() < timeout:
        line = ""
        try:
            line = ser.readline().decode('ascii', errors='ignore').strip()
        except (serial.SerialException, TypeError) as e:
            print(f"Serial port read error: {e}. Assuming failure.")
            return 1e9

        if line:
            if "msh >" in line:
                # print(f"<-- [ECHO] {line}") # 可以注释掉，减少不必要的输出
                continue
            
            print(f"<-- {line}")
            
            if line.startswith("EVAL_RESULT:"):
                try:
                    score = float(line.split(':')[1])
                    if score > 1.0:
                        print(f"Score received: {score}")
                        return score
                    else:
                        print(f"Warning: Received zero or invalid score ({score}). Assuming failure.")
                        return 1e9
                except (ValueError, IndexError):
                    print("Error parsing score. Assuming failure.")
                    return 1e9
    
    print("Error: Did not receive 'EVAL_RESULT:' in time. Assuming failure.")
    return 1e9

# ==============================================================================
# --- MAIN 执行区 ---
# ==============================================================================

if __name__ == '__main__':
    all_results = {}
    
    # --- 在开始前加载历史数据 ---
    print("\n" + "="*60)
    print("           Attempting to load previous results...           ")
    print("="*60)
    TARGET_PROFILES = load_initial_points_from_results(TARGET_PROFILES, RESULTS_FILE)
    print("="*60 + "\n")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, write_timeout=2)
        print(f"Successfully connected to serial port {SERIAL_PORT}.")
        time.sleep(2)
        
        for profile in TARGET_PROFILES:
            profile_name = profile["name"]
            current_target_height = profile["target_height"] 
            space = profile["space"]
            initial_params = profile["initial_params"]
            
            print("\n" + "="*60)
            print(f"   STARTING OPTIMIZATION FOR: {profile_name} (Target Height: {current_target_height}mm)")
            print("="*60)
            print(f"Initial point to test: Kp={initial_params[0]:.8f}, Ki={initial_params[1]:.8f}, Kd={initial_params[2]:.8f}")
            
            print(f"\n--- Evaluating initial point for {profile_name} ---")
            initial_score = objective_function(initial_params)
            
            if initial_score >= 1e9:
                 print(f"\nCritical Error: Failed to evaluate initial point for {profile_name}. Skipping this profile.")
                 continue 
            
            print(f"--- Initial score for {profile_name}: {initial_score} ---")
            
            # 运行贝叶斯优化
            print(f"\n--- Starting Bayesian Optimization for {TOTAL_CALLS_PER_PROFILE - 1} more iterations ---")
            result = gp_minimize(
                func=objective_function,
                dimensions=space,
                n_calls=TOTAL_CALLS_PER_PROFILE,
                n_initial_points=0, # 设置为0，因为我们手动提供了初始点
                x0=[initial_params], # 使用列表包裹，因为gp_minimize期望一个点列表
                y0=[initial_score],
                random_state=123
            )
            
            best_params = {
                "Kp": result.x[0],
                "Ki": result.x[1],
                "Kd": result.x[2]
            }
            all_results[profile_name] = {
                "target_height": current_target_height,
                "best_score": result.fun,
                "best_params": best_params
            }
            
            print("\n" + "-"*60)
            print(f"    OPTIMIZATION FINISHED FOR: {profile_name}")
            print(f"    Best Score: {result.fun:.2f}")
            print(f"    Best Kp: {result.x[0]:.8f}")
            print(f"    Best Ki: {result.x[1]:.8f}")
            print(f"    Best Kd: {result.x[2]:.8f}")
            print("-" * 60)
            
        print("\n" + "="*60)
        print("           ALL OPTIMIZATIONS COMPLETE           ")
        print("==============================================")
        print("Final Gain Scheduling Parameters:")
        
        # 格式化输出最终的C代码
        formatted_c_code = "const pid_profile_t gain_schedule_table[] = {\n"
        for name, data in all_results.items():
             height = data['target_height']
             kp = data['best_params']['Kp']
             ki = data['best_params']['Ki']
             kd = data['best_params']['Kd']
             formatted_c_code += f"    {{{height:.1f}f, {kp:.8f}f, {ki:.8f}f, {kd:.8f}f}}, // Best Score: {data['best_score']}\n"
        formatted_c_code += "};"
        print(formatted_c_code)

        with open(RESULTS_FILE, "w") as f:
            json.dump(all_results, f, indent=4)
        print(f"\nResults have been saved to '{RESULTS_FILE}'")
        print("="*60)

    except serial.SerialException as e:
        print(f"\nFatal Error: Could not communicate with serial port '{SERIAL_PORT}'. Details: {e}")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        if ser and ser.is_open:
            send_cmd("pid_tune -p 0 -i 0 -d 0")
            ser.close()
            print("\nSerial port closed.")
