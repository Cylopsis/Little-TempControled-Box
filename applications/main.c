#include <rtthread.h>
#include <rtdevice.h>
#include "drv_pin.h"
#include "YS4028B12H.h"
#include <stdlib.h> // for atof()
#include <string.h> // for strcmp()
#include <system_vars.h>
#include <math.h> // For fabsf()

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
#define LED_PIN        ((3*32)+12)      // 工作指示灯引脚
#define PTC_PIN        ((2*32)+0)       // PTC恒温发热片继电器控制引脚
#define DHT_DATA_PIN   ((2*32)+1)
#define SAMPLE_PERIOD_MS    1000        // 控制周期/采样周期 (ms)

/* 温控逻辑相关宏 */
#define HYSTERESIS_BAND     2.0f        // 迟滞范围 (+-1.0°C)
#define FAN_SPEED_CIRCULATION 0.05f      // 加热时用于空气循环的低风速（待定，可能不需要）

/*******************************************************************************
 * 全局变量
 ******************************************************************************/
rt_thread_t working_indicate = RT_NULL;
rt_thread_t screen_thread = RT_NULL;

float env_temperature = 25.0f;        // 环境温度
float current_humidity = 50.0f;       // 当前湿度值
float current_temperature = 25.0f;    // 当前温度值
float target_temperature = 25.0f;     // 目标温度值

/* PID 控制器参数 */
float KP = 0.0f;
float KI = 0.0f;
float KD = 0.0f;

/* PID 控制器状态变量 */
float integral_error = 0.0f;
float previous_error = 0.0f;

/* PTC加热器状态变量 */
rt_base_t ptc_state = PIN_LOW;

/* 控制状态与监控变量 */
volatile control_state_t control_state = CONTROL_STATE_IDLE;
float final_fan_speed = 0.0f;
float feedforward_speed_last = 0.0f;
float pid_output_last = 0.0f;

/*******************************************************************************
 * PID 增益调度与前馈
 ******************************************************************************/
typedef struct {
    float temperature;
    float kp;
    float ki;
    float kd;
} pid_profile_t;

const pid_profile_t gain_schedule_table[] = {
    // {温度°C, Kp, Ki, Kd} - 初始占位值
    {20.0f, 0.05f, 0.010f, 0.005f},
    {30.0f, 0.06f, 0.012f, 0.006f},
    {40.0f, 0.07f, 0.015f, 0.007f},
    {50.0f, 0.08f, 0.018f, 0.008f},
    {60.0f, 0.09f, 0.020f, 0.009f},
};
const int num_pid_profiles = sizeof(gain_schedule_table) / sizeof(gain_schedule_table[0]);

typedef struct {
    float temperature;
    float base_fan_speed;   // 前馈风速
} feedforward_profile_t;

// --- 前馈速度表 ---
// 概念: 维持某个高于室温的温度点，所需要的基础风扇转速 (用于散热)
feedforward_profile_t ff_table[] = {
    // {温度°C, 基础风速} - 待定，可能不需要前馈
    {20.0f, 0.10f},
    {30.0f, 0.10f},
    {40.0f, 0.10f},
    {50.0f, 0.10f},
    {60.0f, 0.10f}
};
const int num_ff_profiles = sizeof(ff_table) / sizeof(ff_table[0]);


/*******************************************************************************
 * 函数声明
 ******************************************************************************/
void working_led();
void pid_tune(int argc, char **argv);

/*******************************************************************************
 * 函数定义
 ******************************************************************************/
void working_led() {
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    while (1)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

void update_pid_gains_by_target(float current_target_temp) {
    int lower_index = -1, upper_index = -1;
    for (int i = 0; i < num_pid_profiles; i++) {
        if (gain_schedule_table[i].temperature <= current_target_temp) lower_index = i;
        if (gain_schedule_table[i].temperature >= current_target_temp && upper_index == -1) upper_index = i;
    }

    if (lower_index == -1) { // 低于范围
        KP = gain_schedule_table[0].kp; KI = gain_schedule_table[0].ki; KD = gain_schedule_table[0].kd;
    } else if (upper_index == -1 || lower_index == upper_index) { // 高于范围或精确匹配
        KP = gain_schedule_table[lower_index].kp; KI = gain_schedule_table[lower_index].ki; KD = gain_schedule_table[lower_index].kd;
    } else { // 插值
        const pid_profile_t* p_lower = &gain_schedule_table[lower_index];
        const pid_profile_t* p_upper = &gain_schedule_table[upper_index];
        float ratio = (current_target_temp - p_lower->temperature) / (p_upper->temperature - p_lower->temperature);
        KP = p_lower->kp + ratio * (p_upper->kp - p_lower->kp);
        KI = p_lower->ki + ratio * (p_upper->ki - p_lower->ki);
        KD = p_lower->kd + ratio * (p_upper->kd - p_lower->kd);
    }
}

float get_feedforward_speed(float target_temp) {
    int lower_index = -1, upper_index = -1;
    for (int i = 0; i < num_ff_profiles; i++) {
        if (ff_table[i].temperature <= target_temp) lower_index = i;
        if (ff_table[i].temperature >= target_temp && upper_index == -1) upper_index = i;
    }

    if (lower_index == -1) {
        return ff_table[0].base_fan_speed;
    } else if (upper_index == -1 || lower_index == upper_index) {
        return ff_table[lower_index].base_fan_speed;
    } else {
        const feedforward_profile_t* p_lower = &ff_table[lower_index];
        const feedforward_profile_t* p_upper = &ff_table[upper_index];
        if (p_upper->temperature == p_lower->temperature) return p_lower->base_fan_speed;
        float ratio = (target_temp - p_lower->temperature) / (p_upper->temperature - p_lower->temperature);
        return p_lower->base_fan_speed + ratio * (p_upper->base_fan_speed - p_lower->base_fan_speed);
    }
}

int main(void)
{
#if defined(__CC_ARM)
    rt_kprintf("using armcc, version: %d\n", __ARMCC_VERSION);
#elif defined(__clang__)
    rt_kprintf("using armclang, version: %d\n", __ARMCC_VERSION);
#elif defined(__ICCARM__)
    rt_kprintf("using iccarm, version: %d\n", __VER__);
#elif defined(__GNUC__)
    rt_kprintf("using gcc, version: %d.%d\n", __GNUC__, __GNUC_MINOR__);
#endif

    /* 启动线程 */
    working_indicate = rt_thread_create("WorkingIndicate", working_led, RT_NULL, 256, 11, 20);
    screen_thread = rt_thread_create("ScreenUpdate", screen_on, RT_NULL, 1280, 12, 20);
    if(working_indicate != RT_NULL && screen_thread != RT_NULL)
    {
        rt_thread_startup(working_indicate);
        rt_thread_startup(screen_thread);
        rt_kprintf("Screen & Indicating Threads started successfully.\n");
    }

    /* 初始化PTC */
    rt_pin_mode(PTC_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PTC_PIN, ptc_state);

    /* 初始化风扇 */
    ys4028b12h_cfg_t cfg = &my_ys4028b12h_config;
    ys4028b12h_init(cfg);
    ys4028b12h_set_speed(cfg, 0.0f);

    /* 初始化温度传感器 */
    p3t1755_init(); // 板载
    rt_device_t temp_dev = RT_NULL;
    rt_device_t humi_dev = RT_NULL;
    struct rt_sensor_data temp_data;
    struct rt_sensor_data humi_data;
    temp_dev = rt_device_find("temp_dht");
    humi_dev = rt_device_find("humi_dht");
    if (temp_dev && humi_dev) 
    {
        if(rt_device_open(temp_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK\
        || rt_device_open(humi_dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
        {
            rt_kprintf("Open DHT device failed.\n");
        }
    } else {
        rt_kprintf("DHT device not found.\n");
    }

    if (1 != rt_device_read(temp_dev, 0, &temp_data, 1)) 
    {
        rt_kprintf("Read temp data failed.\n");
    }
    else
    {
        rt_kprintf("[%d] Temp: %d\n", temp_data.timestamp, temp_data.data.temp);
    }
        
    
    /* 初始化 PID 参数 */
    rt_kprintf("Initializing PID for default target: %.1f C\n", target_temperature);
    update_pid_gains_by_target(target_temperature);
    pid_tune(1, RT_NULL); // 显示初始状态

    /* 连接 WiFi */
    rt_wlan_connect("142A_SecurityPlus", "142a8888");

    /***************************************************************************
     * 温控主循环
     **************************************************************************/
    while (1)
    {
        // 读取环境信息
        p3t1755_read_temp(&env_temperature);
        if (1 != rt_device_read(temp_dev, 0, &temp_data, 1)) 
        {
            rt_kprintf("Read temp data failed.\n");
            continue;
        }
        else
        {
            current_temperature = (float)(temp_data.data.temp) / 10.0f;
        }
        if (1 != rt_device_read(humi_dev, 0, &humi_data, 1)) 
        {
            rt_kprintf("Read humi data failed.\n");
            continue;
        }
        else
        {
            current_humidity = (float)(humi_data.data.humi) / 10.0f;
        }
        
        // PTC加热器迟滞控制
        float lower_bound = target_temperature - (HYSTERESIS_BAND / 2.0f);
        float upper_bound = target_temperature + (HYSTERESIS_BAND / 2.0f);
        if (current_temperature < lower_bound) ptc_state = PIN_HIGH;
        else if (current_temperature > upper_bound) ptc_state = PIN_LOW;
        rt_pin_write(PTC_PIN, ptc_state);

        // 风扇控制逻辑
        feedforward_speed_last = 0.0f;
        pid_output_last = 0.0f;
        final_fan_speed = 0.0f;
        if (ptc_state == PIN_HIGH) 
        {
            // --- 状态: 加热 ---
            control_state = CONTROL_STATE_HEATING;
            final_fan_speed = FAN_SPEED_CIRCULATION;
            // 重置PID积分项，防止在加热模式下积分累积
            integral_error = 0.0f;
            previous_error = 0.0f;
            rt_kprintf("Temp:%.2fC Env:%.2fC | HEATING | PTC: ON, Fan: %.0f%%\n", 
                       current_temperature, env_temperature, final_fan_speed * 100);
        }
        else // ptc_state == PIN_LOW
        {
            // --- 状态: 冷却或保温 ---
            if (current_temperature > target_temperature)
            {
                // --- 子状态: 主动冷却 (PID + Feedforward) ---
                control_state = CONTROL_STATE_COOLING;
                float error = current_temperature - target_temperature;
                float dt = SAMPLE_PERIOD_MS / 1000.0f;

                integral_error += error * dt;
                float derivative_error = (error - previous_error) / dt;
                
                // PID 输出
                float pid_output = (KP * error) + (KI * integral_error) + (KD * derivative_error);
                previous_error = error;
                pid_output_last = pid_output;

                // 前馈与PID输出合并
                float ff_speed = get_feedforward_speed(target_temperature);
                feedforward_speed_last = ff_speed;
                final_fan_speed = ff_speed + pid_output;

                // 输出限幅与积分抗饱和
                if (final_fan_speed > 1.0f) {
                    final_fan_speed = 1.0f;
                    integral_error -= error * dt; // 抗饱和
                }
                if (final_fan_speed < 0.0f) {
                    final_fan_speed = 0.0f;
                    integral_error -= error * dt; // 抗饱和
                }
                
                rt_kprintf("Temp:%.2fC Env:%.2fC | COOLING | PTC: OFF, Fan: %.0f%% (FF:%.2f+PID:%.2f)\n",
                           current_temperature, env_temperature, final_fan_speed * 100, ff_speed, pid_output);
            }
            else
            {
                // --- 子状态: 保温/空闲 ---
                control_state = CONTROL_STATE_IDLE;
                final_fan_speed = FAN_SPEED_CIRCULATION;
                rt_kprintf("Temp:%.2fC Env:%.2fC | IDLE    | PTC: OFF, Fan: %.0f%%\n", current_temperature, env_temperature, final_fan_speed * 100);
            }
        }
        
        ys4028b12h_set_speed(cfg, final_fan_speed);
        rt_thread_mdelay(SAMPLE_PERIOD_MS);
    }
    
    return 0;
}

/*******************************************************************************
 * MSH 命令
 ******************************************************************************/
static const char *control_state_to_string(control_state_t state)
{
    switch (state)
    {
        case CONTROL_STATE_HEATING: return "HEATING";
        case CONTROL_STATE_COOLING: return "COOLING";
        case CONTROL_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static void get_status(int argc, char **argv)
{
    rt_kprintf("--- System Status ---\n");
    rt_kprintf("Current Temperature:  %.2f C\n", current_temperature);
    rt_kprintf("Target Temperature:   %.2f C\n", target_temperature);
    rt_kprintf("PTC Heater State:     %s\n", (ptc_state == PIN_HIGH) ? "ON" : "OFF");
    rt_kprintf("Control State:        %s\n", control_state_to_string(control_state));
    rt_kprintf("Fan Command:          %.1f %%\n", final_fan_speed * 100.0f);
    rt_kprintf("--- PID Controller ---\n");
    rt_kprintf("Scheduled Gains: Kp=%.6f, Ki=%.6f, Kd=%.6f\n", KP, KI, KD);
    rt_kprintf("Integral Error:  %.4f\n", integral_error);
    rt_kprintf("Previous Error:  %.4f\n", previous_error);
    rt_kprintf("Feedforward Spd: %.4f\n", feedforward_speed_last);
    rt_kprintf("PID Output:      %.4f\n", pid_output_last);
}
MSH_CMD_EXPORT(get_status, Get current system status for temperature control);

void pid_tune(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("\n--- Usage ---\n");
        rt_kprintf("  pid_tune -t <val>                    (Set target temperature in C)\n");
        rt_kprintf("  pid_tune -p <val> -i <val> -d <val>  (Manual PID override)\n");
        rt_kprintf("  pid_tune -ff                         (Show Feedforward table)\n");
        rt_kprintf("  pid_tune -ff_set <idx> <temp> <spd>  (Set Feedforward entry)\n");
        rt_kprintf("    e.g., pid_tune -ff_set 2 40.0 0.25\n");
        get_status(1, RT_NULL);
        return;
    }

    if (strcmp(argv[1], "-ff") == 0) {
        rt_kprintf("--- Feedforward Table ---\n");
        rt_kprintf("Idx | Temp (C) | Base Speed\n");
        rt_kprintf("----|----------|-----------\n");
        for (int i = 0; i < num_ff_profiles; i++) {
            rt_kprintf("%-3d | %-8.1f | %.4f\n", i, ff_table[i].temperature, ff_table[i].base_fan_speed);
        }
        return;
    }

    if (strcmp(argv[1], "-ff_set") == 0) {
        if (argc != 5) {
            rt_kprintf("Error: Incorrect arguments. Usage: pid_tune -ff_set <idx> <temp> <spd>\n");
            return;
        }
        int index = atoi(argv[2]);
        if (index < 0 || index >= num_ff_profiles) {
            rt_kprintf("Error: Index %d is out of bounds (0-%d).\n", index, num_ff_profiles - 1);
            return;
        }
        ff_table[index].temperature = atof(argv[3]);
        ff_table[index].base_fan_speed = atof(argv[4]);
        rt_kprintf("FF table entry %d updated to: Temp=%.1fC, Speed=%.4f\n", index, ff_table[index].temperature, ff_table[index].base_fan_speed);
        return;
    }
    
    rt_bool_t target_changed = RT_FALSE;
    for (int i = 1; i < argc; i += 2) {
        if (strcmp(argv[i], "-p") == 0) { KP = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-i") == 0) { KI = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-d") == 0) { KD = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-t") == 0) {
            target_temperature = atof(argv[i+1]);
            target_changed = RT_TRUE;
        }
        else { rt_kprintf("Error: Unknown option %s\n", argv[i]); return; }
    }

    if (target_changed) {
        rt_kprintf("Target temperature changed. Re-scheduling PID gains...\n");
        update_pid_gains_by_target(target_temperature);
    }
    
    rt_kprintf("Parameters updated. Current status:\n");
    get_status(1, RT_NULL);
}
MSH_CMD_EXPORT(pid_tune, Tune PID and Feedforward for temperature control);
