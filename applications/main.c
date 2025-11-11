#include <rtthread.h>
#include <rtdevice.h>
#include "drv_pin.h"
#include "YS4028B12H.h"
#include <stdlib.h> // for atof()
#include <string.h> // for strcmp()
#include <system_vars.h>
#include <math.h>   // For fabsf()

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
#define LED_PIN             ((3*32)+12)      // 工作指示灯引脚
#define PTC_PIN             ((2*32)+0)       // PTC恒温发热片继电器控制引脚
#define BTM_PTC_PIN         ((0*32)+22)      // 底板PTC发热片继电器控制引脚
#define DHT_DATA_PIN        ((2*32)+1)       // DHT11数据引脚
#define SAMPLE_PERIOD_MS    1000             // 控制周期/采样周期 (ms)

/* 温控逻辑相关枚举在 system_vars.h 中声明 */

/*******************************************************************************
 * 全局变量
 ******************************************************************************/
rt_thread_t working_indicate = RT_NULL;
rt_thread_t screen_thread = RT_NULL;

float env_temperature = 25.0f;        // 环境温度
float current_humidity = 50.0f;       // 当前湿度值
float current_temperature = 25.0f;    // 当前温度值
float target_temperature = 25.0f;     // 目标温度值

/* 温控与风扇参数，可通过 fan_tune 动态调整 */
float warming_threshold = 3.3f;             // 慢加热阈值 (-3.3°C)
float hysteresis_band = 0.7f;               // 迟滞范围 (+-0.7°C)
float fan_speed_circulation = 0.00f;        // 加热时用于空气循环的低风速
float fan_min = 0.00f;                      // 最小风速（非零时）
float fan_max = 0.63f;                      // 最大风速
float fan_smooth_alpha = 0.3f;              // 风扇命令平滑系数

/* PID 控制器参数 */
float KP = 0.03f;
float KI = 0.002f;
float KD = 0.0f;

/* PID 控制器状态变量 */
float integral_error = 0.0f;
float previous_error = 0.0f;

/* PTC加热器状态变量 */
rt_base_t ptc_state = PIN_LOW;
rt_base_t btm_ptc_state = PIN_LOW;

/* 控制状态与监控变量 */
volatile control_state_t control_state = CONTROL_STATE_IDLE;
float final_fan_speed = 0.0f;          // 平滑后的风扇速度
float fan_speed_cmd = 0.0f;            // 风扇命令（未平滑）
float feedforward_speed_last = 0.0f;
float pid_output_last = 0.0f;

/*******************************************************************************
 * 前馈速度表
 ******************************************************************************/
typedef struct {
    float delta_t;         // 目标温度 - 环境温度
    float base_fan_speed;  // 对应的基础风速
} ff_profile_t;

// 基于 ΔT 的前馈表（需要根据实际测试微调）
ff_profile_t ff_table[] = {
    {  0.0f, 0.00f },     // 目标与环境相同，无需额外散热
    {  5.0f, 0.05f },     
    { 20.0f, 0.10f },     
    { 30.0f, 0.20f },     
    { 40.0f, 0.30f },     
    { 50.0f, 0.40f }      
};
const int num_ff_profiles = sizeof(ff_table) / sizeof(ff_table[0]);

/*******************************************************************************
 * 函数声明
 ******************************************************************************/
void working_led();
extern void remote_start(int argc, char **argv);
void pid_tune(int argc, char **argv);
void fan_tune(int argc, char **argv);
float get_feedforward_speed(float target_temp, float env_temp);

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

float get_feedforward_speed(float target_temp, float env_temp)
{
    float dt = target_temp - env_temp;
    if (dt <= ff_table[0].delta_t) return ff_table[0].base_fan_speed;
    if (dt >= ff_table[num_ff_profiles - 1].delta_t)
        return ff_table[num_ff_profiles - 1].base_fan_speed;

    for (int i = 0; i < num_ff_profiles - 1; i++)
    {
        ff_profile_t *a = &ff_table[i];
        ff_profile_t *b = &ff_table[i + 1];
        if (dt >= a->delta_t && dt <= b->delta_t)
        {
            float ratio = (dt - a->delta_t) / (b->delta_t - a->delta_t);
            return a->base_fan_speed + ratio * (b->base_fan_speed - a->base_fan_speed);
        }
    }
    return 0.0f; // 理论上不会走到这里
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
    screen_thread = rt_thread_create("ScreenUpdate", screen_on, RT_NULL, 2048, 12, 20);
    if(working_indicate != RT_NULL && screen_thread != RT_NULL)
    {
        rt_thread_startup(working_indicate);
        rt_thread_startup(screen_thread);
        rt_kprintf("Screen & Indicating Threads started successfully.\n");
    }

    /* 初始化PTC */
    rt_pin_mode(PTC_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(BTM_PTC_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PTC_PIN, ptc_state);
    rt_pin_write(BTM_PTC_PIN, btm_ptc_state);

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
    
    /* 初始化 PID 参数 */
    rt_kprintf("Initializing PID for default target: %.1f C\n", target_temperature);
    rt_kprintf("Initial PID: KP=%.3f, KI=%.3f, KD=%.3f\n", KP, KI, KD);
    pid_tune(1, RT_NULL); // 显示初始状态

    /* 连接 WiFi */
    rt_wlan_connect("142A_SecurityPlus", "142a8888");
    rt_thread_mdelay(5000); // 等待连接稳定
    /* 启动远程控制服务器 */
    remote_start(0, RT_NULL);

    /***************************************************************************
     * 温控主循环
     ************************************************************************/
    while (1)
    {
        // 读取环境信息
        p3t1755_read_temp(&env_temperature);
        if (1 != rt_device_read(temp_dev, 0, &temp_data, 1)) 
        {
            // DHT11特别容易读取失败，不打印，避免刷屏
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

        // PTC逻辑
        float lower_bound = target_temperature - hysteresis_band - warming_threshold;
        float warming_bound = target_temperature - hysteresis_band / 2.0f;
        float upper_bound = target_temperature + hysteresis_band;

        if (current_temperature < lower_bound)
        {
            // 温度偏低
            control_state = CONTROL_STATE_HEATING;
            ptc_state = PIN_HIGH;
            btm_ptc_state = PIN_HIGH;
        }
        else if (current_temperature < warming_bound)
        {
            // 在目标附近
            control_state = CONTROL_STATE_WARMING;
            ptc_state = PIN_LOW;
            btm_ptc_state = PIN_HIGH;
            
        }
        else if(control_state == CONTROL_STATE_WARMING && current_temperature > target_temperature + hysteresis_band/2.0)
        {
            control_state = CONTROL_STATE_IDLE;
            ptc_state = PIN_LOW;
            btm_ptc_state = PIN_LOW;
        }
        else if (control_state == CONTROL_STATE_COOLING && current_temperature < target_temperature)
        {
            control_state = CONTROL_STATE_IDLE;
            ptc_state = PIN_LOW;
            btm_ptc_state = PIN_LOW;
        }
        else if (current_temperature > upper_bound){
            control_state = CONTROL_STATE_COOLING;
            ptc_state = PIN_LOW;
            btm_ptc_state = PIN_LOW;
        }
        
        rt_pin_write(PTC_PIN, ptc_state);
        rt_pin_write(BTM_PTC_PIN, btm_ptc_state);

        // 风扇控制逻辑
        fan_speed_cmd = fan_speed_circulation;
        feedforward_speed_last = 0.0f;
        pid_output_last = 0.0f;
        float dt = SAMPLE_PERIOD_MS / 1000.0f;

        // 加热阶段
        if (control_state == CONTROL_STATE_HEATING)
        {
            fan_speed_cmd = fan_speed_circulation;
            // 清PID，防止加热阶段积累影响后面冷却
            integral_error = 0.0f;
            previous_error = 0.0f;
            
            // rt_kprintf("Temp:%.2fC Env:%.2fC | HEATING | PTC:%d BTM:%d Fan:%.0f%%\n",
            //            current_temperature, env_temperature,
            //            ptc_state == PIN_HIGH, btm_ptc_state == PIN_HIGH,
            //            fan_speed_cmd * 100.0f);
        }
        else if (control_state == CONTROL_STATE_COOLING)
        {
            // 非加热阶段
            float error = current_temperature - target_temperature;
            float derivative = (error - previous_error) / dt;
            previous_error = error;
            
            integral_error += error * dt;
            
            // PID输出
            float pid_output = KP * error + KI * integral_error + KD * derivative;
            pid_output_last = pid_output;
            
            // 前馈与PID输出合并
            float ff_speed = get_feedforward_speed(target_temperature, env_temperature);
            feedforward_speed_last = ff_speed;
            fan_speed_cmd = ff_speed + pid_output;
            
            // 限幅与抗积分饱和
            if (fan_speed_cmd > fan_max) {
                fan_speed_cmd = fan_max;
                integral_error -= error * dt; // 抗饱和
            }
            if (fan_speed_cmd < fan_min) {
                fan_speed_cmd = fan_min;
                integral_error -= error * dt;
            }
            
            // rt_kprintf("Temp:%.2fC Env:%.2fC | COOLING | Fan:%.0f%% (FF:%.2f PID:%.2f)\n",
            //            current_temperature, env_temperature,
            //            fan_speed_cmd * 100.0f, ff_speed, pid_output);
        }
        else if (control_state == CONTROL_STATE_WARMING)
        {
            // 目标附近
            fan_speed_cmd = fan_speed_circulation;
            // 在这个区域逐步衰减积分，防止偶尔进入冷却时带太大历史误差
            integral_error *= 0.8f;
            
            // rt_kprintf("Temp:%.2fC Env:%.2fC | IDLE    | Fan:%.0f%%\n",
            //             current_temperature, env_temperature,
            //             fan_speed_cmd * 100.0f);
        }

        // 简单一阶滤波，避免PWM抖动
        final_fan_speed = final_fan_speed * (1.0f - fan_smooth_alpha)
                        + fan_speed_cmd   * fan_smooth_alpha;

        // 安全限幅
        if (final_fan_speed > fan_max) final_fan_speed = fan_max;
        if (final_fan_speed < fan_min) final_fan_speed = fan_min;

        ys4028b12h_set_speed(cfg, final_fan_speed);
        
        rt_thread_mdelay(SAMPLE_PERIOD_MS);
    }
    
    return 0;
}

/*******************************************************************************
 * MSH 命令（调试用的）
 ******************************************************************************/
static const char *control_state_to_string(control_state_t state)
{
    switch (state)
    {
        case CONTROL_STATE_HEATING: return "HEATING";
        case CONTROL_STATE_WARMING: return "WARMING";
        case CONTROL_STATE_COOLING: return "COOLING";
        case CONTROL_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static void print_fan_control_params(void)
{
    rt_kprintf("--- Fan Control Parameters ---\n");
    rt_kprintf("Warming Threshold:    %.3f C\n", warming_threshold);
    rt_kprintf("Hysteresis Band:      %.3f C\n", hysteresis_band);
    rt_kprintf("Circulation Speed:    %.3f\n", fan_speed_circulation);
    rt_kprintf("Fan Min:              %.3f\n", fan_min);
    rt_kprintf("Fan Max:              %.3f\n", fan_max);
    rt_kprintf("Smooth Alpha:         %.3f\n", fan_smooth_alpha);
}

static void get_status(int argc, char **argv)
{
    rt_kprintf("--- System Status ---\n");
    rt_kprintf("Current Temperature:  %.2f C\n", current_temperature);
    rt_kprintf("Target Temperature:   %.2f C\n", target_temperature);
    rt_kprintf("Env Temperature:      %.2f C\n", env_temperature);
    rt_kprintf("PTC Heater State:     %s\n", (ptc_state == PIN_HIGH) ? "ON" : "OFF");
    rt_kprintf("Bottom PTC State:     %s\n", (btm_ptc_state == PIN_HIGH) ? "ON" : "OFF");
    rt_kprintf("Control State:        %s\n", control_state_to_string(control_state));
    rt_kprintf("Fan Command:          %.1f %%\n", final_fan_speed * 100.0f);
    rt_kprintf("--- PID Controller ---\n");
    rt_kprintf("Gains: Kp=%.6f, Ki=%.6f, Kd=%.6f\n", KP, KI, KD);
    rt_kprintf("Integral Error:  %.4f\n", integral_error);
    rt_kprintf("Previous Error:  %.4f\n", previous_error);
    rt_kprintf("Feedforward Spd: %.4f\n", feedforward_speed_last);
    rt_kprintf("PID Output:      %.4f\n", pid_output_last);
    print_fan_control_params();
}
MSH_CMD_EXPORT(get_status, Get current system status for temperature control);

void fan_tune(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("\n--- Usage ---\n");
        rt_kprintf("  fan_tune -show\n");
        rt_kprintf("  fan_tune -warm <val> -hys <val>\n");
        rt_kprintf("           -circ <val> -min <val> -max <val> -alpha <val>\n");
        print_fan_control_params();
        return;
    }

    float new_warming_threshold = warming_threshold;
    float new_hysteresis_band = hysteresis_band;
    float new_fan_speed_circulation = fan_speed_circulation;
    float new_fan_min = fan_min;
    float new_fan_max = fan_max;
    float new_fan_smooth_alpha = fan_smooth_alpha;

    rt_bool_t updated = RT_FALSE;
    rt_bool_t show_requested = RT_FALSE;

    int idx = 1;
    while (idx < argc) {
        const char *opt = argv[idx];

        if (strcmp(opt, "-show") == 0) {
            show_requested = RT_TRUE;
            idx += 1;
            continue;
        }

        if (idx + 1 >= argc) {
            rt_kprintf("Error: Option %s requires a value.\n", opt);
            return;
        }

        const char *value_str = argv[idx + 1];
        float value = atof(value_str);

        if (strcmp(opt, "-warm") == 0) {
            if (value < 0.0f) {
                rt_kprintf("Error: warming threshold must be >= 0.\n");
                return;
            }
            new_warming_threshold = value;
            updated = RT_TRUE;
        } else if (strcmp(opt, "-hys") == 0) {
            if (value < 0.0f) {
                rt_kprintf("Error: hysteresis band must be >= 0.\n");
                return;
            }
            new_hysteresis_band = value;
            updated = RT_TRUE;
        } else if (strcmp(opt, "-circ") == 0) {
            if (value < 0.0f || value > 1.0f) {
                rt_kprintf("Error: circulation speed must be within [0, 1].\n");
                return;
            }
            new_fan_speed_circulation = value;
            updated = RT_TRUE;
        } else if (strcmp(opt, "-min") == 0) {
            if (value < 0.0f || value > 1.0f) {
                rt_kprintf("Error: fan_min must be within [0, 1].\n");
                return;
            }
            new_fan_min = value;
            updated = RT_TRUE;
        } else if (strcmp(opt, "-max") == 0) {
            if (value < 0.0f || value > 1.0f) {
                rt_kprintf("Error: fan_max must be within [0, 1].\n");
                return;
            }
            new_fan_max = value;
            updated = RT_TRUE;
        } else if (strcmp(opt, "-alpha") == 0) {
            if (value < 0.0f || value > 1.0f) {
                rt_kprintf("Error: smooth alpha must be within [0, 1].\n");
                return;
            }
            new_fan_smooth_alpha = value;
            updated = RT_TRUE;
        } else {
            rt_kprintf("Error: Unknown option %s\n", opt);
            return;
        }

        idx += 2;
    }

    if (!updated) {
        if (show_requested) {
            print_fan_control_params();
            return;
        }
        rt_kprintf("No parameters updated. Use -show to display current settings.\n");
        return;
    }

    if (new_fan_min > new_fan_max) {
        rt_kprintf("Error: fan_min (%.3f) cannot exceed fan_max (%.3f).\n", new_fan_min, new_fan_max);
        return;
    }

    if (new_fan_speed_circulation < new_fan_min || new_fan_speed_circulation > new_fan_max) {
        rt_kprintf("Error: circulation speed %.3f must be within [%.3f, %.3f].\n",
                   new_fan_speed_circulation, new_fan_min, new_fan_max);
        return;
    }

    warming_threshold = new_warming_threshold;
    hysteresis_band = new_hysteresis_band;
    fan_speed_circulation = new_fan_speed_circulation;
    fan_min = new_fan_min;
    fan_max = new_fan_max;
    fan_smooth_alpha = new_fan_smooth_alpha;

    rt_kprintf("Fan control parameters updated.\n");
    print_fan_control_params();
}
MSH_CMD_EXPORT(fan_tune, Tune fan control parameters at runtime);

void pid_tune(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("\n--- Usage ---\n");
        rt_kprintf("  pid_tune -t <val>                    (Set target temperature in C)\n");
        rt_kprintf("  pid_tune -p <val> -i <val> -d <val>  (Manual PID override)\n");
        rt_kprintf("  pid_tune -ff                         (Show Feedforward table)\n");
        rt_kprintf("  pid_tune -ff_set <idx> <dt> <spd>    (Set Feedforward entry)\n");
        rt_kprintf("    e.g., pid_tune -ff_set 2 15.0 0.20\n");
        get_status(1, RT_NULL);
        return;
    }

    if (strcmp(argv[1], "-ff") == 0) {
        rt_kprintf("--- Feedforward Table (ΔT -> base fan speed) ---\n");
        rt_kprintf("Idx | ΔT(C) | Base Speed\n");
        rt_kprintf("----|--------|----------\n");
        for (int i = 0; i < num_ff_profiles; i++) {
            rt_kprintf("%-3d | %-6.1f | %.4f\n", i, ff_table[i].delta_t, ff_table[i].base_fan_speed);
        }
        return;
    }

    if (strcmp(argv[1], "-ff_set") == 0) {
        if (argc != 5) {
            rt_kprintf("Error: Incorrect arguments. Usage: pid_tune -ff_set <idx> <dt> <spd>\n");
            return;
        }
        int index = atoi(argv[2]);
        if (index < 0 || index >= num_ff_profiles) {
            rt_kprintf("Error: Index %d is out of bounds (0-%d).\n", index, num_ff_profiles - 1);
            return;
        }
        ff_table[index].delta_t = atof(argv[3]);
        ff_table[index].base_fan_speed = atof(argv[4]);
        rt_kprintf("FF table entry %d updated to: ΔT=%.1fC, Speed=%.4f\n", 
                   index, ff_table[index].delta_t, ff_table[index].base_fan_speed);
        return;
    }
    
    // rt_bool_t target_changed = RT_FALSE;
    for (int i = 1; i < argc; i += 2) {
        if (strcmp(argv[i], "-p") == 0) { KP = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-i") == 0) { KI = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-d") == 0) { KD = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-t") == 0) {
            target_temperature = atof(argv[i+1]);
            // target_changed = RT_TRUE;
        }
        else { rt_kprintf("Error: Unknown option %s\n", argv[i]); return; }
    }

    // 移除了增益调度相关，target_changed不再影响PID参数
    
    rt_kprintf("Parameters updated. Current status:\n");
    get_status(1, RT_NULL);
}
MSH_CMD_EXPORT(pid_tune, Tune PID and Feedforward for temperature control);
