#include <rtthread.h>
#include <rtdevice.h>
#include "YS4028B12H.h"
#include <stdlib.h> // for atof()
#include <string.h> // for strcmp()
#include <system_vars.h>
#include <math.h>   // for log()
#include "fsl_pwm.h"
/*******************************************************************************
 * 线程句柄
 ******************************************************************************/
rt_thread_t working_indicate = RT_NULL;
rt_thread_t screen_thread = RT_NULL;
rt_thread_t pid_thread = RT_NULL;

/*******************************************************************************
 * 设备句柄
 ******************************************************************************/
rt_device_t dht_temp_dev = RT_NULL;
rt_device_t dht_humi_dev = RT_NULL;
rt_device_t adc_dev = RT_NULL;
rt_pwm_t pwm_dev = RT_NULL;

/*******************************************************************************
 * 参数定义
 ******************************************************************************/
/* 传感器信息 */
volatile float env_temperature = 25.0f;        // 环境温度
volatile float current_humidity = 50.0f;       // 当前湿度值
volatile float current_temperature = 25.0f;    // 当前温度值
volatile float target_temperature = 40.0f;     // 目标温度值
volatile float ptc_temperature = 25.0f;        // PTC温度值
volatile float ptc_target_temp = 40.0f;        // PTC目标温度

/* 温控参数 */
float warming_threshold = 3.0f;       // 当前保温阈值 (由前馈表决定)
float warming_bias = 5.0f;            // 保温偏置温度（PTC温度高于目标温度多少用于保温）
float heating_bias = 30.0f;           // 加热偏置温度（PTC温度高于目标温度多少用于加热）
float hysteresis_band = 2.0f;         // 迟滞范围
float fan_min = 0.00f;                // 最小风速
float fan_max = 0.63f;                // 最大风速

/* 控制状态与监控变量 */
pid_ctx_t pid_box;  // 外环PID控制PTC目标温度
pid_ctx_t pid_ptc;  // 内环PID控制PTC温度
pid_ctx_t pid_cool; // 风扇 PI
volatile control_state_t control_state = CONTROL_STATE_WARMING;
volatile rt_uint32_t ptc_state = HEAT;
volatile float final_pwm_duty = 0.0f;          // 当前PWM占空比
/*******************************************************************************
 * 前馈表
 ******************************************************************************/
typedef struct {
    float target_temp;         // PTC目标温度
    float base_pwm;            // 对应的PWM占空比
} ff_profile_t;
ff_profile_t ff_table[] = {
    { 20.0f, 0.18f }, 
    { 25.0f, 0.23f },     
    { 30.0f, 0.27f },     
    { 40.0f, 0.36f },     
    { 50.0f, 0.46f },     
    { 60.0f, 0.55f },
    { 70.0f, 0.64f },
    { 80.0f, 0.73f },
    { 90.0f, 0.82f },
    {100.0f, 0.91f }
};
const int num_ff_profiles = sizeof(ff_table) / sizeof(ff_table[0]);

typedef struct {
    float target_temp;            // 目标温度
    float ptc_temp;               // 维持目标温度需要的PTC温度
} warming_ff_entry_t;
static warming_ff_entry_t warming_ff_table[] = {
    { 25.0f, 35.0f },
    { 30.0f, 40.5f },
    { 40.0f, 53.0f },
    { 57.0f, 85.0f },
    { 70.0f, 100.0f }
};
const int num_warming_ff_entries = sizeof(warming_ff_table) / sizeof(warming_ff_table[0]);

/*******************************************************************************
 * 函数声明
 ******************************************************************************/
void working_led();
extern void remote_start(int argc, char **argv);
void tune(int argc, char **argv);
static const char* control_state_to_string(control_state_t state);
rt_err_t initialization();
static float get_feedforward_pwm(float target_temp);
static float get_warming_temp(float target_temp);
static float ntc_adc_to_temp(uint32_t adc_val);
void pid_entry(void *parameter);
/*----------------------------------------------------------------------------*/
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
    struct rt_sensor_data dht_temp_data;
    struct rt_sensor_data dht_humi_data;
    rt_uint32_t adc_value = 0;
    if(initialization(&dht_temp_dev, &dht_humi_dev, &adc_dev, &pwm_dev) != RT_EOK) {
        rt_kprintf("Initialization failed!\n");
        return -RT_ERROR;
    }
    /* 启动线程 */
    pid_thread = rt_thread_create("PIDControl", pid_entry, RT_NULL, 1024, 10, 30);
    if (pid_thread != RT_NULL) {
        rt_thread_startup(pid_thread);
    }
    /* 启动远程控制服务器 */
    remote_start(0, RT_NULL);
    working_indicate = rt_thread_create("WorkingIndicate", working_led, RT_NULL, 256, 11, 20);
    screen_thread = rt_thread_create("ScreenUpdate", screen_on, RT_NULL, 2048, 12, 20);
    if(working_indicate != RT_NULL && screen_thread != RT_NULL)
    {
        rt_thread_startup(working_indicate);
        rt_thread_startup(screen_thread);
        rt_kprintf("Screen & Indicating Threads started successfully.\n");
    }
    /***************************************************************************
     * 温控状态控制循环
     ************************************************************************/
    while (1)
    {
        // 读取环境信息
        p3t1755_read_temp(&env_temperature);
        if (1 != rt_device_read(dht_temp_dev, 0, &dht_temp_data, 1)) continue;
        else current_temperature = (float)(dht_temp_data.data.temp) / 10.0f;
        if (1 != rt_device_read(dht_humi_dev, 0, &dht_humi_data, 1)) 
        {
            rt_kprintf("Read humi data failed.\n");
            continue;
        }
        else current_humidity = (float)(dht_humi_data.data.humi) / 10.0f;

        control_state_t previous_state = control_state;
        float upper_bound = target_temperature + hysteresis_band;
        float lower_bound = target_temperature - hysteresis_band;
        if (current_temperature < lower_bound) {
            control_state = CONTROL_STATE_HEATING;
        } else if (current_temperature > upper_bound + 1) {
            control_state = CONTROL_STATE_COOLING;
        } else if (current_temperature < upper_bound ) {
            control_state = CONTROL_STATE_WARMING;
        }
        
        // 处理状态切换
        if (control_state != previous_state) {
            // rt_kprintf("State Changed: %s -> %s\n", control_state_to_string(previous_state), control_state_to_string(control_state));
            // 切换前关闭PWM
            rt_pwm_set(pwm_dev, 0, PTC_PERIOD, 0);
            rt_thread_mdelay(20);
            if (control_state == CONTROL_STATE_COOLING) {
                ptc_state = COOL;
                rt_pin_write(STATE_PIN, ptc_state);
                pid_cool.integral = 0.0f; // 重置积分
                pid_cool.prev_error = 0.0f;
                pid_box.integral = 0.0f;
                pid_box.prev_error = 0.0f;
                pid_ptc.integral = 0.0f;
                pid_ptc.prev_error = 0.0f;
            } else {
                ptc_state = HEAT;
                rt_pin_write(STATE_PIN, ptc_state);
            }
        }

        // rt_kprintf("PTC Temp: %.2f C | Current Temp: %.2f C, Target Temp: %.2f C, Env Temp: %.2f C, Humidity: %.2f %% | PWM: %.2f %%\n",
        //             ptc_temperature,current_temperature, target_temperature, env_temperature, current_humidity, final_pwm_duty * 100.0f);
        
        rt_thread_mdelay(SAMPLE_PERIOD_MS);
    }
    
    return 0;
}

/*******************************************************************************
 * 函数定义
 ******************************************************************************/
void pid_entry(void *parameter)
{
    rt_kprintf("PID control thread started.\n");
    float dt = CONTROL_PERIOD_MS / 1000.0f;
    float fan_cmd = 0.0f;
    while (1)
    {
        rt_uint32_t adc_value = rt_adc_read(adc_dev, 0);
        ptc_temperature = ntc_adc_to_temp(adc_value);
        
        float error = 0.0f;
        float output = 0.0f;
        
        switch(control_state)
        {
            case CONTROL_STATE_HEATING:
            case CONTROL_STATE_WARMING:
            {
                // 外环PID：控制箱内温度，输出PTC的目标温度
                float outer_error = target_temperature - current_temperature;
                pid_box.integral += outer_error * dt;
                if(pid_box.integral > 100.0f) pid_box.integral = 100.0f;
                if(pid_box.integral < -100.0f) pid_box.integral = -100.0f;
                float outer_derivative = (outer_error - pid_box.prev_error) / dt;
                float outer_output = pid_box.kp * outer_error + pid_box.ki * pid_box.integral + pid_box.kd * outer_derivative;
                // 计算PTC目标温度
                ptc_target_temp = get_warming_temp(target_temperature) + outer_output;
                float ratio = fabs(outer_error + hysteresis_band) / (hysteresis_band * 2);
                if (ratio > 1) ratio = 1;
                float dynamic_bias = warming_bias + (heating_bias - warming_bias) * ratio;
                if (ptc_target_temp > target_temperature + dynamic_bias) ptc_target_temp = target_temperature + dynamic_bias;
                else if (ptc_target_temp < target_temperature + warming_bias) ptc_target_temp = target_temperature + warming_bias;
                if (ptc_target_temp > PTC_MAX_SAFE_TEMP) ptc_target_temp = PTC_MAX_SAFE_TEMP;
                
                pid_box.prev_error = outer_error;

                // 内环PID：控制PTC温度到ptc_target_temp
                if (ptc_temperature >= PTC_MAX_SAFE_TEMP) {
                    output = 0.0f; // 过热保护
                    rt_kprintf("WARNING: PTC Overheat! Temp: %.1f\n", ptc_temperature);
                } else {
                    float inner_error = ptc_target_temp - ptc_temperature;
                    pid_ptc.integral += inner_error * dt;
                    if(pid_ptc.integral > 50.0f) pid_ptc.integral = 50.0f;
                    if(pid_ptc.integral < -50.0f) pid_ptc.integral = -50.0f;
                    float inner_derivative = (inner_error - pid_ptc.prev_error) / dt;
                    output = pid_ptc.kp * inner_error + pid_ptc.ki * pid_ptc.integral + pid_ptc.kd * inner_derivative;
                    output += get_feedforward_pwm(ptc_target_temp);
                    if (output > pid_ptc.out_max) output = pid_ptc.out_max;
                    if (output < pid_ptc.out_min) output = pid_ptc.out_min;
                    pid_ptc.prev_error = inner_error;
                }
                break;
            }
            case CONTROL_STATE_COOLING:
                // 风扇 PI 控制
                error = current_temperature - target_temperature;
                pid_cool.integral += error * dt;
                if(pid_cool.integral > 50.0f) pid_cool.integral = 50.0f;
                if(pid_cool.integral < -50.0f) pid_cool.integral = -50.0f;
                output = pid_cool.kp * error + pid_cool.ki * pid_cool.integral;
                fan_cmd = fan_cmd + 0.37 * (output - fan_cmd);// 一阶滤波平滑输出
                output = fan_cmd;
                if (output > pid_cool.out_max) output = pid_cool.out_max;
                if (output < pid_cool.out_min) output = pid_cool.out_min;
                pid_cool.prev_error = error;
                break;
            default:
                output = 0.0f;
                pid_box.integral *= 0.98f;
                pid_ptc.integral *= 0.98f;
                pid_cool.integral *= 0.98f;
                break;
        }

        
        final_pwm_duty = output;

        rt_uint32_t pulse = (rt_uint32_t)(final_pwm_duty * PTC_PERIOD);
        rt_pwm_set(pwm_dev, 0, PTC_PERIOD, pulse);
        rt_thread_mdelay(CONTROL_PERIOD_MS);
    }
}

rt_err_t initialization()
{
    rt_err_t result = RT_EOK;

    /* 初始化控制状态 */
    ptc_state = HEAT;
    control_state = CONTROL_STATE_WARMING;
    // 加热PID 
    //TODO!:(需要整定，建模中，或者可以使用一些机器学习方法，把目标函数黑盒转换为凸函数，然后做凸优化)
    pid_ptc.kp = 1.37f;
    pid_ptc.ki = 0.10f;
    pid_ptc.kd = 0.70f;
    pid_ptc.out_min = 0.0f;
    pid_ptc.out_max = 1.0f;
    pid_box.kp = 0.10f;
    pid_box.ki = 0.76f;
    pid_box.kd = 0.37f;
    pid_box.out_min = 0.0f;
    pid_box.out_max = 1.0f;
    // 风冷PI (需要整定)
    pid_cool.kp = 0.01f;
    pid_cool.ki = 0.001f;
    pid_cool.kd = 0.0f;
    pid_cool.out_min = fan_min;
    pid_cool.out_max = fan_max;
    rt_pin_mode(STATE_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(STATE_PIN, ptc_state);

    /* 初始化温度传感器 */
    result |= p3t1755_init(); // 板载
    dht_temp_dev = rt_device_find("temp_dht");
    dht_humi_dev = rt_device_find("humi_dht");
    if (dht_temp_dev && dht_humi_dev) {
        result |= rt_device_open(dht_temp_dev, RT_DEVICE_FLAG_RDWR);
        result |= rt_device_open(dht_humi_dev, RT_DEVICE_FLAG_RDWR);
    } else {
        rt_kprintf("DHT device not found.\n");
        result = -RT_ERROR;
    } 

    /* 初始化 ADC */
    adc_dev = (rt_adc_device_t)rt_device_find(PTC_TEMP_ADC);
    if (adc_dev == RT_NULL) {
        rt_kprintf("ADC device not found!\n");
        result = -RT_ERROR;
    } else {
        result |= rt_adc_enable(adc_dev, PTC_ADC_CHANNEL);
    }

    /* 初始化 PWM */
    pwm_dev = (rt_pwm_t)rt_device_find(APP_PTC_PWM_DEV_NAME);
    if (pwm_dev == RT_NULL) {
        rt_kprintf("PWM device not found!\n");
        result = -RT_ERROR;
    } else {
        result |= rt_pwm_set(pwm_dev, 0, PTC_PERIOD, 0);
        result |= rt_pwm_enable(pwm_dev, 0);
    }

    /* 连接 WiFi */
    result |= rt_wlan_connect(APP_WLAN_SSID, APP_WLAN_PASSWORD);
    rt_thread_mdelay(5000); // 等待连接稳定
    return result;
}

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

static float get_feedforward_pwm(float target_temp)
{
    float dt = target_temp;
    if (dt <= ff_table[0].target_temp) return ff_table[0].base_pwm;
    if (dt >= ff_table[num_ff_profiles - 1].target_temp)
        return ff_table[num_ff_profiles - 1].base_pwm;

    for (int i = 0; i < num_ff_profiles - 1; i++)
    {
        ff_profile_t *a = &ff_table[i];
        ff_profile_t *b = &ff_table[i + 1];
        if (dt >= a->target_temp && dt <= b->target_temp)
        {
            float ratio = (dt - a->target_temp) / (b->target_temp - a->target_temp);
            return a->base_pwm + ratio * (b->base_pwm - a->base_pwm);
        }
    }
    return 0.0f; // 理论上不会走到这里
}

static float get_warming_temp(float target_temp)
{
    float ptc_temp = warming_ff_table[0].ptc_temp;
    if (target_temp <= warming_ff_table[0].target_temp)
        ptc_temp = warming_ff_table[0].ptc_temp;
    else if (target_temp >= warming_ff_table[num_warming_ff_entries - 1].target_temp)
        ptc_temp = warming_ff_table[num_warming_ff_entries - 1].ptc_temp;
    else
    {
        for (int i = 0; i < num_warming_ff_entries - 1; i++)
        {
            warming_ff_entry_t *a = &warming_ff_table[i];
            warming_ff_entry_t *b = &warming_ff_table[i + 1];
            if (target_temp >= a->target_temp && target_temp <= b->target_temp)
            {
                float ratio = (target_temp - a->target_temp) / (b->target_temp - a->target_temp);
                ptc_temp = a->ptc_temp + ratio * (b->ptc_temp - a->ptc_temp);
                break;
            }
        }
    }
    return ptc_temp;
}

static float ntc_adc_to_temp(uint32_t adc_val)
{
    if (adc_val >= 65535) return -100.0f;
    float voltage = (float)adc_val * ADC_REF_VOLTAGE / ADC_RESOLUTION;
    float r_ntc = NTC_SERIES_R * voltage / (ADC_REF_VOLTAGE - voltage);
    float ln_r = log(r_ntc / NTC_R25);
    float t_kelvin = 1.0f / ((1.0f / 298.15f) + (ln_r / NTC_B_VALUE));
    
    return t_kelvin - 273.15f;
}

static const char* control_state_to_string(control_state_t state)
{
    switch(state)
    {
        case CONTROL_STATE_HEATING:
            return "HEATING";
        case CONTROL_STATE_WARMING:
            return "WARMING";
        case CONTROL_STATE_COOLING:
            return "COOLING";
        default:
            return "ERROR!!!";
    }
}
/*******************************************************************************
 * MSH 命令（调试用的）
 ******************************************************************************/
static void get_status(int argc, char **argv)
{
    rt_kprintf("----- System Status -----\n");
    rt_kprintf("State:                %s\n", control_state_to_string(control_state));
    rt_kprintf("Box Temp:             %.2f C\n", current_temperature);
    rt_kprintf("Target Temp:          %.2f C\n", target_temperature);
    rt_kprintf("PTC Temp:             %.2f C\n", ptc_temperature);
    rt_kprintf("Humidity:             %.1f %%\n", current_humidity);
    rt_kprintf("PWM Duty Cycle:       %.1f %%\n", final_pwm_duty * 100.0f);
    rt_kprintf("Hysteresis Band:      +/- %.2f C\n", hysteresis_band);

    rt_kprintf("\n----- PID Controllers -----\n");
    
    // 根据当前状态，高亮活动的控制器
    const char* heat_active_str = (control_state != CONTROL_STATE_COOLING) ? " (ACTIVE)" : "";
    rt_kprintf("Outer PID (Box)%s\n", heat_active_str);
    rt_kprintf("  Gains:    Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid_box.kp, pid_box.ki, pid_box.kd);
    rt_kprintf("  Internal: I-Term=%.3f, Prev-Err=%.3f\n", pid_box.integral, pid_box.prev_error);

    rt_kprintf("Inner PID (PTC)%s\n", heat_active_str);
    rt_kprintf("  Gains:    Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid_ptc.kp, pid_ptc.ki, pid_ptc.kd);
    rt_kprintf("  Internal: I-Term=%.3f, Prev-Err=%.3f\n", pid_ptc.integral, pid_ptc.prev_error);

    const char* cool_active_str = (control_state == CONTROL_STATE_COOLING) ? " (ACTIVE)" : "";
    rt_kprintf("Cooling PI%s\n", cool_active_str);
    rt_kprintf("  Gains:    Kp=%.3f, Ki=%.3f\n", pid_cool.kp, pid_cool.ki);
    rt_kprintf("  Internal: I-Term=%.3f, Prev-Err=%.3f\n", pid_cool.integral, pid_cool.prev_error);
}
MSH_CMD_EXPORT(get_status, Get current system status for temperature control);

void tune(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("\n----- Usage -----\n");
        rt_kprintf("  tune target <val>          (Set target temperature in C)\n");
        rt_kprintf("  tune hys <val>             (Set hysteresis band in C)\n");
        rt_kprintf("  tune warmbias <val>        (Set warming bias temperature in C)\n");
        rt_kprintf("  tune heatbias <val>        (Set heating bias temperature in C)\n");
        rt_kprintf("  tune ff <0-ptc/1-warmt> <temp> <val> (Set feedforward value)\n");
        rt_kprintf("  tune box <kp|ki|kd> <val>  (Tune outer box PID)\n");
        rt_kprintf("  tune heat <kp|ki|kd> <val> (Tune inner PTC PID)\n");
        rt_kprintf("  tune cool <kp|ki> <val>    (Tune cooling PI)\n");
        rt_kprintf("\n----- Example -----\n");
        rt_kprintf("  tune heat kp 0.3\n");
        rt_kprintf("  tune target 45.5\n");
        rt_kprintf("\n");
        get_status(0, RT_NULL); // 如果没有参数，则显示当前状态
        return;
    }

    const char *cmd = argv[1];

    if (strcmp(cmd, "target") == 0) {
        if (argc != 3) { rt_kprintf("Usage: tune target <value>\n"); return; }
        target_temperature = atof(argv[2]);
        rt_kprintf("Target temperature set to %.2f C\n", target_temperature);
    }
    else if (strcmp(cmd, "hys") == 0) {
        if (argc != 3) { rt_kprintf("Usage: tune hys <value>\n"); return; }
        hysteresis_band = atof(argv[2]);
        rt_kprintf("Hysteresis band set to +/- %.2f C\n", hysteresis_band);
    }
    else if (strcmp(cmd, "warmbias") == 0) {
        if (argc != 3) { rt_kprintf("Usage: tune warmbias <value>\n"); return; }
        warming_bias = atof(argv[2]);
        rt_kprintf("Warming bias temperature set to %.2f C\n", warming_bias);
    }
    else if (strcmp(cmd, "heatbias") == 0) {
        if (argc != 3) { rt_kprintf("Usage: tune heatbias <value>\n"); return; }
        heating_bias = atof(argv[2]);
        rt_kprintf("Heating bias temperature set to %.2f C\n", heating_bias);
    }
    else if (strcmp(cmd, "ff") == 0) {
        if (argc != 5) { rt_kprintf("Usage: tune ff <target(0-ptc/1-warmt)> <temp> <value>\n"); return; }
        int table_type = atoi(argv[2]);
        float temp = atof(argv[3]);
        float value = atof(argv[4]);
        rt_bool_t found = RT_FALSE;
        if(table_type == 0) {
            for (size_t i = 0; i < num_ff_profiles; i++)
            {
                if (fabsf(ff_table[i].target_temp - temp) < 2.0f) {
                    found = RT_TRUE;
                    ff_table[i].base_pwm = value;
                    rt_kprintf("Feedforward PWM for %.2f C set to %.2f\n", temp, value);
                    return;
                }
            }
            if (found == RT_FALSE)
            {
                // TODO!：插入新条目（排序）
                rt_kprintf("Error: No existing ptc feedforward entry near %.2f C. Addition of new entries not implemented.\n", temp);
            }
        } else if (table_type == 1)
        {
            for (size_t i = 0; i < num_warming_ff_entries; i++)
            {
                if (fabsf(warming_ff_table[i].target_temp - temp) < 2.0f) {
                    found = RT_TRUE;
                    warming_ff_table[i].ptc_temp = value;
                    rt_kprintf("Warming feedforward PTC Temperature for %.2f C set to %.2f\n", temp, value);
                    return;
                }
            }
            if (found == RT_FALSE)
            {
                // TODO!：插入新条目（排序）
                rt_kprintf("Error: No existing warming feedforward entry near %.2f C. Addition of new entries not implemented.\n", temp);
            }
        } else {
            rt_kprintf("Error: Unknown feedforward table type '%d'. Use 0 for ptc, 1 for warmt.\n", table_type);
        }
        return;
    }
    else if (strcmp(cmd, "box") == 0) {
        if (argc != 4) { rt_kprintf("Usage: tune box <kp|ki|kd> <value>\n"); return; }
        const char *param = argv[2];
        float value = atof(argv[3]);
        if (strcmp(param, "kp") == 0) pid_box.kp = value;
        else if (strcmp(param, "ki") == 0) pid_box.ki = value;
        else if (strcmp(param, "kd") == 0) pid_box.kd = value;
        else { rt_kprintf("Error: Unknown box param '%s'. Use kp, ki, or kd.\n", param); return; }
        rt_kprintf("Box PID '%s' set to %f\n", param, value);
    }
    else if (strcmp(cmd, "heat") == 0) {
        if (argc != 4) { rt_kprintf("Usage: tune heat <kp|ki|kd> <value>\n"); return; }
        const char *param = argv[2];
        float value = atof(argv[3]);
        if (strcmp(param, "kp") == 0) pid_ptc.kp = value;
        else if (strcmp(param, "ki") == 0) pid_ptc.ki = value;
        else if (strcmp(param, "kd") == 0) pid_ptc.kd = value;
        else { rt_kprintf("Error: Unknown heat param '%s'. Use kp, ki, or kd.\n", param); return; }
        rt_kprintf("Heat PID '%s' set to %f\n", param, value);
    }
    else if (strcmp(cmd, "cool") == 0) {
        if (argc != 4) { rt_kprintf("Usage: tune cool <kp|ki> <value>\n"); return; }
        const char *param = argv[2];
        float value = atof(argv[3]);
        if (strcmp(param, "kp") == 0) pid_cool.kp = value;
        else if (strcmp(param, "ki") == 0) pid_cool.ki = value;
        else { rt_kprintf("Error: Unknown cool param '%s'. Use kp or ki.\n", param); return; }
        rt_kprintf("Cool PI '%s' set to %f\n", param, value);
    }
    else {
        rt_kprintf("Error: Unknown command '%s'\n", cmd);
        return;
    }

    rt_kprintf("\nParameters updated. Current status:\n");
    get_status(0, RT_NULL); // 每次成功修改后，自动显示最新状态
}
MSH_CMD_EXPORT(tune, Tune system parameters (target, hys, PID gains));
/*******************************************************************************
 * MSH 命令（调参用的）
 ******************************************************************************/
/**
 * @brief  PID 性能评估命令，为Python自动调参脚本提供接口
 * @param  argc 参数个数
 * @param  argv 参数列表
 * @usage  eval_ptc <target_temp> <duration_ms>
 * @note   该命令会临时强制系统进入WARMING状态以独立评估PTC温度控制。
 *         评估指标为积分绝对误差 (Integrated Absolute Error, IAE)，值越小表示性能越好。
 */
void eval_ptc(int argc, char **argv)
{
    if (argc != 3) {
        rt_kprintf("Usage: eval_ptc <target_temp> <duration_ms>\n");
        return;
    }

    float eval_target_temp = atof(argv[1]);
    rt_uint32_t eval_duration_ms = atoi(argv[2]);
    rt_uint32_t sample_interval_ms = CONTROL_PERIOD_MS; // 使用PID线程的周期进行采样

    if (eval_duration_ms < 500 || eval_duration_ms > 300000) {
        rt_kprintf("Error: Duration must be between 500 and 300000 ms.\n");
        return;
    }
    
    rt_kprintf("Starting PTC evaluation: Target=%.2f C, Duration=%d ms\n", eval_target_temp, eval_duration_ms);

    target_temperature = eval_target_temp;
    control_state = CONTROL_STATE_WARMING;
    ptc_state = HEAT;
    rt_pin_write(STATE_PIN, HEAT);
    pid_ptc.integral = 0.0f; // 重置PID积分，确保一个干净的开始
    pid_ptc.prev_error = 0.0f;
    pid_box.integral = 0.0f;
    pid_box.prev_error = 0.0f;

    float total_absolute_error = 0.0f;
    rt_tick_t start_tick = rt_tick_get();
    rt_uint32_t sample_count = 0;

    // 2. 在指定时间内运行评估循环
    while (rt_tick_get() - start_tick < rt_tick_from_millisecond(eval_duration_ms))
    {
        // 我们不直接控制PID，PID线程仍在后台根据当前状态(WARMING)和参数运行
        // 我们只需要在这里同步地读取PTC温度并计算误差
        
        float current_ptc_temp = ptc_temperature; // 直接使用全局变量，它由PID线程高频更新
        float error = current_ptc_temp - eval_target_temp;
        
        total_absolute_error += fabsf(error);
        sample_count++;
        
        rt_thread_mdelay(sample_interval_ms);
    }

    // 3. 计算并输出最终得分
    //    我们对总误差进行平均，使其与评估时长无关
    if (sample_count == 0) {
        rt_kprintf("EVAL_RESULT:999999.0\n"); // 避免除零错误
        return;
    }
    float score = total_absolute_error / sample_count;

    // Python脚本会解析这一行
    rt_kprintf("EVAL_RESULT:%.4f\n", score); 
}
MSH_CMD_EXPORT(eval_ptc, Evaluate PTC PID performance for autotuning);
/**
 * @brief  强制设置系统控制状态 (主要用于外部脚本调试)
 * @param  argc 参数个数
 * @param  argv 参数列表
 * @usage  force_state <warming|heating|cooling>
 * @note   此命令会直接修改 control_state 并执行相应的硬件切换和PID重置。
 */
void force_state(int argc, char **argv)
{
    if (argc != 2) {
        rt_kprintf("Usage: force_state <warming|heating|cooling>\n");
        return;
    }

    control_state_t previous_state = control_state;
    control_state_t new_state = previous_state;
    const char* state_str = argv[1];

    if (strcmp(state_str, "warming") == 0) {
        new_state = CONTROL_STATE_WARMING;
    } else if (strcmp(state_str, "heating") == 0) {
        new_state = CONTROL_STATE_HEATING;
    } else if (strcmp(state_str, "cooling") == 0) {
        new_state = CONTROL_STATE_COOLING;
    } else {
        rt_kprintf("Error: Unknown state '%s'. Use warming, heating, or cooling.\n", state_str);
        return;
    }

    if (new_state != previous_state) {
        control_state = new_state; // 直接修改全局状态变量
        rt_kprintf("State forced from %s to %s\n", control_state_to_string(previous_state), control_state_to_string(new_state));

        // --- 关键：执行与状态切换相关的必要操作 ---
        rt_pwm_set(pwm_dev, 0, PTC_PERIOD, 0); // 切换前先停止输出，更安全
        rt_thread_mdelay(20);

        // 重置三个PID控制器，防止旧状态的累积误差影响新状态
        pid_box.integral = 0.0f;
        pid_box.prev_error = 0.0f;
        pid_ptc.integral = 0.0f;
        pid_ptc.prev_error = 0.0f;
        pid_cool.integral = 0.0f;
        pid_cool.prev_error = 0.0f;

        // 根据新状态设置硬件
        switch(control_state)
        {
            case CONTROL_STATE_HEATING:
            case CONTROL_STATE_WARMING:
                ptc_state = HEAT;
                rt_pin_write(STATE_PIN, HEAT);
                break;
            case CONTROL_STATE_COOLING:
                ptc_state = COOL;
                rt_pin_write(STATE_PIN, COOL);
                break;
        }
    } else {
        rt_kprintf("State is already %s. No change made.\n", state_str);
    }
}
MSH_CMD_EXPORT(force_state, Force system into a specific control state);
