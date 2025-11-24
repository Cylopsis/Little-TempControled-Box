#ifndef SYSTEM_VARS_H
#define SYSTEM_VARS_H

#include <rtthread.h>
#include "drv_pin.h"
/*******************************************************************************
 * 参数定义
 ******************************************************************************/
typedef struct rt_device_pwm* rt_pwm_t;
#define HEAT PIN_LOW
#define COOL PIN_HIGH
#define LED_PIN             ((3*32)+12)      // 工作指示灯引脚
#define STATE_PIN           ((0*32)+21)      // 状态继电器控制引脚 0-制热（PWM信号传递给MOSPTC） 1-降温（PWM信号传递给风扇）
#define DHT_DATA_PIN        ((2*32)+1)       // DHT11数据引脚
#define PTC_TEMP_ADC    	"adc0"
#define PTC_ADC_CHANNEL 	0
#define PTC_PERIOD      	(1000000000 / PKG_USING_PTC_FREQUENCY)
#define NTC_R25             10000.0f  		// 25度时的阻值 10k
#define NTC_B_VALUE         3950.0f   		// B值
#define NTC_SERIES_R        10000.0f  		// 分压串联电阻 10k
#define ADC_REF_VOLTAGE     3300      		// 参考电压（mv）
#define ADC_RESOLUTION      65535.0f  		// 16bit ADC
#define SAMPLE_PERIOD_MS    1000          	// 主线程采样周期 (ms)
#define CONTROL_PERIOD_MS 	100         	// PID控制周期 (ms)
#define PTC_MAX_SAFE_TEMP   120.0f           // PTC最大工作温度 (°C)

/* 传感器信息 */
extern volatile float env_temperature;        // 环境温度
extern volatile float current_humidity;       // 当前湿度值
extern volatile float current_temperature;    // 当前温度值
extern volatile float target_temperature;     // 目标温度值
extern volatile float ptc_temperature;        // PTC温度值
/* 温控参数 */
extern float warming_threshold;       // 当前保温阈值 (由前馈表决定)
extern float warming_bias;            // 保温偏置温度（PTC温度高于目标温度多少用于保温）
extern float heating_bias;           // 加热偏置温度（PTC温度高于目标温度多少用于加热）
extern float hysteresis_band;         // 迟滞范围
extern float fan_min;                // 最小风速
extern float fan_max;                // 最大风速

/* 控制状态与监控变量 */
/* PID 控制器 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} pid_ctx_t;
extern pid_ctx_t pid_heat; // 加热 PID
extern pid_ctx_t pid_cool; // 风扇 PI
typedef enum {
	CONTROL_STATE_HEATING=0,
	CONTROL_STATE_WARMING,
	CONTROL_STATE_COOLING
} control_state_t;
extern volatile control_state_t control_state;
extern volatile rt_uint32_t ptc_state;
extern volatile float final_pwm_duty;          // 当前PWM占空比

// 控制接口
extern void tune(int argc, char **argv);
extern void remote_start(int argc, char **argv);

// OLED显示
extern void screen_on();
#endif /* SYSTEM_VARS_H */
