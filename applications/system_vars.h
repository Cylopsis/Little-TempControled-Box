#ifndef SYSTEM_VARS_H
#define SYSTEM_VARS_H

#include <rtthread.h>

typedef enum {
    CONTROL_STATE_IDLE = 0,
    CONTROL_STATE_HEATING,
    CONTROL_STATE_COOLING
} control_state_t;

extern float env_temperature;        // 环境温度
extern float current_humidity;       // 当前湿度值
extern float current_temperature;    // 当前温度值
extern float target_temperature;     // 目标温度值
extern float KP;
extern float KI;
extern float KD;
extern float integral_error;
extern float previous_error;
extern rt_base_t ptc_state;          // 加热PTC继电器状态
extern rt_base_t btm_ptc_state;      // 底板PTC继电器状态
extern volatile control_state_t control_state; // 当前控制状态
extern float final_fan_speed;        // 最新风扇占空比 (0.0-1.0)
extern float feedforward_speed_last; // 最近一次前馈风速
extern float pid_output_last;        // 最近一次PID输出

// 控制接口
void update_pid_gains_by_target(float current_target_temp);
float get_feedforward_speed(float target_temp);
void pid_tune(int argc, char **argv);

// OLED显示
void screen_on();

#endif /* SYSTEM_VARS_H */
