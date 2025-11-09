#ifndef __UTIL_H__
#define __UTIL_H__

#include <rtthread.h>
#include "drv_pin.h"

#define LED_PIN        ((3*32)+12)
rt_thread_t working_indicate = RT_NULL;


/* PID 控制器参数 */
extern float final_target_height = 250.0f; // 用户设定的最终目标高度 (mm)
extern ramped_target_height = 250.0f; // PID控制器当前正在追踪的、平滑变化的目标高度

extern float KP = 0.0f;
extern float KI = 0.0f;
extern float KD = 0.0f;

extern int num_pid_profiles;
extern const pid_profile_t gain_schedule_table[];
extern const int num_ff_profiles;
extern const feedforward_profile_t ff_table[];

/* PID 控制器状态变量 */
extern float integral_error = 0.0f;
extern float previous_error = 0.0f;

/* PID 控制参数评估 */
rt_bool_t is_evaluating = RT_FALSE;
float total_abs_error = 0.0f;

void working_led();
void pid_eval(int argc, char **argv);
void pid_tune(int argc, char **argv);

#endif /* __UTIL_H__ */