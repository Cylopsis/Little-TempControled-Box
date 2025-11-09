#include "YS4028B12H.h"

ys4028b12h_cfg my_ys4028b12h_config = {
    .period = PKG_USING_YS4028B12H_PERIOD, // 周期
    .pulse = PKG_USING_YS4028B12H_DEFAULT_PAULSE, // 脉冲宽度
    .name = RT_NULL,
    .channel = PKG_USING_YS4028B12H_PWM_CHANNEL,
};

/* 初始化 */
rt_err_t ys4028b12h_init(ys4028b12h_cfg_t cfg)
{
    /* 查找设备 */
    cfg->name = (struct rt_device_pwm *)rt_device_find(PKG_USING_YS4028B12H_PWM_DEV_NAME);
    if (cfg->name == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", cfg->name);
        return RT_ERROR;
    }
    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(cfg->name, cfg->channel, cfg->period, cfg->pulse);
    /* 使能设备 */
    rt_pwm_enable(cfg->name, cfg->channel);
    
    return RT_EOK;
}

rt_err_t ys4028b12h_deinit(ys4028b12h_cfg_t cfg)
{
    if (cfg->name != RT_NULL)
    {
        rt_pwm_disable(cfg->name, cfg->channel);
        
        return RT_EOK;
    }
    
    return RT_ERROR;
}

rt_err_t ys4028b12h_set_speed(ys4028b12h_cfg_t cfg, float speed)
{
    if (cfg->name == RT_NULL)
    {
        rt_kprintf("not find the YS4028B12H device\n");

        return RT_ERROR;
    }
    else if (speed < 0 || speed > 1.0f)
    {
        rt_kprintf("speed must be in range [0, 1.0]\n");

        return RT_ERROR;
    }
    else{

        cfg->pulse = (int)(cfg->period * speed); // 计算脉冲宽度
        rt_pwm_set(cfg->name, cfg->channel,cfg->period, cfg->pulse);

        return RT_EOK;
    }
}

rt_err_t ys4028b12h_control(ys4028b12h_cfg_t cfg, int speed, int dir)
{
    rt_kprintf("TODO: Not know if it can change direction\n");

    return RT_ERROR;
}

rt_err_t ys4028b12h_get(ys4028b12h_cfg_t cfg)
{
    if (cfg->name == RT_NULL)
    {
        rt_kprintf("not find the ys4028b12h device\n");

        return RT_ERROR;
    }

    rt_kprintf("ys4028b12h get device is %s\n",PKG_USING_YS4028B12H_PWM_DEV_NAME);
    rt_kprintf("ys4028b12h get channel is %d\n",cfg->channel);
    rt_kprintf("ys4028b12h get pulse is %d\n",cfg->pulse);
    rt_kprintf("ys4028b12h get period is %d\n",cfg->period);
    
    return RT_EOK;
}

float ys4028b12h_get_speed(ys4028b12h_cfg_t cfg)
{
    if (cfg->name == RT_NULL)
    {
        rt_kprintf("not find the ys4028b12h device\n");
        return RT_ERROR;
    }
    else{
        
        return cfg->pulse / (float)cfg->period;
    }
}
