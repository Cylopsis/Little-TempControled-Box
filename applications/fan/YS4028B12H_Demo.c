#include "YS4028B12H.h"


static int set_speed(int argc, char **argv)
{
    ys4028b12h_cfg_t cfg = &my_ys4028b12h_config;
    float speed = atof(argv[1]);
    rt_err_t result = ys4028b12h_set_speed(cfg, speed);
    
    if (result != RT_EOK) {
        rt_kprintf("Failed to set speed: %d\n", result);
        return -1;
    } else {
        rt_kprintf("YS4028B12H speed set to: %f\n", speed);
    }
    
    return 0;
}

MSH_CMD_EXPORT(set_speed,set speed);