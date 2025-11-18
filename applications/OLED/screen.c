#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <u8g2_port.h>
#include <system_vars.h>

void screen_on()
{
    u8g2_t u8g2;
    char buf[32];

    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f( &u8g2, U8G2_R0, u8x8_byte_rtthread_hw_i2c, u8x8_gpio_and_delay_rtthread);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    while (1)
    {
        u8g2_ClearBuffer(&u8g2);
        switch (control_state)
        {
            case CONTROL_STATE_HEATING:
                rt_sprintf(buf, "HEATING");
                break;
            case CONTROL_STATE_COOLING:
                rt_sprintf(buf, "COOLING");
                break;
            case CONTROL_STATE_IDLE:
                rt_sprintf(buf, "IDLE");
                break;
            case CONTROL_STATE_WARMING:
                rt_sprintf(buf, "WARMING");
                break;
            default:
                break;
        }
        u8g2_DrawStr(&u8g2, 10, 18, buf);
        rt_sprintf(buf, "Current Temp: %.2f C", current_temperature);
        u8g2_DrawStr(&u8g2, 10, 36, buf);
        rt_sprintf(buf, "Target Temp:  %.2f C", target_temperature);
        u8g2_DrawStr(&u8g2, 10, 54, buf);
        rt_sprintf(buf, "Env Temp:     %.2f C", env_temperature);
        u8g2_DrawStr(&u8g2, 10, 72, buf);
        u8g2_SendBuffer(&u8g2);
        rt_thread_mdelay(1000);
    }
}
