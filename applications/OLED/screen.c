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

    /* full buffer example, setup procedure ends in _f */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    while (1)
    {
        sprintf(buf, "Testing");
        u8g2_DrawStr(&u8g2, 10, 18, buf);
        sprintf(buf, "Current Temp: %.2f C", current_temperature);
        u8g2_DrawStr(&u8g2, 10, 36, buf);
        u8g2_SendBuffer(&u8g2);
        rt_thread_mdelay(500);
    }
}
