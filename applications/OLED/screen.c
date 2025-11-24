#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <u8g2_port.h>
#include <system_vars.h>

#define OLED_I2C_PIN_SCL                    22  // P0_22
#define OLED_I2C_PIN_SDA                    23  // P0_23

void screen_on()
{
    u8g2_t u8g2;
    char buf[32];

    // Initialization
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_rtthread);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_CLOCK, OLED_I2C_PIN_SCL);
    u8x8_SetPin(u8g2_GetU8x8(&u8g2), U8X8_PIN_I2C_DATA, OLED_I2C_PIN_SDA);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);

    while (1)
    {
        const char *state_label = "STANDBY";
        float metric_values[4];
        const char *metric_labels[4] = {"CUR", "TGT", "ENV", "PTC"};
        const float slider_half_span = 30.0f; // 越小温差条波动越明显
        float delta = 0.0f;
        float slider_ratio = 0.0f;

        u8g2_ClearBuffer(&u8g2);

        switch (control_state)
        {
            case CONTROL_STATE_HEATING:
                state_label = "HEATING";
                break;
            case CONTROL_STATE_COOLING:
                state_label = "COOLING";
                break;
            case CONTROL_STATE_WARMING:
                state_label = "WARMING";
                break;
            default:
                break;
        }

        metric_values[0] = current_temperature;
        metric_values[1] = target_temperature;
        metric_values[2] = env_temperature;
        metric_values[3] = ptc_temperature;

        delta = current_temperature - target_temperature;
        if (slider_half_span > 0.0f)
        {
            slider_ratio = delta / slider_half_span;
            if (slider_ratio < -1.0f)
            {
                slider_ratio = -1.0f;
            }
            else if (slider_ratio > 1.0f)
            {
                slider_ratio = 1.0f;
            }
        }
        // 状态
        u8g2_SetFont(&u8g2, u8g2_font_7x13B_tf);
        u8g2_uint_t state_width = u8g2_GetStrWidth(&u8g2, state_label);
        u8g2_DrawStr(&u8g2, (128 - state_width) / 2, 14, state_label);
        u8g2_DrawHLine(&u8g2, 0, 18, 128);

        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        for (rt_uint8_t i = 0; i < 4; i++)
        {
            rt_uint8_t col = i % 2;
            rt_uint8_t row = i / 2;
            rt_uint8_t x = col ? 66 : 2;
            rt_uint8_t y = 28 + (row * 10);

            rt_sprintf(buf, "%s:%5.1fC", metric_labels[i], metric_values[i]);
            u8g2_DrawStr(&u8g2, x, y, buf);
        }
        u8g2_DrawVLine(&u8g2, 64, 20, 24);
        u8g2_DrawHLine(&u8g2, 0, 40, 128);

        // 温差
        u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
        rt_sprintf(buf, "DELTA %+.1fC", delta);
        u8g2_DrawStr(&u8g2, 2, 50, buf);

        // 温度条
        const rt_uint8_t slider_x = 2;
        const rt_uint8_t slider_y = 52;
        const rt_uint8_t slider_width = 124;
        const rt_uint8_t slider_height = 10;
        const rt_uint8_t slider_center = slider_x + slider_width / 2;
        float offset = slider_ratio * ((slider_width / 2.0f) - 3.0f);
        int indicator_x = (int)(slider_center + offset);

        u8g2_DrawFrame(&u8g2, slider_x, slider_y, slider_width, slider_height);
        u8g2_DrawVLine(&u8g2, slider_center, slider_y - 2, slider_height + 4);
        u8g2_DrawBox(&u8g2, indicator_x - 1, slider_y + 1, 3, slider_height - 2);

        u8g2_SendBuffer(&u8g2);
        rt_thread_mdelay(1000);
    }
}
