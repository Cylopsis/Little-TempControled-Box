#include<util.h>

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

void pid_tune(int argc, char **argv)
{
    // 显示帮助信息
    if (argc < 2) {
        rt_kprintf("--- PID & Feedforward Status ---\n");
        rt_kprintf("  Final Target: %.2f mm, Ramped Target: %.2f mm\n", final_target_height, ramped_target_height);
        rt_kprintf("  Kp: %f, Ki: %f, Kd: %f\n", KP, KI, KD);
        rt_kprintf("\n--- Usage ---\n");
        rt_kprintf("  pid_tune -t <val>                    (Set target height)\n");
        rt_kprintf("  pid_tune -p <val> -i <val> -d <val>  (Manual PID override)\n");
        rt_kprintf("  pid_tune -ff                         (Show Feedforward table)\n");
        rt_kprintf("  pid_tune -ff_set <idx> <h> <spd>     (Set Feedforward entry)\n");
        rt_kprintf("    e.g., pid_tune -ff_set 2 100.0 0.45\n");
        return;
    }

    // 处理特殊命令
    if (strcmp(argv[1], "-ff") == 0) {
        rt_kprintf("--- Feedforward Table ---\n");
        rt_kprintf("Idx | Height (mm) | Base Speed\n");
        rt_kprintf("----|-------------|-----------\n");
        for (int i = 0; i < num_ff_profiles; i++) {
            rt_kprintf("%-3d | %-11.1f | %.4f\n", i, ff_table[i].height, ff_table[i].base_fan_speed);
        }
        return;
    }

    if (strcmp(argv[1], "-ff_set") == 0) {
        if (argc != 5) {
            rt_kprintf("Error: Incorrect arguments for -ff_set.\n");
            rt_kprintf("Usage: pid_tune -ff_set <index> <height> <speed>\n");
            return;
        }
        int index = atoi(argv[2]);
        if (index < 0 || index >= num_ff_profiles) {
            rt_kprintf("Error: Index %d is out of bounds (0-%d).\n", index, num_ff_profiles - 1);
            return;
        }
        ff_table[index].height = atof(argv[3]);
        ff_table[index].base_fan_speed = atof(argv[4]);
        rt_kprintf("Feedforward table entry %d updated to: Height=%.1f, Speed=%.4f\n",
                   index, ff_table[index].height, ff_table[index].base_fan_speed);
        return;
    }

    // 解析传统的PID和目标设置参数
    for (int i = 1; i < argc; i += 2) {
        if (strcmp(argv[i], "-p") == 0) { KP = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-i") == 0) { KI = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-d") == 0) { KD = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-t") == 0) { final_target_height = atof(argv[i+1]); }
        else { rt_kprintf("Error: Unknown option %s\n", argv[i]); return; }
    }
    rt_kprintf("Parameters updated. Current status:\n");
    pid_tune(1, RT_NULL); // 显示更新后的状态
}
MSH_CMD_EXPORT(pid_tune, Tune PID and Feedforward parameters);