#include <rtthread.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <sys/errno.h>
#include <stdio.h>
#include "system_vars.h"
#include "drv_pin.h"

#define SERVER_PORT     5000    // 服务器监听的端口
#define RECV_BUFSZ      256     // 接收缓冲区大小
#define SEND_BUFSZ      1024    // 发送缓冲区大小
#define MAX_ARGS        16      // 命令行参数最大数量

static rt_thread_t server_thread = RT_NULL;

static const char *control_state_to_string(control_state_t state)
{
    switch (state)
    {
    case CONTROL_STATE_HEATING: return "HEATING";
    case CONTROL_STATE_WARMING: return "WARMING";
    case CONTROL_STATE_COOLING: return "COOLING";
    case CONTROL_STATE_IDLE:
    default:
        return "IDLE";
    }
}

/**
 * @brief TCP服务器线程入口函数
 * @param parameter 线程参数 (未使用)
 */
static void remote_server_thread_entry(void *parameter)
{
    int sock, connected;
    struct sockaddr_in server_addr, client_addr;
    socklen_t sin_size;
    
    char recv_buf[RECV_BUFSZ];
    char send_buf[SEND_BUFSZ];
    char *argv[MAX_ARGS]; // 用于存放分割后的命令参数指针
    int argc;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        rt_kprintf("[Remote] Socket error\n");
        goto __exit;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    rt_memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
        rt_kprintf("[Remote] Unable to bind\n");
        goto __exit;
    }

    if (listen(sock, 5) == -1)
    {
        rt_kprintf("[Remote] Listen error\n");
        goto __exit;
    }

    rt_kprintf("[Remote] TCP Server waiting for client on port %d...\n", SERVER_PORT);

    while (1)
    {
        sin_size = sizeof(struct sockaddr_in);
        
        // 接受客户端连接 (阻塞)
        connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
        if (connected < 0)
        {
            rt_kprintf("[Remote] Accept connection failed! errno = %d\n", errno);
            continue;
        }
        rt_kprintf("[Remote] Got a connection from (%s, %d)\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        // 与客户端交互循环
        while (1)
        {
            int bytes_received = recv(connected, recv_buf, RECV_BUFSZ - 1, 0);
            if (bytes_received <= 0)
            {
                rt_kprintf("[Remote] Client disconnected or recv error.\n");
                closesocket(connected);
                break;
            }

            recv_buf[bytes_received] = '\0';
            char* p = strpbrk(recv_buf, "\r\n");
            if (p) *p = '\0'; // 去掉换行符
            // for (int i = 0; i < bytes_received; i++) {
            //     if (recv_buf[i] == '\r' || recv_buf[i] == '\n') {
            //         recv_buf[i] = '\0';
            //     }
            // }
            // rt_kprintf("[Remote] Received command: '%s'\n", recv_buf);
            argc = 0;
            char *saveptr; // for strtok_r
            char *ptr = strtok_r(recv_buf, " ", &saveptr);
            while (ptr != NULL && argc < MAX_ARGS) {
                argv[argc++] = ptr;
                ptr = strtok_r(NULL, " ", &saveptr);
            }
 
            if (argc == 0) {
                continue; // 空命令
            }
 
            // --- 根据第一个参数分发命令 ---
            if (strcmp(argv[0], "get_status") == 0)
            {
                const float fan_percent = final_fan_speed * 100.0f;
                const char *ptc_state_str = (ptc_state == PIN_HIGH) ? "ON" : "OFF";
                const char *btm_ptc_str = (btm_ptc_state == PIN_HIGH) ? "ON" : "OFF";
                const char *ctrl_state_str = control_state_to_string(control_state);

                int len = snprintf(send_buf, sizeof(send_buf), "{"\
                    "\"current_temperature\":%.2f,"\
                    "\"target_temperature\":%.2f,"\
                    "\"current_humidity\":%.2f,"\
                    "\"env_temperature\":%.2f,"\
                    "\"ptc_state\":\"%s\","\
                    "\"btm_ptc_state\":\"%s\","\
                    "\"control_state\":\"%s\","\
                    "\"fan_speed\":%.4f,"\
                    "\"fan_speed_percent\":%.2f,"\
                    "\"feedforward_speed\":%.4f,"\
                    "\"pid_output\":%.4f,"\
                    "\"pid_kp\":%.6f,"\
                    "\"pid_ki\":%.6f,"\
                    "\"pid_kd\":%.6f,"\
                    "\"integral_error\":%.4f,"\
                    "\"previous_error\":%.4f,"\
                    "\"warming_threshold\":%.4f,"\
                    "\"hysteresis_band\":%.4f,"\
                    "\"fan_speed_circulation\":%.4f,"\
                    "\"fan_min\":%.4f,"\
                    "\"fan_max\":%.4f,"\
                    "\"fan_smooth_alpha\":%.4f"\
                    "}\r\n",
                    current_temperature,
                    target_temperature,
                    current_humidity,
                    env_temperature,
                    ptc_state_str,
                    btm_ptc_str,
                    ctrl_state_str,
                    final_fan_speed,
                    fan_percent,
                    feedforward_speed_last,
                    pid_output_last,
                    KP, KI, KD,
                    integral_error,
                    previous_error,
                    warming_threshold,
                    hysteresis_band,
                    fan_speed_circulation,
                    fan_min,
                    fan_max,
                    fan_smooth_alpha);

                if (len < 0 || len >= SEND_BUFSZ)
                {
                    rt_kprintf("[Remote] JSON buffer overflow detected\n");
                    continue;
                }

                // 发送响应
                if (send(connected, send_buf, (size_t)len, 0) < 0) {
                    rt_kprintf("[Remote] Send response failed.\n");
                    closesocket(connected);
                    break;
                }
            }
            else if (strcmp(argv[0], "pid_tune") == 0)
            {         
                pid_tune(argc, argv);

                const char ok_msg[] = "OK\r\n";
                send(connected, ok_msg, sizeof(ok_msg) - 1, 0);
            }
            else if (strcmp(argv[0], "fan_tune") == 0)
            {
                fan_tune(argc, argv);

                const char ok_msg[] = "OK\r\n";
                send(connected, ok_msg, sizeof(ok_msg) - 1, 0);
            }
            else
            {
                // 对于未知命令，发送错误信息
                sprintf(send_buf, "ERROR: Unknown command '%s'.\r\n", argv[0]);
                send(connected, send_buf, strlen(send_buf), 0);
            }
        }
    }

__exit:
    if (sock >= 0) closesocket(sock);
    rt_kprintf("[Remote] Server thread exited.\n");
}

/**
 * @brief MSH命令，用于启动远程控制服务器线程
 */

void remote_start(int argc, char **argv)
{
    if (server_thread != RT_NULL)
    {
        rt_kprintf("[Remote] Server is already running.\n");
        return;
    }

    server_thread = rt_thread_create("RemoteTCPSrv",
                                     remote_server_thread_entry,
                                     RT_NULL,
                                     3172,
                                     12,
                                     20);

    if (server_thread != RT_NULL)
    {
        rt_thread_startup(server_thread);
        rt_kprintf("[Remote] TCP server started successfully.\n");
    }
    else
    {
        rt_kprintf("[Remote] Failed to create TCP server thread.\n");
    }
}
MSH_CMD_EXPORT(remote_start, Start the remote control TCP server);

