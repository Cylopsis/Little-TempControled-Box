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
#define SEND_BUFSZ      630    // 发送缓冲区大小
#define MAX_ARGS        16      // 命令行参数最大数量

static rt_thread_t server_thread = RT_NULL;

static const char *control_state_to_string(control_state_t state)
{
    switch (state)
    {
    case CONTROL_STATE_HEATING: return "HEATING";
    case CONTROL_STATE_WARMING: return "WARMING";
    case CONTROL_STATE_COOLING: return "COOLING";
    default:                    return "ERROR!!!";
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
            if (p) *p = '\0'; 
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
                const float fan_percent = final_pwm_duty * 100.0f;
                const char *ptc_state_str = (ptc_state == HEAT) ? "ON" : "OFF";
                const char *ctrl_state_str = control_state_to_string(control_state);

                int len = snprintf(send_buf, sizeof(send_buf), "{"\
                    "\"current_ptc_temperature\":%.2f,"\
                    "\"current_temperature\":%.2f,"\
                    "\"target_temperature\":%.2f,"\
                    "\"current_humidity\":%.2f,"\
                    "\"env_temperature\":%.2f,"\
                    "\"ptc_state\":\"%s\","\
                    "\"control_state\":\"%s\","\
                    "\"current_pwm\":%.2f,"\
                    "\"heat_kp\":%.2f,"\
                    "\"heat_ki\":%.2f,"\
                    "\"heat_kd\":%.2f,"\
                    "\"cool_kp\":%.2f,"\
                    "\"cool_ki\":%.2f,"\
                    "\"warming_threshold\":%.2f,"\
                    "\"hysteresis_band\":%.2f"\
                    "}\r\n",
                    ptc_temperature,
                    current_temperature,
                    target_temperature,
                    current_humidity,
                    env_temperature,
                    ptc_state_str,
                    ctrl_state_str,
                    final_pwm_duty,
                    pid_heat.kp,
                    pid_heat.ki,
                    pid_heat.kd,
                    pid_cool.kp,
                    pid_cool.ki,
                    warming_threshold,
                    hysteresis_band
                );

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
            else if (strcmp(argv[0], "tune") == 0)
            {         
                tune(argc, argv);

                const char ok_msg[] = "OK\r\n";
                send(connected, ok_msg, sizeof(ok_msg) - 1, 0);
            }
            else
            {
                sprintf(send_buf, "ERROR: Unknown command '%s'.\r\n", argv[0]);
                send(connected, send_buf, strlen(send_buf), 0);
            }

            rt_thread_mdelay(30);
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
                                     11,
                                     30);

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

