## 1. 项目概述

本项目是一个简易的、较高精度的、易于扩展的桌面级温控系统。实现对箱内温度的精确控制，并通过Wi-Fi接入网络，配套了功能完善的Web可视化监控和调参Dashboard，实现了完整的物联网闭环。整个项目充分利用了RT-Thread多线程、设备驱动框架和网络组件的能力，实现了一个软硬件结合的完整解决方案。

- **硬件平台**：NXP FRDM-MCXA156
- **RT-Thread版本**：5.2.1
- **核心功能**：
    - 整机最大功耗24W
    - 温控范围常温~70°C（接入制冷片/更高温的PTC可以拓展范围）
    - 较高精度的恒温控制（最大波动范围3°C，稳态波动范围1°C）
    - 多传感器数据融合
    - OLED本地显示
    - Web远程监控与在线调参
- **项目亮点**：级联PID+前馈复合控制算法、三态控制状态机、TCP-WebSocket桥接的Web可视化方案。

## 2. RT-Thread使用情况概述

RT-Thread作为项目的核心操作系统，为功能的实现提供了坚实的基础。其稳定可靠的内核、丰富的组件和简洁的API，让整个开发过程事半功倍。

- **内核与调度器**：项目创建了多个线程来处理不同任务，包括主控线程、PID控制线程、OLED刷新线程、网络服务线程和LED指示线程。利用RT-Thread的抢占式调度器，确保了温度控制等高优先级任务的实时性。
- **设备驱动框架**：通过RT-Thread统一的设备模型，轻松地操作了多个硬件外设：
    - `Pin`设备：用于控制LED指示灯和加热/散热模式切换的继电器。
    - `ADC`设备：用于读取NTC热敏电阻的电压值，进而计算PTC加热片的实时温度。
    - `PWM`设备：用于精确控制PTC加热片和散热风扇的功率输出。
    - `I2C`设备：用于驱动OLED显示屏（基于u8g2软件包）和读取板载P3T1755环境温度传感器。
    - `Sensor`框架：用于读取DHT11、P3T1755传感器数据。
- **网络协议栈**：使用了内置的lwIP协议栈和SAL套接字抽象层，快速实现了稳定可靠的TCP服务器，为远程监控提供了数据通道。
- **WLAN无线框架**：通过`rt_wlan_connect`接口，便捷地实现了Wi-Fi网络连接功能。
- **FinSH/MSH命令行**：通过自定义的`get_status`和`tune`命令，所有关键参数都可以在运行时通过串口动态调整。
- **软件包生态**：
    - **u8g2**：本地UI图形库。
    - **dhtxx**：DHT11温湿度传感器软件包。
    - **p3t1755**: 板载I2C温度传感器软件包。

## 3. 硬件框架

系统硬件由核心控制、传感器、执行器和人机交互四部分组成

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20251127/5de04516a3b92a29e155a00d876c7628.png.webp)

- **核心控制器**：NXP FRDM-MCXA156开发板。
- **传感器模块**：
    - **箱内温湿度**：DHT11传感器，通过RT-Thread Sensor框架读取。
    - **PTC表面温度**：NTC热敏电阻，通过ADC采样并使用Steinhart-Hart模型计算，用于内环控制和过温保护。
    - **环境温度**：开发板板载的P3T1755 I2C传感器。
- **执行器模块**：
    - **加热**：LR7843 MOSFET驱动PTC陶瓷发热片，通过PWM信号调节功率。
    - **散热**：12V直流风扇，同样由PWM信号驱动。
    - **模式切换**：通过一个继电器切换PWM输出信号到MOSFET或风扇，实现加热/散热模式的自动切换。
- **人机交互模块**：
    - **本地**：SSD1306 OLED显示屏，实时显示系统状态、当前温度、目标温度等关键信息。
    - **远程**：通过Wi-Fi连接，在PC或手机浏览器上访问可视化Dashboard。

### 硬件连线图
![circuit.png](https://oss-club.rt-thread.org/uploads/20251127/17e4d7bc78cffd71293aad90213edec9.png.webp)

![hardware.jpg](https://oss-club.rt-thread.org/uploads/20251127/11839c2581b664ca0c233e70f5a2eedc.jpg.webp)

![hardware2.jpg](https://oss-club.rt-thread.org/uploads/20251127/3faf2989e715328d6abcc60f13b473af.jpg.webp)

## 4. 软件框架说明

软件的核心是一个基于`main.c`中的三态状态机和`pid_entry`线程中的级联PID控制算法。

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20251127/f453bb47356db5995312e82b356afa88.png.webp)

### 软件模块说明

1.  **主控与状态机 (`main.c`)**
    - `main`函数负责初始化所有设备（传感器、PWM、ADC、Wi-Fi），并创建各个应用线程。
    - `main`函数内的`while(1)`循环是系统的**主状态机**。它周期性地读取箱内温度，并与目标温度和迟滞范围(`hysteresis_band`)比较，自动在`HEATING`（加热）、`WARMING`（保温）、`COOLING`（散热）三种状态间切换。
    - 状态切换时，会通过`STATE_PIN`控制继电器，将PWM信号通路切换到对应的执行器（PTC或风扇），并重置PID积分项，防止状态突变。

2.  **核心控制算法 (`pid_entry`线程)**
	这是一个独立的线程，以更高的频率(`CONTROL_PERIOD_MS`)运行，负责核心的温度控制算法。
    - **级联PID+前馈（加热/保温模式）**:
        - **外环PID** (`pid_box`): 根据**箱内温度**与目标温度的差值，计算出一个期望的**PTC目标温度** (`ptc_target_temp`)。这使得PTC的加热速率能根据箱内离目标的远近动态调整。
        - **内环PID** (`pid_ptc`): 根据**PTC实际温度**与外环给出的`ptc_target_temp`的差值，计算出PWM的调节量。这可以快速响应PTC自身的温度波动，实现更稳定的热量输出。
        - **前馈控制**: 建立了一个`ptc_target_temp`到`base_pwm`的映射表 (`ff_table`)。PID的输出是叠加在这个PWM基础值之上的微调，这大大加快了系统收敛速度，减少了PID积分饱和的风险。
    - **PI控制（散热模式）**:
        - 切换到散热模式后，算法切换为简单的`pid_cool` PI控制器，根据箱内温度与目标温度的差值直接控制风扇的转速。
    - **过温保护**: 算法实时监测PTC温度，一旦超过设定的安全阈值(`PTC_MAX_SAFE_TEMP`)，立即将PWM输出置零，确保系统安全。

3.  **远程控制服务 (`remote.c`)**
    - 在`remote_server_thread_entry`线程中，创建了一个TCP服务器，监听`5000`端口。
    - 服务器接收两种文本命令：
        - `get_status`: 将系统中所有关键的实时变量（各处温度、湿度、PID参数、控制状态、PWM占空比等）打包成一个JSON字符串返回。
        - `tune ...`: 将收到的参数直接传递给`main.c`中的`tune()`函数，实现了对目标温度、PID增益、前馈表等所有关键参数的运行时修改。

4.  **OLED显示 (`screen.c`)**
    - `screen_on`线程负责驱动OLED屏幕。
    - UI界面清晰地展示了当前的工作模式（HEATING/COOLING/WARMING）、4个温度值（当前、目标、环境温度、PTC当前温度），以及一个直观的温差指示条。

## 5. 演示效果

#### 本地OLED显示

![OLED.jpg](https://oss-club.rt-thread.org/uploads/20251127/193085bc5be924f6819969f5c976af94.jpg.webp)
*OLED实时显示系统关键数据*

#### 远程Web Dashboard

![screenshot_dashboard.gif](https://oss-club.rt-thread.org/uploads/20251127/6a9f360f66992d61afe0c685750ed7bb.gif)
*通过浏览器访问的实时监控仪表盘，包含仪表、状态指示和在线调参区*


![history.png](https://oss-club.rt-thread.org/uploads/20251127/b2c6c96cb4e76d7e15dcf703e880b076.png.webp)
*温度历史曲线图，可以直观地分析系统的响应速度、超调和稳态误差（K线图可以得到的信息更多，而且看起来很有趣）*

#### MSH 命令行调试
通过串口连接，可以直接使用`get_status`查看系统状态，或使用`tune`命令修改参数。

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20251127/240fd615d1677a2ce6648c63fbd3d920.png.webp)

## 6. 改进方向

当然，项目中也存在一些可以持续优化的方向：

1. **硬件**

	把DHT11换成更好的（比如DHT22），DHT11的精度和响应速度都不太行，但是我手里暂时没有其它的温度传感器了。

2. **参数优化**

	目前的参数还不太好，还有很大的优化空间。由于温度的变化很慢，所以如果通过实验测定最优参数的话会非常消耗时间，目前正在尝试通过建模拟真的方式寻找最优参数，因为期中有点忙，所以还没做完。

![screenshot_image.png](https://oss-club.rt-thread.org/uploads/20251127/2cba140d4030571d70458b8d7248e82f.png.webp)

3.  **功能扩展**：
    *   **断网续控与数据缓存**: 增加网络状态监测，在Wi-Fi断开时，系统能继续自主运行，并将历史数据存储在本地Flash中，待网络恢复后自动上传。
    *   **云平台接入**: 将当前的TCP私有协议/WebSocket方案替换为标准的**MQTT协议**，方便接入阿里云、腾讯云等主流物联网平台，实现更强大的设备管理、数据分析和远程OTA功能。
    *   **多段温度曲线**：允许用户通过Web界面预设一条随时间变化的温度曲线（例如：先60℃保温30分钟，再升到80℃保温1小时），使温控箱能用于更复杂的场景，如电路板老化、食品发酵等。

## 7. 代码地址

本项目已开源，欢迎各位老师和开发者批评指正
- **GitHub**: `https://github.com/Cylopsis/Little-TempControled-Box`

## 7. 总结与未来展望

本项目成功地在NXP FRDM-MCXA156平台上，利用RT-Thread构建了一个功能完整、具备远程监控能力的智能温控箱。目前的实现验证了级联PID+前馈控制方案的可行性，并展现了Web可视化在嵌入式调试与监控中的巨大优势。
