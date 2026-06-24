# 2026 Smart Farm Lower Machine Framework

本工程是面向 2026 百度智慧交通创意组“智慧农场”任务整理的下位机干净框架。
核心原则是：上位机负责全部任务逻辑和识别决策，下位机只负责稳定执行、闭环控制和状态回传。

## 保留模块

- `CORE`：STM32F407 启动文件、系统文件、中断模板。
- `DSP_LIB`：CMSIS DSP 库。
- `FWLIB`：STM32F4 标准外设库。Keil 工程里只编译当前需要的 `misc/adc/gpio/rcc/tim/usart`。
- `Drive_program` / `Drive_include`：板级初始化，包含 SysTick、GPIO、编码器、VIN ADC、调试串口 USART1。
- `Hardware_program` / `Hardware_include`：当前确定会用到的蜂鸣器、电机 PWM、步进电机基础函数。
- `Framework`：今年下位机框架入口，包含调度、通信协议、运动闭环、执行机构命令缓存。

## 已删除的去年工程入口

Keil 工程不再引用去年的多任务状态机、PS2、SBUS、USB、CAN、旧串口解析、旧 `User_program/main.c`、旧传感器任务等模块。
清理后的编译入口集中在 `Framework/main.c`。

## 中断和主循环分工

- `USART6_IRQHandler`：只收上位机 12 字节固定帧，校验完整后放入单帧缓冲区，不解析命令，不发送数据。
- `TIM6_DAC_IRQHandler`：1ms 调度中断，优先级高于 USART6。只做速度环和置标志，不做串口发送。
- `main` 主循环：根据标志调用 `Protocol_Poll()`、`Telemetry_SendOdo()`、`Telemetry_SendBat()`、`Telemetry_SendHB()` 和 `Actuator_Tick()`。

这样可以避免阻塞串口发送卡住 TIM6，也能减少串口中断和速度环同时改目标值造成的数据撕裂。

## 通信协议

固定 12 字节二进制帧：

```text
AA 55 | CMD | DATA0..DATA7 | XOR
```

`XOR` 为前 11 字节异或。16 位数据使用大端序。

当前命令：

- `0x01 SPD`：四轮速度目标，DATA 为 A/B/C/D 四个 `int16`。
- `0x02 STOP`：立即停车。
- `0x03 MODE`：运动模式预留。
- `0x04 PING`：心跳请求。
- `0x10 ARM`：机械臂四轴目标预留。
- `0x11 GRP`：夹爪开合预留。
- `0x12 FIRE`：投放/发射机构预留。
- `0x13 LFT`：升降机构预留。
- `0x14 STO`：仓储/分拣机构预留。

回传：

- `0x81 HB`：版本号、运动使能、执行机构使能、毫秒计数低 16 位、协议错误计数低 16 位。
- `0x82 ODO`：四路编码器增量。
- `0x83 BAT`：电压 ADC 换算值。

## 运动方向约定

`SPD` 中 A/B/C/D 四个速度目标为正数时，定义为整车前进方向。
根据实车自测，底层电机原始方向映射固定为：A/C 使用正输出前进，B/D 使用负输出前进。
这个符号转换只放在 `Framework/motion.c`，上位机不要再额外反向。

## 后续接线位置

如果队伍后续确定了舵机、步进、投放或仓储机构的具体硬件，把实际驱动映射放到 `Framework/actuator.c` 的 `Actuator_Tick()` 中。
不要把机械动作放进 USART6 中断里，也不要把阻塞串口发送放进 TIM6 中断里。
