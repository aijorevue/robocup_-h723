# H7 麦克纳姆底盘与 RK 机械臂联动工程

本工程运行在 STM32H723VGT6 / DM-MC02 上，负责四轮麦克纳姆底盘、BMI088、LCD、FS-i6S 遥控器、两路 MG90S，以及通过 USB CDC 与 RK3588S 机械臂视觉程序联动。

## 代码入口

- `src/main.c`：红/蓝场路线状态机与三个机械臂任务点。
- `src/route_controller.c`：编码器里程计、BMI088 航向闭环、平移/转向/绕行控制和 H7-RK 协议。
- `src/rc_override.c`：CH5 使能后的遥控器接管与退出。
- `src/rc_protocol.c`、`src/rc_control.c`：SBUS/iBUS 解码、通道检查、死区和速度档位。
- `src/motor_output.c`：四个达妙电机的 CAN 命令和反馈。
- `src/board.c`：时钟、GPIO、ADC、UART、FDCAN、PWM 和 USB 等板级驱动。
- `src/lcd_display.c`：电压、场地、航向、里程、RK 和启动状态显示。
- `src/run_log.c`：行车记录采样、Flash 保存和串口导出。
- `include/app_config.h`：路线距离、速度、PID、超时和硬件映射参数。

H7-RK 协议与联调步骤见 `docs/h7-rk-workflow.md`。

## 启动与控制权

1. 上电初始化 LCD、USB、FDCAN、UART 和 PWM，等待 LCD 摇杆选择场地。
2. 摇杆向上选择红场，向下选择蓝场；显示立即更新，释放后开始路线。
3. 自动路线运行期间，CH5 拉高可在任意阶段由遥控器接管底盘。
4. RC 短暂丢帧时继续发送最后一条有效指令；持续丢失或 CH5 关闭后，零速、失能并回到 LCD 启动门。
5. 路线故障后仍可用 RC 接管；退出 RC 后可重新选择场地启动完整路线。

## 自动路线

红场与蓝场使用相同距离，横移、转向、平台位移和绕行方向互为镜像：

1. 向场地侧横移 `0.8 m`，回正航向。
2. 前进 `4.1 m`，再次回正。
3. 向场地侧转 `90 deg`，执行 `DISC_CATCH`。
4. 后退 `1.6 m`，再转 `90 deg`，前进 `1.6 m`。
5. 再转 `90 deg`，执行三次 `PLATFORM_PICK`，两次横移均为 `0.35 m`。
6. 后退 `0.9 m` 并横移 `0.1 m`，合成斜行。
7. 再转 `90 deg`，异步启动 `COLUMN_CATCH`。
8. 以车头前方 `0.5 m` 为圆心绕行 `270 deg`，再后退 `0.3 m`。
9. 停止 `COLUMN_CATCH`，两路 MG90S 转到 `95 deg` 后回零并关闭 PWM。
10. 底盘零速、失能，保存并输出行车日志，然后回到启动门。

## 当前控制参数

| 参数 | 数值 | 配置宏 |
| --- | ---: | --- |
| 平移最高速度 | `1.800 m/s` | `ROUTE_TRANSLATION_SPEED_M_S` |
| 平移加速度 | `2.000 m/s^2` | `ROUTE_TRANSLATION_ACCEL_M_S2` |
| 长直线最高速度 | `1.800 m/s` | `ROUTE_LONG_FORWARD_SPEED_M_S` |
| 长直线加速度 | `2.000 m/s^2` | `ROUTE_LONG_FORWARD_ACCEL_M_S2` |
| 转向最高角速度 | `2.200 rad/s` | `ROUTE_TURN_MAX_SPEED_RAD_S` |
| 转向角加速度 | `3.800 rad/s^2` | `ROUTE_TURN_ACCEL_RAD_S2` |
| 绕行最高角速度 | `1.000 rad/s` | `ROUTE_ORBIT_MAX_SPEED_RAD_S` |
| 绕行角加速度 | `1.800 rad/s^2` | `ROUTE_ORBIT_ACCEL_RAD_S2` |
| 直行航向 PD | `KP=7.50, KD=0.22` | `HEADING_KP`, `HEADING_KD` |
| 横移航向 PD | `KP=5.00, KD=0.25` | `STRAFE_HEADING_KP`, `STRAFE_HEADING_KD` |
| 转向 PD | `KP=2.20, KD=0.20` | `ROUTE_TURN_KP`, `ROUTE_TURN_KD` |
| 路段稳定时间 | `80 ms` | `ROUTE_SEGMENT_SETTLE_MS` |

平移距离由四轮编码器反馈构成的里程计闭环控制，BMI088 陀螺仪积分 yaw 并参与航向/转角闭环；加速度计仅以较低权重修正速度观测，不单独积分位置。

## 构建

```powershell
cd D:\小车底盘\test\test1
.\build.ps1 -Clean -Configuration Release
```

输出：

```text
build\DM_MC02_Gyro_3m.elf
build\DM_MC02_Gyro_3m.hex
build\DM_MC02_Gyro_3m.bin
```

构建不会自动烧录。实车烧录前必须架空四轮、固定车体并准备动力急停。
