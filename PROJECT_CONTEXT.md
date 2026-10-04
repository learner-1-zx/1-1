# 智能分拣工程上下文

## 工程概况

- MCU：STM32F407VET6
- HAL 配置工具：STM32CubeMX
- 编译工具：Keil MDK-ARM
- Keil 工程：`MDK-ARM/diankong.uvprojx`
- CubeMX 配置：`diankong.ioc`

## 当前外设

- ADC1：电池电压采样，通道为 ADC1_IN4
- TIM1：PWM 相关功能
- TIM6：基础定时器
- USART2：实际连接设备待确认
- USART3：实际连接设备待确认

## 电机控制信号

- X 轴：`X_STEP`、`X_DIR`、`X_EN`
- Y 轴：`Y_STEP`、`Y_DIR`、`Y_EN`
- T 轴：`T_STEP`、`T_DIR`、`T_EN`

## 输入信号

- `X_MIN`、`X_MAX`
- `Y_MIN`、`Y_MAX`
- `T_HOME`
- `EMERGENCY_STOP`
- `GRIP_SENSOR`

## 当前程序状态

[main.c](Core/Src/main.c) 已完成 HAL、GPIO、TIM1、USART2、USART3、ADC1、TIM6
的初始化。主循环目前尚未实现完整业务逻辑。

## 架构约束

1. 不把全部业务逻辑堆在 `main.c` 中。
2. 步进电机控制、串口协议、安全保护和 ADC 检测独立模块化。
3. 限位和急停具有最高安全优先级。
4. CubeMX 生成代码尽量只在 `USER CODE` 区域修改。
5. 每次修改后都要能够重新编译 Keil 工程。

## 待确认事项

1. USART2 和 USART3 的实际连接设备。
2. X、Y、T 三个轴的机械含义。
3. 步进驱动器的使能电平和方向电平。
4. 急停、限位和抓取传感器的有效电平。
5. 自动分拣动作流程。
