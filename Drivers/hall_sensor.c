#include "hall_sensor.h"

/* 霍尔开关读取：实际信号接 PD12/T_HOME，引脚模式和上拉在 gpio.c
 * 中统一配置；这里只做电平读取和极性换算。
 * 低有效开集电极：有磁铁 = 低电平，无磁铁 = 上拉高电平。 */

 //原始读电平函数
GPIO_PinState HallSensor_ReadRaw(void)
{
  return HAL_GPIO_ReadPin(HALL_SENSOR_GPIO_Port, HALL_SENSOR_Pin);
}

//逻辑触发判断函数
uint8_t HallSensor_IsTriggered(void)
{
  GPIO_PinState value = HallSensor_ReadRaw();

  //条件编译 宏开关 如果在别的地方定义了HALL_SENSOR_ACTIVE_HIGH 宏
  //则认为是高电平触发 否则就是低电平触发
#if HALL_SENSOR_ACTIVE_HIGH
  return (value == GPIO_PIN_SET) ? 1U : 0U;
#else
  return (value == GPIO_PIN_RESET) ? 1U : 0U;
#endif
}
//1U 代表无符号的整数