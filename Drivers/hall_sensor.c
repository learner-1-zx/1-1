#include "hall_sensor.h"

/* 霍尔开关读取：实际信号接 PD12/T_HOME，引脚模式和上拉在 gpio.c
 * 中统一配置；这里只做电平读取和极性换算。
 * 低有效开集电极：有磁铁 = 低电平，无磁铁 = 上拉高电平。 */

GPIO_PinState HallSensor_ReadRaw(void)
{
  return HAL_GPIO_ReadPin(HALL_SENSOR_GPIO_Port, HALL_SENSOR_Pin);
}

uint8_t HallSensor_IsTriggered(void)
{
  GPIO_PinState value = HallSensor_ReadRaw();

#if HALL_SENSOR_ACTIVE_HIGH
  return (value == GPIO_PIN_SET) ? 1U : 0U;
#else
  return (value == GPIO_PIN_RESET) ? 1U : 0U;
#endif
}
