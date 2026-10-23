#include "host_stub.h"
#include "hall_sensor.h"
#include <string.h>

uint8_t      host_bytes[HOST_TRACE_MAX];
uint32_t     host_byte_count;
host_xfer_t  host_xfers[HOST_XFER_MAX];
uint32_t     host_xfer_count;
uint32_t     host_cs_low;
uint32_t     host_cs_high;
uint32_t     host_rst_low;
uint32_t     host_rst_high;
uint8_t      host_dc_level = 1U;
uint8_t      host_blk_level;
uint32_t     host_delay_ms;
uint32_t     host_tick_ms;
uint32_t     host_error_handler_calls;
uint32_t     host_pwm_z_start;
uint32_t     host_pwm_grip_start;
uint32_t     host_pwm_start_it;
uint8_t      host_hall_level;

/* 外设句柄实体：仿真时用普通内存代替寄存器，__HAL_TIM_SET_COMPARE
 * 会直接写到 host_timX_regs.CCRx 上。 */
TIM_TypeDef host_tim1_regs;
TIM_TypeDef host_tim2_regs;
TIM_TypeDef host_tim6_regs;
SPI_TypeDef host_spi2_regs;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
SPI_HandleTypeDef hspi2;

void host_reset(void)
{
  memset(host_bytes, 0, sizeof(host_bytes));
  host_byte_count = 0U;
  host_xfer_count = 0U;
  host_cs_low = 0U;
  host_cs_high = 0U;
  host_rst_low = 0U;
  host_rst_high = 0U;
  host_dc_level = 1U;
  host_blk_level = 0U;
  host_delay_ms = 0U;
  host_tick_ms = 0U;
  host_error_handler_calls = 0U;
  host_pwm_z_start = 0U;
  host_pwm_grip_start = 0U;
  host_pwm_start_it = 0U;
  host_hall_level = 0U;
  memset(host_xfers, 0, sizeof(host_xfers));
  memset(&host_tim1_regs, 0, sizeof(host_tim1_regs));
  memset(&host_tim2_regs, 0, sizeof(host_tim2_regs));

  /* 真机上由 MX_SPI2_Init / MX_TIMx_Init 设置 Instance 指针，仿真里手动设置。 */
  hspi2.Instance = &host_spi2_regs;
  htim1.Instance = &host_tim1_regs;
  htim2.Instance = &host_tim2_regs;
  htim6.Instance = &host_tim6_regs;
}

uint8_t host_xfer_byte(uint32_t index, uint16_t offset)
{
  if ((index >= host_xfer_count) || (offset >= host_xfers[index].len))
  {
    return 0U;
  }
  return host_bytes[host_xfers[index].start + offset];
}

/* ---------------- HAL 桩 ---------------- */

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                       GPIO_PinState PinState)
{
  if ((GPIOx == LCD_CS_GPIO_Port) && (GPIO_Pin == LCD_CS_Pin))
  {
    if (PinState == GPIO_PIN_RESET)
    {
      host_cs_low++;
    }
    else
    {
      host_cs_high++;
    }
  }
  else if ((GPIOx == LCD_DC_GPIO_Port) && (GPIO_Pin == LCD_DC_Pin))
  {
    host_dc_level = (PinState == GPIO_PIN_SET) ? 1U : 0U;
  }
  else if ((GPIOx == LCD_RST_GPIO_Port) && (GPIO_Pin == LCD_RST_Pin))
  {
    if (PinState == GPIO_PIN_RESET)
    {
      host_rst_low++;
    }
    else
    {
      host_rst_high++;
    }
  }
  else if ((GPIOx == LCD_BLK_GPIO_Port) && (GPIO_Pin == LCD_BLK_Pin))
  {
    host_blk_level = (PinState == GPIO_PIN_SET) ? 1U : 0U;
  }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  if ((GPIOx == HALL_SENSOR_GPIO_Port) && (GPIO_Pin == HALL_SENSOR_Pin))
  {
    return (host_hall_level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  }
  return GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
                                   const uint8_t *pData,
                                   uint16_t Size, uint32_t Timeout)
{
  (void)Timeout;

  if ((hspi == NULL) || (hspi->Instance != &host_spi2_regs))
  {
    return HAL_ERROR;
  }
  if ((host_xfer_count >= HOST_XFER_MAX) ||
      ((host_byte_count + Size) > HOST_TRACE_MAX))
  {
    return HAL_ERROR;
  }

  host_xfers[host_xfer_count].dc = host_dc_level;
  host_xfers[host_xfer_count].start = host_byte_count;
  host_xfers[host_xfer_count].len = Size;
  memcpy(&host_bytes[host_byte_count], pData, Size);
  host_byte_count += Size;
  host_xfer_count++;

  return HAL_OK;
}

void HAL_Delay(uint32_t Delay)
{
  host_delay_ms += Delay;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  /* Z 轴：TIM2_CH2（PA1）；夹爪：TIM1_CH1（PA8）。 */
  if ((htim == &htim2) && (Channel == TIM_CHANNEL_2))
  {
    host_pwm_z_start++;
  }
  else if ((htim == &htim1) && (Channel == TIM_CHANNEL_1))
  {
    host_pwm_grip_start++;
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim,
                                       uint32_t Channel)
{
  /* 当前两路舵机都用硬件 PWM，不再需要更新/比较中断。 */
  (void)htim;
  (void)Channel;
  host_pwm_start_it++;
  return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
  return host_tick_ms;
}

void Error_Handler(void)
{
  /* 主机仿真不进入死循环，只记录调用次数。 */
  host_error_handler_calls++;
}
