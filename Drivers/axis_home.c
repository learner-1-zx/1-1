#include "axis_home.h"

/* X/Y 零点开关读取：引脚模式和上拉在 gpio.c 中统一配置，这里只做
 * 电平读取、极性换算和 10 ms 左右的软件消抖。
 * 机械限位开关压到行程端点时触点抖动比霍尔明显，所以消抖时间
 * 由调用方传入（回零用 SORT_XY_HOME_DEBOUNCE_MS）。 */

typedef struct
{
  uint8_t active_seen;
  uint32_t active_since_ms;
  uint8_t release_seen;
  uint32_t release_since_ms;
} AxisHomeDebounce_t;

static AxisHomeDebounce_t home_debounce[AXIS_HOME_COUNT];

GPIO_PinState AxisHome_ReadRaw(AxisHomeId_t id)
{
  switch (id)
  {
    case AXIS_HOME_X:
      return HAL_GPIO_ReadPin(X_HOME_GPIO_Port, X_HOME_Pin);
    case AXIS_HOME_Y:
      return HAL_GPIO_ReadPin(Y_HOME_GPIO_Port, Y_HOME_Pin);
    default:
      return GPIO_PIN_SET;
  }
}

uint8_t AxisHome_IsTriggered(AxisHomeId_t id)
{
  GPIO_PinState value;

  if (id >= AXIS_HOME_COUNT)
  {
    return 0U;
  }
  value = AxisHome_ReadRaw(id);
#if AXIS_HOME_ACTIVE_HIGH
  return (value == GPIO_PIN_SET) ? 1U : 0U;
#else
  return (value == GPIO_PIN_RESET) ? 1U : 0U;
#endif
}

void AxisHome_ResetDebounce(void)
{
  uint8_t i;

  for (i = 0U; i < (uint8_t)AXIS_HOME_COUNT; ++i)
  {
    home_debounce[i].active_seen = 0U;
    home_debounce[i].active_since_ms = 0U;
    home_debounce[i].release_seen = 0U;
    home_debounce[i].release_since_ms = 0U;
  }
}

uint8_t AxisHome_TriggeredDebounced(AxisHomeId_t id, uint32_t debounce_ms)
{
  uint32_t now;

  if (id >= AXIS_HOME_COUNT)
  {
    return 0U;
  }
  if (AxisHome_IsTriggered(id) == 0U)
  {
    home_debounce[id].active_seen = 0U;
    return 0U;
  }

  now = HAL_GetTick();
  if (home_debounce[id].active_seen == 0U)
  {
    home_debounce[id].active_seen = 1U;
    home_debounce[id].active_since_ms = now;
    return 0U;
  }
  return ((uint32_t)(now - home_debounce[id].active_since_ms) >= debounce_ms)
             ? 1U : 0U;
}

uint8_t AxisHome_ReleasedDebounced(AxisHomeId_t id, uint32_t debounce_ms)
{
  uint32_t now;

  if (id >= AXIS_HOME_COUNT)
  {
    return 0U;
  }
  if (AxisHome_IsTriggered(id) != 0U)
  {
    home_debounce[id].release_seen = 0U;
    return 0U;
  }

  now = HAL_GetTick();
  if (home_debounce[id].release_seen == 0U)
  {
    home_debounce[id].release_seen = 1U;
    home_debounce[id].release_since_ms = now;
    return 0U;
  }
  return ((uint32_t)(now - home_debounce[id].release_since_ms) >= debounce_ms)
             ? 1U : 0U;
}
