#include "sorting_config.h"

/* 运动指令与虚拟按键共用 USART3：SORT_DEBUG_CONSOLE=0 时没人解析字节，
 * 这里也一起裁掉，避免代码在正式固件里空转。 */
#if SORT_BENCH_MOTION && !SORT_DEBUG_CONSOLE
#error "SORT_BENCH_MOTION requires SORT_DEBUG_CONSOLE=1 (shared USART3 parser)"
#endif

#if SORT_BENCH_MOTION

#include "bench_motion.h"
#include "debug_uart.h"
#include "safety_manager.h"
#include "servo_gripper.h"
#include "servo_z.h"
#include "sorting_task.h"
#include "stepper_motor.h"

/* 序列状态：每步只做一件事，由 BenchMotion_Process() 每轮推进。 */
typedef enum
{
  BM_SEQ_IDLE = 0,
  BM_SEQ_MOVE_XY,
  BM_SEQ_GRIP_OPEN,
  BM_SEQ_Z_DOWN,
  BM_SEQ_GRIP_CLOSE,
  BM_SEQ_Z_UP,
  BM_SEQ_WAIT_T,
  BM_SEQ_DONE
} BenchMotionSeq_t;

/* 命令解析：大写字母开头 = 运动指令，按“一行一条”解析（需要回车/换行），
 * 这样 X1200、R1200,3 这类带数字的命令不会因为“还没输完”而误执行。
 * 单字符按键（0~3 / A~D / s / h / x）仍然即时生效、不需要回车。 */
#define BENCH_MOTION_LINE_MAX          24U

static char line_buf[BENCH_MOTION_LINE_MAX];
static uint8_t line_len;
static uint8_t line_active;
static uint32_t line_last_ms;

static BenchMotionSeq_t seq_state;
static uint32_t seq_started_ms;
static uint32_t seq_stage_ms;
static int32_t last_x;
static int32_t last_y;
static uint8_t last_bin;
static uint8_t turret_pending;   /* 有一次 T 运动还没做位置归一化 */

static void Motion_Print(const char *tag, const char *text)
{
  DebugUart_Write(tag);
  DebugUart_Write(" ");
  DebugUart_Write(text);
  DebugUart_Write("\r\n");
}

static void Motion_PrintInt(const char *tag, const char *label, int32_t value)
{
  DebugUart_Write(tag);
  DebugUart_Write(" ");
  DebugUart_WriteInt(label, value);
}

/* 所有指令的公共门控：安全、无任务、序列空闲。
 * 与 main.c 的 BenchKey_Allowed() 含义一致，避免和自动流程抢电机。 */
static uint8_t Motion_Allowed(const char *tag)
{
  if (BenchMotion_IsBusy() != 0U)
  {
    Motion_Print(tag, "REJECT SEQ BUSY");
    return 0U;
  }
  if (SafetyManager_IsSafe() == 0U)
  {
    Motion_Print(tag, "REJECT UNSAFE (emergency stop)");
    return 0U;
  }
  if (SortingTask_HasMission() != 0U)
  {
    Motion_Print(tag, "REJECT MISSION RUNNING");
    return 0U;
  }
  return 1U;
}

void BenchMotion_Init(void)
{
  line_len = 0U;
  line_active = 0U;
  line_last_ms = 0U;
  seq_state = BM_SEQ_IDLE;
  seq_started_ms = 0U;
  seq_stage_ms = 0U;
  last_x = 0;
  last_y = 0;
  last_bin = 0U;
  turret_pending = 0U;
}

uint8_t BenchMotion_IsBusy(void)
{
  return ((seq_state != BM_SEQ_IDLE) && (seq_state != BM_SEQ_DONE)) ? 1U : 0U;
}

int32_t BenchMotion_GetLastX(void)
{
  return last_x;
}

uint8_t BenchMotion_GetLastBin(void)
{
  return last_bin;
}

void BenchMotion_PrintHelp(void)
{
  DebugUart_Write("X<step>  move X (0..20000)        X       show X\r\n");
  DebugUart_Write("Y<step>  move Y (disabled now)    T1..T6  turret to bin\r\n");
  DebugUart_Write("ZT/ZB/ZM Z top1200/bottom2500/mid1850\r\n");
  DebugUart_Write("Z<us>    set Z pulse (1200..2500) Z       show Z\r\n");
  DebugUart_Write("GO/GC    gripper open/close       G<us>   set grip pulse\r\n");
  DebugUart_Write("R        run sequence (last X/bin) RA      abort sequence\r\n");
  DebugUart_Write("R1200,3  set X=1200 bin=3 then run the sequence\r\n");
  DebugUart_Write("motion cmds need CR/LF; 0..3, s, h still instant\r\n");
}

/* ---------------- 单轴/单器件动作 ---------------- */

static void Motion_MoveX(int32_t steps)
{
  if (Motion_Allowed("X") == 0U)
  {
    return;
  }
  if (StepperMotor_IsPositionValid(STEPPER_X) == 0U)
  {
    Motion_Print("X", "REJECT NOT HOMED (run HOME first)");
    return;
  }
  if (StepperMotor_IsTargetValid(STEPPER_X, steps) == 0U)
  {
    Motion_Print("X", "REJECT RANGE 0..SORT_X_TRAVEL_STEPS");
    return;
  }
  if (StepperMotor_IsBusy(STEPPER_X) != 0U)
  {
    Motion_Print("X", "BUSY (wait or use RA)");
    return;
  }
  if (StepperMotor_MoveTo(STEPPER_X, steps) != HAL_OK)
  {
    Motion_Print("X", "REJECT (MoveTo failed)");
    return;
  }
  last_x = steps;   /* 供后续 R 直接复用 */
  Motion_PrintInt("X", "-> ", steps);
}

static void Motion_MoveY(int32_t steps)
{
#if SORT_ENABLE_Y_AXIS
  if (Motion_Allowed("Y") == 0U)
  {
    return;
  }
  if (StepperMotor_IsPositionValid(STEPPER_Y) == 0U)
  {
    Motion_Print("Y", "REJECT NOT HOMED (run HOME first)");
    return;
  }
  if (StepperMotor_IsTargetValid(STEPPER_Y, steps) == 0U)
  {
    Motion_Print("Y", "REJECT RANGE");
    return;
  }
  if (StepperMotor_IsBusy(STEPPER_Y) != 0U)
  {
    Motion_Print("Y", "BUSY");
    return;
  }
  if (StepperMotor_MoveTo(STEPPER_Y, steps) != HAL_OK)
  {
    Motion_Print("Y", "REJECT (MoveTo failed)");
    return;
  }
  last_y = steps;
  Motion_PrintInt("Y", "-> ", steps);
#else
  (void)steps;
  Motion_Print("Y", "DISABLED (SORT_ENABLE_Y_AXIS=0)");
#endif
}

/* 第 bin 仓（1..6）相对当前 T 位置的目标步数。
 * 与 sorting_task.c 的 SortingTask_StartTurret() 同一套规则：
 * 单向转按 SORT_T_ROTATE_SIGN 补角，允许最短路径时取 ±半圈内。 */
static uint8_t Motion_TurretTarget(uint8_t bin, int32_t *target)
{
  const int32_t rev = (int32_t)SORT_T_STEPS_PER_REV;
  int32_t desired;
  int32_t current;
  int32_t delta;

  if ((bin < 1U) || (bin > (uint8_t)SORT_T_BOX_COUNT))
  {
    return 0U;
  }
  desired = SORT_T_HOME_OFFSET_STEP + SORT_T_BIN_STEP((int32_t)bin - 1U);
  current = StepperMotor_GetPosition(STEPPER_T);

#if SORT_T_USE_SHORTEST_PATH
  delta = (desired - current) % rev;
  if (delta < 0)
  {
    delta += rev;
  }
  if (delta > (rev / 2))
  {
    delta -= rev;
  }
  *target = current + delta;
#elif (SORT_T_ROTATE_SIGN > 0)
  delta = (desired - current) % rev;
  if (delta < 0)
  {
    delta += rev;
  }
  *target = current + delta;
#else
  delta = (current - desired) % rev;
  if (delta < 0)
  {
    delta += rev;
  }
  *target = current - delta;
#endif
  return 1U;
}

static void Motion_TurretBin(uint8_t bin)
{
  int32_t target;

  if (Motion_Allowed("T") == 0U)
  {
    return;
  }
  if ((bin < 1U) || (bin > (uint8_t)SORT_T_BOX_COUNT))
  {
    Motion_Print("T", "REJECT BIN 1..6");
    return;
  }
  if (StepperMotor_IsPositionValid(STEPPER_T) == 0U)
  {
    Motion_Print("T", "REJECT NOT HOMED (run HOME first)");
    return;
  }
  if (StepperMotor_IsBusy(STEPPER_T) != 0U)
  {
    Motion_Print("T", "BUSY");
    return;
  }
  if (Motion_TurretTarget(bin, &target) == 0U)
  {
    Motion_Print("T", "REJECT TARGET");
    return;
  }
  if (StepperMotor_MoveTo(STEPPER_T, target) != HAL_OK)
  {
    Motion_Print("T", "REJECT (MoveTo failed)");
    return;
  }
  last_bin = bin;
  turret_pending = 1U;
  Motion_PrintInt("T", "BIN=", (int32_t)bin);
}

static void Motion_ZSet(uint16_t pulse_us, const char *name)
{
  if (Motion_Allowed("Z") == 0U)
  {
    return;
  }
  ZServo_JumpTo(pulse_us);
  DebugUart_Write("Z ");
  DebugUart_Write(name);
  DebugUart_WriteInt(" US=", (int32_t)ZServo_GetPulse());
}

static void Motion_GripOpen(void)
{
  if (Motion_Allowed("G") == 0U)
  {
    return;
  }
  /* JumpTo 一次到位：和 KEY0 的“按下即开爪”一致，
   * 标定模式（SERVO_GRIPPER_CALIB_MODE=1）下也不会变成空操作。 */
  ServoGripper_JumpTo(SERVO_GRIPPER_OPEN_PULSE_US, 0U);
  DebugUart_WriteInt("G OPEN US=", (int32_t)ServoGripper_GetPulse());
}

static void Motion_GripClose(void)
{
  if (Motion_Allowed("G") == 0U)
  {
    return;
  }
  ServoGripper_JumpTo(SERVO_GRIPPER_CLOSE_PULSE_US, 1U);
  DebugUart_WriteInt("G CLOSE US=", (int32_t)ServoGripper_GetPulse());
}

static void Motion_GripPulse(int32_t pulse_us)
{
  if (Motion_Allowed("G") == 0U)
  {
    return;
  }
  if ((pulse_us < (int32_t)SERVO_GRIPPER_MIN_PULSE_US) ||
      (pulse_us > (int32_t)SERVO_GRIPPER_MAX_PULSE_US))
  {
    DebugUart_WriteInt("G REJECT RANGE MIN=",
                       (int32_t)SERVO_GRIPPER_MIN_PULSE_US);
    DebugUart_WriteInt("G REJECT RANGE MAX=",
                       (int32_t)SERVO_GRIPPER_MAX_PULSE_US);
    return;
  }
  ServoGripper_JumpTo((uint16_t)pulse_us,
                      (pulse_us <= (int32_t)SERVO_GRIPPER_START_PULSE_US) ? 1U
                                                                          : 0U);
  DebugUart_WriteInt("G US=", (int32_t)ServoGripper_GetPulse());
}

/* ---------------- 全流程序列（R） ---------------- */

void BenchMotion_Abort(void)
{
  if (BenchMotion_IsBusy() == 0U)
  {
    Motion_Print("SEQ", "IDLE (nothing to abort)");
    return;
  }
  /* 只停脉冲、保留驱动器使能（和 SortingTask_Stop 的软停一致），
   * 舵机保持当前位置，避免半空掉块。 */
  StepperMotor_Stop(STEPPER_X);
#if SORT_ENABLE_Y_AXIS
  StepperMotor_Stop(STEPPER_Y);
#endif
  StepperMotor_Stop(STEPPER_T);
  seq_state = BM_SEQ_IDLE;
  Motion_Print("SEQ", "ABORT");
}

static void Motion_StartSequence(uint8_t has_x, int32_t x,
                                 uint8_t has_bin, int32_t bin)
{
  int32_t t_target;

  if (Motion_Allowed("R") == 0U)
  {
    return;
  }
  if (has_bin != 0U)
  {
    if ((bin < 1) || (bin > (int32_t)SORT_T_BOX_COUNT))
    {
      Motion_Print("R", "REJECT BIN 1..6");
      return;
    }
    last_bin = (uint8_t)bin;
  }
  if (last_bin == 0U)
  {
    Motion_Print("R", "REJECT: set bin first (T3 or R1200,3)");
    return;
  }
  if (has_x != 0U)
  {
    if (StepperMotor_IsTargetValid(STEPPER_X, x) == 0U)
    {
      Motion_Print("R", "REJECT X RANGE");
      return;
    }
    last_x = x;
  }
  if ((StepperMotor_IsPositionValid(STEPPER_X) == 0U) ||
      (StepperMotor_IsPositionValid(STEPPER_T) == 0U))
  {
    Motion_Print("R", "REJECT NOT HOMED (run HOME first)");
    return;
  }
  if ((StepperMotor_IsBusy(STEPPER_X) != 0U) ||
#if SORT_ENABLE_Y_AXIS
      (StepperMotor_IsBusy(STEPPER_Y) != 0U) ||
#endif
      (StepperMotor_IsBusy(STEPPER_T) != 0U))
  {
    Motion_Print("R", "REJECT AXIS BUSY");
    return;
  }
  if (Motion_TurretTarget(last_bin, &t_target) == 0U)
  {
    Motion_Print("R", "REJECT TARGET");
    return;
  }

  /* X/Y 和 T 同时起步：转盘找仓和 XY 定位并行，节省节拍。 */
  (void)StepperMotor_MoveTo(STEPPER_X, last_x);
#if SORT_ENABLE_Y_AXIS
  (void)StepperMotor_MoveTo(STEPPER_Y, last_y);
#endif
  (void)StepperMotor_MoveTo(STEPPER_T, t_target);
  turret_pending = 1U;

  seq_state = BM_SEQ_MOVE_XY;
  seq_started_ms = HAL_GetTick();
  seq_stage_ms = seq_started_ms;

  DebugUart_WriteInt("SEQ START X=", last_x);
  DebugUart_WriteInt("SEQ START BIN=", (int32_t)last_bin);
  Motion_Print("SEQ", "MOVE XY + T");
}

/* ---------------- 命令解析（一行一条） ---------------- */

static const char *Motion_SkipSpace(const char *p)
{
  while ((*p == ' ') || (*p == '\t'))
  {
    ++p;
  }
  return p;
}

/* 读十进制整数；成功返回 1，并把指针推到数字之后。 */
static uint8_t Motion_ReadInt(const char **pp, int32_t *value)
{
  const char *p = Motion_SkipSpace(*pp);
  int32_t v = 0;
  uint8_t digits = 0U;
  int8_t sign = 1;

  if (*p == '-')
  {
    sign = -1;
    ++p;
  }
  while ((*p >= '0') && (*p <= '9'))
  {
    if (digits < 6U)   /* 六位足够覆盖 0..20000，防止误输入溢出 */
    {
      v = (v * 10) + (int32_t)(*p - '0');
    }
    ++digits;
    ++p;
  }
  if (digits == 0U)
  {
    return 0U;
  }
  *value = (sign < 0) ? -v : v;
  *pp = p;
  return 1U;
}

/* X<步数> / Y<步数> / T<仓号>
 * Z<T|B|M|脉宽> / G<O|C|脉宽>
 * R[<步数>[,<仓>]] / RA
 * 省略数字时只打印当前值。 */
static void Motion_ParseLine(const char *line)
{
  const char *p = Motion_SkipSpace(line + 1);
  int32_t v1 = 0;
  int32_t v2 = 0;
  uint8_t has_v1;
  uint8_t has_v2 = 0U;

  switch (line[0])
  {
    case 'X':
    case 'Y':
      has_v1 = Motion_ReadInt(&p, &v1);
      if (*Motion_SkipSpace(p) != '\0')
      {
        Motion_Print("CMD", "BAD ARGUMENT");
        return;
      }
      if (has_v1 == 0U)
      {
        Motion_PrintInt((line[0] == 'X') ? "X" : "Y", "POS=",
                        StepperMotor_GetPosition((line[0] == 'X') ? STEPPER_X
                                                                  : STEPPER_Y));
      }
      else if (line[0] == 'X')
      {
        Motion_MoveX(v1);
      }
      else
      {
        Motion_MoveY(v1);
      }
      break;

    case 'T':
      has_v1 = Motion_ReadInt(&p, &v1);
      if (*Motion_SkipSpace(p) != '\0')
      {
        Motion_Print("CMD", "BAD ARGUMENT");
        return;
      }
      if (has_v1 == 0U)
      {
        Motion_PrintInt("T", "POS=", StepperMotor_GetPosition(STEPPER_T));
      }
      else if ((v1 < 1) || (v1 > (int32_t)SORT_T_BOX_COUNT))
      {
        Motion_Print("T", "REJECT BIN 1..6");
      }
      else
      {
        Motion_TurretBin((uint8_t)v1);
      }
      break;

    case 'Z':
      if ((*p == 'T') || (*p == 't'))
      {
        p = Motion_SkipSpace(p + 1);
        if (*p != '\0')
        {
          Motion_Print("CMD", "BAD ARGUMENT");
          return;
        }
        Motion_ZSet(ZSERVO_PULSE_TOP_US, "TOP");
        break;
      }
      if ((*p == 'B') || (*p == 'b'))
      {
        p = Motion_SkipSpace(p + 1);
        if (*p != '\0')
        {
          Motion_Print("CMD", "BAD ARGUMENT");
          return;
        }
        Motion_ZSet(ZSERVO_PULSE_BOTTOM_US, "BOTTOM");
        break;
      }
      if ((*p == 'M') || (*p == 'm'))
      {
        p = Motion_SkipSpace(p + 1);
        if (*p != '\0')
        {
          Motion_Print("CMD", "BAD ARGUMENT");
          return;
        }
        Motion_ZSet(ZSERVO_CALIB_START_US, "MID");
        break;
      }
      has_v1 = Motion_ReadInt(&p, &v1);
      if (*Motion_SkipSpace(p) != '\0')
      {
        Motion_Print("CMD", "BAD ARGUMENT");
        return;
      }
      if (has_v1 == 0U)
      {
        Motion_PrintInt("Z", "US=", (int32_t)ZServo_GetPulse());
      }
      else if ((v1 < (int32_t)ZSERVO_PULSE_MIN_US) ||
               (v1 > (int32_t)ZSERVO_PULSE_MAX_US))
      {
        DebugUart_WriteInt("Z REJECT RANGE MIN=",
                           (int32_t)ZSERVO_PULSE_MIN_US);
        DebugUart_WriteInt("Z REJECT RANGE MAX=",
                           (int32_t)ZSERVO_PULSE_MAX_US);
      }
      else
      {
        Motion_ZSet((uint16_t)v1, "SET");
      }
      break;

    case 'G':
      if ((*p == 'O') || (*p == 'o'))
      {
        p = Motion_SkipSpace(p + 1);
        if (*p != '\0')
        {
          Motion_Print("CMD", "BAD ARGUMENT");
          return;
        }
        Motion_GripOpen();
        break;
      }
      if ((*p == 'C') || (*p == 'c'))
      {
        p = Motion_SkipSpace(p + 1);
        if (*p != '\0')
        {
          Motion_Print("CMD", "BAD ARGUMENT");
          return;
        }
        Motion_GripClose();
        break;
      }
      has_v1 = Motion_ReadInt(&p, &v1);
      if (*Motion_SkipSpace(p) != '\0')
      {
        Motion_Print("CMD", "BAD ARGUMENT");
        return;
      }
      if (has_v1 == 0U)
      {
        Motion_PrintInt("G", "US=", (int32_t)ServoGripper_GetPulse());
      }
      else
      {
        Motion_GripPulse(v1);
      }
      break;

    case 'R':
      if ((*p == 'A') || (*p == 'a'))
      {
        p = Motion_SkipSpace(p + 1);
        if (*p != '\0')
        {
          Motion_Print("CMD", "BAD ARGUMENT");
          return;
        }
        BenchMotion_Abort();
        break;
      }
      has_v1 = Motion_ReadInt(&p, &v1);
      p = Motion_SkipSpace(p);
      if (*p == ',')
      {
        ++p;
        has_v2 = Motion_ReadInt(&p, &v2);
        if (has_v2 == 0U)
        {
          Motion_Print("CMD", "R NEEDS BIN AFTER COMMA");
          return;
        }
        p = Motion_SkipSpace(p);
      }
      if (*p != '\0')
      {
        Motion_Print("CMD", "BAD ARGUMENT");
        return;
      }
      Motion_StartSequence(has_v1, v1, has_v2, v2);
      break;

    default:
      Motion_Print("CMD", "UNKNOWN (h = help)");
      break;
  }
}

uint8_t BenchMotion_HandleByte(uint8_t byte)
{
  if (line_active == 0U)
  {
    switch (byte)
    {
      case (uint8_t)'X':
      case (uint8_t)'Y':
      case (uint8_t)'T':
      case (uint8_t)'Z':
      case (uint8_t)'G':
      case (uint8_t)'R':
        /* 大写字母开头的运动指令：进入行缓冲，等回车/换行再解析。
         * 这样 X1200、R1200,3 不会在输到一半时被误执行。 */
        line_active = 1U;
        line_len = 1U;
        line_buf[0] = (char)byte;
        line_last_ms = HAL_GetTick();
        return 1U;
      default:
        /* 不是运动指令（0~3 按键、A~D 长按、s/h/x 等）交给按键控制台。 */
        return 0U;
    }
  }

  if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
  {
    line_buf[line_len] = '\0';
    line_active = 0U;
    line_len = 0U;
    Motion_ParseLine(line_buf);
    return 1U;
  }

  /* 退格：手动输入写错时可以改，不把垃圾字符带进解析。 */
  if ((byte == (uint8_t)'\b') || (byte == 0x7FU))
  {
    if (line_len > 1U)
    {
      line_len--;
    }
    return 1U;
  }

  if (line_len < (BENCH_MOTION_LINE_MAX - 1U))
  {
    line_buf[line_len++] = (char)byte;
    line_last_ms = HAL_GetTick();
  }
  else
  {
    line_active = 0U;
    line_len = 0U;
    Motion_Print("CMD", "TOO LONG (dropped)");
  }
  return 1U;
}

/* ---------------- 主循环推进 ---------------- */

void BenchMotion_Process(void)
{
  uint32_t now = HAL_GetTick();

  /* 行缓冲超时：只发了 X/一半数字又没回车时把残留丢掉，
   * 否则后面的 0~3 按键会被当成命令参数吞掉。 */
  if ((line_active != 0U) &&
      ((uint32_t)(now - line_last_ms) > BENCH_MOTION_LINE_TIMEOUT_MS))
  {
    line_active = 0U;
    line_len = 0U;
    Motion_Print("CMD", "DROPPED (end motion commands with CR/LF)");
  }

  /* T 轴空闲后把位置归一化到 0..一圈：单向转多任务会持续累加，
   * 这里只改软件坐标、不动电机（与 SortingTask_NormalizeTurretPosition 相同）。 */
  if ((turret_pending != 0U) && (StepperMotor_IsBusy(STEPPER_T) == 0U))
  {
    int32_t rev = (int32_t)SORT_T_STEPS_PER_REV;
    int32_t pos = StepperMotor_GetPosition(STEPPER_T);
    int32_t norm = pos % rev;

    if (norm < 0)
    {
      norm += rev;
    }
    if (norm != pos)
    {
      StepperMotor_SetPosition(STEPPER_T, norm);
    }
    turret_pending = 0U;
  }

  if ((seq_state == BM_SEQ_IDLE) || (seq_state == BM_SEQ_DONE))
  {
    return;
  }

  /* 急停或来了自动任务：立刻停轴并退出序列，避免两边同时驱动同一根轴。
   * 视觉任务在协议层已经被拒（BUSY），这里主要兜住急停和异常路径。 */
  if ((SafetyManager_IsSafe() == 0U) || (SortingTask_HasMission() != 0U))
  {
    StepperMotor_Stop(STEPPER_X);
#if SORT_ENABLE_Y_AXIS
    StepperMotor_Stop(STEPPER_Y);
#endif
    StepperMotor_Stop(STEPPER_T);
    seq_state = BM_SEQ_IDLE;
    Motion_Print("SEQ", "ABORT SAFE/MISSION");
    return;
  }

  switch (seq_state)
  {
    case BM_SEQ_MOVE_XY:
    {
      uint8_t xy_done = (StepperMotor_IsBusy(STEPPER_X) == 0U) ? 1U : 0U;

#if SORT_ENABLE_Y_AXIS
      if (StepperMotor_IsBusy(STEPPER_Y) != 0U)
      {
        xy_done = 0U;
      }
#endif
      if (xy_done != 0U)
      {
        ServoGripper_JumpTo(SERVO_GRIPPER_OPEN_PULSE_US, 0U);
        Motion_Print("SEQ", "GRIP OPEN");
        seq_state = BM_SEQ_GRIP_OPEN;
        seq_stage_ms = now;
      }
      else if ((uint32_t)(now - seq_started_ms) > BENCH_MOTION_SEQ_XY_TIMEOUT_MS)
      {
        seq_state = BM_SEQ_IDLE;
        Motion_Print("SEQ", "TIMEOUT XY");
      }
      break;
    }

    case BM_SEQ_GRIP_OPEN:
      if ((uint32_t)(now - seq_stage_ms) >= BENCH_MOTION_GRIP_SETTLE_MS)
      {
        ZServo_JumpTo(ZSERVO_PULSE_BOTTOM_US);
        DebugUart_WriteInt("SEQ Z DOWN US=", (int32_t)ZServo_GetPulse());
        seq_state = BM_SEQ_Z_DOWN;
        seq_stage_ms = now;
      }
      break;

    case BM_SEQ_Z_DOWN:
      if ((uint32_t)(now - seq_stage_ms) >= BENCH_MOTION_Z_SETTLE_MS)
      {
        ServoGripper_JumpTo(SERVO_GRIPPER_CLOSE_PULSE_US, 1U);
        Motion_Print("SEQ", "GRIP CLOSE");
        seq_state = BM_SEQ_GRIP_CLOSE;
        seq_stage_ms = now;
      }
      break;

    case BM_SEQ_GRIP_CLOSE:
      if ((uint32_t)(now - seq_stage_ms) >= BENCH_MOTION_GRIP_SETTLE_MS)
      {
        ZServo_JumpTo(ZSERVO_PULSE_TRAVEL_US);
        DebugUart_WriteInt("SEQ Z UP US=", (int32_t)ZServo_GetPulse());
        seq_state = BM_SEQ_Z_UP;
        seq_stage_ms = now;
      }
      break;

    case BM_SEQ_Z_UP:
      if ((uint32_t)(now - seq_stage_ms) >= BENCH_MOTION_Z_SETTLE_MS)
      {
        Motion_Print("SEQ", "WAIT TURRET");
        seq_state = BM_SEQ_WAIT_T;
        seq_stage_ms = now;
      }
      break;

    case BM_SEQ_WAIT_T:
      if (StepperMotor_IsBusy(STEPPER_T) == 0U)
      {
        seq_state = BM_SEQ_DONE;
        Motion_Print("SEQ", "DONE (drop move belongs to the auto flow)");
      }
      else if ((uint32_t)(now - seq_stage_ms) > BENCH_MOTION_SEQ_T_TIMEOUT_MS)
      {
        seq_state = BM_SEQ_IDLE;
        Motion_Print("SEQ", "TIMEOUT TURRET");
      }
      break;

    default:
      seq_state = BM_SEQ_IDLE;
      break;
  }
}

#else /* SORT_BENCH_MOTION */
/* SORT_BENCH_MOTION=0：保留空编译单元，正式固件不包含任何测试运动代码。 */
typedef int BenchMotion_Disabled_t;
#endif /* SORT_BENCH_MOTION */
