#include "sorting_config.h"

/* 运动指令与虚拟按键共用 USART3：SORT_DEBUG_CONSOLE=0 时没人解析字节，
 * 这里也一起裁掉，避免代码在正式固件里空转。 */
#if SORT_BENCH_MOTION && !SORT_DEBUG_CONSOLE
#error "SORT_BENCH_MOTION requires SORT_DEBUG_CONSOLE=1 (shared USART3 parser)"
#endif

#if SORT_BENCH_MOTION

#include "bench_motion.h"
#include "bench_console.h"
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

/* 小写单词指令缓冲：zup / zdown / xmax / zero / zk / gb / ok ...
 * 和大写行缓冲分开，避免把 X1200 这种“还没输完”的命令提前执行。 */
static char word_buf[BENCH_MOTION_LINE_MAX];
static uint8_t word_len;
static uint8_t word_active;
static uint32_t word_last_ms;

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
  word_len = 0U;
  word_active = 0U;
  word_last_ms = 0U;
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
  DebugUart_Write("-- lowercase words (CR/LF or 100 ms idle ends a word) --\r\n");
  DebugUart_Write("ok       echo ok (power-on also sends ok)\r\n");
  DebugUart_Write("zup      Z up to TOP 1200 us     zdown   Z down to 2150 us\r\n");
  DebugUart_Write("ztop/zbottom  Z 1200/2150 us (top / lowest working)\r\n");
  DebugUart_Write("xmax/xmin  X to soft max/min     ymax/ymin  Y to soft max/min\r\n");
  DebugUart_Write("tjog/tjog-  turret +2000/-2000 steps (works before homing)\r\n");
  DebugUart_Write("zero     re-home (READY) or reset fault then home (ERROR)\r\n");
  DebugUart_Write("zk       gripper open            gb        gripper close\r\n");
  DebugUart_Write("stop/reset/status/help           run       run R with last X/bin\r\n");
  DebugUart_Write("abort    stop R sequence         x1200/y500/t3/z2200/g1650\r\n");
  DebugUart_Write("-- uppercase one-line commands (need CR/LF) --\r\n");
#if SORT_XY_BENCH_MODE
  DebugUart_Write("X<step>  move X (abs, soft limit) X       show X\r\n");
  DebugUart_Write("Y<step>  move Y (abs, soft limit) Y       show Y\r\n");
  DebugUart_Write("T1..T6  turret to bin (T not homed in bench)\r\n");
#else
  DebugUart_Write("X<step>  move X (0..20000)        X       show X\r\n");
  DebugUart_Write("Y<step>  move Y                   T1..T6  turret to bin\r\n");
#endif
  DebugUart_Write("ZT/ZB    Z top1200/bottom2150 (no mid)\r\n");
  DebugUart_Write("Z<us>    set Z pulse (1200..2150) Z       show Z\r\n");
  DebugUart_Write("GO/GC    gripper open/close       G<us>   set grip pulse\r\n");
  DebugUart_Write("R        run sequence (last X/bin) RA      abort sequence\r\n");
  DebugUart_Write("R1200,3  set X=1200 bin=3 then run the sequence\r\n");
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
#if SORT_XY_BENCH_MODE
  /* 台架模式统一用慢速（和四键一键到底一致），避免串口定位比按键快
   * 一个数量级；正式任务在 Submit 里会重新套用 SORT_XY_MAX_SPEED_STEPS_S。 */
  StepperMotor_SetSpeed(STEPPER_X, SORT_XY_BENCH_END_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_X, SORT_XY_ACCEL_STEPS_S2);
#endif
  if (StepperMotor_IsTargetValid(STEPPER_X, steps) == 0U)
  {
    Motion_Print("X", "REJECT RANGE (soft limit)");
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
#if SORT_XY_BENCH_MODE
  StepperMotor_SetSpeed(STEPPER_Y, SORT_XY_BENCH_END_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_Y, SORT_XY_ACCEL_STEPS_S2);
#endif
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

/* T 轴手动点动：不经过回零状态机，也不要求 T 位置有效，专门用来确认
 * “TB6600 / STEP / DIR / EN / 24V / 共地”这条硬件链路是否正常。
 * 位置无效时按当前位置 0 处理，软限位仍是 ±3 圈，不会无限转。 */
static void Motion_TJog(int32_t steps)
{
  if (Motion_Allowed("T") == 0U)
  {
    return;
  }
  if (StepperMotor_IsBusy(STEPPER_T) != 0U)
  {
    Motion_Print("T", "BUSY (wait or use abort)");
    return;
  }
  if (StepperMotor_IsPositionValid(STEPPER_T) == 0U)
  {
    Motion_Print("T", "WARN NOT HOMED (jog from assumed 0)");
  }
  StepperMotor_SetSpeed(STEPPER_T, SORT_T_MAX_SPEED_STEPS_S);
  StepperMotor_SetAcceleration(STEPPER_T, SORT_T_ACCEL_STEPS_S2);
  if (StepperMotor_MoveRelative(STEPPER_T, steps) != HAL_OK)
  {
    Motion_Print("T", "REJECT (soft limit / MoveTo failed)");
    return;
  }
  Motion_PrintInt("T JOG ", "STEPS=", steps);
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
  /* 开合标志按标定窗口中点判断（正式模式 1720~2030 的中点 = 1875 us）：
   * 不能用 START_PULSE 当阈值——正式模式的 START_PULSE 就是张开位，
   * 那样 G2030（完全张开）还会被标成“已闭合”。 */
  ServoGripper_JumpTo((uint16_t)pulse_us,
                      (pulse_us <=
                       (int32_t)((SERVO_GRIPPER_CLOSE_PULSE_US +
                                  SERVO_GRIPPER_OPEN_PULSE_US) / 2U))
                          ? 1U
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
#if SORT_ENABLE_Y_AXIS
      (StepperMotor_IsPositionValid(STEPPER_Y) == 0U) ||
#endif
      (StepperMotor_IsPositionValid(STEPPER_T) == 0U))
  {
    /* 台架模式（SORT_XY_BENCH_MODE=1）只回 X/Y，T 位置永远无效，
     * 所以 R/T 在台架模式下会被这一条拒绝；跑带转盘的全流程要先
     * 把 SORT_XY_BENCH_MODE 改回 0，让状态机把 T 一起回零。 */
    Motion_Print("R", "REJECT NOT HOMED (run HOME first)");
    return;
  }
#if SORT_ENABLE_Y_AXIS
  if (StepperMotor_IsTargetValid(STEPPER_Y, last_y) == 0U)
  {
    Motion_Print("R", "REJECT Y RANGE");
    return;
  }
#endif
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

  /* X/Y 和 T 同时起步：转盘找仓和 XY 定位并行，节省节拍。
   * 三个 MoveTo 都必须成功；任何一路失败都要把已经启动的轴停掉，
   * 否则序列会在“轴根本没动”的情况下继续开爪/下降。 */
  if (StepperMotor_MoveTo(STEPPER_X, last_x) != HAL_OK)
  {
    Motion_Print("R", "REJECT X MoveTo failed");
    return;
  }
#if SORT_ENABLE_Y_AXIS
  if (StepperMotor_MoveTo(STEPPER_Y, last_y) != HAL_OK)
  {
    StepperMotor_Stop(STEPPER_X);
    Motion_Print("R", "REJECT Y MoveTo failed");
    return;
  }
#endif
  if (StepperMotor_MoveTo(STEPPER_T, t_target) != HAL_OK)
  {
    StepperMotor_Stop(STEPPER_X);
#if SORT_ENABLE_Y_AXIS
    StepperMotor_Stop(STEPPER_Y);
#endif
    Motion_Print("R", "REJECT T MoveTo failed");
    return;
  }
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

/* ---------------- 小写单词指令（全流程联调） ---------------- */

static char Motion_ToLower(char c)
{
  if ((c >= 'A') && (c <= 'Z'))
  {
    return (char)(c - 'A' + 'a');
  }
  return c;
}

/* 忽略大小写比较；word 已转小写，这里再转一次以便直接比较原始行。 */
static uint8_t Motion_WordEqual(const char *word, const char *expect)
{
  uint8_t i = 0U;

  while ((word[i] != '\0') && (expect[i] != '\0'))
  {
    if (Motion_ToLower(word[i]) != expect[i])
    {
      return 0U;
    }
    ++i;
  }
  return ((word[i] == '\0') && (expect[i] == '\0')) ? 1U : 0U;
}

/* 单词缓冲允许的字符：字母、数字、逗号、负号。
 * 空格/制表符直接忽略，所以 "run 1200,3" 等价 "run1200,3"。 */
static uint8_t Motion_IsWordChar(uint8_t byte)
{
  if (((byte >= (uint8_t)'a') && (byte <= (uint8_t)'z')) ||
      ((byte >= (uint8_t)'A') && (byte <= (uint8_t)'Z')) ||
      ((byte >= (uint8_t)'0') && (byte <= (uint8_t)'9')) ||
      (byte == (uint8_t)',') || (byte == (uint8_t)'-'))
  {
    return 1U;
  }
  return 0U;
}

/* 形如 x1200 / t3 / z2200 / g1650 的“前缀 + 整数”形式。
 * 返回 1 表示已经完整匹配到词尾；prefix 必须是小写。 */
static uint8_t Motion_WordNumber(const char *word, char prefix, int32_t *value)
{
  const char *p;

  if (Motion_ToLower(word[0]) != prefix)
  {
    return 0U;
  }
  p = word + 1;
  if ((*p != '-') && ((*p < '0') || (*p > '9')))
  {
    return 0U;
  }
  if (Motion_ReadInt(&p, value) == 0U)
  {
    return 0U;
  }
  return (*Motion_SkipSpace(p) == '\0') ? 1U : 0U;
}

/* zero：恢复零点。READY 时重新走完整回零；ERROR/急停锁存时先清故障
 * （ResetFault 内部会确认急停输入已松开），再自动回零。 */
static void Motion_HomeZero(void)
{
  SortingState_t st;

  if (BenchMotion_IsBusy() != 0U)
  {
    Motion_Print("ZERO", "REJECT SEQ BUSY");
    return;
  }
  if (SortingTask_HasMission() != 0U)
  {
    Motion_Print("ZERO", "REJECT MISSION RUNNING");
    return;
  }

  st = SortingTask_GetState();
  if ((st == SORT_ERROR) || (st == SORT_EMERGENCY_STOP))
  {
    if (SortingTask_ResetFault() != 0U)
    {
      Motion_Print("ZERO", "FAULT CLEARED, HOMING");
    }
    else
    {
      Motion_Print("ZERO", "REJECT (emergency stop still on)");
    }
    return;
  }
  if (SafetyManager_IsSafe() == 0U)
  {
    Motion_Print("ZERO", "REJECT UNSAFE");
    return;
  }
  if (st == SORT_SYS_READY)
  {
    if (SortingTask_RequestHoming() != 0U)
    {
      Motion_Print("ZERO", "HOMING START");
    }
    else
    {
      Motion_Print("ZERO", "REJECT HOMING");
    }
    return;
  }
  /* 上电启动中 / 正在回零：不重复下发，打印状态方便判断。 */
  DebugUart_WriteInt("ZERO REJECT BUSY STATE=", (int32_t)st);
}

/* stop：软停止。READY 且无任务时没有可停的东西，直接提示。 */
static void Motion_SoftStop(void)
{
  if ((SortingTask_HasMission() == 0U) &&
      (SortingTask_GetState() == SORT_SYS_READY))
  {
    Motion_Print("STOP", "IDLE (nothing to stop)");
    return;
  }
  SortingTask_Stop();
  Motion_Print("STOP", "REQUESTED (ERROR, use reset/zero)");
}

/* reset：清急停锁存并重新回零（急停仍按下时拒绝）。 */
static void Motion_FaultReset(void)
{
  /* READY 时 SortingTask_ResetFault() 只清安全故障、不会重新回零，
   * 这里单独提示，避免电脑端误以为又开始回零了。 */
  if (SortingTask_GetState() == SORT_SYS_READY)
  {
    Motion_Print("RESET", "ALREADY READY (use zero to re-home)");
    return;
  }
  if (SortingTask_ResetFault() != 0U)
  {
    Motion_Print("RESET", "OK, HOMING");
  }
  else
  {
    Motion_Print("RESET", "REJECT (emergency stop still on)");
  }
}

static void Motion_ShowStatus(void)
{
#if BENCH_CONSOLE_NO_STATUS
  Motion_Print("STATUS", "DISABLED (BENCH_CONSOLE_NO_STATUS=1)");
#else
  BenchConsole_PrintStatus();
#endif
}

/* 单词执行入口。所有动作都复用上面的正式驱动函数，因此安全/无任务
 * 门控、软限位、Z/夹爪限幅与原有大写指令完全一致。 */
static void Motion_ExecuteWord(const char *word)
{
  int32_t value;

  if (Motion_WordEqual(word, "ok"))
  {
    /* 心跳：不经过任何门控，急停/任务中也要能回 ok。 */
    DebugUart_Write("ok\r\n");
    return;
  }

  /* Z 轴：工作高度 / 机械端点 / 指定脉宽 */
  if (Motion_WordEqual(word, "zup"))
  {
    Motion_ZSet(ZSERVO_PULSE_TRAVEL_US, "UP(TRAVEL)");
    return;
  }
  if (Motion_WordEqual(word, "zdown"))
  {
    Motion_ZSet(ZSERVO_PULSE_GRASP_US, "DOWN(GRASP)");
    return;
  }
  if (Motion_WordEqual(word, "ztop"))
  {
    Motion_ZSet(ZSERVO_PULSE_TOP_US, "TOP");
    return;
  }
  if (Motion_WordEqual(word, "zbottom"))
  {
    Motion_ZSet(ZSERVO_PULSE_BOTTOM_US, "BOTTOM");
    return;
  }
  if (Motion_WordEqual(word, "z"))
  {
    Motion_PrintInt("Z", "US=", (int32_t)ZServo_GetPulse());
    return;
  }

  /* X/Y：软限位两端与零点 */
  if (Motion_WordEqual(word, "xmax"))
  {
    Motion_MoveX(SORT_X_MAX_STEP);
    return;
  }
  if (Motion_WordEqual(word, "xmin") || Motion_WordEqual(word, "xzero"))
  {
    Motion_MoveX(SORT_X_MIN_STEP);
    return;
  }
  if (Motion_WordEqual(word, "ymax"))
  {
    Motion_MoveY(SORT_Y_MAX_STEP);
    return;
  }
  if (Motion_WordEqual(word, "ymin") || Motion_WordEqual(word, "yzero"))
  {
    Motion_MoveY(SORT_Y_MIN_STEP);
    return;
  }
  if (Motion_WordEqual(word, "x"))
  {
    /* 兼容原来的单字符 x：释放虚拟按键。 */
    BenchConsole_ReleaseKeys();
    return;
  }
  if (Motion_WordEqual(word, "y"))
  {
    Motion_PrintInt("Y", "POS=", StepperMotor_GetPosition(STEPPER_Y));
    return;
  }

  /* 转盘手动点动：不依赖 T 回零，查驱动器/接线用 */
  if (Motion_WordEqual(word, "tjog") || Motion_WordEqual(word, "tjogp"))
  {
    Motion_TJog(BENCH_MOTION_T_JOG_STEPS);
    return;
  }
  if (Motion_WordEqual(word, "tjog-") || Motion_WordEqual(word, "tjogn"))
  {
    Motion_TJog(-BENCH_MOTION_T_JOG_STEPS);
    return;
  }

  /* 零点 / 停止 / 复位 / 状态 */
  if (Motion_WordEqual(word, "zero") || Motion_WordEqual(word, "home"))
  {
    Motion_HomeZero();
    return;
  }
  if (Motion_WordEqual(word, "stop") || Motion_WordEqual(word, "halt"))
  {
    Motion_SoftStop();
    return;
  }
  if (Motion_WordEqual(word, "reset"))
  {
    Motion_FaultReset();
    return;
  }
  if (Motion_WordEqual(word, "status") || Motion_WordEqual(word, "st"))
  {
    Motion_ShowStatus();
    return;
  }
  if (Motion_WordEqual(word, "help"))
  {
    BenchConsole_PrintHelp();
    return;
  }
  if (Motion_WordEqual(word, "h"))
  {
    BenchConsole_PrintHelp();
    return;
  }

  /* 夹爪 */
  if (Motion_WordEqual(word, "zk") || Motion_WordEqual(word, "go") ||
      Motion_WordEqual(word, "gopen") || Motion_WordEqual(word, "open"))
  {
    Motion_GripOpen();
    return;
  }
  if (Motion_WordEqual(word, "gb") || Motion_WordEqual(word, "gc") ||
      Motion_WordEqual(word, "gclose") || Motion_WordEqual(word, "close"))
  {
    Motion_GripClose();
    return;
  }
  if (Motion_WordEqual(word, "g"))
  {
    Motion_PrintInt("G", "US=", (int32_t)ServoGripper_GetPulse());
    return;
  }

  /* 序列 */
  if (Motion_WordEqual(word, "run") || Motion_WordEqual(word, "start") ||
      Motion_WordEqual(word, "r"))
  {
    Motion_StartSequence(0U, 0, 0U, 0);
    return;
  }
  if (Motion_WordEqual(word, "abort"))
  {
    BenchMotion_Abort();
    return;
  }

  /* 小写数字指令：x1200 / y500 / t3 / z2200 / g1650 */
  if (Motion_WordNumber(word, 'x', &value))
  {
    Motion_MoveX(value);
    return;
  }
  if (Motion_WordNumber(word, 'y', &value))
  {
    Motion_MoveY(value);
    return;
  }
  if (Motion_WordNumber(word, 't', &value))
  {
    Motion_TurretBin((uint8_t)value);
    return;
  }
  if (Motion_WordNumber(word, 'z', &value))
  {
    if ((value < (int32_t)ZSERVO_PULSE_MIN_US) ||
        (value > (int32_t)ZSERVO_PULSE_MAX_US))
    {
      DebugUart_WriteInt("Z REJECT RANGE MIN=",
                         (int32_t)ZSERVO_PULSE_MIN_US);
      DebugUart_WriteInt("Z REJECT RANGE MAX=",
                         (int32_t)ZSERVO_PULSE_MAX_US);
    }
    else
    {
      Motion_ZSet((uint16_t)value, "SET");
    }
    return;
  }
  if (Motion_WordNumber(word, 'g', &value))
  {
    Motion_GripPulse(value);
    return;
  }

  /* 和按键控制台的未知命令提示保持一致，电脑端只认一个字符串。 */
  DebugUart_Write("UNKNOWN CMD (h = help)\r\n");
}

/* 结束当前单词并执行；空词（只收到 CR/LF）直接忽略。 */
static void Motion_FinishWord(void)
{
  word_buf[word_len] = '\0';
  word_active = 0U;
  word_len = 0U;
  if (word_buf[0] != '\0')
  {
    Motion_ExecuteWord(word_buf);
  }
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
        Motion_Print("CMD", "ZM REMOVED (no mid; use ZT/ZB)");
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
      DebugUart_Write("UNKNOWN CMD (h = help)\r\n");
      break;
  }
}

uint8_t BenchMotion_HandleByte(uint8_t byte)
{
  /* 小写单词缓冲：凑满一个词后由 CR/LF、非单词字符或 Process() 的空闲
   * 超时触发执行。s/h/x 等单字符也会走这里，但会延迟
   * BENCH_MOTION_WORD_IDLE_MS 后才执行，输出和原来完全一致。 */
  if (word_active != 0U)
  {
    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
    {
      Motion_FinishWord();
      return 1U;
    }
    if ((byte == (uint8_t)'\b') || (byte == 0x7FU))
    {
      if (word_len > 0U)
      {
        word_len--;
      }
      return 1U;
    }
    if ((byte == (uint8_t)' ') || (byte == (uint8_t)'\t'))
    {
      /* 忽略词内空格，便于输入 "run 1200,3" 这种写法。 */
      word_last_ms = HAL_GetTick();
      return 1U;
    }
    if (Motion_IsWordChar(byte) != 0U)
    {
      if (word_len < (BENCH_MOTION_LINE_MAX - 1U))
      {
        word_buf[word_len++] = Motion_ToLower((char)byte);
        word_last_ms = HAL_GetTick();
        return 1U;
      }
      word_active = 0U;
      word_len = 0U;
      Motion_Print("CMD", "TOO LONG (dropped)");
      return 1U;
    }
    /* 大写运动指令 / 数字按键等：先结束当前单词，再按普通字节处理。 */
    Motion_FinishWord();
  }

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
        break;
    }

    if ((byte >= (uint8_t)'a') && (byte <= (uint8_t)'z'))
    {
      word_active = 1U;
      word_len = 1U;
      word_buf[0] = (char)byte;
      word_last_ms = HAL_GetTick();
      return 1U;
    }
    /* 不是运动指令（0~3 按键、A~D 长按等）交给按键控制台。 */
    return 0U;
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

  /* 小写单词的空闲结尾：串口助手不勾“发送新行”时，收到最后一个字符
   * 后 100 ms 内没有新字符就执行。必须在行缓冲超时之前处理，
   * 否则单字符 s/h/x 要等满 1.5 s 才有反应。 */
  if ((word_active != 0U) &&
      ((uint32_t)(now - word_last_ms) > BENCH_MOTION_WORD_IDLE_MS))
  {
    Motion_FinishWord();
  }

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
