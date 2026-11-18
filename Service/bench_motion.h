#ifndef __BENCH_MOTION_H
#define __BENCH_MOTION_H

#include <stdint.h>

/* USART3 台架“运动指令”控制台（需要 SORT_DEBUG_CONSOLE=1，共用同一个串口）。
 *
 * 目的：视觉（USART2）和自动分拣流程还没并联调好时，用电脑串口手动跑
 * “到坐标 → 开爪 → Z 到最下 → 闭爪 → Z 上升 → 转盘到指定仓”，
 * 所有动作都直接调用正式驱动（StepperMotor / ZServo / ServoGripper），
 * 不经过状态机，所以不会和队友正在改的 sorting_task.c 抢逻辑。
 *
 * 命令（大写字母开头，**每条后面按回车/换行**；串口助手勾选“发送新行”即可）：
 *   单字符按键 0~3 / A~D / s / h / x 仍然即时生效，不需要回车。
 *   X<步数>       X 轴移动到绝对微步（0..SORT_X_MAX_STEP），例：X1200
 *   X             只打印当前 X 位置
 *   Y<步数>       Y 轴移动（SORT_ENABLE_Y_AXIS=0 时提示 DISABLED）
 *   T<1..6>       转盘转到指定仓（单向/最短路径按 sorting_config.h 配置）
 *   T             只打印当前 T 位置
 *   ZT / ZB       Z 到最上(1200) / 最低工作位(2150) us
 *   Z<脉宽>       Z 到指定脉宽（1200..2150），例：Z2000
 *   Z             只打印当前 Z 脉宽
 *   GO / GC       夹爪张开 / 闭合（JumpTo 一次到位，标定模式下同样有效）
 *   G<脉宽>       夹爪到指定脉宽（按 SERVO_GRIPPER_MIN/MAX 限幅）
 *   G             只打印当前夹爪脉宽
 *   R             用上次的 X 和仓号跑一遍全流程
 *   R<步数>       设定 X 后用上次仓号跑
 *   R<步数>,<仓>  设定 X 和仓号后跑，例：R1200,3
 *   RA            中止正在跑的序列
 *
 * 2026-11-05 新增小写“单词指令”（全流程联调最常用，不需要记大小写缩写）：
 *   ok            回显 ok（另外上电时固件自动发一次 ok）
 *   zup            Z 到最上（ZSERVO_PULSE_TRAVEL_US，1200 us）
 *   zdown          Z 到最低工作位（ZSERVO_PULSE_GRASP_US，2150 us）
 *   ztop/zbottom         Z 到 1200/2150 us（最上/最低工作位）
 *   xmax / xmin    X 到软限位上限 SORT_X_MAX_STEP / 零点 SORT_X_MIN_STEP
 *   ymax / ymin    Y 到软限位上限 / 零点
 *   tjog / tjog-   转盘相对 +2000 / -2000 微步（不要求先回零，查 T 驱动器用）
 *   zero / home    恢复零点：READY 重新回零；ERROR/急停松开后先复位再回零
 *   zk / go       机械爪张开（同 GO）
 *   gb / gc       机械爪闭合（同 GC）
 *   stop          软停止；reset 清故障并回零；status 打印状态
 *   run           用上次 X/仓号跑 R 序列；abort 中止
 *   x1200 / y500 / t3 / z2200 / g1650   小写数字指令，等价于大写版本
 * 单词可以用回车/换行结束；串口助手不勾“发送新行”时，最后一个字符后
 * BENCH_MOTION_WORD_IDLE_MS 内没有新字符也会自动执行。
 * 大写 X/Y/T/Z/G/R 仍按原来的“一行一条”解析，两者互不影响。
 *
 * 全流程 R 的顺序（每一步都从 USART3 打印，方便对着机械看）：
 *   1) X 与新仓同时起步（T 与 XY 并行）；
 *   2) X 到位 → 夹爪张开 → 等 GRIP_SETTLE；
 *   3) Z 到最低工作位（2150）→ 等 Z_SETTLE；
 *   4) 夹爪闭合 → 等 GRIP_SETTLE；
 *   5) Z 到最上（1200）→ 等 Z_SETTLE；
 *   6) 等转盘到位 → SEQ DONE。
 * 注意：本序列不含“移动到投放口/开爪投放”，那部分属于正式流程（队友）。
 *
 * 安全/互锁：任何指令都要求“安全 + 无分拣任务 + 序列空闲”；
 * 序列运行期间 Protocol 会拒收视觉任务（回 BUSY），避免两边同时动轴。 */

/* 序列的等待/超时（毫秒）；与 sorting_config.h 的动作时间保持一致。 */
#define BENCH_MOTION_GRIP_SETTLE_MS      350U
#define BENCH_MOTION_Z_SETTLE_MS         600U
#define BENCH_MOTION_SEQ_XY_TIMEOUT_MS   30000U
#define BENCH_MOTION_SEQ_T_TIMEOUT_MS    20000U
/* 运动指令是“一行一条”：收到 X/Y/T/Z/G/R 后开始缓冲，
 * 超过这个时间还没收到回车/换行就把这半条命令丢掉并提示，
 * 避免残留的命令行把后面的 0~3 按键吞成参数。 */
#define BENCH_MOTION_LINE_TIMEOUT_MS     1500U

/* 小写单词指令的“自动结尾”时间：串口助手不勾“发送新行”时，
 * 收到 zup 的最后一个 p 后 100 ms 没有新字符就执行。
 * 115200 下整词只要约 0.3 ms，留 100 ms 足够容错；
 * 手动逐字键入时建议每个词后加回车，或保持每字间隔 < 100 ms。 */
#define BENCH_MOTION_WORD_IDLE_MS        100U

/* tjog / tjog- 每次相对转动的微步数：16640 步/圈时 2000 步 ≈ 43°，
 * 肉眼能明显看到转盘动，又不会一下转太远。 */
#define BENCH_MOTION_T_JOG_STEPS         2000

/* 清空解析状态与序列（上电初始化时调用）。 */
void BenchMotion_Init(void);
/* 收到一个串口字节：返回 1 表示已被运动指令消费，0 交回按键控制台。 */
uint8_t BenchMotion_HandleByte(uint8_t byte);
/* 主循环调用：推进序列、T 轴位置归一化；非阻塞。 */
void BenchMotion_Process(void);
/* 是否正在跑 R 序列。 */
uint8_t BenchMotion_IsBusy(void);
/* 中止序列（不关驱动器，只停止推进；急停由 SafetyManager 处理）。 */
void BenchMotion_Abort(void);
/* 打印运动指令帮助（供 USART3 控制台 'h' 使用）。 */
void BenchMotion_PrintHelp(void);
/* 供状态打印：最近一次设定的 X / 仓号。 */
int32_t BenchMotion_GetLastX(void);
uint8_t BenchMotion_GetLastBin(void);

#endif /* __BENCH_MOTION_H */
