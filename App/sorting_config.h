#ifndef __SORTING_CONFIG_H
#define __SORTING_CONFIG_H

#include <stdint.h>

/* 分拣流程的可调参数集中放在这里，避免散落在状态机和驱动中。
 *
 * 重要：带“待标定”的数值都是占位值，必须实机确认后才能用于正式比赛。
 * 机械结构：UM2 式 Cartesian 龙门，X/Y 为两根独立轴，可直接分别定位。
 */

/* ---------------- 调试开关 ---------------- */

/* 台架 Z 轴按键（KEY2/KEY3）：1 = 控 Z（当前）；0 = 不控 Z，可改用 X 点动。
 * 与 servo_z.h 的 ZSERVO_BENCH_KEY2/3_JUMP 配合：
 *   ZSERVO_BENCH_KEY2/3_JUMP = 1 → KEY2 一键到最下 2500 us、KEY3 一键到最上 1200 us；
 *   = 0 → 每次 ±ZSERVO_STEP_US（50 us），按住连发。
 * X 调试时把本项置 0、SORT_BENCH_KEY_X_JOG 置 1。
 * 正式比赛 SORT_ENABLE_BENCH_KEYS=0 时整段不编译。 */
#define SORT_BENCH_KEY_Z_SERVO          1

/* 台架 X 轴点动（KEY2/KEY3）：1 = 点动 X；0 = 不点动（当前）。
 * 只在 SORT_BENCH_KEY_Z_SERVO = 0 时生效（Z 按键优先级更高）。
 * 调 X 轴时临时把 Z_SERVO 置 0、本项置 1，调完改回。 */
#define SORT_BENCH_KEY_X_JOG            0

/* 台架按键（KEY0/KEY1 夹爪，KEY2/KEY3 Z 轴或 X 轴）只在安全且无任务时生效。
 * 正式比赛必须改为 0，彻底关闭手动入口。 */
#define SORT_ENABLE_BENCH_KEYS          1

/* 台架按键动作方式（KEY0 = 张开方向/脉宽增大，KEY1 = 闭合方向/脉宽减小）：
 * 1 = 按下一次直接跳到开/合位，视觉未就绪时手动模拟夹爪状态（当前）；
 * 0 = 每次按下步进 SERVO_GRIPPER_STEP_US（50 us），用于找机械临界。
 * 标定分支代码保留，重新标定时把这里和 servo_gripper.h 的
 * SERVO_GRIPPER_CALIB_MODE 一起改（0 + 1）。
 * 比赛固件靠 SORT_ENABLE_BENCH_KEYS=0 关闭，不依赖本项。 */
#define SORT_BENCH_KEY_JUMP             1

/* 一次到位的目标脉宽（只在 SORT_BENCH_KEY_JUMP=1 时有效）：
 * 0 = SERVO_GRIPPER_OPEN/CLOSE_PULSE_US（1730/1420 us），
 *     与自动流程完全一致，两端各留 20 us 机械余量，推荐；
 * 1 = SERVO_GRIPPER_MAX/MIN_PULSE_US（1750/1400 us），
 *     实测机械临界；长时间停在临界会顶住齿条，舵机堵转发热，
 *     只在需要复现“极限位置”时短时使用。 */
#define SORT_BENCH_KEY_AT_LIMIT         0

/* 按键消抖：按下电平必须连续保持这么久才算一次有效按键。
 * 太小会被触点抖动重复触发，太大则轻点无效；板载轻触开关用 20 ms。 */
#define SORT_BENCH_KEY_DEBOUNCE_MS      20

/* 标定模式（SORT_BENCH_KEY_JUMP = 0）下按住 KEY1 连续点动：
 * 首次按下立即走一步，之后每 REPEAT_MS 再走一步，松手即停。
 * 50 us/步 × 7 步 ≈ 1 s 走完 350 us 行程；嫌太快就加大到 200~250 ms。
 * 投放口标定（SORT_XY_CALIB_MODE=1，当前）下 KEY0 不连发：按下即开爪，
 * 长按满 SORT_DROP_KEY_LONG_PRESS_MS 才存投放口 X，连发会重置长按计时。
 * Z 轴逐档（SORT_BENCH_KEY_Z_SERVO=1 且 ZSERVO_BENCH_KEY2/3_JUMP=0）
 * 沿用同一节拍：KEY2/KEY3 每 REPEAT_MS 走一步 ZSERVO_STEP_US（50 us）。
 * Z 轴一键到位（ZSERVO_BENCH_KEY2/3_JUMP=1，当前）只在按下沿走一次。 */
#define SORT_BENCH_KEY_REPEAT_MS        150

/* 允许按键的条件（实现见 main.c 的 BenchKey_Allowed()）：
 * 安全输入正常（SafetyManager_IsSafe() != 0），且没有正在执行的分拣任务。
 * 这样上电回零期间、READY、回零失败后的 ERROR 都能立即手动模拟夹爪，
 * 不必等 T 轴回零结束（没接 T_HOME 时要等满超时时间）；
 * 任务执行期间按键不响应，避免和状态机抢夹爪。
 * KEY2/KEY3 另有一层门控：X 轴点动只在 X 空闲时生效（Z 轴标定模式下
 * 只动舵机脉宽，没有“忙”状态，门控仍由安全 + 无任务保证）。
 * 急停（SafetyManager_IsSafe() == 0）任何时候都不放行。 */

/* ---------------- X/Y 回零（2026-10-26 新增） ---------------- */

/* Y 轴硬件尚未接好：0 = 只调试 X，Y 驱动器不上使能、不回零也不参与运动；
 * 1 = X/Y 都按真实零点开关回零。改回 1 前先确认 Y 限位开关已接到 PD5。 */
#define SORT_ENABLE_Y_AXIS              0

/* 回零方向：+1 步数增大，-1 步数减小。两个零点开关都装在步数最小端，
 * 所以默认 -1（朝负方向找零点）；实机方向反了（朝行程外走）就改 +1。 */
#define SORT_X_HOME_DIR                 (-1)
#define SORT_Y_HOME_DIR                 (-1)

/* 回零速度（按 1/16 细分、20T/2 mm 同步轮约 80 微步/mm 估算）：
 *   搜索 800 步/秒 ≈ 10 mm/s，先压到开关；
 *   退让 400 步/秒，反向走到开关释放；
 *   二次接近 120 步/秒 ≈ 1.5 mm/s，用慢速触发边沿当零点，重复性好。
 * 三个速度都远低于 SORT_XY_MAX_SPEED_STEPS_S，满足“初始化慢慢回零”。 */

/* 首次接线 / 找方向的“超低速验证档”（2026-10-26）：
 * 1 = 搜索 400、退让 200、接近 60 步/秒，超时 90 s；
 * 0 = 正常速度（800 / 400 / 120，超时 40 s）。
 *
 * 按 1/16 细分、3200 微步/圈折算，400 步/秒 = 电机 8 秒/圈、
 * 同步轮 20T/2 mm 时滑台约 5 mm/s —— 5 秒能转大半圈，肉眼能看出
 * 是单向走还是来回摆。正常档 800 步/秒 = 4 秒/圈、10 mm/s。
 *
 * 用法：新装的轴在正常速度下“嗡嗡响但不走”时，先置 1 上电观察：
 *   - 能平稳单向转   -> 是细分跳线/启动速度/共振问题。
 *     A4988 的 MS1/MS2/MS3 没全部接高时是整步，实际转速比代码
 *     快 16 倍：800 微步/秒变成 800 整步/秒 ≈ 4 转/秒，
 *     42 步进 + 同步带直接启动基本必失步，表现就是原地抖。
 *   - 只在小角度来回摆 -> 查电机相线分组、STEP/DIR 接线、A4988 电流。
 * 确认完必须改回 0。 */
#define SORT_XY_HOME_DEBUG_SLOW         1

#if SORT_XY_HOME_DEBUG_SLOW
#define SORT_XY_HOME_SPEED_STEPS_S      400U
#define SORT_XY_HOME_BACKOFF_SPEED_STEPS_S 200U
#define SORT_XY_HOME_APPROACH_SPEED_STEPS_S 60U
/* 400 步/秒 × 90 s = 36000 步，足够覆盖 22000 步的保护上限。 */
#define SORT_XY_HOME_TIMEOUT_MS         90000U
#else
#define SORT_XY_HOME_SPEED_STEPS_S      800U
#define SORT_XY_HOME_BACKOFF_SPEED_STEPS_S 400U
#define SORT_XY_HOME_APPROACH_SPEED_STEPS_S 120U
/* 40000 ms 覆盖 250 mm 行程 @10 mm/s 再加余量；超时即判回零失败。 */
#define SORT_XY_HOME_TIMEOUT_MS         40000U
#endif

/* 机械限位开关触点抖动比霍尔大，取 15 ms（转盘霍尔是 10 ms）。 */
#define SORT_XY_HOME_DEBOUNCE_MS        15U

/* 夹爪对射传感器确认。传感器极性和安装位置未标定前先关闭。 */
#define SORT_GRIP_VERIFY_ENABLE         0
#define SORT_GRIP_SENSOR_ACTIVE_LOW     1

/* ---------------- 取放位置（绝对步数，待标定） ---------------- */

/* 固定投放口：夹爪把物块送到这里，转盘把目标仓转到其下方。
 * 坐标以 X/Y 回零后的零点为 0，由 STM32 自己保存，不走上位机；
 * 上位机只发送物块（取料）位置 pick_x_step / pick_y_step。
 * 标定方法（SORT_XY_CALIB_MODE=1）：回零后用 KEY2/KEY3 慢速点动 X
 * 到投放口，按 KEY0 存成投放 X（USART3 打印 DROP_X=...），
 * 再把打印出来的数写回这里，长期生效。 */
#define SORT_DROP_X_STEP                0
#define SORT_DROP_Y_STEP                0

/* 任务完成后的停车位；设为 0/0 表示回到原点上方。 */
#define SORT_PARK_X_STEP                0
#define SORT_PARK_Y_STEP                0
#define SORT_RETURN_TO_PARK             1

/* X/Y 软限位（绝对步数）。回零完成后零点 = 0，行程向零点相反一侧展开：
 *   SORT_X/Y_HOME_DIR = -1（零点在步数最小端，默认）→ 0 ~ +行程；
 *   SORT_X/Y_HOME_DIR = +1（零点在步数最大端）      → -行程 ~ 0。
 * 20000 微步只是保守占位：1/16 细分 + 20T/2 mm 同步轮约 80 微步/mm
 * → 20000 步 ≈ 250 mm。实测行程后按“实测_mm × 80”收紧，
 * 宁小勿大（顶到机械端会丢步异响）。 */
#define SORT_X_TRAVEL_STEPS             20000
#define SORT_Y_TRAVEL_STEPS             20000
#if (SORT_X_HOME_DIR < 0)
#define SORT_X_MIN_STEP                 0
#define SORT_X_MAX_STEP                 SORT_X_TRAVEL_STEPS
#else
#define SORT_X_MIN_STEP                 (-SORT_X_TRAVEL_STEPS)
#define SORT_X_MAX_STEP                 0
#endif
#if (SORT_Y_HOME_DIR < 0)
#define SORT_Y_MIN_STEP                 0
#define SORT_Y_MAX_STEP                 SORT_Y_TRAVEL_STEPS
#else
#define SORT_Y_MIN_STEP                 (-SORT_Y_TRAVEL_STEPS)
#define SORT_Y_MAX_STEP                 0
#endif

/* 回零搜索/退让的最大步数保护（2026-10-26）：
 * 走了“行程 + 2000 步”还没等到开关触发/释放，说明
 *   ① 零点开关没接好或接到 NC/GND（一直低/一直高）、
 *   ② SORT_X/Y_HOME_DIR 方向反了，正顶着机械端、
 *   ③ 机械卡死或电机相线/电流有问题（丢步，位置在涨但轴不动）。
 * 立即停机报 SORT_ERR_HOMING，避免顶着机械端长时间失步
 * （电机发热、同步带/联轴器磨损）。调试时把
 * SORT_X_TRAVEL_STEPS 临时改小可以更快进入 ERROR，方便用
 * KEY2/KEY3 点动确认电机和方向。 */
#define SORT_X_HOME_MAX_STEPS           (SORT_X_TRAVEL_STEPS + 2000)
#define SORT_Y_HOME_MAX_STEPS           (SORT_Y_TRAVEL_STEPS + 2000)

/* ---------------- X/Y 运动参数（待标定） ---------------- */

/* X 轴（A4988）机械与细分：
 * 电机 42BYGH47（1.8°，200 整步/圈，0.55N 保持转矩），A4988 驱动。
 * A4988 细分由 MS1/MS2/MS3 决定：全空/全低 = 整步，全高 = 1/16。
 * 台架实测前先按实际跳线改 SORT_X_DRIVER_MICROSTEP，否则点动步距和速度
 * 会整体差一个比例。Y 轴参数仍沿用 SORT_XY_*，机械与 X 相同。 */
#define SORT_X_MOTOR_FULL_STEPS_PER_REV 200U
#define SORT_X_DRIVER_MICROSTEP         16U

/* 电机轴一整圈的微步数，供台架点动和文档估算用。 */
#define SORT_X_STEPS_PER_MOTOR_REV \
  (SORT_X_MOTOR_FULL_STEPS_PER_REV * SORT_X_DRIVER_MICROSTEP)

#define SORT_XY_MAX_SPEED_STEPS_S       4000U
#define SORT_XY_ACCEL_STEPS_S2          20000U

/* ---------------- T 轴（转盘）参数 ---------------- */

/* 机械：42-40 两相步进电机（1.8°，200 整步/圈）+ 行星减速箱 5.2:1。
 * 驱动器细分以 TB6600 拨码实测为准；下面按 1/16 细分计算：
 *   200 * 16 * 5.2 = 16640 步/圈（转盘输出轴）
 * 若拨码改成 1/8，只把 SORT_T_DRIVER_MICROSTEP 改成 8 即可，
 * 每仓步数、软限位都会自动跟着变；速度和超时仍建议按实机复核。
 * 注意：16640 不能被 6 整除，仓位目标必须用 SORT_T_BIN_STEP()
 * 四舍五入，不能简单写 bin*2773，否则每转一圈会少 2 步并累积。 */
#define SORT_T_MOTOR_FULL_STEPS_PER_REV 200U
#define SORT_T_DRIVER_MICROSTEP         16U
#define SORT_T_GEAR_RATIO_X10           52U   /* 5.2:1；直连写 10，用整数避免浮点 */
#define SORT_T_BOX_COUNT                6

#define SORT_T_STEPS_PER_REV            \
  ((SORT_T_MOTOR_FULL_STEPS_PER_REV * SORT_T_DRIVER_MICROSTEP * \
    SORT_T_GEAR_RATIO_X10) / 10U)

/* 标称每仓步数，只用于台架点动和文档；正式目标用下面的四舍五入式。 */
#define SORT_T_STEPS_PER_BIN            (SORT_T_STEPS_PER_REV / SORT_T_BOX_COUNT)

/* 第 index 仓（0..5）相对零位的步数。6 段之和严格等于一圈：
 *   0, 2773, 5547, 8320, 11093, 13867，最后一段回 0 是 2773。 */
#define SORT_T_BIN_STEP(index) \
  ((((int32_t)(index)) * (int32_t)SORT_T_STEPS_PER_REV + \
    (int32_t)(SORT_T_BOX_COUNT / 2)) / (int32_t)SORT_T_BOX_COUNT)

/* box_id 编号基准：1 表示协议使用 1..6；0 表示使用 0..5。
 * 必须和 OpenART 发送端保持一致。 */
#define SORT_BOX_ID_ONE_BASED           1

/* 回零后 1 号仓相对 T_HOME 触发点的偏移步数。
 *
 * 2026-10-23 实机：霍尔触发点还不是“1 号仓正对投放口”，
 * 要再逆时针转一个小角度才是真正零点（磁铁位置暂不调整）。
 * 现在先保持 0，等机械位置固定后用下面公式补偿：
 *   N = 偏差角度 / 360 * SORT_T_STEPS_PER_REV
 * 若 +步数方向就是逆时针，填 +N；反之填 -N（或 REV-N）。
 * 最好直接把磁铁挪到“1 号仓正对投放口”的位置；软件偏移只做小角度
 * 微调，否则单向转换到 1 号仓可能要多绕近一圈。
 * 改完先用协议逐个仓复核对位，不要直接跑自动流程。 */
#define SORT_T_HOME_OFFSET_STEP         0

/* 回零方向：+1 向步数增大方向找 T_HOME，-1 相反。
 * 建议与 SORT_T_ROTATE_SIGN 同号：正常换仓和回零接近零位都从
 * 同一侧压向磁铁，齿轮间隙方向一致，零位重复性更好。 */
#define SORT_T_HOME_DIR                 (+1)

/* 1 表示允许正反转走最短路径；0 表示只单向转。
 * 没有滑环/线缆余量时保持 0，避免绕线。 */
#define SORT_T_USE_SHORTEST_PATH        0

/* 单向转的旋转方向：+1 步数增大，-1 步数减小。
 * 只在 SORT_T_USE_SHORTEST_PATH=0 时生效。 */
#define SORT_T_ROTATE_SIGN              (+1)

/* T 轴软限位。转盘位置在每个任务结束后归一化到 0..STEPS_PER_REV，
 * 所以单向转目标最多约两圈；按整圈步数的 3 倍自动留余量，
 * 这样修改 STEPS_PER_REV 后不需要重新手算软限位。 */
#define SORT_T_MIN_STEP                 (-(3 * SORT_T_STEPS_PER_REV))
#define SORT_T_MAX_STEP                 (3 * SORT_T_STEPS_PER_REV)

/* 15 rpm 左右：16640 * 15 / 60 ≈ 4160 步/秒，先取 4000（14.4 rpm）。
 * 6 仓换一次约 0.7 s，加减速约 0.2 s；若起步丢步/异响，先把
 * SORT_T_ACCEL_STEPS_S2 降到 10000 或把最大速度降到 3000。 */
#define SORT_T_MAX_SPEED_STEPS_S        4000U
#define SORT_T_ACCEL_STEPS_S2           20000U
/* 回零搜索 2000 步/秒 ≈ 7.2 rpm（一圈 8.3 s）；慢速接近 300 步/秒
 * ≈ 1.1 rpm，触发点重复性更好。 */
#define SORT_T_HOME_SPEED_STEPS_S       2000U
#define SORT_T_HOME_APPROACH_SPEED_STEPS_S 300U
/* 退让速度：搜索触发后反向走到霍尔释放，不再用固定步数，
 * 因此磁铁有效宽度 W 不需要测量，换磁铁也不用改参数。 */
#define SORT_T_HOME_BACKOFF_SPEED_STEPS_S 600U
#define SORT_T_HOME_TIMEOUT_MS          20000U
#define SORT_T_HOME_DEBOUNCE_MS         10U

/* 台架 X 轴点动：KEY2 正向、KEY3 反向，一次走一个电机整圈（3200 微步）。
 * 只在安全、无任务、X 空闲且 ZSERVO_CALIB_MODE = 0 时生效；X 现在有 PD6
 * 零点开关，回零后位置是相对零点的绝对步数，软限位会拦住越界点动，
 * 不会再“无开关乱走”。
 * 带 40T/20T 同步带时直线行程随传动比变化，先按整圈点动对方向；
 * 方向反了就对调 KEY2/KEY3 的符号（见 main.c 的 BenchX_Jog）。
 * ！！Z 轴齿条标定（servo_z.h 的 ZSERVO_CALIB_MODE = 1，当前）期间
 *    KEY2/KEY3 归 Z 轴脉宽用，本项暂时不生效；标定完把 ZSERVO_CALIB_MODE
 *    改回 0 即恢复 X 轴点动。
 * 正式比赛 SORT_ENABLE_BENCH_KEYS=0 时整段不编译。 */
#define SORT_BENCH_X_JOG_STEPS          SORT_X_STEPS_PER_MOTOR_REV

/* 投放口标定（台架按键，2026-10-26） ---------------- */

/* 1 = 投放口标定模式：
 *     KEY0：**按下即张开夹爪**；按住满 SORT_DROP_KEY_LONG_PRESS_MS 后
 *           才额外执行一次“把当前 X 存成投放口 X”，每次按住只存一次；
 *     KEY1：Y 启用（SORT_ENABLE_Y_AXIS=1）时存当前 Y，Y 未启用时仍闭合夹爪；
 *     存完通过 USART3 打印 DROP_X= / DROP_Y=（失败打印 REJECT）。
 *     KEY2/KEY3 每次只走 SORT_CALIB_JOG_STEPS（慢速点动）。
 * 0 = 恢复 KEY0/KEY1 控制夹爪（按下即到开/合位）、KEY2/KEY3 整圈点动。
 * 标定值只存在 RAM，掉电丢失；长期值要写回 SORT_DROP_X_STEP/Y_STEP。
 * ！！SORT_BENCH_KEY_Z_SERVO = 1（当前）时 KEY2/KEY3 被 Z 轴占用，
 *    这里的 X 慢速点动不生效；调 X 时把 Z_SERVO 置 0、X_JOG 置 1。
 *    KEY0 的“短按开爪 + 长按存投放口”仍然有效。 */
#define SORT_XY_CALIB_MODE              1
#define SORT_CALIB_JOG_STEPS            100
#define SORT_CALIB_JOG_SPEED_STEPS_S    300U

/* KEY0 长按判据（只在 SORT_XY_CALIB_MODE=1 时生效）：
 * 短按 < 1000 ms 只开爪；按住满 1000 ms 才存投放口 X。
 * 取 1 s 是为了让“开爪”和“存坐标”两个动作能明确区分开；
 * 觉得太慢可改 800，但不要小于 500（会被手抖/连按误触发）。 */
#define SORT_DROP_KEY_LONG_PRESS_MS     1000U

/* ---------------- Z 轴台架模式（2026-10-27 标定完成） ---------------- */

/* 驱动开关在 Drivers/servo_z.h：
 *   - ZSERVO_CALIB_MODE = 1（当前）：台架手动——上电停在 1850 us（行程中点），
 *     Travel/Approach/Grasp 不改脉宽（视觉未就绪，不跑全流程）；
 *     = 0：正式——上电到运输位 1220 us，自动流程正常控制 Z。
 *   - ZSERVO_BENCH_KEY2/3_JUMP = 1（当前）：KEY2 一键到最下 2500 us、
 *     KEY3 一键到最上 1200 us（实测端点），按下沿一次、不连发；
 *     = 0：每次 ±ZSERVO_STEP_US（50 us），按住连发。
 *   - 实测行程 1200~2500 us，限幅同值；工作高度占位 1220/1850/2480。
 * ！！端点 1200/2500 是机械临界，别长时间停住；自动流程用 1220/2480。
 * ！！视觉联调通过、跑自动流程时把 ZSERVO_CALIB_MODE 改 0。 */

/* 1 = 用 USART3（PB10/PB11，115200）打印回零结果、失败码和标定步数；
 * 正式比赛可以改 0，彻底去掉调试串口输出和对应代码。 */
#define SORT_DEBUG_UART                 1

/* box_id 不在 1..6 时，是否允许退回使用协议里的 drop_step。 */
#define SORT_T_ALLOW_DROP_STEP_FALLBACK 1

/* ---------------- 动作等待时间（单位 ms，待实测） ---------------- */

/* Z 舵机从初始高度到接近高度。 */
#define SORT_Z_APPROACH_MS              600U
/* 夹爪张开到稳定。 */
#define SORT_GRIP_OPEN_MS               400U
/* Z 舵机从接近高度下降到抓取高度。 */
#define SORT_Z_DESCEND_MS               600U
/* 夹爪闭合到稳定。 */
#define SORT_GRIP_CLOSE_MS              400U
/* 夹住确认的采样延时。 */
#define SORT_GRIP_VERIFY_MS             50U
/* Z 舵机从抓取高度抬回初始高度。 */
#define SORT_Z_LIFT_MS                  600U
/* T 轴到位后的稳定时间，防止刚停就开爪。 */
#define SORT_T_SETTLE_MS                200U
/* 投放开爪到物块落下的时间。 */
#define SORT_DROP_OPEN_MS               400U
/* 回停车位/回安全位的等待时间。 */
#define SORT_RETURN_MS                  300U

/* ---------------- 超时保护 ---------------- */

#define SORT_XY_TIMEOUT_MS              10000U
/* 最远换 5 个仓：5*2773≈13865 步，4000 步/秒约 3.5 s；留 3 倍余量。 */
#define SORT_T_TIMEOUT_MS               10000U
#define SORT_MISSION_TIMEOUT_MS         45000U

#endif
