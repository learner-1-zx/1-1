#ifndef __SORTING_CONFIG_H
#define __SORTING_CONFIG_H

#include <stdint.h>

/* 分拣流程的可调参数集中放在这里，避免散落在状态机和驱动中。
 *
 * 重要：带“待标定”的数值都是占位值，必须实机确认后才能用于正式比赛。
 * 机械结构：UM2 式 Cartesian 龙门，X/Y 为两根独立轴，可直接分别定位。
 */

/* ---------------- 调试开关 ---------------- */

/* 台架按键（KEY0/KEY1 夹爪，KEY2/KEY3 T 轴单仓点动）只在安全且无任务时生效。
 * 正式比赛必须改为 0，彻底关闭手动入口。 */
#define SORT_ENABLE_BENCH_KEYS          1

/* 台架按键动作方式（KEY0 = 张开方向，KEY1 = 闭合方向）：
 * 0 = 每次按下步进 SERVO_GRIPPER_STEP_US（10 us），用于找机械临界；
 * 1 = 按下一次直接跳到开/合位，用于视觉未就绪时手动模拟夹爪状态。
 * 比赛固件靠 SORT_ENABLE_BENCH_KEYS=0 关闭，不依赖本项。 */
#define SORT_BENCH_KEY_JUMP             1

/* 一次到位的目标脉宽（只在 SORT_BENCH_KEY_JUMP=1 时有效）：
 * 0 = SERVO_GRIPPER_OPEN/CLOSE_PULSE_US（1780/1410 us），
 *     与自动流程完全一致，两端各留 20 us 机械余量，推荐；
 * 1 = SERVO_GRIPPER_MAX/MIN_PULSE_US（1800/1390 us），
 *     实测机械临界；长时间停在临界会顶住齿条，舵机堵转发热，
 *     只在需要复现“极限位置”时短时使用。 */
#define SORT_BENCH_KEY_AT_LIMIT         0

/* 按键消抖：按下电平必须连续保持这么久才算一次有效按键。
 * 太小会被触点抖动重复触发，太大则轻点无效；板载轻触开关用 20 ms。 */
#define SORT_BENCH_KEY_DEBOUNCE_MS      20

/* 允许按键的条件（实现见 main.c 的 BenchKey_Allowed()）：
 * 安全输入正常（SafetyManager_IsSafe() != 0），且没有正在执行的分拣任务。
 * 这样上电回零期间、READY、回零失败后的 ERROR 都能立即手动模拟夹爪，
 * 不必等 T 轴回零结束（没接 T_HOME 时要等满超时时间）；
 * 任务执行期间按键不响应，避免和状态机抢夹爪。
 * KEY2/KEY3 的 T 轴点动另有一层门控：只在 READY 且 T 空闲时生效。
 * 急停（SafetyManager_IsSafe() == 0）任何时候都不放行。 */

/* X/Y 没有回零开关时，上电把软件位置清零，操作者必须先把机构推到原点。
 * 装上 X/Y 回零开关后改为 0，并补充真正的回零流程。 */
#define SORT_ASSUME_XY_ORIGIN           1

/* 夹爪对射传感器确认。传感器极性和安装位置未标定前先关闭。 */
#define SORT_GRIP_VERIFY_ENABLE         0
#define SORT_GRIP_SENSOR_ACTIVE_LOW     1

/* ---------------- 取放位置（绝对步数，待标定） ---------------- */

/* 固定投放口：夹爪把物块送到这里，转盘把目标仓转到其下方。 */
#define SORT_DROP_X_STEP                0
#define SORT_DROP_Y_STEP                0

/* 任务完成后的停车位；设为 0/0 表示回到原点上方。 */
#define SORT_PARK_X_STEP                0
#define SORT_PARK_Y_STEP                0
#define SORT_RETURN_TO_PARK             1

/* X/Y 软限位（绝对步数，待按行程标定）。
 * 先给一个足够大的范围，装上回零开关后再收紧到实际行程。 */
#define SORT_X_MIN_STEP                 (-100000)
#define SORT_X_MAX_STEP                 (100000)
#define SORT_Y_MIN_STEP                 (-100000)
#define SORT_Y_MAX_STEP                 (100000)

/* ---------------- X/Y 运动参数（待标定） ---------------- */

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
 * 改完先单仓点动确认，不要直接跑自动流程。 */
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

/* 台架 T 轴点动：KEY2 正向、KEY3 反向，一次走一个仓距（≈60°）。
 * 只在 READY 且 T 空闲时生效；点动后 T 位置标志被清除，
 * 必须重新回零（发 HOME 或复位）才能接收自动任务。
 * 正式比赛 SORT_ENABLE_BENCH_KEYS=0 时整段不编译。 */
#define SORT_BENCH_T_JOG_STEPS          SORT_T_STEPS_PER_BIN

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
