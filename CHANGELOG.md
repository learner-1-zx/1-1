# CHANGELOG — 精简修改记录

> 只记录“当前有效”的变更。详细调试过程看 Git 历史，不要写回本文件。
> 当前参数以 `AI_CONTEXT.md` 为准。

## 2026-11-18（文档整理）

- 新增 `AI_CONTEXT.md`：AI 默认入口、当前有效参数、代码地图、构建命令。
- 新增 `AI_RULES.md`：**新建 .py 必须先说明目的并征得同意**、X/Y 代码边界等。
- 新增 `.clinerules`：精简版 AI 规则。
- 新增本 `CHANGELOG.md`。
- 大文档（IO/LESSONS/PROJECT_CONTEXT/SORTING_FLOW）
  顶部加“历史资料，当前值看 AI_CONTEXT”的提示。
- 删除 `XY_INTEGRATION_GUIDE.md`：一次性合并队友 XY 分支的旧说明，
  行程/投放口/协议（0x05/0x87）已过时，保留会误导。
- 不再维护旧的 `ls.md` 调试记录。

## 2026-11-08（视觉/投放口/Z）

### 视觉坐标映射

- `SORT_VISION_COORD_MAP=1`：`pick_x_step/pick_y_step` 字段改为
  **视觉坐标（默认 mm，相对视觉零点）**。
- 视觉零点：机械 **X=2200 / Y=3000**。
- 比例：**900 微步 = 2 cm = 20 mm → 45 微步/mm**。
- 映射函数 `SortingTask_MapVisionMission()`；
  `protocol.c` 在 `SortingTask_Submit()` 前调用。
- 支持 `NUM/DEN` 单位切换（cm=450/1，0.1mm=45/10）和 X/Y `SIGN`。
- 转换后仍走软限位；USART3 打印 `VISION_X/Y`、`MAP_X_STEP/Y_STEP`。

### 投放口

- 投放口固定为 **X/Y 限位零点 0/0**；
- `SORT_XY_CALIB_MODE=0`，关闭“当前位置存投放口”的标记方法；
- `SORT_T_ALLOW_DROP_STEP_FALLBACK=0`，不再用 `drop_step` 回退；
- 零点由回零时限位开关二次触发边沿建立。

### Z 轴

- `ZSERVO_CALIB_MODE=0`，上电直接 **ztop=1200 us**；
- 最低工作位 **2150 us**；限幅 1200~2150；
- `Travel/Approach/Grasp = 1200/1200/2150`，**取消 1850 zmid**；
- `zmid` 命令删除；`ZM` 提示 `ZM REMOVED`；
- 台架 KEY2 一键 2150、KEY3 一键 1200。

### 串口/台架

- 上电 `ok`；每次发 `ok` 回 `ok`；
- 小写单词指令：`zup/zdown/ztop/zbottom/xmax/xmin/zero/zk/gb/tjog/tjog-/run/abort`；
- 版本标记 `FW=2026-11-08 T-PARALLEL ULN2003`；
- `s` 状态增加 `T_POS_OK/T_BUSY/T_HOME`；
- 台架模式 X/Y 与 T **同时回零**。

### T 轴

- TB6600 + ULN2003（三路反相）极性修正：
  `T_STEP_ACTIVE_LOW=0`、`T_DIR_INVERT=0`、`T_EN_ACTIVE_LOW=1`；
- 速度按旧测试上限：1200/1250，回零 1000/200/400；
- 已知问题：T 仍有噪声大/来回打转，未最终解决；
  旧测试资料见 Git 提交 `59207c1:TURNTABLE_TEST_2026-10-12.md`。

## 2026-10-31

- 新增 USART3 台架控制台与运动指令（大写 X/Y/T/Z/G/R）。
- 补齐视觉协议与上位机工具。

## 2026-10-27

- Z 轴齿条标定完成（历史值 1200/2500；2026-11-08 已改为 1200/2150）。

## 2026-10-25

- 夹爪 BDS300 重新标定：闭合 1420、张开 1730 us。

## 2026-10-12

- 转盘 T 轴单独测试记录：`TURNTABLE_TEST_2026-10-12.md`（Git 59207c1）。
