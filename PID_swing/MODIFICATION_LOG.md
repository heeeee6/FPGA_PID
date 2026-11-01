# PID_swing 修改日志

本文件用于记录 FPGA 工程的每次功能性修改。每条记录都列出改动文件、
验证结果和撤回方法。

## 基线

- 日期：2026-11-01
- Git 基线：`2c98701`（第二版——DS）
- 说明：该基线保留了 STM32 `Mode31` 风格的 1 kHz 状态机和 5 ms 角度环，
  不包含后续未稳定的 2 ms 原始角度实验。

## 修改记录

### 2026-11-01 / 恢复 STM32 风格参数调节界面

- 状态：已完成
- 目标：
  - 恢复中心角度、起摆力度和起摆时间的菜单调节。
  - 在 OLED 上实时显示中心角度、起摆力度、起摆时间和 K1 按下状态。
  - 保持 STM32 `Menu.c` 的上/下/确认/返回按键语义。
  - 使用附件给出的默认参数：角度环 `0.32/0.01/0.42`，
    位置环 `0.40/0.00/4.00`，中心角度 `2094`，起摆力度 `45`。
- 默认起摆时间：`186 ms`。
- 实际操作：
  - PAGE0/1/2 中 K1 启停，K2/K3 修改位置目标，K4 切换页面。
  - PAGE3 浏览时：K1 上一项，K2 下一项，K3 进入/退出编辑，
    K4 返回 PAGE0。
  - PAGE3 编辑中：K1 减小，K2 增大，K3 完成编辑，K4 返回。
  - 编辑中心角度或起摆时间时，按住 K1/K2 每 50 ms 连续改变 1。
  - 进入参数编辑和进入 PAGE3 时强制撤销运行请求，避免 K1 同时触发
    调参和电机启动。
  - OLED 普通页第 7 行显示 `C:中心角度 P:起摆力度 T:起摆时间`；
    第 8 行显示 K1 电平、运行请求和位置目标。
  - OLED 的 PAGE3 显示菜单光标、编辑状态、四个参数、K1 状态和运行状态。
- 涉及文件：
  - `rtl/pendulum_ui.v`
  - `rtl/oled_ssd1306_ui.v`
  - `rtl/pid_swing_top.v`
  - `sim/tb_oled_ssd1306_ui.sv`
  - `sim/tb_stm32_menu_restore.sv`
  - `PID_swing.xpr`
- OLED 数字格式化已改为在视觉捕获阶段预先寄存，避免显示格式化组合逻辑
  进入 50 MHz 关键路径。
- 验证：
  - 7 个 XSim 测试全部通过。
  - 新增 `tb_stm32_menu_restore` 验证短按、长按连续调节、编辑停机互锁、
    PAGE3 返回和 PAGE0 的 K1 启动。
  - Vivado 2018.3 综合、实现和比特流生成通过。
  - 路由后时序：WNS `+0.318 ns`，TNS `0.000 ns`，
    WHS `+0.167 ns`，THS `0.000 ns`，无时序失败端点。
- 比特流：
  - `pid_swing_hw_top_stm32_menu_restored.bit`
  - SHA-256：
    `D4DE52DF24D54703891F6A73B415C07648896F2821F71FF3B79078B604594DB7`
- 撤回源码：

```powershell
git restore --source=2c98701 -- `
  PID_swing/rtl/oled_ssd1306_ui.v `
  PID_swing/rtl/pendulum_ui.v `
  PID_swing/rtl/pid_swing_top.v `
  PID_swing/sim/tb_oled_ssd1306_ui.sv `
  PID_swing/PID_swing.xpr
Remove-Item -LiteralPath `
  PID_swing/sim/tb_stm32_menu_restore.sv, `
  PID_swing/MODIFICATION_LOG.md
```

上述命令应在 `D:\FPGA\FPGA_PID` 中执行。删除动作只针对本记录新增的两个文件。

### 2026-11-01 / PAGE3 直接启停并确认参数进入控制链

- 状态：已完成
- 修改原因：进入第四显示页（内部编号 `PAGE3`）后，原按键表将 K1 用作
  菜单上一项，因此无法在该页直接启动倒立摆。
- 实际操作：
  - PAGE3 浏览状态：K1 启动/停止，K2 下一参数项，K3 进入编辑，
    K4 返回 PAGE0。离开 PAGE3 时撤销运行请求。
  - PAGE3 编辑状态：K1 减小，K2 增大，K3 完成编辑，K4 返回。
  - 在 PAGE3 完成参数修改后，可直接按 K1 启动，不需要先切回 PAGE0。
  - OLED PAGE3 按键提示同步改为 `K1 RUN / K2 NEXT / K3 ED`。
- PAGE3 参数到控制模块的对应关系：
  - 中心角度 `center_angle` 进入 `angle_center_map`、STM32 角度误差
    以及双环 PID 的中心参考。
  - 起摆力度 `start_pwm` 进入 `swing_balance_ctrl.start_pwm_value`。
  - 起摆时间 `start_time_ms` 进入
    `swing_balance_ctrl.start_time_value`。
  - PWM 偏移 `pwm_offset` 进入
    `swing_balance_ctrl.pwm_offset_value`。
  - `USE_STM32_UI=1` 和 `RUNTIME_PARAMS=1` 已使上述值替代编译期固定值。
- 涉及文件：
  - `rtl/pendulum_ui.v`
  - `rtl/oled_ssd1306_ui.v`
  - `sim/tb_stm32_menu_restore.sv`
- 验证：
  - 7 个 XSim 测试全部通过。
  - `tb_stm32_menu_restore` 新增验证：PAGE3 的 K1 启停、编辑时强制停机、
    参数保留，以及编辑结束后不离开 PAGE3 即可再次启动。
  - Vivado 2018.3 综合、实现和比特流生成通过。
  - 路由后时序：WNS `+0.535 ns`，TNS `0.000 ns`，
    WHS `+0.163 ns`，THS `0.000 ns`，无时序失败端点。
- 比特流：
  - `pid_swing_hw_top_page4_k1.bit`
  - SHA-256：
    `B46B286B29F50422C2D66F77B41E811D877103CA3D9FFC9C554BDA9B982C9CAA`
- 如需仅退回本次按键行为，可恢复上一个比特流：
  `pid_swing_hw_top_stm32_menu_restored.bit`。源码层面可在
  `D:\FPGA\FPGA_PID` 执行 `git diff -- PID_swing/rtl`，
  查看并对本次 PAGE3 分支修改做反向应用。

### 2026-11-01 / 移植参考 FPGA 倒立摆工程的控制结构

- 状态：已完成
- 参考工程：
  `https://github.com/shisanzerotwo/fpga-inverted-pendulum`
  - 参考版本：`81b6013`
  - 重点学习：`inverted-pendulum/fpga/pendulum/src/pid.v`
    和 `balance_ctrl.v`
- 移植思路：
  - PID 的 P/I/D 分子先完整相加，最后只做一次除以 Q8 缩放和一次向零
    截断，避免当前实现逐项右移造成的正负不对称和分数丢失。
  - 增加真实限幅方向上的条件积分，避免电机已经饱和时继续累积积分。
  - 起摆过程继续保留 1%/ms 安全斜率；进入平衡态后参考工程的做法是让
    PID 结果直接控制电机，因此平衡态取消内部命令斜率。
  - TB6612 的方向和占空比改为只在 20 kHz PWM 周期边界锁存，禁止一个
    PWM 周期中途换向或产生残缺脉冲。
  - 角度 ADS1115 重新解耦：I2C 使用 400 kHz，角度每 1 ms 优先读取，
    四路电位器在后台约每 20 ms 扫描一轮。
  - 平衡状态落摆退出时同时清除 `armed`，确保 TB6612 进入安全输出。
- 涉及文件：
  - `rtl/pid_fixed.v`
  - `rtl/swing_balance_ctrl.v`
  - `rtl/tb6612_pwm.v`
  - `rtl/ads1115_reader.v`
  - `sim/tb_ads1115_reader.sv`
  - `sim/tb_pid_fixed_precision.sv`
  - `sim/tb_tb6612_pwm_latched.sv`
  - `PID_swing.xpr`
- 验证：
  - 9 个 XSim 测试全部通过。
  - 新增 PID 精度测试覆盖正负小数截断和最终统一除法。
  - 新增 TB6612 测试覆盖 PWM 周期中途改方向不会立即生效。
  - Vivado 2018.3 综合、实现和比特流生成通过。
  - 路由后时序：WNS `+0.255 ns`，TNS `0.000 ns`，
    WHS `+0.069 ns`，THS `0.000 ns`，无时序失败端点。
- 与参考工程仍有意的差异：
  - 当前增益可在 OLED/电位器中实时调整，因此保留运行时 Q8 参数输入；
    参考工程使用编译期整数系数。
  - 当前仍使用两颗 ADS1115；参考工程使用 GD32 采样后通过 1 Mbaud UART
    将原始 12 位码值送入 FPGA。
  - 当前最终电机命令仍限制在 ±35%；参考工程的位置式 PID 输出上限为
    ±100%。这是后续实机需要判断的剩余差异。
- 比特流：
  - `pid_swing_hw_top_reference_control.bit`
  - SHA-256：
    `1C322B6DB67210DF6B3FD52E71F7E75D282C94885BC9502049466F933C1BF5AD`
- 撤回方法：在 `D:\FPGA\FPGA_PID` 执行
  `git restore --source=f459084 -- PID_swing/rtl PID_swing/sim PID_swing/PID_swing.xpr`
  可恢复到本次修改前源码；必要时删除本记录新增的两个测试文件。

### 2026-11-01 / 按附件参数优化并验证左右输出对称性

- 状态：已完成
- 参数来源：`C:\Users\ASUS\Desktop\相关参数.txt`
- 默认参数更新：
  - 角度环：`Kp=0.32`、`Ki=0.01`、`Kd=0.42`
  - 位置环：`Kp=0.40`、`Ki=0.00`、`Kd=4.00`
  - 中心角度：`2067`
  - 起摆力度：`45`
  - 启动时间：`100 ms`
- 针对左侧响应偏慢的处理：
  - 检查 PID 最终截断、饱和、正负方向 PWM 映射和占空比计算。
  - 新增等幅正负命令测试，`+25%` 与 `-25%` 必须产生完全相同的
    25% 占空比，只允许方向位不同。
  - PID 正负小数已统一向零截断，不再存在负数先产生 `-1` 而正数产生
    `0` 的方向偏置。
  - 角度 PID 内部输出限幅与最终 `MOTOR_COMMAND_LIMIT=35` 保持一致。
- 涉及文件：
  - `rtl/pendulum_ui.v`
  - `rtl/pid_swing_top.v`
  - `sim/tb_stm32_menu_restore.sv`
  - `sim/tb_tb6612_pwm_latched.sv`
- 验证：
  - 9 个 XSim 测试全部通过。
  - Vivado 2018.3 综合、实现和比特流生成通过。
  - 路由后时序：WNS `+0.140 ns`，TNS `0.000 ns`，
    WHS `+0.162 ns`，THS `0.000 ns`，无时序失败端点。
- 说明：仿真已证明代码输出的正反向 PWM 完全对称。如果实物仍表现为
  左转明显慢于右转，下一步应测量相同 PWM 下的正反向稳态转速或编码器
  速度；剩余差异来自电机、齿轮、静摩擦、机械负载或角度中心标定，不应
  继续通过给 PID 正方向加偏置来处理。
- 比特流：
  - `pid_swing_hw_top_params_optimized.bit`
  - SHA-256：
    `F76A38F52FA12425C262E10225367F15A05958312504237704617598637D60F2`
- 撤回方法：恢复上一个
  `pid_swing_hw_top_reference_control.bit`，源码按本次日志中的三个文件
  反向应用。

### 2026-11-01 / 直立启动直接平衡与单 Kp 调参配置

- 状态：已完成
- 修改原因：装置在上电启动时如果摆杆已经处于直立窗口，不应先执行固定
  起摆踢击；应确认摆杆稳定在直立窗口后直接进入 PID 平衡模式。
- 启动状态机修改：
  - K1 运行请求到来时若摆杆已在直立窗口，立即跳过 3 秒启动保持，
    进入 `S_START_CHECK`。
  - 非直立启动仍执行原来的 3 秒保持，再进入 `S_START_CHECK`。
  - 使用两个相隔 40 ms 的直立采样确认位置。
  - 两次均位于 1700～2300 捕获窗口时，直接进入 `MODE_BALANCE`，
    不执行起摆脉冲。
  - 任一确认样本不在窗口时，退回原有 `S_L3/S_L4` 起摆流程。
- 默认调参值修改：
  - 角度环 `Kp=0.25`，`Ki=0`，`Kd=0`。
  - 位置环 `Kp=0`，`Ki=0`，`Kd=0`。
  - 中心角度 `2067`，起摆力度 `45`，启动时间 `100 ms`。
- 涉及文件：
  - `rtl/pendulum_ui.v`
  - `rtl/swing_balance_ctrl.v`
  - `sim/tb_pid_swing_top.sv`
  - `sim/tb_stm32_menu_restore.sv`
  - `sim/tb_swing_startup_balance.sv`
- 验证：
  - 10 个 XSim 测试全部通过。
  - 新增测试验证：直立启动经 40 ms 确认后直接进入平衡；悬挂启动仍
    回到原起摆踢击流程。
  - Vivado 2018.3 综合、实现和比特流生成通过。
  - 路由后时序：WNS `+0.379 ns`，TNS `0.000 ns`，
    WHS `+0.029 ns`，THS `0.000 ns`，无时序失败端点。
- 比特流：
  - `pid_swing_hw_top_upright_start_kp025.bit`
  - SHA-256：
    `213F9B1FC6B30355FAAC465E49C9DA282F75ACB6088F6EA9B48743C6E54E5042`
- 说明：直立启动仍需要先按 K1 发出运行请求；K1 是电机运行和 PID
  使能的人为确认，不会在上电后自动驱动电机。

### 2026-11-02 / K1 直立中心自校准与起摆捕获窗口跟随

- 状态：已完成
- 修改原因：
  - 传感器每次上电或重新摆放后的实际直立码值不一定等于固定中心
    `2067`。固定中心会造成左右相同机械倾角的 PID 误差大小不同。
  - 固定捕获区间 `1700～2300` 与实际直立码值偏离后，摆杆经过直立区
    也可能无法进入 PID；40 ms 峰值判断还可能漏掉快速穿越。
- K1 启动行为：
  - K1 从停止变为运行时，如果 ADC 已就绪且当前角度位于已存中心
    正负 `300` 码值内，则把当前角度锁存为新的 `CENTER`。
  - 悬挂位置通常远离直立中心，因此悬挂启动不会覆盖已校准中心。
  - OLED 中的 `C` 会立即显示本次采样得到的新中心；后续 PID 角度误差
    随即使用该中心，左右对称倾角会产生对称的误差。
- 起摆状态机行为：
  - 直立捕获窗口改为 `CENTER` 正负 `300` 码值，不再使用固定绝对
    的 `1700/2300`。
  - 左右摆幅峰值判断同样跟随 `CENTER`。
  - 平衡失效窗口改为 `CENTER` 正负 `1000` 码值。
  - 判断状态增加连续两个 1 ms 样本快速捕获，同时保留原来的两个
    40 ms 样本捕获，降低摆杆快速经过直立区而完全漏检的概率。
- 未改动参数：
  - 角度环仍为 `Kp=0.25`、`Ki=0`、`Kd=0`。
  - 位置环仍全部为 `0`。
  - 中心默认值仍为 `2067`，起摆力度 `45`，起摆时间 `100 ms`。
- 涉及文件：
  - `rtl/pendulum_ui.v`
  - `rtl/swing_balance_ctrl.v`
  - `rtl/pid_swing_top.v`
  - `sim/tb_stm32_menu_restore.sv`
  - `sim/tb_swing_startup_balance.sv`
  - `sim/tb_pid_swing_top.sv`
- 验证：
  - 10 个 XSim 测试全部通过。
  - 新增验证 K1 直立中心捕获、悬挂启动不覆盖中心、中心改变后捕获
    窗口跟随，以及原固定窗口之外的直立位置可以进入平衡。
  - Vivado 2018.3 综合、布局布线、DRC 和位流生成通过。
  - 路由后时序：WNS `+0.182 ns`，TNS `0.000 ns`，
    WHS `+0.095 ns`，THS `0.000 ns`，无时序失败端点。
  - DRC：`0 Error`。
- 位流：
  - `pid_swing_hw_top_upright_autocenter.bit`
  - SHA-256：
    `58130A0473DD74B28AA29BF34598E34DA4DBAFF7ADDDA95A29B56BB7541AB17B`
- 实测操作：
  - 上电后先用手把摆杆扶到希望作为倒立零点并尽量静止的位置，再按
    K1。OLED 的 `C` 会变成当前传感器码值。
  - 如果希望继续使用手动中心，可在 `pid_swing_top.v` 的 `u_ui`
    参数中设置 `AUTO_CENTER_ON_START(0)`。
  - 中心校准后若某一方向仍然明显更迟钝，应测量该方向电机启动死区；
    不要再通过给 Kp 或某一方向增加隐藏偏置来掩盖机械不对称。
- 撤回：
  - 上板行为可回退到上一个已验证位流：
    `pid_swing_hw_top_upright_start_kp025.bit`
    SHA-256
    `213F9B1FC6B30355FAAC465E49C9DA282F75ACB6088F6EA9B48743C6E54E5042`。
  - 若只关闭自动中心，在 `pid_swing_top.v` 的 `u_ui` 实例化参数中
    增加 `AUTO_CENTER_ON_START(0)`。
  - 原 `1700/2300` 是不对称固定窗口，不建议重新使用；如需恢复旧
    上板控制效果，直接回退上述旧位流。
