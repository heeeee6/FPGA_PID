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
