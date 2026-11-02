# FPGA 旋转倒立摆控制系统

基于 SmartZynq V1.3B（XC7Z020）的旋转倒立摆自动起摆与双环 PID 控制工程。
项目使用 Verilog 实现传感器采集、起摆状态机、角度/位置双环 PID、OLED 参数
界面和 TB6612 电机驱动，可在运行时通过按键与电位器完成主要调试。

## 主要功能

- 左右交替脉冲自动起摆，进入直立捕获窗口后切换到双环 PID。
- K1 启动时自动采样并锁存当前直立中心，降低传感器零点偏移的影响。
- 捕获窗口跟随中心：`CENTER ± 300`；落摆保护窗口：`CENTER ± 1000`。
- 1 ms 连续样本快速捕获与 40 ms 摆幅峰值判断并行工作。
- 角度环 5 ms 更新，位置环 50 ms 更新。
- 20 kHz PWM，最终电机命令限幅 `±35%`。
- 四页 OLED 界面，支持中心、起摆力度、起摆时间、PWM 偏移及双环参数调节。
- 双 ADS1115 采集，OLED 使用独立 I2C 总线。

## 硬件组成

- 主控：SmartZynq V1.3B，`xc7z020clg484-1`
- 时钟：板载 50 MHz PL 时钟
- 角度采集：ADS1115 #1，地址 `0x48`，AIN0 接角度传感器
- 电位器采集：ADS1115 #2，地址 `0x49`，AIN0～AIN3 接 RP1～RP4
- 显示：SSD1306 OLED，地址 `0x3C`
- 电机驱动：TB6612
- 反馈：正交编码器 A/B 相
- 输入：K1～K4 低电平有效按键

### 数字接口

| 信号 | J6 | FPGA 引脚 | 连接 |
|---|---:|---|---|
| `motor_pwm` | 1 | U22 | TB6612 PWMA |
| `motor_in1` | 2 | T22 | TB6612 AIN1 |
| `motor_in2` | 3 | W22 | TB6612 AIN2 |
| `motor_stby` | 4 | V22 | TB6612 STBY |
| `motor_enc_a` | 5 | Y21 | 编码器 A |
| `motor_enc_b` | 6 | Y20 | 编码器 B |
| `oled_scl` | 7 | AB22 | OLED SCL |
| `oled_sda` | 8 | AA22 | OLED SDA |
| `rst_n` | 9 | AB21 | 低电平复位 |
| `run_enable` | 10 | AA21 | 高电平允许运行 |
| `ads_scl` | 11 | AB19 | 两颗 ADS1115 SCL |
| `ads_sda` | 12 | AB20 | 两颗 ADS1115 SDA |
| K1 | 13 | AA19 | 按键到 GND |
| K2 | 14 | Y19 | 按键到 GND |
| K3 | 15 | AB16 | 按键到 GND |
| K4 | 16 | AA16 | 按键到 GND |

J6 的 VADJ 必须设置为 **3.3 V**。ADS1115、OLED、编码器和全部数字信号均使用
3.3 V 逻辑，并与 FPGA 共地。

## 按键操作

### PAGE0～PAGE2

| 按键 | 功能 |
|---|---|
| K1 | 启动/停止 |
| K2 | 位置目标增加 408 |
| K3 | 位置目标减少 408 |
| K4 | 下一页 |

PAGE1 中 RP1、RP2、RP3 分别调节角度环 Kp、Ki、Kd；PAGE2 中相同旋钮调节
位置环。旋钮需先回到当前参数附近才会接管控制。

### PAGE3

| 按键 | 浏览状态 | 编辑状态 |
|---|---|---|
| K1 | 启动/停止 | 当前参数减小 |
| K2 | 下一项 | 当前参数增大 |
| K3 | 进入编辑 | 完成编辑 |
| K4 | 返回 PAGE0 | 返回 PAGE0 |

中心角度和起摆时间支持长按 K1/K2 连续调节。

## 默认参数

```text
角度环：Kp=0.25, Ki=0, Kd=0
位置环：Kp=0, Ki=0, Kd=0
CENTER=2067
起摆力度=45%
起摆时间=100 ms
PWM 偏移=0
```

这些默认值是调试起点，不是对所有机械结构通用的整定结果。

## 工程结构

```text
rtl/         Verilog RTL
sim/         XSim 自检测试
constr/      Vivado 引脚与时序约束
scripts/     工程更新、仿真、实现和位流脚本
```

主要顶层：

- `rtl/pid_swing_hw_top.v`：物理板顶层
- `rtl/pid_swing_top.v`：控制与调度顶层
- `rtl/swing_balance_ctrl.v`：起摆与平衡状态机
- `rtl/pid_fixed.v`：Q8.8 定点 PID
- `rtl/pendulum_ui.v`：按键、菜单与参数管理
- `rtl/ads1115_reader.v`：双 ADS1115 采集

## 构建与仿真

需要 Vivado 2018.3，在工程目录执行：

```powershell
vivado -mode batch -source scripts/update_project.tcl
./scripts/run_sim.ps1
vivado -mode batch -source scripts/check_impl_route.tcl
vivado -mode batch -source scripts/write_bitstream.tcl
```

已生成的主要位流：

```text
pid_swing_hw_top_upright_autocenter.bit
```

当前验证结果：

- 10 个 XSim 测试通过
- 路由后 WNS `+0.182 ns`
- 路由后 WHS `+0.095 ns`
- DRC `0 Error`

## 调试提示

- 上电后先将摆杆放在期望的直立位置，再按 K1 完成中心采样。
- 若机构静摩擦或联轴器间隙较大，应先检查正反方向启动死区，再调整 PID。
- 如果 STBY 已在外部固定接高，J6-4 可不连接，但 FPGA 将无法通过软件进入待机。
- 调试起摆和闭环参数时必须保留电机电源急停。
- 机械中心、传感器安装和电机方向应在低 PWM 下完成校准。

详细修改记录见 [MODIFICATION_LOG.md](MODIFICATION_LOG.md)。
