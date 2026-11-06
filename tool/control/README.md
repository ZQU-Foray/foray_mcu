# tool/control：控制律的设计与辨识工具（本机）

板上不解 Riccati 方程、不做辨识：K 与模型参数都在电脑上算好，填进
`config/profiles/<profile>/params.hpp`。这些工具需要 numpy、scipy，CI 只对它们做
ruff 与 black，**自检要在本机跑**。

## 环境

已装好的虚拟环境：`~/.local/venvs/control`（Python 3.12、numpy 2.5.4、scipy 1.18.1）。
重建：

```sh
python3 -m venv ~/.local/venvs/control
~/.local/venvs/control/bin/pip install -r tool/control/requirements.txt
```

## lqr_design.py：模型 + Q、R → K

```sh
# 直接给 Q、R（对角元，个数与状态、输入数一致）
~/.local/venvs/control/bin/python tool/control/lqr_design.py double-integrator --dt 0.001 --q 2.56e6,0 --r 1

# 双积分器也可以按闭环 ω_n、ζ 反算 Q（q_θ = ω_n⁴·r，q_ω = (4ζ² − 2)·ω_n²·r，ζ ≥ 0.707）
~/.local/venvs/control/bin/python tool/control/lqr_design.py double-integrator --dt 0.001 --bandwidth 40 --damping 0.707

# 自检（复现设计表的四行、与连续公式对账、DARE 残差、两条命令一致）
~/.local/venvs/control/bin/python tool/control/lqr_design.py --selftest
```

做的是：零阶保持离散化（`scipy.linalg.expm`）→ 解离散 DARE（`solve_discrete_are`）→
K = (R + Bdᵀ·P·Bd)⁻¹·Bdᵀ·P·Ad。Q 的对角元可以有个别为 0（只罚某些状态），R 的每个
对角元必须大于 0（输入不能没有代价）。输出 K（按状态名逐个打印）、闭环极点（z 与
s = ln(z)/dt）、闭环 ω_n 与 ζ（ζ > 1 时按两个实极点换算，仍是设计时给的阻尼），
以及一段能直接粘贴的 C++：

```cpp
// tool/control/lqr_design.py double-integrator --dt 0.001 --q 2.56e+06,0 --r 1
// 闭环 ω_n 40 rad/s、ζ 0.7071
1555.38F, // angle_gain
55.7742F, // rate_gain
```

注释里的命令能原样重跑，以后改 K 照着重跑即可（生成的结果要可复现）。
`double-integrator` 是加了 ESO 补偿以后的单轴（`θ̇ = ω`、`ω̇ = u₀`），**K 与 J 无关**：
J 测准了只改 b₀，K 不用重算。

云台只有两个状态，工具把 |s| 最小的两个极点当作一对来报 ω_n、ζ；以后轮腿的状态
多了，这两个可能不是同一个模态（一个实极点加一个复极点对里的一个），届时要改成
按模态配对或逐个列出。

## models.py：被控对象

每个模型一个函数，返回连续的 A、B 与状态名、输入名；函数名就是 `lqr_design.py`
的模型参数。现在只有 `double_integrator()`（云台、大 yaw）。以后加轮腿时在这里写
`wheel_leg(leg_length)`（按平衡点线性化的 6 状态、2 输入），`lqr_design.py` 的
`--schedule leg_length=0.12:0.30:20` 与生成的 `lqr_schedule.hpp` 再做。

## identify_axis.py：录到的数据 → 模型参数

辨识 J（→ b₀ = 1/J）、库仑摩擦 Fc、重力 G（pitch）或常值力矩 c（yaw）与延迟。
回归式（τ 是下发的总力矩，含重力前馈；两个轴分开做）：

```text
τ[k−d] = J·ω̇[k] + B·ω[k] + Fc·tanh(ω/ωs) + G·cos θ[k]     ← pitch
τ[k−d] = J·ω̇[k] + B·ω[k] + Fc·tanh(ω/ωs) + c               ← yaw
```

- ω̇ 用 Savitzky–Golay 求导（窗口 21 点、3 次），ω 用同一窗口平滑，ωs = 0.01 rad/s；
  每段按 `time_us` 换成均匀采样（dt 取中位数）再算；
- 延迟 d 在 0–15 拍里取 R² 最大的（`--max-delay` 可改），并在峰值附近抛物线插值、报成小数拍；
  中心差分的时间对齐会多算约半拍，实物上按需减 0.5。峰值落在上限上时会提示加大上限
  （步兵 yaw 经同步带约 9 拍，pitch 约 5 拍）；
- 没有可用样本（比如录制时云台没在出力）时直接说明原因并退出；
- 只用出力模式（HOLD / TRACK / MANUAL）的样本，进入出力后的前 0.2 s 不要；
- pitch 不加常数项（`cos θ` 在小范围里几乎是常数，和常数项分不开）；
- B 估不准（粘滞摩擦与库仑摩擦分不开），只作参考、不用它。

输入是 EmberProbe 按切片导出的 CSV：每段一个目录（里面是切片）或一个 CSV。
切片按 `index` 列拼起来、检查没有缺口；时间列是 `time_us`；除 `index`、`time_us`、
`mode` 外还要有 `{轴}_rad`、`{轴}_rate`、`{轴}_torque_newton_meter`、
`{轴}_feedback_torque_newton_meter` 四列（允许前缀，如
`g_gimbal_capture.pitch_rad`）。**第一次导出后要核对实际的列名。**

```sh
~/.local/venvs/control/bin/python tool/control/identify_axis.py build/capture/pitch_sweep_2hz --axis pitch
~/.local/venvs/control/bin/python tool/control/identify_axis.py build/capture/pitch_sweep_* --axis pitch   # 多段合起来拟合
~/.local/venvs/control/bin/python tool/control/identify_axis.py --simulate --axis pitch
~/.local/venvs/control/bin/python tool/control/identify_axis.py ... --axis pitch --gravity-feedforward 0.4
```

输出：J、b₀、B（标"不可靠"）、Fc、G 或 c、延迟（拍数与 ms）、R²、残差 RMS；电机
反馈力矩对下发力矩的比例与延迟；建议的 b₀ 与 ω₀ 上限 `0.5/(延迟 + 0.5 ms)`。
给了 `--gravity-feedforward` 时，G 与它相差超过 10% 会提示先改 `params.hpp`。

`--simulate` 自己生成闭环仿真数据（PI 角速度环 + P 角度环、0.15 rad 正弦目标
1/2/4 Hz、力矩延迟 2 拍、陀螺噪声 σ 0.01、角度噪声 1e-4），跑同一条回归，要求
J ±5%、G ±5%（pitch；yaw 的 c ±15%）、Fc ±15%、延迟（减掉中心差分的半拍后）
±0.5 拍。真值写在脚本开头。

## 提交前

```sh
~/.local/venvs/llvm18/bin/ruff check --isolated tool/control
~/.local/venvs/llvm18/bin/black --check tool/control
git update-index --chmod=+x tool/control/*.py   # 新脚本；exFAT 上文件权限不可靠
```

本机在 exFAT 上，所有文件都显示可执行，ruff 可能报 EXE002（假报错）；要看 CI 的
真实结果，在 `git archive` 出来的副本里跑。
