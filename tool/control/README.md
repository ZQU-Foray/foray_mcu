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
K = (R + Bdᵀ·P·Bd)⁻¹·Bdᵀ·P·Ad。输出 K（按状态名逐个打印）、闭环极点（z 与
s = ln(z)/dt）、闭环 ω_n 与 ζ，以及一段能直接粘贴的 C++：

```cpp
// tool/control/lqr_design.py double-integrator --dt 0.001 --q 2.56e+06,0 --r 1
// 闭环 ω_n 40 rad/s、ζ 0.7071
1555.38F, // angle_gain
55.7742F, // rate_gain
```

注释里的命令能原样重跑，以后改 K 照着重跑即可（生成的结果要可复现）。
`double-integrator` 是加了 ESO 补偿以后的单轴（`θ̇ = ω`、`ω̇ = u₀`），**K 与 J 无关**：
J 测准了只改 b₀，K 不用重算。

## models.py：被控对象

每个模型一个函数，返回连续的 A、B 与状态名、输入名；函数名就是 `lqr_design.py`
的模型参数。现在只有 `double_integrator()`（云台、大 yaw）。以后加轮腿时在这里写
`wheel_leg(leg_length)`（按平衡点线性化的 6 状态、2 输入），`lqr_design.py` 的
`--schedule leg_length=0.12:0.30:20` 与生成的 `lqr_schedule.hpp` 再做。

## identify_axis.py：录到的数据 → 模型参数

`tool/control/identify_axis.py` 读 EmberProbe 导出的录制切片，辨识 J、b₀、库仑摩擦、
重力与延迟，输出建议的 b₀、ω₀ 上限。用法：

```sh
~/.local/venvs/control/bin/python tool/control/identify_axis.py build/capture/pitch_sweep_2hz --axis pitch
~/.local/venvs/control/bin/python tool/control/identify_axis.py --simulate --axis pitch
```

## 提交前

```sh
~/.local/venvs/llvm18/bin/ruff check --isolated tool/control
~/.local/venvs/llvm18/bin/black --check tool/control
git update-index --chmod=+x tool/control/*.py   # 新脚本；exFAT 上文件权限不可靠
```

本机在 exFAT 上，所有文件都显示可执行，ruff 可能报 EXE002（假报错）；要看 CI 的
真实结果，在 `git archive` 出来的副本里跑。
