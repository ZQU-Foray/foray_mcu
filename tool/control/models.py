"""被控对象模型库：给 lqr_design.py 与 identify_axis.py 用。

每个模型是一个函数，返回 Model（连续的 A、B、状态名、输入名）。函数名就是
lqr_design.py 的模型参数（把下划线换成连字符也认）。
以后加轮腿时在这里写 wheel_leg(leg_length)，A、B 按平衡点线性化。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Model:
    """连续的线性模型 ẋ = A·x + B·u。A 是 States × States，B 是 States × Inputs。"""

    a: np.ndarray
    b: np.ndarray
    states: tuple[str, ...]
    inputs: tuple[str, ...]

    def __post_init__(self) -> None:
        if self.a.shape != (len(self.states), len(self.states)):
            raise ValueError("A 的形状与状态名个数不符")
        if self.b.shape != (len(self.states), len(self.inputs)):
            raise ValueError("B 的形状与状态名、输入名个数不符")


def double_integrator() -> Model:
    """加了 ESO 补偿以后的单轴：θ̇ = ω，ω̇ = u₀（角加速度指令）。云台、大 yaw 都用它。"""
    a = np.array([[0.0, 1.0], [0.0, 0.0]])
    b = np.array([[0.0], [1.0]])
    return Model(a, b, states=("angle", "rate"), inputs=("acceleration",))


MODELS = {"double-integrator": double_integrator}
