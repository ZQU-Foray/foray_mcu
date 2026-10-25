### 通用代码库，一般情况下适用于h723和f407

### 目录结构

```text
Libraries/
├─ Algorithm/            纯算法与数学工具（不包含架构与产品语义）
│  ├─ alg_controller/    PID 控制算法
│  ├─ alg_crc/           CRC 校验
│  ├─ alg_estimate/      状态估计（KalmanFilter、QuaternionEkf、Vqf、ESO、IMU 轴映射）
│  ├─ alg_filter/        数字滤波器（Filter.h、ButterworthLowPass.hpp）
│  ├─ alg_fsm/           有限状态机
│  ├─ alg_kinematics/    底盘运动学与里程计
│  ├─ alg_math/          基础数学（Vector3/Quaternion/Matrix/Matrix3/BasicMath）
│  └─ alg_slope/         斜坡规划
├─ Component/            通用组件（架构件：组合算法、定义模式与策略，不含产品参数）
│  └─ AxisController.hpp 轴级执行器骨架（模式/串级/限幅链/失败策略/诊断）
├─ Device/               设备驱动与设备抽象（BMI088、电机组、蜂鸣器、WS2812）
├─ Protocol/             协议编解码（DJI 电调、SBUS）
└─ README.md
```

### 层次约定

- **Algorithm 只放纯算法**：数学模型、滤波器、状态机、协议无关的计算；不定义产品语义。
- **Component 放架构件**：把算法组合成可复用的结构（例如轴级执行器的模式、限幅链、失败策略、诊断）。
  架构件依赖算法件（`Component → Algorithm`），反之不成立。
- **产品参数不进 Libraries**：默认模式、失败策略取值、目标钳位范围、增益与限幅数值由
  `Application/Task/<模块>` 的薄壳提供，见 `Application/Task/1_chassis`、`Application/Task/2_gimbal`。

### 状态估计算法

姿态解算与陀螺仪零偏估计算法位于 `Algorithm/alg_estimate/`（`KalmanFilter`、
`QuaternionEkf`、`Vqf` 等），由 `Application/Imu` 侧组合使用；历史版本存档在
`Tests/backups/`，仅供参考。

### 热路径公共组件（只在这里实现一次，不得各算法自行复制）

| 组件                                                       | 位置                                          | 适用场合                                                                                                                                                                                |
| ---------------------------------------------------------- | --------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `alg_math::Matrix3`                                        | `Algorithm/alg_math/Matrix3.h`                | 3×3 **双精度**矩阵（扁平存储、直接索引）。面向每个样本都要跑的热路径；通用可读性优先的场合仍用 `Matrix<Rows, Columns>`（float + `At()`）                                                |
| `alg_filter::ButterworthLowPass<Channels>`                 | `Algorithm/alg_filter/ButterworthLowPass.hpp` | 二阶 Butterworth 低通：时间常数参数化、**double 状态**、多通道、带"先按 τ 秒平均再取稳态初值"的初始化语义，并提供 `InternalState` 读写（检查点/离线逐段续算，恢复时校验快照与配置同源） |
| `alg_math::Square` / `Limit` / `LimitInPlace` / `RadToDeg` | `Algorithm/alg_math/BasicMath.h`              | 标量工具                                                                                                                                                                                |

约定与现状：

- `alg_filter::ButterworthFilter`（`Filter.h`）是按**截止频率**参数化、`float` 状态、标量、
  从零初值的旧版本，当前**无任何消费者**；它与 `ButterworthLowPass` 描述同一个双线性变换
  （参数换算 `f_c = √2 / (2π·τ)`）。新增代码一律用 `ButterworthLowPass`，旧类待统一清理。
- 低通**状态与系数必须保持 double**：float 状态在长时运行下会出现数值问题；
  `ButterworthLowPass` 已把这条约束固化在组件内部，调用方不再各自踩坑。
- `Vqf`（`Algorithm/alg_estimate/Vqf.*`）只保留算法本体（更新步、静止判定、零偏估计），
  其低通与 3×3 矩阵运算全部复用上述公共组件，不在算法内私建数学实现。
  对外接口已覆盖上游 6 维通路的全部实用入口，含调试/回放需要的三项：
  **零偏注入** `SetBiasEstimate`、**静止判据可观测量** `GetRestObservables`
  （两路偏离/阈值比值，1.0 即判据边界）、**完整状态读写** `GetInternalState`/`SetInternalState`
  （快照自校验配置同源，上游不校验）。

### 第三方算法来源

- `Algorithm/alg_estimate/Vqf.{hpp,cpp}`：移植自 MIT 许可的上游参考实现
  <https://github.com/dlaidig/vqf>（D. Laidig, T. Seel, *Information Fusion* 2023, 91, 187–204，
  doi:10.1016/j.inffus.2022.10.014）。上游为 MIT；本仓库按其 6 维（陀螺+加速度计）
  通路实现，未引入磁力计相关部分。
