# P2 机械搬迁：规则、复现与验证（2026-10-26）

把 `legacy/` 中的 1:1 项搬到目标位置，不改行为。决策见 [decision.md](../../../../decision.md) Decision-20261026-08。

## 提交

| 提交 | 内容 |
|---|---|
| `2e2665e` | `move.py --move`：323 个文件 `git mv`（全部 R100，内容一字未改），删除 5 个旧构建文件。本提交单独无法编译 |
| `25ffea5` | `move.py --rewrite`：118 个文件只改 `#include`（157 处）与头文件保护宏（57 个）所在的行 |
| `1152e68` | 手工适配：各层 CMakeLists 改为新路径，工具与 CI 配置更新 |

搬迁单独成为一个提交，是为了让 `git log --follow` 逐提交都能识别改名。只有一行 include 的文件（`basic_math.cpp`、`fsm.cpp`）改写后，合在一起看相似度低于 50%；分开提交后，仍能追溯到迁移源最早的提交。

## 规则

[move.py](move.py) 按规则生成 [移动表.tsv](移动表.tsv)，脚本里有规则的定义。

- **去向**：

  | 迁移前 | 搬迁后 |
  |---|---|
  | `Platform/Interface/*.hpp` | `src/hal/` |
  | `Platform/Interface/Detail/*.h` | `src/hal/detail/` |
  | `Libraries/Algorithm/*` | `src/lib/`（保留 `alg_*` 子目录名） |
  | `Libraries/Component/` | `src/lib/component/` |
  | `Libraries/Device/` | `src/driver/` |
  | `Libraries/Protocol/` | `src/driver/protocol/` |
  | `Application/Task/` 的任务外壳 | `app/tasks/` |
  | `ChassisController` | `app/chassis/` |
  | `RemoteReceiver` | `app/remote/` |
  | `Module/GimbalModule` | `app/gimbal/` |
  | DM_MC02 的 CubeMX 生成物 | `platform/st/stm32h7/board_dm/cubemx/` |
  | `Port` | `platform/st/stm32h7/board_dm/port/` |

- **文件名**：
  - snake_case，如 `Bmi088Accel.cpp` → `bmi088_accel.cpp`，`DjiMotorModelMap.hpp` → `dji_motor_model_map.hpp`；
  - C++ 头统一 `.hpp`，迁移源 Algorithm 下的 `Pid.h` 等改为 `pid.hpp`；C 头保持 `.h`；
  - 被 CubeMX 生成代码按名字 include 的 `port/Init.h` 保持原名。
- **include**：
  - 按搬迁前的查找顺序（同目录 → `legacy/` → `Platform/Interface` → `test/host` → Port → PortKit）解析到具体文件，再写成相对层根目录的路径：`"hal/can.hpp"`、`"driver/bmi088/bmi088.hpp"`、`"tasks/imu_task.hpp"`、`"support/mock_motor.hpp"`；
  - 影子头相对影子根；port 内部用同目录文件名；
  - 指向 P2.5 才搬的文件，保持 `"Application/..."` 写法；
  - 生成区不改。
- **头文件保护宏**：由新的 include 路径大写得到，如 `"hal/detail/can.h"` → `HAL_DETAIL_CAN_H`；port 头按 `platform/` 下的路径得到。

## 复现

在 P1 的最后一个提交（`70d4915`）上执行：

```sh
python docs/ai/decisions/2026-10-26-P2-机械搬迁/move.py --move      # 然后提交
python docs/ai/decisions/2026-10-26-P2-机械搬迁/move.py --rewrite   # 以 HEAD~1 的文件清单重算移动表
```

CMake、工具与 CI 的改动是手工完成的，见提交 `1152e68`。

## 验证

| 项 | 结果 |
|---|---|
| 符号集合（与 P1 存档的 `symbols-*.txt` 比较） | Debug 1426 个、Release 1038 个，一致；Windows xPack GCC 15.2.1 与 Linux Arm GNU 15.2.Rel1 各验证一次 |
| 编译选项 | 112 个固件源文件，除 include 目录外与 P1 逐项相同 |
| 依赖图 | target 不变；框架与已搬迁的 app target 不再依赖 `legacy_include`，少了 21 条边 |
| 宿主测试 | 14/14（MSYS2 g++ 16.2、WSL g++ 11.4），零告警 |
| 命名、格式、架构检查 | 命名 0 处；格式 131 个文件通过；文本扫描与两张依赖图检查通过 |
| 可追溯性 | 改名识别 321/323（另 2 个是只有一行的文件，逐提交追溯仍然完整）；抽查的文件都能 `git log --follow` 到迁移源最早的提交 |

## 留给 P2.5 的事

- 搬迁剩余文件：`Application/{Entry,Imu,Task/TaskManager}`、`Platform/PortKit`、`Port/UsbCdc.{c,h}`。
- 删除 `legacy_include`、`legacy_library()`，以及 `.clang-tidy` 头文件过滤中的 `legacy/Application` 一段（原定 P2，推迟原因见 Decision-20261026-08 第 6 点）。
- `UsbCdc.h`、`Entry.h` 与 `Init.h` 一样，被生成代码按名字 include，保持原名。
