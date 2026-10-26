# P0.5 命名统一：过程、复现与验证（2026-10-26）

把 `legacy/` 与 `test/host/` 中的 C++ 标识符按 ROS 2 风格（根目录 `.clang-tidy`）统一改名。决策见 [decision.md](../../../../decision.md) Decision-20261026-05。

## 范围

- 改名：`legacy/Application`、`legacy/Libraries`、`legacy/Platform/Interface/*.hpp`（C++ 包装层）、`test/host` 中的 C++ 代码。
- 不改：C ABI（`Platform/Interface/Detail/*.h`、`Port/`、`PortKit/`、`Application_Init` 等"模块_函数"）、CMSIS-RTOS2 的 `os*`、CubeMX 生成区与厂商代码、文件名（P2 搬迁时改）、Markdown 历史文档。

## 提交

| 提交 | 内容 |
|---|---|
| `6e9fa0a` | 新增 `tool/ci/naming.py`；`.clang-tidy` 微调（头文件过滤、C ABI 不改名） |
| `2b3fbf8` | 改名：102 个文件，只做标识符替换，行数不变 |
| `0c51793` | 改名后重新格式化：48 个文件，去掉空白后与改名提交逐文件一致 |

后两个提交登记在 `.git-blame-ignore-revs`。

## 复现

在 `6e9fa0a` 上执行（Linux 或 WSL；Windows 下 MSYS2 的 libstdc++ 16 与 clang 18 不兼容，宿主测试会有 1 个单元解析失败）：

```sh
pip install clang-tidy==18.1.8 pyyaml==6.0.2
src=legacy/Platform/STM32H7/DM_MC02
for cfg in Debug Release; do
  cmake -S $src -B build/tidy/$cfg -G Ninja -DCMAKE_BUILD_TYPE=$cfg \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/$src/cmake/gcc-arm-none-eabi.cmake -DARM_GCC_ROOT=<arm-none-eabi-gcc 根目录>
done
cmake -S test/host -B build/tidy/host -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

python tool/ci/naming.py --fix -p build/tidy/Debug -p build/tidy/Release -p build/tidy/host \
  --report 改名表.tsv                                      # 1. 自动改名
bash docs/ai/decisions/2026-10-26-P0.5-改名/manual-fixups.sh   # 2. 手工修正
python tool/ci/format.py --fix                              # 3. 重新格式化（单独提交）
```

第 1 步的输出与本目录的 [改名表.tsv](改名表.tsv) 逐字节一致（两次独立运行已核对）。第 2 步的脚本要读取本目录的改名表，所以要从仓库根执行。

三个编译数据库缺一不可：

- Debug 打开 `IMU_ENABLE_DIAGNOSTICS`，诊断代码只在这个配置下可见；
- Release 走 `#else` 分支（关掉诊断时的空实现）；
- 宿主测试覆盖测试代码与测试替身。

被 `#if` 关掉的代码对 clang-tidy 不可见，以后新增编译开关时，CI 的 `lint-naming` 也要覆盖对应配置。

## 自动改名的结果

`naming.py` 汇总 58 个翻译单元和 2 个孤立头文件的结果：命名不合规 2190 处，写回 9383 处修改，涉及 101 个文件。孤立头文件是 `ExtendedStateObserver.hpp` 和 `UsbCdc.hpp`，没有任何翻译单元包含它们。

| 类别 | 处数 | 规则 |
|---|---:|---|
| 方法 534、函数 88 | 622 | `snake_case` |
| 公开成员 412、私有成员 211、受保护成员 12 | 635 | `snake_case`，私有与受保护成员加尾下划线 |
| 编译期常量 241、枚举值 185 | 426 | `UPPER_SNAKE_CASE` |
| 参数 185、局部常量 133、局部变量 86 | 404 | `snake_case` |
| 全局变量 99、全局常量 4 | 103 | 全局变量加 `g_` 前缀；全局常量为 `UPPER_SNAKE_CASE` |

逐项清单见 [改名表.tsv](改名表.tsv)：文件、行、列、类别、原名、新名，行列号是改名前的位置。新名为 `-` 的 19 处是 clang-tidy 无法自动修正的。

## 手工修正（[manual-fixups.sh](manual-fixups.sh)）

每一项都由编译失败或残留扫描发现：

1. **clang-tidy 无法自动修正的 19 处**：`Bmi088::publish` 是成员函数模板，模板参数上的 `x_axis`、`get_read_count()` 等访问依赖类型；`MotorGroup` 经模板参数访问 DJI 型号映射的 `MAXIMUM_MOTORS`、`make_config`。按同一规则手工改名。
2. **未实例化模板里的调用**：涉及 `Matrix.h`、`KalmanFilter.hpp`、`ExtendedStateObserver.hpp`、`Fsm.h`、`AxisController.hpp`、`MockMotor.hpp`，以及 `Spi.hpp`、`Uart.hpp`、`UsbCdc.hpp` 中模板重载里的转发调用。clang-tidy 看不到这些调用，编译器也只在实例化时检查，靠 [residual.py](residual.py) 扫描发现。
3. **改名后与局部变量同名**：
   - `alg_math::Norm` 改成 `norm` 后，被同名局部变量 `norm` 遮住，调用改为限定形式 `alg_math::norm(value)`，两个名字都不变；
   - 测试辅助函数 `Config()` 改成 `config()` 后和局部变量同名，改为 `default_config()`。
4. **注释中的旧名字**（[comments.py](comments.py)）：共 205 处、39 个文件。只改多段驼峰、映射唯一、且不是现存类型名的名字（如 `InitRtos`、`PostKernel`、`MaximumMotors`）。
5. **常量名里的数字分段**（[digit-segments.pl](digit-segments.pl)）：clang-tidy 会把数字并到前一个词上，例如 `ACCEL_ODR100_HZ`、`GYRO_BANDWIDTH2000_HZ532_HZ`、`POWER5_V`、`CHANNEL1`。修正规则如下：
   - "数值+单位"和序号单独成段：`ACCEL_ODR_100_HZ`、`GYRO_BANDWIDTH_2000_HZ_532_HZ`、`POWER_5V`、`CHANNEL_1`、`LEVEL_0`、`DIGITAL_CHANNEL_17_FLAG`（对应字段为 `digital_channel_17`）；
   - 名字里固有的数字保持原样：型号（`BMI088`、`GM6020_CURRENT`、`c610`）、引脚名（`INTERRUPT1`）、术语（`OSR2`、`crc8`）、数学常量（`SQRT2`）、标号（`WHEEL_M1`）。原先拆错的 `G_M6020_CURRENT` 改回 `GM6020_CURRENT`。

   修正后，C++ 包装层的枚举名与 C 层的宏后缀逐一对应，例如 `CHANNEL_1` 对应 `CAN_PORT_CHANNEL_1`，`POWER_5V` 对应 `GPIO_PORT_PIN_POWER_5V`。`docs/ai/architecture.md` §8.2 的示例 `Channel::CHANNEL_1` 也采用这一写法。

## 验证

| 项 | 结果 |
|---|---|
| legacy 固件 `.bin`（Windows，xPack GCC 15.2.1，基线同一安装路径） | Debug `45d47493…`、Release `76f98d86…`，与基线 SHA-256 逐字节一致 |
| 符号表（`arm-none-eabi-nm -S --size-sort -C`） | Debug 1447 个、Release 1058 个，与基线相同；"地址+大小+类型"集合一致，名字变化的符号 Debug 379 个、Release 219 个。改名后的符号表见 [Debug.nm.txt](Debug.nm.txt)、[Release.nm.txt](Release.nm.txt)，与 `../baseline-a2cfd16/` 按地址一一对应 |
| 宿主测试 | 13/13 通过（WSL g++ 11.4；Windows MSYS2 g++ 16.2） |
| 三种配置编译 | 固件 Debug、Release（Arm GNU Toolchain 15.2.Rel1，Linux）与宿主测试均无错误 |
| 命名检查 | `naming.py --check`（三个数据库，含孤立头）0 处 |
| 残留扫描 | 只剩与合规名字同名的误报：局部变量 `tick`、参数 `notification`、模板参数 `Index` 等 |

`.bin` 逐字节一致，说明改名没有改变任何生成代码，满足 plan.md 的 P0.5 验收条件。

## 注意事项

- **调试与历史数据**：芯片里旧固件的符号仍是旧名。例如 `test/data/imu/` 记录的 `gyroCapture` 位于 `0x24000788`，对应新构建的 `g_gyro_capture`。按地址对照时，基线符号表和本目录的符号表地址相同，可以一一对应。
- **`PI` 与 CMSIS-DSP 宏同名**：`alg_math::pi` 改名为 `PI` 后，和 `arm_math.h` 的 `#define PI` 同名。目前手写代码都不包含 `arm_math.h`，没有影响；以后同一翻译单元同时包含两者会编译失败，P3a 写平台层时注意。
- **注释里的单个单词名没改**：例如 `Init`、`Error`，它们和英文单词无法区分。
- **历史文档保留旧名**：评审报告、基线符号表、`test/host/README.md` 的适配记录、`legacy/` 下的 README 与 VALIDATION 都保持原样，按本目录的改名表对照。
