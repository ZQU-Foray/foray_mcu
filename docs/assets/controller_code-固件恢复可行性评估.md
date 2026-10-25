# 能否从已烧录固件找回“最新进度”的工程代码？

> 评估对象：`foray_mcu_controller` / DM_MC02（STM32H723VGTx，Cortex-M7，1 MB 内部 Flash）
> 已知基线：`git main` @ `a2cfd16`（2026-10-23 11:17 +08），本次工作区为 2026-10-25 11:28 全新 clone
> 评估日期：2026-10-25　·　评估性质：**可行性评估（未执行恢复）**　·　前提：固件“未加密”，只剩单片机内的已烧录镜像

---

## 0. 结论（先看这里）

| 问题 | 结论 |
|---|---|
| 能否从固件“还原出原始工程源码”（文本级，含命名/注释/宏/目录，能直接 `git commit`） | **不能。0%。** 编译器与烧录过程把这些信息永久丢弃了 |
| 能否恢复“最新进度的行为、参数、关键逻辑”，并在旧基线基础上重建出**可编译、行为等价**的代码 | **有条件可行。** 本质是“以旧基线为骨架的差分恢复”，不是从零逆向一个固件 |
| 本项目（STM32H723 + GCC 15.2 + 有完整旧基线）的总体可行性 | **中等偏上（取决于 4 个变量，见 §5）**。最有利条件是：旧基线可复现构建、约一半代码是公开库可自动识别、C++ 关闭了 RTTI/异常/虚函数 |
| 最大的“一票否决”风险 | ① Flash 读保护（RDP）不是 0 → 读不出；② 烧录镜像 ≠ 最后进度 → 未烧录的修改永远丢失；③ 丢的是 Release(-Os) 镜像或大改架构 → 成本迅速超过重写 |

**一句话**：固件里只有“编译后的行为”，没有“工程源码”。想找回工程，做法是：**先确认固件能读、是最后进度；用旧源码重建对照镜像；函数级差分出“丢失的增量”；只反编译增量并写回旧基线**。如果增量超过约 2–4 k 行，重写通常比逆向更划算。

---

## 1. 本次已核实的事实（不是推测）

### 1.1 项目与构建

| 项 | 值 |
|---|---|
| MCU / Flash | STM32H723VGTx，Cortex-M7，内部 Flash 1 MB @ `0x08000000` |
| 构建系统 | CMake + Ninja，`CMakePresets.json`（Debug / Release） |
| 工具链 | Arm GNU Toolchain 15.2（`cmake/gcc-arm-none-eabi.cmake`，presets 指向 `C:/Users/pomelo/Documents/Codex/stm32-tools/gcc-15.2.rel1`） |
| 语言 / 选项 | C++17；Debug `-O0 -g3`，Release `-Os -g0`；`-fno-rtti -fno-exceptions -fno-threadsafe-statics`；无 LTO；`-Wl,--gc-sections`；`--specs=nano.specs` |
| 项目自有源码规模 | 约 **23,196 行**（Application 3,290 / Libraries 8,801 / Platform Interface+PortKit+Port 4,039 / CubeMX Core 5,336 / USB_DEVICE 1,730） |
| 代码特征 | **0 处虚函数**（无 vtable）、**0 处运行时 assert/`__FILE__`**、仅 `static_assert`（编译期，不产生二进制痕迹） |

### 1.2 基线可复现构建（本机实测，已验证）

用本机 xpack GCC 15.2.1 + CMake 4.4 + Ninja 从 `a2cfd16` 完整构建 Debug/Release，**0 error**：

| 配置 | 烧录镜像 `.bin` | `.text` | `.rodata` | `.data`(LMA) | `.bss` | ELF 大小 |
|---|---|---|---|---|---|---|
| Debug `-O0 -g3` | **238,320 B** | 200,200 B | 16,184 B | 21,192 B | 157,056 B | 3,778,912 B（其中约 3.5 MB 是调试信息） |
| Release `-Os -g0` | **100,664 B** | 96,228 B | — | 4,432 B | 105,064 B | 188,616 B |

生成物（可复现对照物）：`Platform/STM32H7/DM_MC02/build/{Debug,Release}/mc02_cpp.{elf,hex,bin,map}`

基线镜像 SHA-256（本机工具链指纹，供日后与芯片 dump 对比）：

```
Debug   (238,320 B) 45D47493659656306952BDE64062BB80BD4AB7F6E95618B2F00A82A00AD44204
Release (100,664 B) 76F98D86969985B7DD3AB9C0A7A1EAE829B87F9DA9E51BF4E58BA4DDD8ED7B02
```

> 注意：`-g3` 的 ELF 有 3.78 MB，而真正烧进芯片的只有 **238 KB** —— 这直接证明调试信息不会进 Flash。

### 1.3 固件镜像的构成（Debug 构建，实测 map 统计）

| 来源 | 函数/代码段数 | 代码字节 | 占比 |
|---|---|---|---|
| ST HAL 驱动（公开源码） | 246 | 74,936 | 37.5% |
| 项目 Libraries（旧基线已知） | 219 | 39,722 | 19.9% |
| C/C++ 运行库、DSP、CubeMX 生成代码等 | ~300 | ~38,000 | ~19% |
| 项目 Application（旧基线已知） | 167 | 26,084 | 13.0% |
| FreeRTOS（公开源码） | 87 | 13,106 | 6.6% |
| ST USB Device 库（公开源码） | 57 | 8,042 | 4.0% |
| **合计** | **1,098** | **199,998** | 100% |

关键点：**项目自有代码只占约一半，且这“一半”中的绝大部分在旧基线里已经存在**。真正需要人工恢复的对象不是 200 KB 代码，而是“固件相对旧基线发生差异的那一小部分”。

### 1.4 本机恢复工具链现状

| 工具 | 状态 |
|---|---|
| `arm-none-eabi-gcc/g++ 15.2.1`（xpack） | ✅ 已在 PATH |
| Apache OpenOCD 0.12.0（xpack） | ✅ 已在 PATH |
| CMake 4.4.4 / Ninja 1.13.2 / Python 3.13 | ✅ 已装 |
| Ghidra / IDA / Binary Ninja / rizin | ❌ 未装（Ghidra 免费，需下载） |
| 旧构建产物（`.elf/.hex/.map`）、IDE 本地历史 | ❌ 本机未检索到（本机为 10-25 新搭环境） |

---

## 2. 单片机 Flash 里到底有什么、没有什么

**STM32 的“烧录”只写入 ELF 的可加载段（`PT_LOAD`）**。`OpenOCD program <elf>`（或 `.hex/.bin`）只会把 `.isr_vector / .text / .rodata / .data 初值 / .init_array` 写进 Flash；`.debug_*`、`.symtab/.strtab`、`.comment` 等都是非加载节，**从不进入芯片**。即便当初是 `-g3` Debug 构建，源码级信息也只在丢失的 `build/` 目录里。

### 2.1 固件里**有**的（可稳定恢复）

- 全部机器码（Thumb-2）、函数边界、调用关系、条件跳转与跳转表；
- **`.rodata`（约 16 KB）**：字符串常量、浮点常量、查找表、USB 描述符、FreeRTOS 任务名（`imuTask/chassisTask/gimbalTask/defaultTask` 实测就在镜像里）；
- **`.data` 初值（约 21 KB）**：全局对象/数组/配置表的**默认值**（很多整定参数、默认配置就是这种方式烧进 Flash 的）；
- 向量表、`.init_array` 全局构造函数列表、中断入口；
- 构建类型指纹：Debug/Release 镜像大小差 2.4 倍；Debug 镜像还会带 libstdc++ 未优化断言的字符串（如 `%s:%d: %s: Assertion '%s' failed.` 以及 `operator[] [with _Tp = component::AxisStatus; ...]` 这类 **C++ 模板实例化类型名**）——这是意外收获，对恢复类型/模板有辅助价值；
- 若固件是 Debug 构建，反编译质量会明显更好（`-O0` 无内联、无重排、栈帧清晰）。

### 2.2 固件里**没有**的（不要抱期望）

- **符号表、调试信息**：函数名、变量名、类型/结构体定义、源文件、行号——全部没有（这是“还原源码”不可能的根本原因）；
- **注释、宏定义、模板原始写法、类名**：`-fno-rtti` 连 RTTI 类型名都没有，且项目无虚函数 → 连 vtable 都没有；
- 被 `--gc-sections` 裁掉的未使用代码（本来也不影响运行）；
- **非 MCU 资产**：CubeMX `.ioc` 文本、CMake/IDE 配置、上位机 Python 工具、测试、文档、历史提交——`.gitignore` 明确排除了 `Tools/`、`scripts/`、`*.py`、`Tests/*`、`User_Configs/` 等，这些**即使固件在手也 100% 救不回来**。

### 2.3 特别提醒：“未加密” ≠ “可读”

STM32 的读取保护（RDP，Option Bytes）与“加密算法”是两回事：

| RDP 等级 | 用调试口读 Flash | 说明 |
|---|---|---|
| Level 0（默认） | ✅ 可读全片 | 正常情况，直接 dump 即可 |
| Level 1 | ❌ 不能直接读 | 解除保护（regression）通常会触发**全片擦除**——如果芯片是唯一副本，等于亲手销毁固件 |
| Level 2 | ❌ 完全不可调试、不可逆 | 基本无解。J-Link/SEGGER 文档明确：RDP=2 后无法连接，且保护不能复位 |

> 所以第一步不是反编译，而是**在断电/复位前确认 RDP 等级**。在确认之前，不要点任何“unlock / regression / erase”按钮。

---

## 3. 为什么本项目的可行性高于“从零逆向固件”

1. **有完整旧基线**：`a2cfd16` 是真实、可编译的工程。差分恢复的最大成本节省来自这里——反编译目标从“整个固件”缩小为“增量”。
2. **基线可复现**：本机已实测构建成功（0 error）。用同版本工具链、同选项重新构建旧版本，就能生成函数名/地址/大小完全已知的“对照 ELF + map”。
3. **公开代码占一半**：HAL 1.11.6、FreeRTOS 10.6.2、ST USB Device、CMSIS-DSP、newlib 都是公开源码。用旧源码重建后，固件里这些函数可被自动匹配识别，人工面进一步缩小。
4. **C++ 特性反而有利**：无 RTTI、无异常、无虚函数、无动态多态，调用图基本是静态的，没有 vtable/typeinfo/异常表还原这些“硬骨头”。
5. **工具链与烧录方式已知**：GCC 15.2、CMake/Ninja、OpenOCD/ST-Link，重建环境可精确还原（建议补装 Arm GNU 15.2.rel1 以获得更高字节级一致性）。

**不利因素**：`--gc-sections` 会丢弃未引用代码；无符号/无调试信息；若烧录的是 Release `-Os`，内联与合并会让函数边界和语义恢复难 1.5–2.5 倍。

---

## 4. 推荐路线：差分恢复（若确认要做）

### Phase 0 — 取证与保护确认（0.5–1 天）

1. 确认 RDP 等级（CubeProgrammer 的 Option Bytes 页最直观，或 OpenOCD 读 `FLASH_OPTSR_CUR`）；**RDP≠0 时停下评估，不要 regression**；
2. 接调试器 dump 整片 1 MB，**读两遍并比对 hash**；保存 `.bin` + 元数据（芯片 UID、Option Bytes、时间、板卡编号）；
3. 确认外部 OSPI/QSPI 上没有代码（本工程链接脚本只把代码放内部 Flash，正常可忽略）；
4. 确认“最后一个烧录镜像 = 最新进度”：和开发者对时间线，若之后还有未烧录的修改，那部分固件里没有，**永远无法恢复**。

参考命令（OpenOCD 0.12）：

```powershell
# 先只读状态，确认能连上（不要写任何东西）
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset halt; mdw 0x5200201C 1; shutdown"

# 全片 dump（1 MB），做两份并核对 hash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset halt; dump_image mc02_flash_a.bin 0x08000000 0x100000; shutdown"
```

### Phase 1 — 基线重建 + 构建类型判定 + 差分（2–4 天）

1. 用 GCC 15.2 重建 `a2cfd16` 的 Debug 与 Release（本机已可复现，见 §1.2）；
2. **构建类型判定**：dump 镜像大小 ≈ 238 KB → Debug；≈ 101 KB → Release。再用字符串特征交叉验证（Debug 独有的 `Assertion`/`operator[]` 类型名）；
3. **关键判断**：若 dump 与重建 Debug bin **逐字节一致**（或 99%+ 一致）→ 芯片里就是旧基线本身，**没有“最新进度”可恢复**，立刻停止投入；
4. **差分**：Ghidra 载入 dump（ARM Cortex-M7，基址 `0x08000000`）自动分析，再与重建 ELF 做函数级匹配（Ghidra Version Tracking / Diaphora / 自研“归一化指令哈希”），产出：未变函数表、变化函数表、新增函数表、数据表差异。**此时就能量化“丢失增量”的真实规模**——这是决定继续还是改重写的决策点。

### Phase 2 — 增量反编译与写回（主要成本，见 §4 工作量表）

- 只反编译“变化/新增”函数；未变函数直接用旧基线源码与符号名；
- 从 `.rodata`/`.data` 与反编译代码里恢复整定参数、查找表、结构体偏移、协议常量；
- 每恢复一个函数：重编译 → 与 dump 中该函数做指令序列/CFG 对比 → 迭代到一致；
- 结构体/接口如有变化，优先从访问偏移和初始化代码反推。

### Phase 3 — 验证（3–10 天）

- 用旧版本 host 测试思路重建关键单元测试；
- 实机行为对比（USB CDC/调试档案/IMU 数据流）；
- 在备份 dump 的前提下烧录测试板回归。

### 工作量经验估算（1 名熟悉该工程的嵌入式工程师，Debug `-O0` 镜像，有完整旧基线）

| 丢失增量（新增/修改的有效代码行） | 典型内容 | 人工工作量 | 日历时间 | 可行性 |
|---|---|---|---|---|
| ≈0（镜像 = 旧基线） | — | 0 | 0.5–1 天（只需比对） | 无需恢复 |
| < 100 行（参数级） | PID/滤波/限幅/协议常量整定 | 2–5 人天 | 约 1 周 | **高**：参数可比较直接读出 |
| 100–500 行 | 1–3 个功能点、接口未大改 | 8–15 人天 | 2–3 周 | **中高** |
| 500–1,500 行 | 新模块/任务接线/结构体扩展 | 20–40 人天 | 1–2 月 | **中** |
| 1,500–4,000 行 | 多模块联动、局部重构 | 50–100 人天 | 2–4 月 | **中低** |
| > 4,000 行或架构级重构 | 换框架、大改接口/任务模型 | ≥100 人天 | ≥4 月 | **低**（建议按固件行为规格重写） |

> 折算基准：约 80–200 行有效代码/人天（含反编译、写回、验证）。Release `-Os` 镜像 ×1.5–2.5；AI 辅助可提速 1.3–2 倍，但验证成本不降。以上为经验估算，不是承诺。

---

## 5. 决定可行性的 4 个变量（需要先确认）

| # | 变量 | 影响 | 需要的答案 |
|---|---|---|---|
| 1 | **RDP 等级 / 能否读出** | 一票否决 | RDP=0？能否连上调试口？ |
| 2 | **烧录镜像是否等于最后进度** | 未烧录部分 0% | 最后一次烧录是什么时间？之后还改过什么？ |
| 3 | **丢失增量规模/性质** | 决定是恢复还是重写 | 大致加了哪些模块/多少行？动没动 TaskManager、接口、结构体、协议？ |
| 4 | **Debug 还是 Release 构建** | 难度 ×1.5–2.5 | dump 大小一测便知 |

第 5 个（非技术但更关键）：**除了芯片，还有没有别的副本**——见 §6。

---

## 6. 成本更低的替代路径（强烈建议在动手逆向之前先做完）

按“性价比”排序：

1. **确认“丢失”的范围**：当前工作区是 10-25 11:28 clone 的 `main` @ `a2cfd16`。若“最新进度”只是未推送的本地修改，优先找：同事本地仓库副本、远端其他分支/PR（当前 `origin` 为私有 GitHub，本机无凭据，无法远程核查）、CI 构建产物。
2. **丢失的电脑/硬盘**（如果还在）：
   - **JetBrains CLion Local History**（默认保留数天，可逐文件恢复未提交修改）——本机没有 JetBrains 目录，需去丢失机器确认；
   - Windows 卷影副本 / 文件历史 / OneDrive 版本历史；
   - 回收站、编辑器临时文件、`build/` 目录（**只要当初的 `.elf` 还在（3.7 MB），符号 + DWARF 全在，恢复成本直接降一个数量级**；`.map` 次之）。
3. **烧录痕迹**：OpenOCD/J-Link 工程常常保留 `build/Debug/mc02_cpp.{elf,hex,map,bin}`；工位共享盘、微信/QQ 传输记录、邮件附件里也可能有。
4. **本机扫描结果**：本次评估机器（10-25 新搭环境）未发现旧构建产物、IDE 历史或相关压缩包；VS Code 历史仅有机型无关的 settings.json，回收站只有两个 `.exe`。**说明至少这台机器不是“丢失现场”，需要去原机器/其他同事处确认。**

> 顺序建议：**先花 30–60 分钟找 `.elf/.hex/.map/本地历史/同事副本`；找不到，再考虑 §4 的固件差分恢复。** 这是本评估最重要的行动建议。

---

## 7. 风险与操作红线

- **先备份，后操作**：dump 完成并校验前，不要烧录、不要复位刷写、不要改 Option Bytes；
- **不要对 RDP≥1 的芯片执行 unlock/regression**：会触发全片擦除，唯一固件可能瞬间消失；
- 若板子还通着电且在运行，先不要反复复位（虽然 Flash 内容不变，但 RAM 里的运行态/调试档案可能对验证有用）；
- 若固件是 Release `-Os`，预期工期按上限估；若丢失增量动了核心头文件/接口，实际“差异函数数”可能远大于“改动行数”；
- 版权/合规：自有代码，无额外问题；
- 恢复出的代码建议按“新基线”重新走一遍 CI/主机测试，并在重构后与原固件行为做对照，再决定是否替换生产固件。

---

## 8. 结论矩阵（把变量填进去即可决策）

| 场景 | 可读性 | 增量规模 | 可能性 | 建议 |
|---|---|---|---|---|
| Debug 镜像 + RDP0 + 增量 ≤500 行 | ✅ | 小 | **中高（约 70–85%）** | 做差分恢复，2–3 周 |
| Debug 镜像 + RDP0 + 增量 0.5–2 k 行 | ✅ | 中 | **中（约 40–60%）** | 先做 Phase 0–1 量化，再决定继续/重写 |
| Release 镜像 或 增量 >2 k 行 | ✅ | 中/大 | **低–中（20–40%）** | 建议“以固件为行为规格”重写；只恢复参数/行为 |
| 镜像 ≠ 最后进度 | ✅ | — | 未烧录部分 **0%** | 只能恢复到最后一次烧录的版本 |
| RDP = 1 | ❌ 直接 dump | — | 低（需在片 dumper 方案，成功率不确定、风险高） | 单独评估，勿贸然 regression |
| RDP = 2 | ❌ | — | **0%** | 不可行 |

---

## 附录 A：本评估执行过的本地验证

```powershell
# 基线复现构建（本机实测通过，0 error；build/ 已被 .gitignore 忽略）
cmake -S Platform/STM32H7/DM_MC02 -B Platform/STM32H7/DM_MC02/build/Debug -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE=Platform/STM32H7/DM_MC02/cmake/gcc-arm-none-eabi.cmake `
  -DARM_GCC_ROOT=C:/Users/pomelo/tools/xpack-arm-none-eabi-gcc -DCMAKE_BUILD_TYPE=Debug
cmake --build Platform/STM32H7/DM_MC02/build/Debug -j 8

# 生成物：mc02_cpp.elf / .hex / .bin / .map
# Debug:   FLASH 238,320 B (22.73%)   Release: FLASH 100,664 B (9.60%)
```

## 附录 B：参考

- STM32 读取保护（RDP）等级与不可逆性：[SEGGER Knowledge Base — ST STM32](https://kb.segger.com/STM32)
- OpenOCD 内存读取/dump 命令：[OpenOCD User's Guide — General Commands](https://openocd.org/doc/html/General-Commands.html)
- Ghidra（免费，支持 ARM Cortex-M7 / Thumb-2 反编译）：[NationalSecurityAgency/ghidra](https://github.com/NationalSecurityAgency/ghidra)
- 函数级二进制差分（配合 IDA）：[Diaphora](https://github.com/joxeankoret/diaphora)
- Arm GNU Toolchain 15.2 下载（与仓库 pin 的版本对齐）：[Arm GNU Toolchain Downloads](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
