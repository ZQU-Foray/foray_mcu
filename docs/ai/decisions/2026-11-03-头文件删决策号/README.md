# 代码注释删掉决策号（2026-11-03）

> AI 撰写。负责人 2026-11-03 要求"头文件注释中的决策号全部都删掉"，随后追加"都删了但是别删文档的"，见 [decision.md](../../../../decision.md) Decision-20261103-06。

## 第一步：头文件（`f71cdb0`）

### 范围

已跟踪的手写头文件（`*.h`、`*.hpp`，不含 `cubemx/`、`third_party/`、`legacy/`）里的 `Decision-YYYYMMDD-NN`：55 个文件、77 处。
### 做法

在提交 `78acdf5` 上执行：

```bash
git ls-files '*.h' '*.hpp' | grep -v -e '/cubemx/' -e third_party -e '^legacy/' \
  | xargs grep -l "Decision-20" | xargs python3 docs/ai/decisions/2026-11-03-头文件删决策号/drop_decision_refs.py
```

脚本处理 76 处，报告 1 处未处理。之后手工改了两处：

- `heater_control.hpp`："之后按 Decision-20261103-02 修正"改为"之后按类说明里的两处修正"；
- `monitor.hpp`：删掉决策号后，独占一行的句号并回上一行。

### 结果

- bringup 与 link 的 Debug、Release 四个固件，改前改后 `.bin` 逐字节相同。
- 以下检查都通过：宿主测试 39/39、格式、命名（5 个编译数据库）、文本扫描、依赖图。
- 提交 `f71cdb0` 已登记到 `.git-blame-ignore-revs`。

## 第二步：代码与配置文件（`4540b4b`）

### 范围

已跟踪的非文档文件，共 76 个：`.cpp`、CMake、Python、CI 配置、`.clang-format`、`.clang-tidy`、`CODEOWNERS`、`.gitignore`、标准帧文件。

不动的：

- `*.md` 与 `docs/` 下的全部文件，包括各阶段记录里的脚本；
- `.git-blame-ignore-revs`，它的注释是每个被忽略提交的来由记录；
- 生成区、`third_party/`、`legacy/`，这些地方本来就没有决策号。

### 做法

在 `69b3019` 上执行：

```bash
git grep -l "Decision-20" -- . ':!*.md' ':!docs/' ':!.git-blame-ignore-revs' \
  | xargs python3 docs/ai/decisions/2026-11-03-头文件删决策号/drop_decision_refs.py
```

脚本报告 14 处未处理，其中 13 处手工改写。这些句子把决策号当主语或宾语用，例如"与 Decision-20261025-04 一致""Decision-20261029-02 修好的情形""见 decision.md Decision-…"。剩下 1 处是标准帧的文件头，改了生成器 `tool/robolink/make_golden.py` 后重新生成。

之后手工修了三类：

- 删完只剩标点的续行，5 处，并回上一行；
- 测试文件首行的类名与"在"之间补空格，6 处；
- 板级 CMake 的分隔线前补空格，1 处。

`bringup_test.cpp` 的一处行尾注释变短后，由 `tool/ci/format.py --fix` 重排。

### 结果

- 四个固件与 `78acdf5` 逐字节相同。
- 以下检查都通过：宿主测试 39/39、格式、命名（5 个编译数据库）、文本扫描、依赖图、标准帧 `--check`、ruff 与 black（在干净副本上）。
- 非文档文件里已没有 `Decision-YYYYMMDD-NN`。
