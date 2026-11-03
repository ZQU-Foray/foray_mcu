# 头文件注释删掉决策号（2026-11-03）

> AI 撰写。负责人 2026-11-03 要求"头文件注释中的决策号全部都删掉"，见 [decision.md](../../../../decision.md) Decision-20261103-06。

## 范围

已跟踪的手写头文件（`*.h`、`*.hpp`，不含 `cubemx/`、`third_party/`、`legacy/`）里的 `Decision-YYYYMMDD-NN`：55 个文件、77 处。`.cpp`、CMake、脚本、CI 配置里的决策号不在本次范围。

## 做法

在提交 `78acdf5` 上执行：

```bash
git ls-files '*.h' '*.hpp' | grep -v -e '/cubemx/' -e third_party -e '^legacy/' \
  | xargs grep -l "Decision-20" | xargs python3 docs/ai/decisions/2026-11-03-头文件删决策号/drop_decision_refs.py
```

脚本处理 76 处，报告 1 处未处理。之后手工改了两处：

- `heater_control.hpp`："之后按 Decision-20261103-02 修正"改为"之后按类说明里的两处修正"；
- `monitor.hpp`：删掉决策号后，独占一行的句号并回上一行。

## 结果

- bringup 与 link 的 Debug、Release 四个固件，改前改后 `.bin` 逐字节相同。
- 以下检查都通过：宿主测试 39/39、格式、命名（5 个编译数据库）、文本扫描、依赖图。
- 提交 `f71cdb0` 已登记到 `.git-blame-ignore-revs`。
