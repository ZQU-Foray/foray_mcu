# docs 索引（foray_mcu）

| 文档 | 内容 | 状态 |
|---|---|---|
| [reviews/2026-10-25-foray_mcu-架构评审报告.md](reviews/2026-10-25-foray_mcu-架构评审报告.md) | 结构重构方案完整评审报告（**v1.0，已确认**）：现状事实、负责人确认的前提、M1–M4 必改项、目标 DAG 与依赖规则、平台可移植性约束、框架与产品边界、浮点策略、契约语言（C++）、命名与格式、P0–P5 迁移路径、Q1–Q18 与反馈表、实测记录 | ✅ 已确认 |
| [reviews/archive/](reviews/archive/) | 评审报告 v0.1–v0.4 原文存档（仅供对照，不再维护） | 📄 已归档 |
| [architecture.md](architecture.md) | 分层判据与依赖规则的唯一人读入口（占位；P0 完善） | 📄 占位 |
| [decisions/2026-10-25-迁移基线与对照.md](decisions/2026-10-25-迁移基线与对照.md) | 迁移基线数据、SHA、工具链版本、对照判据与 P0 复核步骤 | 📄 已交付，待 P0 复核 |
| [assets/controller_code-架构图.png](assets/controller_code-架构图.png) | 迁移源 ControllerCode 的既有架构图（PNG 内嵌 5 页：总览 / 附录A 分层明细 / 附录B 运行时 / 附录C 修订说明 / 附录D 待修项） | 📄 已归档 |
| [assets/controller_code-固件恢复可行性评估.md](assets/controller_code-固件恢复可行性评估.md) | 迁移源基线数据的来源文档（在 ControllerCode a2cfd16 中未入 git，**原样转存，未做内容修改**；含原机器的本机路径） | 📄 已转存 |
| `decisions/`（其余） | 评审决议、后续关键决策归档（P0 起） | 🟡 待补 |

仓库级文档（五件套）见根目录：[README.md](../README.md)、[AGENTS.md](../AGENTS.md)、[plan.md](../plan.md)、[tree.md](../tree.md)、[decision.md](../decision.md)。
