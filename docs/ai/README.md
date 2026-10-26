# docs/ai 索引（foray_mcu）

> 本目录下的文档都由 AI 撰写（Decision-20261026-03），供人审阅；人工撰写的文档放在 `docs/` 的其他位置。
> 其中的结论以负责人确认、`decision.md` 登记为准。

| 文档 | 内容 | 状态 |
|---|---|---|
| [reviews/2026-10-25-foray_mcu-架构评审报告.md](reviews/2026-10-25-foray_mcu-架构评审报告.md) | 结构重构方案完整评审报告（**v1.0，已确认**）：现状事实、负责人确认的前提、M1–M4 必改项、目标 DAG 与依赖规则、平台可移植性约束、框架与产品边界、浮点策略、契约语言（C++）、命名与格式、P0–P5 迁移路径、Q1–Q18 与反馈表、实测记录 | ✅ 已确认 |
| [reviews/archive/](reviews/archive) | 评审报告 v0.1–v0.4 原文存档（仅供对照，不再维护） | 📄 已归档 |
| [architecture.md](architecture.md) | 分层判据、16 条依赖规则（检查方式与生效阶段）、目录白名单、例外登记表的唯一人读入口 | ✅ v1（2026-10-26） |
| [decisions/2026-10-25-迁移基线与对照.md](decisions/2026-10-25-迁移基线与对照.md) | 迁移基线数据、SHA、工具链版本、对照判据、P0 复现结果与芯片 Flash 存档记录；符号表在 `decisions/baseline-a2cfd16/` | ✅ P0 已复核 |
| [../assets/controller_code-架构图.png](../assets/controller_code-架构图.png) | 迁移源 ControllerCode 的既有架构图（人工绘制，不在本目录；PNG 内嵌 5 页：总览 / 附录A 分层明细 / 附录B 运行时 / 附录C 修订说明 / 附录D 待修项） | 📄 已归档 |
| [assets/controller_code-固件恢复可行性评估.md](assets/controller_code-固件恢复可行性评估.md) | 迁移源基线数据的来源文档（在 ControllerCode a2cfd16 中未入 git，**原样转存，未做内容修改**；含原机器的本机路径；按写作时间与风格判断为 AI 撰写） | 📄 已转存 |
| [decisions/2026-10-26-P0.5-改名/](decisions/2026-10-26-P0.5-改名/README.md) | P0.5 命名统一的过程、复现命令、改名表、手工修正脚本与验证结果；改名后的符号表（与基线按地址一一对应） | ✅ P0.5 已验收 |
| [decisions/2026-10-26-丢失功能与恢复依据.md](decisions/2026-10-26-丢失功能与恢复依据.md) | `a2cfd16` 之后丢失的功能（L1–L6，含加热启动与超调优化）、原固件的行为证据与重做验收依据 | 🟡 持续更新 |
| `decisions/`（其余） | 评审决议、后续关键决策归档（P0 起） | 🟡 待补 |

仓库级文档（五件套）见根目录：[README.md](../../README.md)、[AGENTS.md](../../AGENTS.md)、[plan.md](../../plan.md)、[tree.md](../../tree.md)、[decision.md](../../decision.md)。
