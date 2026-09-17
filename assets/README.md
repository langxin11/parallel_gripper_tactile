<!-- --8<-- [start:content] -->
# 模型资产

本目录按夹爪维护 MJCF、网格、来源与生成说明。先选择夹爪，再查看对应的模型变体和触觉布局。

| 夹爪 | 模型与加载 | 资产维护 |
| --- | --- | --- |
| Robotiq 2F-85 | [模型入口][robotiq]、[离散 taxel 与 touch_grid][robotiq-tactile] | 从基础 MJCF 生成触觉变体。 |
| DMgripper | [模型入口][dm]、[Pillar 布局与接触映射][dm-tactile] | [Onshape 导出、后处理与验收][dm-export]。 |

公共数组、单位和力的定义见[触觉接口约定][tactile-contract]；实验运行与结果解释由主项目文档维护。

<!-- --8<-- [start:maintenance] -->
## 资产独立性的验收标准 {#asset-maintenance}

这里的“相对独立”指资产能交代自身来源、处理过程和加载要求，不要求复制主项目的控制器或实验框架。

| 标准 | 可检查的证据 |
| --- | --- |
| 来源与许可明确 | 给出上游地址、可用的版本或导出配置、修改说明及许可文件；尚未查明的内容明确标记。 |
| 输入与产物明确 | 列出基础模型、网格、生成变体及其对应关系，给出可执行命令和输出位置。 |
| 生成与运行依赖分离 | CAD 导出凭据和导出工具只在重新生成时需要；加载已提交模型无需连接 CAD 服务。 |
| 专用工具随资产维护 | 模型专用生成、转换与检查脚本归属对应资产目录，公共 CLI 只负责调用；迁移时保持既有参数与输出契约。 |
| 模型能被检查 | 明确关节、执行器、闭环约束、碰撞与触觉命名的检查项，并能通过项目资源校验。 |
| 加载契约稳定 | `configs/model/` 中的 `model.path`、模型家族和触觉配置与实际 MJCF 一致；迁移路径时同步检查调用方。 |
| 正文只有一份 | 模型专属说明随资产维护；站点引用同一正文，不复制为另一份手工维护的说明。 |

> **当前工具边界：**专用工具尚未全部归入资产目录。生成和命名逻辑仍在主包
> `src/parallel_gripper_tactile/asset_tools.py`，DM 碰撞变体脚本仍在
> `scripts/prepare_flat_sphere_collision_model.py`。当前 `pgt assets` 仍依赖主项目环境。

## 维护边界

- **资产目录：**来源、网格、MJCF、导出流程、模型变体、碰撞几何和具体传感器布局。
- **主项目：**公共触觉语义、运行时读取器、控制器、实验配置组合与结果。
- **研究报告：**模型适用条件、实验依据和冻结结果；资产入口链接到这些依据，不另写一套结论。

修改模型后，先核对对应资产页，再按[测试策略][testing]执行适用验证。
<!-- --8<-- [end:maintenance] -->
<!-- --8<-- [end:content] -->

[robotiq]: grippers/robotiq_2f85/README.md
[robotiq-tactile]: grippers/robotiq_2f85/README.md#tactile-model
[dm]: grippers/dm_gripper/README.md
[dm-tactile]: grippers/dm_gripper/README.md#tactile-model
[dm-export]: grippers/dm_gripper/onshape-export.md
[tactile-contract]: ../docs/tactile-conventions.md
[testing]: ../docs/testing.md