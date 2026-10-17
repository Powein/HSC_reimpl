# 参数与材料诊断记录

本目录提供完整行程开发中的参数探索、材料比较和设备初始化诊断。各案例的 `parameters.json` 保存配置，`status.json` 保存推进帧数、终止状态和诊断信息。

| 目录 | 诊断内容 | 记录的结果 |
|---|---|---|
| `table1_interpretation/` | 车载胎 E 为 1e9、1e10 Pa 的参数探索 | 参数切换后终止，状态保存在两个案例目录中 |
| `stable_neo_hookean/` | Stable Neo-Hookean 胎体的完整行程尝试 | 较软地面胎最小体积比为 −0.018602；保留体积异常与停止状态 |
| `log_nh_device_initialization/` | 对数 Neo-Hookean 的设备端初始化 | 在第 0 帧记录设备异常与进程退出码 |

完整行程采用[对数 Neo-Hookean 与体积路径约束](../../../reproduction/hsc_backend/NONINVERSION.md)。两组正式数据位于 [ground_1e8](../ground_1e8/status.json)、[ground_1e6](../ground_1e6/status.json)，均已完成整车通过；[结果报告](../report.md)汇总这两条完成轨迹。
