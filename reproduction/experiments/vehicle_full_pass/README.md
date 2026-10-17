# 汽车完整越过轮胎：两组刚度实验

本入口推进车辆通过一排三个地面轮胎。按用户选定的单变量方案，车载轮胎均为 `1e10 Pa`，地面轮胎的杨氏模量分别为 `1e8`、`1e6 Pa`；其余场景、驱动、接触和求解设置相同。参数以 [config.json](config.json) 为准，实际运行值另存到每组结果的 `parameters.json`。

时间步 `1/30 s` 来自 HC.pdf 表 1。Fig.19 明确给出了车载胎 `1e10 Pa`、地面胎 `1e8 Pa`，作为本次基准组；地面胎 `1e6 Pa` 是用户选择的较软对照，不称为论文原始案例。资产继续使用 Chrono 衍生装配，一排三个障碍胎用于这次完整行程，规模与论文不同。

两组均已完成：地面胎 `1e8 Pa` 组为 213 帧、7.100 秒，`1e6 Pa` 组为 209 帧、6.967 秒；HSC 分别调用 6,161、6,402 次，无其他求解器回退。完整数值、动画和与论文的差别见[结果报告](../../../result/vehicle_full_pass/report.md)。

## 运行与结果

在项目根目录的 PowerShell 中执行：

```powershell
./reproduction/experiments/vehicle_full_pass/run.ps1
```

默认依次计算两组，全部完成后自动生成对照表、预览和报告。只运行一组、或恢复中断计算：

```powershell
./reproduction/experiments/vehicle_full_pass/run.ps1 -Case ground_1e8
./reproduction/experiments/vehicle_full_pass/run.ps1 -Case ground_1e8 -Resume
```

结果独立保存到项目根目录的 `result/vehicle_full_pass/`，不混入早期装配短测或 HSC 接触初期结果。完整通过的结果会保留；重复执行同一配置直接跳过，不覆盖。修改参数后应同时修改 `result_directory` 和 `experiment`，分别使用新的结果目录和原生缓存目录。

原生运行环境读取 `reproduction/hsc_backend/runtime.json`。CUDA 缓存和求解器断点放在该文件的 `root/runs/<experiment>/<case>/` 下，`experiment` 由配置给定，当前为 `vehicle_full_pass_noninverting`；完整路径写入结果的 `provenance.json`。这是本机运行缓存；代码和数据中的实验路径均相对项目根目录解析。原生断点在 E 盘，避免系统盘空间不足，也避免增量编译清理 `build/python/` 时丢失断点。恢复需要原生断点、相同参数、相同场景源码和相同后端二进制；更换资产后应新建实验目录。

## 完整通过的判据

四个车轮的轮心必须依次越过对应侧的障碍轮胎中心，越过时轮心位于障碍的横向范围内且高于障碍顶部。车辆所有实体几何的最后端，还必须越过当前全部障碍的最前端至少 `0.15 m`，连续保持五帧，并且运行中发生过网格接触。障碍被推向前方时，判据随其位置更新；车头接触、仅前轴越过、从侧面绕过均不算完成。

满足上述条件才写入 `COMPLETED_FULL_PASS`。达到 `max_frames`、Newton 不收敛、线搜索失败、HSC 失败、单元体积非正或进程退出异常均记为失败。`max_frames` 是保护上限，不是预先指定的成功帧数。主程序每帧检查原生求解状态，不另行重复运行装配测试。

两组均使用 `LogNeoHookean`。原生线搜索检查整条试探路径的单元体积，要求 `det(F)>1e-8`；实现、数学定义和异常记录见[防翻转说明](../../hsc_backend/NONINVERSION.md)。

## 代码职责

| 文件 | 职责 |
|---|---|
| `config.json` | 两组实验共用参数，以及唯一改变的地面胎 E |
| `run.ps1` / `run.py` | 选择本机运行环境、启动案例、保存进程日志和退出状态 |
| `run_case.py` | 组装场景、推进 HSC、提取几何测量、保存快照和恢复断点 |
| `completion.py` | 不依赖求解器的完整通过判据 |
| `storage.py` | 增量复制 HSC 记录、追加轨迹、原子更新状态文件 |
| `summarize.py` | 由保存数据生成 JSON / CSV 对照表 |
| `render.py` | 由保存网格生成完整行程图、刚度曲线和动画 |
| `report.py` / `finalize.py` | 读取完成数据生成结果文档，统一管理统计、绘图和报告输出 |
| `watch.py` | 跟随正在运行的两组进度，两组成功后生成最终输出 |
| `test_completion.py` | 检查车头通过、推着障碍走、绕行等误判边界，不运行物理仿真 |

场景定义复用 `reproduction/vehicle_assembly/scene.py`，碰撞代理和原始资产也继续由该装配模块维护。HSC 原生实现位于 `reproduction/hsc_backend/`。绘图的网格准备和相机投影位于 `vehicle_assembly/mesh_render.py`，绘图入口不加载 CUDA 求解器。

## 输出文件

| 路径（相对 `result/vehicle_full_pass/`） | 内容 |
|---|---|
| `<case>/status.json` | 当前状态、已完成帧数、通过判据、累计 HSC 指标 |
| `<case>/parameters.json` / `scene_manifest.json` | 实验参数副本、实际场景配置、材料、质量和接触排除表 |
| `<case>/provenance.json` | 源码及后端 SHA-256、上游 commit、原生工作目录 |
| `<case>/source_snapshot/` | 与该组来源记录中的 SHA-256 对应的源码副本 |
| `<case>/trajectory.jsonl` | 每帧位置、轮胎变形、胎圈误差、悬挂行程和完整通过判据 |
| `<case>/hsc_stats.jsonl` / `frame_stats.jsonl` | 每次线性求解及每帧非线性收敛记录 |
| `<case>/geometry.npz` / `frames/frame_*.npz` | 不变拓扑、逐时刻 ABD 变换和 FEM 顶点；快照为 float32，物理计算和指标为 float64 |
| `<case>/checkpoint.json` | 原生断点帧号、日志位置和 Python 回调状态 |
| `<case>/run.log` | 标准输出、原生 warning 和异常信息 |
| `comparison.json` / `comparison.csv` | 从轨迹计算的两组数值对照 |
| `preview/` | 固定相机行程图、变形曲线和动画 |
| `report.md` | 两组均完成后，从实际数据生成的结果文档 |
| `postprocess.json` | 最终输出清单及数据、绘图代码的指纹，避免重复生成 |

逐帧快照分别压缩保存，日志只追加，避免每帧重写整段历史。中断恢复时，尚未进入断点的日志尾部另存为 `.interrupted-*`，再从断点继续。只有不超过当前已完成帧号的快照进入绘图。

`attempts/table1_interpretation/` 保留用户答复前启动、随后终止的车载胎刚度探索记录，状态为 `STOPPED_SUPERSEDED_BY_USER_SELECTION`。`attempts/stable_neo_hookean/` 保留防翻转修正前的两组记录，较软组曾出现负体积。这些记录不进入正式两组对照。正式目录名 `ground_1e8`、`ground_1e6` 直接表示地面轮胎刚度。每个案例使用进程锁，多个入口不会重复计算同一目录。

`attempts/log_nh_device_initialization/` 保存新材料接入时的设备端异常日志；这次尝试只完成初始化，没有推进任何时间步。之后改用显式小矩阵计算并完成 GPU 检查，再启动正式计算。

在已配置 Python 环境中重建统计和预览：

```powershell
$runtime = Get-Content reproduction/hsc_backend/runtime.json -Raw | ConvertFrom-Json
& $runtime.python reproduction/experiments/vehicle_full_pass/summarize.py
& $runtime.python reproduction/experiments/vehicle_full_pass/render.py --animate
& $runtime.python reproduction/experiments/vehicle_full_pass/report.py
```

HSC 仍采用 ACA、Schur PCG 和 Neumann 预条件；强关节耦合使用已有的 ABD 自适应多项式延长，准备阶段部分位于 CPU。该实现、显式胎圈连接、固定 ABD 刚度及替代网格与论文存在差别；本实验不用于声称论文性能或原场景逐项复现。
