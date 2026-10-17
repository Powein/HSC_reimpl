# 汽车完整越过轮胎：两组刚度实验

本模块运行 HMMWV 衍生车辆驶过一排三个地面轮胎的实验。两组车载胎固定为 `E=1e10 Pa`，地面胎分别为 `1e8 Pa`、`1e6 Pa`；车辆、驱动、接触和求解参数共用同一份配置。

[参考结果](../../../result/vehicle_full_pass/report.md)已完成两组完整行程，分别推进 213 帧、209 帧，模拟时长为 7.100 s、6.967 s。每次 Newton 线性系统由原生 HSC 求解，共调用 6,161 次、6,402 次。

## 运行入口

首次使用先按[安装与运行指南](../../../REPOSITORY.md)准备 Python 依赖和 HSC 后端。以下命令均从仓库根目录执行。

默认配置入口：

```powershell
./reproduction/experiments/vehicle_full_pass/run.ps1
```

入口依次处理两组，全部完成后生成统计、图像、动画和报告。默认 [config.json](config.json) 对应仓库中已完成的参考结果，重复执行会保留结果并跳过计算。

新计算先复制配置，分别设置新的 `experiment` 和 `result_directory`，然后指定配置文件：

```powershell
python reproduction/experiments/vehicle_full_pass/run.py --config reproduction/experiments/vehicle_full_pass/config.local.json
```

配置文件的创建步骤见[新实验指南](../../../REPOSITORY.md#运行新的两组刚度实验)。`run.py` 支持 `--config`、`--case` 和 `--resume`；PowerShell 包装入口提供默认配置下的 `-Case`、`-Resume`：

```powershell
./reproduction/experiments/vehicle_full_pass/run.ps1 -Case ground_1e8
./reproduction/experiments/vehicle_full_pass/run.ps1 -Case ground_1e8 -Resume
```

## 实验配置

| 配置项 | 默认值 | 含义 |
|---|---:|---|
| `dt` | 1/30 s | 时间步，对应论文表 1 的 Vehicle 案例 |
| `cases[].loose_E` | 1e8 / 1e6 Pa | 两组地面胎杨氏模量 |
| `materials.mounted_E` | 1e10 Pa | 两组共用的车载胎杨氏模量 |
| `materials.poisson` / `density` | 0.3 / 1000 kg/m³ | 各向同性实体胎的泊松比与密度 |
| `material_model` | `log_neo_hookean` | 对数 Neo-Hookean 材料及单元路径可行性检查 |
| `materials.abd_kappa` / `bead_k` | 1e9 / 1e8 | ABD 刚度参数与胎圈连接刚度；胎圈单位为 N/m |
| `obstacles` | 3 | 一排三个平放地面轮胎 |
| `start_x` | −2.6 m | 车身初始 X 位置 |
| `motor_speed_rad_s` / `motor_ramp_time_s` | 2 rad/s / 0.5 s | 车轮目标角速度与启动渐增时间 |
| `hsc.max_rank` | 256 | ACA 秩上限 |
| `hsc.neumann_order` / `omega` | 5 / 0.7 | Neumann 初始项数及权重 |
| `hsc.adaptive_abd_inverse` / `abd_inverse_tol` | 1 / 1e-5 | 启用 ABD 多项式延长及其逆误差目标 |
| `hsc.max_iter` / `full_residual_limit` | 12000 / 0.1 | Schur PCG 迭代上限与完整系统残差上限 |
| `newton_max_iter` / `max_frames` | 400 / 600 | 每帧 Newton 迭代上限与实验帧数上限 |
| `clearance_m` / `clear_frames` | 0.15 m / 5 | 整车越过障碍的间距与连续保持帧数 |
| `snapshot_every` / `checkpoint_every` | 2 / 10 | 几何快照与原生断点的保存间隔（帧） |

`E=1e10/1e8 Pa` 的车载胎/地面胎组合对应论文 Fig.19；地面胎 `E=1e6 Pa` 是项目增加的柔软对照。车辆使用 Chrono HMMWV 衍生资产，包含 12 ABD、7 FEM 胎体、78,197 个 Tet4。悬挂、质量、接触排除和装配坐标见[车辆装配说明](../../vehicle_assembly/README.md)，实际运行值保存在各组 `scene_manifest.json`。

胎体采用由 E、ν、密度定义的各向同性实体弹性模型。每次线搜索沿整条试探路径要求 `det(F)>1e-8`，每帧再核对实际单元体积，见[材料与体积约束](../../hsc_backend/NONINVERSION.md)。HSC 的分块、残差定义与 CPU/CUDA 分工见[后端说明](../../hsc_backend/README.md)。

## 完整通过的判据

[completion.py](completion.py) 从求解后的实体几何计算以下条件：

1. 四个车轮的轮心分别从对应侧障碍中心的后方越到前方；越过时，轮心位于障碍横向范围内，并高于障碍顶部。
2. 取全部障碍初始最前端和当前最前端的较大值作为边界，车辆全部实体的最后端越过该边界至少 `clearance_m`。
3. 运行中出现过网格接触，且上述条件连续保持 `clear_frames` 帧。

满足条件后状态写为 `COMPLETED_FULL_PASS`。帧数上限、Newton 与线搜索收敛、HSC 残差及单元体积均受检查；计算失败时，入口保存 `FAILED` 状态和诊断日志。`test_completion.py` 覆盖四轮通过、障碍移动和横向位置等判据边界。

## 进度与断点

`<结果目录>/<case>/status.json` 提供当前状态、完成帧数和 HSC 累计指标，`run.log` 保存进程输出。每组使用独立进程锁，运行期间增量追加轨迹和求解记录。

后端路径从 `reproduction/hsc_backend/runtime.json` 读取。原生工作目录位于其 `root/runs/<experiment>/<case>/` 下，完整位置写入 `provenance.json`。断点恢复需要该目录中的原生数据、项目内的 `checkpoint.json`，以及匹配的配置、场景源码和后端二进制。

恢复时，断点之后的日志尾部保存为 `.interrupted-*`，再从断点帧继续。绘图按已完成帧号选择有效快照。新的配置和资产组合使用新的实验名与结果目录。

## 输出文件

以下路径相对于配置中的 `result_directory`。

| 路径 | 内容 |
|---|---|
| `<case>/status.json` | 状态、完成帧数、通过判据和累计 HSC 指标 |
| `<case>/parameters.json` | 本次运行的实验配置及案例参数 |
| `<case>/scene_manifest.json` | 实际材料、质量、几何、关节和接触排除表 |
| `<case>/provenance.json` / `source_snapshot/` | 源码与后端 SHA-256、上游版本、原生工作目录及对应源码副本 |
| `<case>/trajectory.jsonl` | 逐帧位置、轮辋坐标系中的胎体位移、胎圈误差、悬挂行程与通过判据 |
| `<case>/hsc_stats.jsonl` | 逐次线性求解的秩、残差、迭代数和接触数量 |
| `<case>/frame_stats.jsonl` | 逐帧 Newton 与线搜索收敛记录 |
| `<case>/geometry.npz` / `frames/frame_*.npz` | 固定拓扑、ABD 变换和 FEM 顶点；快照为 float32，物理计算与指标为 float64 |
| `<case>/checkpoint.json` | 原生断点帧号、日志位置和 Python 回调状态 |
| `<case>/run.log` | 标准输出、原生提示和异常信息 |
| `comparison.json` / `comparison.csv` | 两组指标及单变量配置核对结果 |
| `preview/` | 固定相机行程图、变形曲线和动画 |
| `report.md` | 由两组已完成数据生成的结果报告 |
| `postprocess.json` | 后处理生成的输出清单与输入指纹 |

参考结果中的[诊断记录](../../../result/vehicle_full_pass/attempts/README.md)按实验目的另行分类，提供参数与终止状态。

## 代码与后处理

| 文件 | 职责 |
|---|---|
| `run.ps1` / `run.py` | 选择运行环境、启动案例、记录进程状态 |
| `run_case.py` | 装配场景、推进 HSC、测量几何、保存状态与断点 |
| `completion.py` | 根据几何测量判断完整通过 |
| `storage.py` | 增量日志、轨迹追加和状态文件原子更新 |
| `summarize.py` | 从保存数据生成 JSON、CSV 对照 |
| `render.py` | 从保存网格生成图表与动画 |
| `report.py` | 从对照数据生成报告 |
| `finalize.py` | 统一执行统计、绘图、报告生成并记录输入指纹 |
| `watch.py` | 跟随运行进度，在两组完成后生成最终输出 |

场景复用 `vehicle_assembly/scene.py`，绘图复用 `vehicle_assembly/mesh_render.py`。安装项目 Python 依赖后，在仓库根目录执行以下命令可重建参考结果的展示文件：

```powershell
python reproduction/experiments/vehicle_full_pass/summarize.py
python reproduction/experiments/vehicle_full_pass/render.py --animate
python reproduction/experiments/vehicle_full_pass/report.py
```

三个入口均支持 `--result result/<实验名>`。报告生成器检查两组的共同配置、场景与后端版本、收敛状态和体积有效性，再生成数值表格。
