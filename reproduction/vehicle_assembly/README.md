# 车辆装配

本模块从 Project Chrono 的 HMMWV 资产构建刚柔耦合车辆，提供闭合碰撞代理、四套独立悬挂、轮毂驱动和胎圈连接。[scene.py](scene.py) 供组件实验与 [HSC 完整行程](../experiments/vehicle_full_pass/README.md)共同使用。

![碰撞代理与组件实验](preview/assembly_overview.png)

## 部件与连接

| 部件 | 实现与规模 |
|---|---|
| 车身 ABD | 原碰撞外形的闭合凸包，布尔切除四个轮罩；283 顶点、562 三角形 |
| 轮辋 ABD | 沿 Y 轴的闭合圆柱；98 顶点、192 三角形，半径 0.23543 m |
| 主轴 ABD | 0.10 × 0.12 × 0.12 m 闭合盒体，合并主轴与滑块的机构作用 |
| 四套悬挂 | 车身—主轴滑动关节、弹簧、轴向阻尼与行程限位 |
| 车轮转动 | 主轴—轮辋转动关节及目标角速度驱动，通过关节反力带动车辆 |
| FEM 胎体 | 每胎 3,548 顶点、11,171 个 Tet4；完整行程使用对数 Neo-Hookean |
| 胎圈连接 | 每胎 39 个 `BC_CONN` 角节点，经 `SoftVertexStitch` 连接到轮辋锚点 |
| 接触 | libuipc IPC barrier、连续碰撞检测（CCD）与摩擦，统一处理 ABD/FEM |

连接关系为：`车身 → 滑动关节、弹簧和阻尼 → 主轴 → 转动关节 → 轮辋 → 胎圈约束 → 胎体`。车辆前进方向为 +X，竖直方向为 +Z，轮轴沿 Y。

[build_proxies.py](build_proxies.py) 使用 Trimesh 和 Manifold 生成闭合代理；轮辋半径依据胎体内孔边段的最小径向距离，预留 6 mm 径向间隙。闭合性、法向、体积和三角形质量的记录见 [proxy_audit.json](assets/proxy_audit.json)。代理同时用于碰撞与 ABD 质量积分，重心和惯量由代理几何计算。

### 胎圈、弹簧与阻尼

胎圈采用零原长能量 `E = 0.5 kb ||x_tire − (A_rim X_anchor + p_rim)||²`。锚点随轮辋 ABD 变换运动，刚柔交叉项参与全局求解。每个连接点的刚度为 `1e8 N/m`。

悬挂弹簧采用 `E = 0.5 ks (L − L0)²`，作为隐式能量项进入 Newton。轴向阻尼使用上一帧关节位移差估算速度，施加 `F = −c v`，按帧显式更新。关节、胎圈和行程限位均使用有限刚度约束，运行数据记录其位移与连接误差。

### 接触配置

实体轮辋—胎体、轮胎之间、胎体自接触和地面接触均进入碰撞流程。接触排除针对有意重叠的车身—所属主轴、主轴—所属轮辋，以及用于绑定的虚拟锚点。虚拟锚点专用于约束映射，质量与接触由实体代理承担。每次运行在 `scene_manifest.json` 保存完整接触排除表。

IPC 接触、摩擦和关节项先装配到全局矩阵，再由运行入口选定的线性求解器处理。完整行程入口使用 [HSC 后端](../hsc_backend/README.md)；本模块的组件入口使用 pyuipc 0.0.28 自带求解器。

## 参数

| 参数 | 数值 | 来源或定义 |
|---|---:|---|
| 车身质量 | 2086.52 kg | Chrono HMMWV JSON |
| 单个轮辋质量 | 18.8 kg | Chrono 车轮 JSON |
| 单个合并主轴质量 | 34.155 kg | spindle 14.705 kg + upright 19.45 kg |
| 胎体泊松比 / 密度 | 0.3 / 1000 kg/m³ | 项目材料参数；单胎约 63.8095 kg |
| 等效悬挂刚度 | 45,374.0375 N/m | Chrono 前悬弹簧近零斜率 × 运动比 0.5 的平方 |
| 等效悬挂阻尼 | 4,753.8923 N·s/m | Chrono 前悬阻尼 × 同一运动比的平方 |
| 悬挂行程 / 初始弹簧长度 | ±0.15 m / 0.35 m | 简化滑动机构的参数 |
| 整车弹簧原长 | 约 0.46266 m | 按每角承担四分之一车身重力设置预压 |
| 重力 / 摩擦系数 | 9.8 m/s² / 0.6 | 项目场景参数 |
| 接触激活距离 | 1 mm | IPC 参数 |
| 组件实验时间步 | 0.005 s | `run.ps1` / `validate_assembly.py` |
| 完整行程时间步 | 1/30 s | 完整行程配置，对应论文表 1 |

`paper_stiffness` 配置使用车载胎 `E=1e10 Pa`、地面胎 `E=1e8 Pa`，对应论文 Fig.19；`assembly` 配置使用 `1e8/1e6 Pa`，便于观察组件变形。完整行程另由独立配置控制两组地面胎刚度。

胎体采用各向同性实体弹性，原 C3D10 网格取角节点转换为线性 Tet4。悬挂使用四套相同的等效滑动机构，胎圈采用 Chrono 节点集绑定。论文附录中的轮毂、轮辐接触传动与本项目节点绑定的对应关系见[结果报告](../../result/vehicle_full_pass/report.md)。

接触刚度由 libuipc 根据场景调整，输入请求值为 `1e10`。例如四轮平地案例的初始有效值约为 `6.891e11`，16 胎装配案例约为 `5.579e11`；有效值及原生提示保存在日志和 [verification.json](verification.json) 中。

## 运行组件实验

在仓库根目录安装 Python 3.12 依赖及 pyuipc 组件运行环境：

```powershell
./reproduction/vehicle_assembly/setup.ps1 -Python python
```

然后按需要选择案例：

```powershell
# 固定轮辋与胎圈连接
./reproduction/vehicle_assembly/run.ps1 -Python python -Case bead -Steps 40

# 单轮悬挂与驱动
./reproduction/vehicle_assembly/run.ps1 -Python python -Case suspension -Steps 120

# 四轮整车在平地滚动
./reproduction/vehicle_assembly/run.ps1 -Python python -Case vehicle -Steps 80

# 四个车载胎与十二个地面胎的装配
./reproduction/vehicle_assembly/run.ps1 -Python python -Case vehicle -Profile paper_stiffness -Obstacles 12 -Steps 12
```

默认创建带时间戳的结果与日志目录。`-Label` 设置名称，`-TimeoutSeconds` 设置运行时限，默认 300 s；`-MotorSpeed`、`-StartX` 分别设置驱动角速度和起点。结果保存在 `reproduction/vehicle_assembly/results/<Label>/`。每 10 步保存部分结果，完整运行记录最终状态。

资产重建命令为 `python reproduction/vehicle_assembly/build_proxies.py`。该脚本默认采用凸包与轮罩布尔切除；`--coacd` 提供 CoACD 凸分解实验路径。正式完整行程的环境准备和新实验运行见[安装与运行指南](../../REPOSITORY.md)。

## 已保存的组件结果

以下结果使用 Windows、Python 3.12、pyuipc 0.0.28 和 RTX 3060 Ti 8 GB，材料为组件入口的 Stable Neo-Hookean。

| 案例 | 时间范围 | 测量结果 |
|---|---:|---|
| 固定轮辋与重力下的 FEM 胎体 | 40 步 / 0.20 s | 胎圈最大误差 0.278 μm，单元体积为正 |
| 单轮悬挂与轮毂驱动 | 120 步 / 0.60 s | 转角 0.480000 rad，胎圈最大误差 1.969 μm，下沉 24.916 mm |
| 四轮平地滚动 | 80 步 / 0.40 s | 车身前移 0.13084 m，胎圈最大误差 12.775 μm |
| 四车载胎与十二地面胎 | 12 步 / 0.06 s | 21 ABD、16 FEM；胎圈最大误差 11.044 μm，最小体积比 0.99824 |
| 首排三个障碍的接触初期 | 40 步 / 0.20 s | 车身前移 0.10681 m，胎圈最大误差 16.847 μm，最小体积比 0.44746 |

[测量汇总](verification.json)、[曲线](preview/validation_curves.png)和[悬挂动画](preview/suspension_test.gif)提供组件响应。首排接触的独立几何检查覆盖 9 个保存时刻、189 次轮胎对检查，2,704 个候选四面体对中正体积相交数为 0，距离容差为 `1e-9 m`；具体范围见 [independent_overlap_check.json](results/obstacle_entry/independent_overlap_check.json)。

整车完整通过的两组 HSC 数据统一保存在 [result/vehicle_full_pass](../../result/vehicle_full_pass/report.md)。

## 文件与来源

| 文件或目录 | 用途 |
|---|---|
| `scene.py` | 可复用场景、材料、关节、胎圈和接触配置 |
| `assets/` / `build_proxies.py` | 闭合代理、参数审计与生成脚本 |
| `validate_assembly.py` / `run.ps1` | 组件实验与进程管理 |
| `check_tire_overlap.py` | 保存帧中不同轮胎的独立四面体相交检查 |
| `audit_results.py` / `render_results.py` | 已保存组件案例的测量汇总与绘图 |
| `mesh_render.py` | 共享网格加载、相机投影与展示工具 |
| `references/` / `references_manifest.json` | 13 份参考源码或规范及其固定版本、SHA-256 |
| `licenses/` / `environment.json` | 依赖许可证与组件实验环境 |

主要来源是 [Project Chrono HMMWV](https://github.com/projectchrono/chrono/tree/c5651d36a204a12e5c349eda0b141f47551f9c36/data/vehicle/hmmwv)、[Chrono FEATire](https://github.com/projectchrono/chrono/blob/c5651d36a204a12e5c349eda0b141f47551f9c36/src/chrono_vehicle/wheeled_vehicle/tire/FEATire.cpp)及 [libuipc ABD–FEM 连接示例](https://github.com/spiriMirror/libuipc-samples/blob/a7ca63fb33a3d6964e28ef5d187e2c817d20b8f5/examples/22_soft_vertex_stitch/main.py)。代理构造使用 [Trimesh](https://trimesh.org/) 和 [Manifold](https://github.com/elalish/manifold)。许可证和各来源的使用范围见[第三方来源说明](../vehicle_research/THIRD_PARTY_NOTICES.md)。
