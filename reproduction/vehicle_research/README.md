# 车辆与轮胎资产

本目录提供 Project Chrono HMMWV 原始资产、线性 Tet4 轮胎、静态场景预览，以及 libuipc 接触流程的参考源码。物理装配由相邻的 [vehicle_assembly](../vehicle_assembly/README.md) 模块完成，[完整行程实验](../experiments/vehicle_full_pass/README.md)使用其中一排三个地面轮胎进行两组刚度计算。

![整车与十二个地面轮胎的静态资产布局](prepared/scene_preview.png)

## 可用资产

原始文件保存在 `upstream/projectchrono__chrono/`，来自固定版本 [Project Chrono HMMWV](https://github.com/projectchrono/chrono/tree/c5651d36a204a12e5c349eda0b141f47551f9c36/data/vehicle/hmmwv)。

| 文件 | 几何规模 | 在项目中的用途 |
|---|---|---|
| `hmmwv_chassis_simple.obj` | 7,678 顶点、15,330 三角形 | 车身外观展示 |
| `hmmwv_chassis_col.obj` | 880 顶点、1,685 三角形 | 闭合车身碰撞代理的形状来源 |
| `hmmwv_rim.obj` | 6,424 顶点、12,381 三角形 | 轮辋外观与尺度参考 |
| `hmmwv_tire_coarse_closed.obj` | 306 顶点、612 三角形 | 闭合轮胎表面模型 |
| `tire/HMMWV_lugged_coarse.inp` | 21,359 节点、11,171 个 C3D10 | 有限元胎体及轮辋连接节点集 |
| 车辆、悬挂、车轮与轮胎 JSON | 轴距、安装点、质量、惯量、弹簧与阻尼曲线 | 装配坐标及等效机构参数来源 |

原始外观和碰撞参考网格的边界、非流形边、退化面记录在 [asset_audit.json](prepared/asset_audit.json) 中。用于仿真的闭合车身、轮辋和主轴代理位于 [vehicle_assembly/assets](../vehicle_assembly/README.md#部件与连接)，其质量和碰撞均使用代理几何。

### 转换后的轮胎

| 文件 | 内容 |
|---|---|
| [tire_tet4.npz](prepared/tire_tet4.npz) | 3,548 顶点、11,171 个 Tet4、6,184 个边界三角形及原始节点编号、节点集 |
| [tire_tet4.msh](prepared/tire_tet4.msh) | Gmsh 2.2 ASCII 网格，便于导入其他工具 |
| [tire_tet4_surface.obj](prepared/tire_tet4_surface.obj) | 从体积网格提取的轮胎表面 |
| [vehicle_over_tires_static.obj](prepared/vehicle_over_tires_static.obj) | 整车与十二个平放车轮的静态资产布局 |
| [scene_manifest.json](prepared/scene_manifest.json) | 静态布局的坐标、旋转、单位和部件定义 |
| [asset_audit.json](prepared/asset_audit.json) | 网格计数、质量检查和尺度变换 |

`prepare_assets.py` 取 C3D10 的四个角节点构成线性 Tet4，将轮轴从 X 旋转到 Y，再等比缩放到最大半径 0.47 m。转换后全部单元体积为正，边界闭合且三角形非退化。

Chrono 的 `BC_CONN` 节点集用于轮辋连接，转换后保留 39 个角节点；`BC_INT` 是内表面相关节点集，保留 291 个角节点。车辆装配利用 `BC_CONN` 将胎体绑定到 ABD 轮辋。节点集的定义可查阅随附的 [FEATire.cpp](upstream/projectchrono__chrono/src/chrono_vehicle/wheeled_vehicle/tire/FEATire.cpp) 和 [ChFEATire.cpp](upstream/projectchrono__chrono/src/chrono_vehicle/wheeled_vehicle/tire/ChFEATire.cpp)。

静态布局使用四个车载胎和十二个地面胎，共 178,736 个 Tet4；完整行程配置选用四个车载胎和三个地面胎，共 78,197 个 Tet4。仿真的实际连接、材料和初始状态记录在各组结果的 `scene_manifest.json`。

## 接触实现索引

车辆使用 libuipc 的 CUDA IPC 接触系统。下面列出本目录保存的参考源码，路径均相对于 `upstream/spiriMirror__libuipc/`。

| 环节 | 实现 | 源码位置 |
|---|---|---|
| 接触候选 | 扫掠 AABB 与 BVH，筛选点–三角形、边–边等几何对 | `src/backends/cuda/collision_detection/filters/stackless_bvh_simplex_trajectory_filter.cu` |
| 法向接触 | IPC barrier 能量、梯度与半正定 Hessian | `src/backends/cuda/contact_system/contact_models/ipc_simplex_normal_contact.cu` |
| 连续碰撞检测 | 加性 CCD 与保守推进，给出安全步长上界 | `src/backends/cuda/utils/distance/details/ccd.inl` |
| 切向摩擦 | 基于前一状态的正则化摩擦势 | `src/backends/cuda/contact_system/contact_models/ipc_simplex_frictional_contact.cu` |
| 刚柔耦合 | 将空间接触导数投影到 ABD/FEM 自由度 | `src/backends/cuda/coupling_system/abd_fem_linear_subsystem.cu` |
| 非线性步进 | Newton、CCD 步长约束与能量线搜索 | `src/backends/cuda/line_search/line_searcher.cu` |

对仿射顶点 `x = A X + p`，接触项通过常量 Jacobian 映射为 `g_q = Jᵀ g_x`、`H_q = Jᵀ H_x J`。这些项与弹性、悬挂和胎圈项一起进入全局矩阵，再由 [HSC 后端](../hsc_backend/README.md)求解。轮胎单元的连续体积约束见[材料实现](../hsc_backend/NONINVERSION.md)。

## 重建资产与接触样例

使用 Windows PowerShell、Python 3.12，从仓库根目录安装基础依赖并重建静态资产：

```powershell
./reproduction/setup.ps1 -Python python
python reproduction/vehicle_research/prepare_assets.py
```

`fetch_sources.py` 根据固定 commit 下载清单内的精选文件，并核对已有文件的 SHA-256：

```powershell
python reproduction/vehicle_research/fetch_sources.py
```

接触样例使用 pyuipc 0.0.28。安装组件运行环境后，可检查 CUDA 加载和小型刚柔接触：

```powershell
./reproduction/vehicle_assembly/setup.ps1 -Python python
python reproduction/vehicle_research/probe_cuda.py
python reproduction/vehicle_research/contact_probe.py
```

[已保存的接触样例](prepared/contact_probe_result.json)为一个 ABD 方块落在 FEM 方块上，时间步 0.005 s，共 120 步。逐帧四面体检查记录的体积相交数为 0，FEM 最低点距地面至少约 0.6604 mm，最后一步压缩约 3.9515 mm。样例参数见 [contact_probe_config.json](prepared/contact_probe_config.json)。

## 来源与版本

[download_manifest.json](download_manifest.json)记录 54 份资产、源码和许可证的 URL、commit、字节数及 SHA-256。`sources/` 保存检索时的仓库与资源目录信息。

| 来源 | 固定版本 | 用途 |
|---|---|---|
| Project Chrono | `c5651d36a204a12e5c349eda0b141f47551f9c36` | HMMWV 资产、节点集和机构参数 |
| libuipc | `9c748a759bb9952a7aeedd59076a70591dc61d6c` | IPC、FEM/ABD、关节及原生 HSC 接入基础 |
| libuipc-samples | `a7ca63fb33a3d6964e28ef5d187e2c817d20b8f5` | 接触、滑动关节、转动关节与驱动示例 |
| Stiff-GIPC | `bb2849a7b292099581907937860d96ecfdf42588` | GPU IPC、ACCD 和刚柔求解的参考源码 |
| IPC Toolkit | `4b3059c8539eed52fc9319c44756fa2136b88fa6` | 距离、barrier、摩擦和 CCD 的参考源码 |

论文车辆案例见[作者项目页](https://sites.google.com/view/multibody)的 Fig.19 及技术附录 Fig.3。各资产和软件的许可证、衍生内容见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
