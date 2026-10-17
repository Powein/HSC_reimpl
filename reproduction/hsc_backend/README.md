# HSC 原生后端

本模块将 HSC 接入 libuipc 的 Newton/IPC 时间推进。碰撞、摩擦、FEM/ABD 弹性、悬挂和胎圈约束共同装配全局 Hessian，由 `linear_hsc` 计算更新方向，再交给 CCD 步长限制、线搜索和状态回写。

后端基于 libuipc 提交 `9c748a759bb9952a7aeedd59076a70591dc61d6c`，提供 ACA 压缩、Schur PCG、Neumann 预条件和逐次求解日志。[两组完整行程](../../result/vehicle_full_pass/report.md)使用这一后端，分别完成 213 帧和 209 帧。

## 求解流程

按 ABD 自由度 q、FEM 自由度 x 分块，全局线性系统写为：

\[
\begin{bmatrix} A & E \\ E^T & D \end{bmatrix}
\begin{bmatrix}\Delta q\\\Delta x\end{bmatrix}
=\begin{bmatrix}b_q\\b_x\end{bmatrix}.
\]

自由度区间读取 `ABDLinearSubsystem` 和 `FEMLinearSubsystem` 的实际 offset/count。接入层将上游保存的三角块补为对称矩阵，并收集刚柔耦合块的非零接口列。

1. 全主元 ACA 以给定秩上限近似耦合矩阵 `E ≈ U Vᵀ`，使用机器精度级零主元保护。
2. 对 ABD 块 A 使用完整 12×12 块 Jacobi 逆、权重 `ω=0.7` 和五项 Neumann 级数，得到逆近似 P。
3. 根据 ABD 逆误差延长多项式，形成 Schur 算子 `S = D − V(Uᵀ P U)Vᵀ`。
4. 在 CUDA 上求解 `S Δx = bₓ − V Uᵀ P bq`。FEM 预条件采用完整 3×3 块 Jacobi 逆和五项 Neumann 级数。
5. 回代 `Δq = P(bq − U Vᵀ Δx)`，检查完整系统残差和下降方向，再将方向交回 Newton。

较小的 ABD 矩阵与 ACA 准备使用 CPU Eigen；大规模稀疏乘法、FEM 预条件和 Schur PCG 在 CUDA 上执行。

### ABD 逆近似

默认使用倍增公式 `P₂ᵣ = Pᵣ + Pᵣ(I − A Pᵣ)` 延长 ABD Neumann 多项式，直至 `||I − A P||F / sqrt(n_ABD) ≤ 1e-5`，最多 48 次倍增。这里的 `n_ABD` 是 ABD 自由度数，每次求解都会记录倍增次数与逆误差。

论文讨论的固定五项近似对应 `-FixedNeumann` / `--fixed-neumann` 模式；自适应模式用于车辆强关节耦合。两种模式均经过完整系统残差检查。固定五项车辆诊断的完整系统相对残差约为 0.998，其终止记录保存在 `results/paper_contact_40/`；自适应模式的完整行程实测残差见下表。

### 残差与日志

Schur PCG 使用相对残差范数目标 `1e-4`。随后用原始全局矩阵重新计算完整系统相对残差，上限为 `0.1`。两种残差分别反映 Schur 迭代精度，以及包含 ACA、ABD 逆近似影响的完整方向精度。

每次调用在 `hsc_stats.jsonl` 中记录 ACA 秩与误差、逆误差、PCG 次数、两类残差、接触数量、耗时及 `fallback` 字段。HSC 收敛或方向检查失败时，运行入口保存失败状态与日志。

## 完整行程配置与结果

| 项目 | 配置或实测值 |
|---|---|
| 场景 | 12 ABD、7 FEM 胎体；ABD / FEM 自由度为 144 / 74,508 |
| 时间步与材料 | 1/30 s；对数 Neo-Hookean，整条线搜索路径要求 `det(F)>1e-8` |
| 车载胎 E | 两组共用 1e10 Pa |
| 地面胎 E | 1e8 / 1e6 Pa |
| ACA 秩上限 / 实际秩 | 256 / 84–96 |
| Neumann 初始项数 / ABD 倍增次数 | 5 / 13–16 |
| Schur PCG 迭代上限 | 12000 |
| HSC 调用次数 | 6,161 / 6,402 |
| HSC 求解占比 | 两组均为全部线性求解 |
| 最大完整系统相对残差 | 9.8978e-5 / 9.9794e-5 |

完整配置在 [config.json](../experiments/vehicle_full_pass/config.json)。材料能量、导数和可行性检查见[轮胎材料与体积约束](NONINVERSION.md)。

## 构建与运行

按[安装与运行指南](../../REPOSITORY.md#构建-hsc-后端)准备 Windows、Python 3.12、Visual Studio 2022 Community、项目依赖和几何工具。构建目录由 `build_location.json` 管理；在仓库根目录依次运行：

```powershell
python reproduction/hsc_backend/prepare_build.py
python reproduction/hsc_backend/prepare_sdk.py
python reproduction/hsc_backend/prepare_dependencies.py
python reproduction/hsc_backend/build_backend.py --jobs 4
```

准备脚本使用固定版本 libuipc、vcpkg、NVIDIA CUDA redistributable 和 Microsoft Windows SDK，并核对下载校验值。构建完成后，`runtime.json` 记录本机 Python、原生库和构建根目录。

完整行程使用[实验入口](../experiments/vehicle_full_pass/README.md)。本模块也提供按指定步数运行的接触实验：

```powershell
./reproduction/hsc_backend/run.ps1 -Steps 40 -Obstacles 3 -MotorSpeed 2 -StartX -2.6 -Label contact_run1
```

该入口默认 `paper_stiffness`、`dt=0.005 s`、Stable Neo-Hookean 和 ACA 秩上限 128，结果写入 `reproduction/hsc_backend/results/<Label>/`。`-Obstacles` 支持 0–12，`-Rank` 设置 ACA 秩上限，`-FixedNeumann` 选择固定五项模式。默认标签带时间戳；指定标签时，为每次实验使用独立名称。

从保存的接触实验快照绘图：

```powershell
$runtime = Get-Content reproduction/hsc_backend/runtime.json -Raw | ConvertFrom-Json
& $runtime.python reproduction/hsc_backend/render_trajectory.py reproduction/hsc_backend/results/contact_run1
```

已保存的 [40 帧接触实验](results/paper_contact_adaptive_40/status.json)覆盖 0.200 s，调用 HSC 601 次，车身前移 0.114381 m，最大完整系统残差为 9.8111e-5。其[轨迹图](results/paper_contact_adaptive_40/preview/trajectory.png)和[动画](results/paper_contact_adaptive_40/preview/trajectory.gif)可用于查看接触初期响应。

## 源码与来源

| 路径 | 内容 |
|---|---|
| `linear_hsc.cu` | 原生 HSC 分块、ACA、Neumann、Schur PCG 与求解统计 |
| `overlays/` | 对数 Neo-Hookean 的 C++、CUDA 和 Python 接口 |
| `apply_backend.py` / `integration.patch` | 后端补丁应用与完整改动记录 |
| `pristine/` | 补丁所依据的上游原文件 |
| `prepare_*.py` / `build_backend.py` | 环境准备和编译流程 |
| `run_vehicle.py` / `run.ps1` | 指定步数的 HSC 车辆接触实验 |
| `check_log_nh.py` / `log_nh_check.json` | 材料导数、路径检查与设备执行的数值核对 |

算法依据为[论文及技术附录](https://sites.google.com/view/multibody)中的分块系统、ACA、Schur 求解和 Neumann 近似。后端接口基于 [libuipc 固定版本](https://github.com/spiriMirror/libuipc/tree/9c748a759bb9952a7aeedd59076a70591dc61d6c)；其根许可证与 NOTICE 保存在 [UPSTREAM-LICENSE](UPSTREAM-LICENSE)、[UPSTREAM-NOTICE](UPSTREAM-NOTICE)。各来源的使用范围与子目录许可见[第三方来源说明](../vehicle_research/THIRD_PARTY_NOTICES.md)。
