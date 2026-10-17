# HC / HSC 核心算法复现

当前完整行程实验使用独立入口 [experiments/vehicle_full_pass](experiments/vehicle_full_pass/README.md)，两组刚度的结果保存到项目根目录 `result/vehicle_full_pass/`。本页保留初期核心算法验证的参数和数据。

后续场景进度：已为 HMMWV 替代资产补齐闭合碰撞代理、四套悬挂、轮毂转动和胎圈连接，并将接触、约束及弹性共同装配的矩阵接入原生 HSC，进入实际 Newton/IPC 时间推进。详见 [HSC 原生接入](hsc_backend/README.md) 和 [汽车装配实现](vehicle_assembly/README.md)；前期来源检索见 [资产与碰撞方案](vehicle_research/README.md)。原生版本对强关节耦合加入了自适应 ABD Neumann 延长，仍使用替代资产；整个障碍阵列的穿越、论文固定阶算法与全 GPU 性能对照尚未完成。

已完成 **核心数值算法的部分复现，以及部分原始 CUDA 函数的 GPU 验证**。尚未复现论文中的棋盘、扭转环、直升机等完整物理场景，也没有验证论文的帧耗时或加速比。

本目录是新增的验证工程。上级 `Guo-2026-HSC/` 的 34 个原始文件与用户提供的 ZIP 逐字节一致，没有修改。所有数值表格均由实际运行产生。

## 运行

在 Windows PowerShell 中执行：

```powershell
& 'C:\Users\Powei\Desktop\复现HC\reproduction\run.ps1'
```

本机依赖已下载至 `_deps/`。脚本优先使用 Codex 附带的 Python 3.12；也可通过 `-Python '你的python.exe绝对路径'` 指定解释器。重新安装依赖时运行同目录的 `setup.ps1`。依赖版本完整保存在 `requirements-lock.txt`。不需要全局安装 CUDA Toolkit；原函数 GPU 检查通过 NVRTC 12.9 和已有 NVIDIA 驱动运行。

只运行 CPU 部分：

```powershell
& 'C:\Users\Powei\Desktop\复现HC\reproduction\run.ps1' -SkipGpu
```

运行脚本保存 stdout/stderr，设置 600 秒硬超时，超时只结束本次实验进程。最后一次完整运行内部计时约 11 秒，退出码为 0。该时间是整个小型验证脚本的耗时，不能与论文的每帧耗时比较。若在 Codex 受限进程中读取下载的二进制依赖失败，应使用获准的本地运行环境；普通 PowerShell 可直接运行。本次最终运行在获准的本地进程中完成。

## 已得到的结果

| 检查 | 实际结果 |
|---|---|
| CPU 数值检查 | 13/13 通过 |
| 原始 CUDA 函数检查 | 12 类，每类 20 个固定种子样本，全部通过 |
| GPU | NVIDIA GeForce RTX 3060 Ti，compute capability 8.6，8 GB |
| ACA 精确低秩重建 | 相对误差约 10⁻¹⁶ |
| 小型精确 Schur 对照与独立直接解 | 解的相对误差 6.10×10⁻¹³ |
| 原始 GPU 函数与解析式 | 最大绝对误差 2.14×10⁻¹⁴ |
| 原始 GPU 导数与有限差分 | 最大绝对误差 8.34×10⁻¹⁰ |

主实验使用 `ω=0.7`、五阶 Neumann、180 个可变形自由度和 36 个仿射自由度。以下是三个**人工构造的对称正定系统**，不是论文中的场景、网格或物理时间步。

| 构造刚度尺度 | 单次 HSC 原系统相对残差 | 外层校正次数 | 校正后原系统相对残差 | 相对直接解误差 |
|---:|---:|---:|---:|---:|
| 10² | 1.025×10⁻³ | 6 | 8.708×10⁻¹¹ | 2.234×10⁻⁹ |
| 10⁴ | 2.388×10⁻³ | 5 | 8.762×10⁻¹¹ | 8.565×10⁻⁹ |
| 10⁶ | 2.430×10⁻³ | 4 | 4.103×10⁻¹¹ | 1.198×10⁻⁷ |

残差很小不等于前向误差同样小；最后一个系统的条件数约 4.23×10⁵，因此同时报告相对解误差。所有原系统残差都使用原始矩阵 H 重新计算，而非只检查 ACA 压缩后的系统。

一阶 Neumann 在本次设定的 50 次外层校正预算内，三个系统均未达到 10⁻⁹ 目标；三阶、五阶、八阶均达到目标。这些未达标记录保留在结果中，不能把它们解释成无限迭代也不收敛。

在刚度尺度 10⁶ 的固定秩消融中，ACA 最大秩为 4、8、12、18 时，外层校正分别需要 45、10、5、4 次。这体现了这个测试系统上的压缩误差与校正次数关系，不是论文场景性能结论。

![数值验证曲线](results/core_validation.png)

## 实现范围与论文的对应

| 论文内容 | 本次处理 |
|---|---|
| 正文公式 (2)、(3)：分块系统与 Schur 补 | `core.py` 实现，使用独立直接解核验 |
| 正文 Algorithm 1：全主元 ACA | NumPy 实现；固定最大秩，并保留数值零主元保护 |
| 正文公式 (5)、(6)：Neumann 逆近似 | 分别使用完整 3×3、12×12 块逆；验证显式矩阵多项式恒等式 |
| 可变形子系统 PCG | 对 Schur 算子迭代，用 D 的 Neumann 算子预条件 |
| 非精确求解后的误差校正 | 在人工二次能量上执行带解析步长的外层校正；不等同论文完整非线性 IPC 时间积分 |
| 部分作者 CUDA 代码 | 保持函数体不变，抽取至独立 NVRTC 编译单元并在 GPU 执行 |
| 附录 COO 排序归约 ACA | 已阅读算法，没有实现 GPU `sort_by_key/reduce_by_key` 版本 |
| 旋转关节、FEM/ABD、IPC、CCD、摩擦、场景资产 | 本页初期核心验证不包含；后续 `vehicle_assembly/` 使用 Chrono/libuipc 实现替代场景，`hsc_backend/` 接入原生 HSC；作者原场景仍未公开 |

GPU 检查包含仿射投影及梯度/Hessian、`rotational_*` 能量和导数、3×3/12×12 稀疏矩阵乘法、3×3/12×12 对角块乘加。其编译单元只补充 `memset` 的依赖与调用包装，并记录每个提取函数的 SHA-256。**没有声称整个原始库已经成功编译。**

公开包的 `rotational_energy` 实际计算两个仿射点之间的二次距离。其有限差分通过，验证的是这个函数本身；不能据此认定正文公式 (11) 的完整转动约束已复现。

## 原始资料核对

来源为用户的 `HC.pdf`、`Guo-2026-HSC.zip`，以及从[作者项目页](https://sites.google.com/view/multibody)下载的技术附录和当前代码包。当前公开代码 ZIP 与用户 ZIP 的 SHA-256 完全相同，均为 `ec833f3d707233b9d3efb9c5e979b10e7ef6a2dcdb6e7835a8658cb30b0e455b`。下载来源和哈希保存在 `downloads/provenance.json`。正文 16 页、附录 4 页均通过结构预检；结构预检不代表论文科学结论已经验证。

核对发现：

1. README 明确将代码定位为需要集成的头文件库，没有独立构建、可执行程序或 benchmark driver。
2. `sparse_blocks_nonsquare_cuda.cuh` 缺失，两个现有文件引用了它；README 提到的 `PCG_fem_abd_coupling_cuda` 和 KKT 组件也没有相应实现文件。
3. 未找到 ACA 和 Schur/HSC 主流程、完整物理时间积分、碰撞/CCD/摩擦、场景网格和配置、原始实验矩阵或日志。
4. 原包 PCG 默认 `ω=0.6667`、Neumann 阶数 1；论文使用 `ω=0.7` 并讨论五阶。原包 `tol` 比较的是平方残差，默认绝对阈值 10⁻¹²；本验证显式采用未平方相对范数。
5. 原包 `jacobi_cuda.cuh` 对大于 3 的块只取标量对角倒数。本参考实现对仿射部分使用完整 12×12 块逆，因此不是原包 PCG 的逐行移植。
6. 附录第 2 页明确 ACA 仅按固定最大秩停止。主实验采用 `k_max=min(m,n)=36`，本例在秩 18 时触及浮点零主元保护。另做秩 4/8/12/18 消融；容差截断试验单独标为非论文诊断。论文场景各自使用的秩预算仍未给出。

附录还给出了刚体误差 `||F Fᵀ-I||²_F > 10⁻⁸` 时，将刚度按 `min(1.2κ,10¹⁷)` 更新、初始 `κ=10⁹` 等信息。这些参数已提取到 `paper_parameters.json`，本验证工程尚未实现相应非线性物理流程。

## 文件索引

| 文件 | 用途 |
|---|---|
| `core.py` | 独立可读的 CPU 算法参考与测试系统构造 |
| `cuda_original.py` | 原始 CUDA 函数抽取、NVRTC 编译、GPU 调用和检查 |
| `run.py` / `run.ps1` | 一键运行与日志 |
| `paper_parameters.json` | 论文/附录参数、出处和未解决的报告细节 |
| `results/numerical_checks.json` | 13 项 CPU 数值检查 |
| `results/gpu_checks.json` | GPU 误差、阈值、设备和原函数哈希 |
| `results/experiments.json` | 完整残差曲线、秩消融、谱半径、收敛状态 |
| `results/summary.csv` | 12 组刚度/阶数实验汇总 |
| `results/fixture_*.npz` | 真实输入矩阵、右端、已知解与独立直接解 |
| `results/core_validation.png` / `.svg` | 数值结果图，可直接查看或导出 |
| `results/source_audit.json` / `environment.json` | 源码完整性与运行环境 |
| `results/status.json` / `run.log` / `stderr.log` | 最终运行状态和日志 |
| `downloads/HC-technical-document.pdf` | 作者技术附录 |

## 继续复现完整场景所需材料

初期核对发现的 HSC/ACA 驱动、FEM/ABD 时间推进和 IPC/摩擦/CCD 缺口，现已通过 `hsc_backend/` 与 libuipc 补齐，车辆也已进入真实的 Newton/IPC 循环。完整行程的两组地面胎刚度实验由 `experiments/vehicle_full_pass/` 管理，实际状态见根目录 `result/vehicle_full_pass/`。

仍缺少作者原场景的完整网格、逐部件材质与初始条件、关节/边界配置、ACA 秩预算和原始运行指标。当前替代场景的运行结果，不能据此写成作者原场景或论文性能已经复现；初期的 **CANNOT_VERIFY** 结论仍适用于这些尚未获得材料支持的对照。

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-10-17 (Asia/Shanghai)
- Verification Status: VERIFIED，仅限已执行的核心数值检查和指定 GPU 函数；论文完整场景 CANNOT_VERIFY
- Version Label: hc_core_exp_result_v1
- Experiment ID: hc-core-20261017
- Type: numerical simulation / reproducibility validation
- Status: completed，exit code 0；低阶消融未达标状态原样保留
- Source: 用户 PDF/ZIP 与作者公开附录；没有添加用户已读声明
- Execution: 本会话单代理执行，无外部模型传送
