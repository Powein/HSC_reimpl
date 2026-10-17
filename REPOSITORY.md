# 安装与运行指南

本指南从克隆后的仓库根目录开始，分别介绍查看结果、构建 HSC 后端和运行新实验。全部命令使用 Windows PowerShell；下文的 `python` 指向 Python 3.12 的 64 位解释器。

## 查看和分析已有结果

[完整行程报告](result/vehicle_full_pass/report.md)、[动画](result/vehicle_full_pass/preview/passage.gif)和[CSV 对照表](result/vehicle_full_pass/comparison.csv)均可直接查看。两组原始数据位于 `result/vehicle_full_pass/ground_1e8/` 和 `ground_1e6/`，文件格式见[实验说明](reproduction/experiments/vehicle_full_pass/README.md#输出文件)。

安装绘图依赖后，可以从保存的轨迹重新生成统计、图像和报告：

```powershell
python --version
./reproduction/setup.ps1 -Python python
python reproduction/experiments/vehicle_full_pass/summarize.py
python reproduction/experiments/vehicle_full_pass/render.py --animate
python reproduction/experiments/vehicle_full_pass/report.py
```

依赖按 [requirements-lock.txt](reproduction/requirements-lock.txt) 安装到 `reproduction/_deps/`。绘图读取已保存的几何状态，统计与报告读取 JSON/JSONL 数据。处理其他实验目录时，三个 Python 入口均支持 `--result result/<实验名>`；`--animate` 用于同时生成 GIF。

## 构建 HSC 后端

| 环境项 | 构建脚本使用的配置 |
|---|---|
| 操作系统与解释器 | Windows x64、Python 3.12 |
| C++ 编译器 | Visual Studio 2022 Community，安装 C++ 桌面开发工具 |
| GPU | CUDA GPU；默认编译目标 `sm_86`，已有结果运行于 RTX 3060 Ti 8 GB |
| CUDA 编译依赖 | NVIDIA 官方 CUDA 12.8.1 redistributable，由准备脚本下载 |
| Windows SDK | Microsoft Windows SDK 10.0.26100.0，由准备脚本安装到构建目录 |
| 源码和包管理 | Git、网络连接；libuipc 和 vcpkg 使用脚本固定的版本 |

Visual Studio 路径和 CUDA 架构在 [build_backend.py](reproduction/hsc_backend/build_backend.py) 中配置。使用其他 Visual Studio 安装位置时，调整 `environment()` 中的 `vs` 路径；使用其他 GPU 编译目标时，同时调整 `UIPC_CUDA_ARCHITECTURES` 和 `CMAKE_CUDA_ARCHITECTURES`。

先安装项目依赖和几何工具：

```powershell
./reproduction/setup.ps1 -Python python
python -m pip install --no-deps --target reproduction/vehicle_research/_geometry -r reproduction/vehicle_assembly/requirements-geometry.txt
```

后端使用独立的构建目录和 Python 虚拟环境。默认在系统临时目录创建 `hc-hsc-build-*` 目录；也可以先在容量充足的磁盘上创建一个路径为 ASCII 字符的目录，再创建 `reproduction/hsc_backend/build_location.json`，将 `root` 设为该目录的绝对路径，例如：

```json
{"root": "D:/HSC-build"}
```

将示例盘符改为本机实际位置，然后依次运行：

```powershell
python reproduction/hsc_backend/prepare_build.py
python reproduction/hsc_backend/prepare_sdk.py
python reproduction/hsc_backend/prepare_dependencies.py
python reproduction/hsc_backend/build_backend.py --jobs 4
```

四个脚本分别准备固定版本源码与 CUDA、Windows SDK、C++ 依赖，再应用补丁并编译。成功后生成 `reproduction/hsc_backend/runtime.json`，记录后端路径、运行解释器、构建根目录和源码版本。实验入口通过该文件选择原生 `linear_hsc`。算法、参数和补丁结构见 [HSC 后端说明](reproduction/hsc_backend/README.md)。

## 运行新的两组刚度实验

仓库的默认配置指向已完成的参考结果。重复运行同一配置时，入口保留这些结果并跳过计算。新计算使用独立的配置文件、实验名和结果目录：

```powershell
Copy-Item reproduction/experiments/vehicle_full_pass/config.json reproduction/experiments/vehicle_full_pass/config.local.json
```

编辑 `config.local.json`，将下面两个字段改为新名称，其余参数保持原值：

```json
"experiment": "vehicle_full_pass_run1",
"result_directory": "result/vehicle_full_pass_run1"
```

`experiment` 决定原生缓存目录名，`result_directory` 决定项目内的数据目录。每个新实验使用各自的名称。随后执行：

```powershell
python reproduction/experiments/vehicle_full_pass/run.py --config reproduction/experiments/vehicle_full_pass/config.local.json
```

入口依次运行 `ground_1e8`、`ground_1e6`。两组车载胎固定为 `E=1e10 Pa`，只改变地面胎杨氏模量；全部完成后自动生成对照表、预览和报告。参数、完成判据与各脚本职责见[完整行程实验](reproduction/experiments/vehicle_full_pass/README.md)。

在另一个 PowerShell 窗口查看状态和最新日志：

```powershell
Get-Content result/vehicle_full_pass_run1/ground_1e8/status.json
Get-Content result/vehicle_full_pass_run1/ground_1e8/run.log -Tail 30
```

单独运行一组时添加 `--case ground_1e8`。恢复同一台机器上的中断计算时使用同一配置，并添加 `--resume`：

```powershell
python reproduction/experiments/vehicle_full_pass/run.py --config reproduction/experiments/vehicle_full_pass/config.local.json --case ground_1e8 --resume
```

恢复需要对应的原生断点、相同参数、场景源码及后端二进制。原生工作目录为 `runtime.json` 中的 `root/runs/<experiment>/<case>/`，确切路径记录在结果的 `provenance.json`；项目内的 `checkpoint.json` 保存与该目录配套的帧号和回调状态。

## 目录与数据约定

| 路径 | 内容与用途 |
|---|---|
| `Guo-2026-HSC/` | 作者发布的 CUDA 头文件与 BSD-3-Clause 许可证 |
| `reproduction/experiments/` | 具名实验的配置、运行、统计和绘图代码 |
| `result/vehicle_full_pass/` | 两组完整行程数据、图表、动画和报告 |
| `reproduction/vehicle_assembly/assets/` | 仿真使用的闭合车身、主轴和轮辋代理 |
| `reproduction/vehicle_research/upstream/` | 固定版本的原始资产、参考源码和许可证 |
| `reproduction/vehicle_research/prepared/` | 转换后的 Tet4 轮胎和静态场景预览 |
| `reproduction/hsc_backend/` | HSC、材料实现、补丁和构建入口 |
| `reproduction/results/` 及各模块 `results/` | 核心算法、装配和接触实验的测量记录 |
| `reproduction/_deps/`、`reproduction/vehicle_research/_geometry/` | 安装脚本生成的本机 Python 依赖 |
| 后端构建根目录 | 本机源码、编译工具、二进制、原生运行缓存与断点 |

各组 `parameters.json` 和 `scene_manifest.json` 保存实际运行参数，`provenance.json` 保存源码与后端 SHA-256，`source_snapshot/` 保存对应版本的源码。历史结果中的绝对路径用于定位原始运行环境；在新机器上构建后，入口使用新生成的 `runtime.json`。

`.gitattributes` 保留文件字节和换行，以便核对来源清单及源码快照的 SHA-256。[诊断记录目录](result/vehicle_full_pass/attempts/README.md)提供材料与初始化实验的参数和终止状态。论文、技术附录可从[作者项目页](https://sites.google.com/view/multibody)获取，第三方版本与许可证见[来源说明](reproduction/vehicle_research/THIRD_PARTY_NOTICES.md)。
