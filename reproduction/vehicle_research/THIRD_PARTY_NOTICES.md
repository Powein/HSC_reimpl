# 第三方来源与衍生内容

本目录的 `upstream/` 保存固定版本的精选资产、参考源码与许可证。[download_manifest.json](download_manifest.json)提供逐文件 URL、commit、字节数和 SHA-256，供核对来源与原始内容。

## 来源和使用范围

| 来源 | 许可证原文 | 使用范围 |
|---|---|---|
| Project Chrono | [BSD-3-Clause](upstream/projectchrono__chrono/LICENSE) | HMMWV 车身、轮辋、轮胎、配置及加载器参考 |
| libuipc 根目录 | [Apache-2.0](upstream/spiriMirror__libuipc/LICENSE) | IPC、CCD、FEM/ABD 和关节后端；组件实验使用 pyuipc 0.0.28，完整行程使用本地构建的 0.9.0 |
| libuipc Python 子目录 | [GPL-3.0](upstream/spiriMirror__libuipc/python/LICENSE) | 固定源码版本中的 Python 子目录许可文件 |
| libuipc-samples | [Apache-2.0](upstream/spiriMirror__libuipc-samples/LICENSE) | 关节、驱动、连接与接触示例参考 |
| Stiff-GIPC | [MPL-2.0](upstream/KemengHuang__Stiff-GIPC/LICENSE)及 [README 使用说明](upstream/KemengHuang__Stiff-GIPC/README.md) | GPU 接触与刚柔求解方案的参考源码 |
| IPC Toolkit | [MIT](upstream/ipc-sim__ipc-toolkit/LICENSE) | 距离、barrier、摩擦与 CCD 的参考源码 |
| 作者 HSC 代码 | [BSD-3-Clause](../../Guo-2026-HSC/LICENSE.txt) | `Guo-2026-HSC/` 中的作者原始代码与函数核对来源 |

libuipc 根目录、README 和发布元数据标注 Apache-2.0，所固定 commit 的 `python/LICENSE` 为 GPL-3.0；两份原文按目录保留。Stiff-GIPC 同时保留根许可证和 README 中的学术、商业使用说明。使用和分发具体文件时，应一并查阅相应目录及依赖的许可声明。

Chrono HMMWV 的 [README](upstream/projectchrono__chrono/data/vehicle/hmmwv/README.md)给出 JSON 模型参数的来源，包括 James Aardema 的公开 HMMWV 后球头失效分析报告。资产随附 Chrono 根目录的 BSD-3-Clause 许可证。几何处理依赖的许可证副本保存在 `reproduction/vehicle_assembly/licenses/`，版本记录在该模块的 `environment.json`。

## 资产衍生过程

[prepare_assets.py](prepare_assets.py)完成以下转换：

1. 将 C3D10 的四个角节点提取为线性 Tet4，保留原始节点编号及节点集。
2. 将轮胎绕 Z 旋转 90°，轮轴由 X 对齐到 Y，等比缩放到最大半径 0.47 m。
3. 根据 Chrono 参考坐标与项目布局，放置车身、四个车载胎及十二个地面胎，生成静态 OBJ、NPZ、MSH、JSON 与预览。

[build_proxies.py](../vehicle_assembly/build_proxies.py)进一步使用 Trimesh 和 Manifold 构建闭合车身、轮辋、主轴代理。胎圈连接使用 Chrono 的 `BC_CONN` 节点集，悬挂采用由 Chrono 参数换算的等效滑动机构。

`prepared/` 中的场景预览用于查看静态布局；`result/vehicle_full_pass/` 的图像与逐帧顶点来自一排三个地面胎的实际 HSC 模拟。两类数据分别保存在资产目录和实验结果目录中。

## 后端衍生代码

HSC 后端基于 libuipc 提交 `9c748a759bb9952a7aeedd59076a70591dc61d6c`，加入 `linear_hsc`、对数 Neo-Hookean 及体积可行性线搜索。代码、补丁与来源记录见 [hsc_backend](../hsc_backend/README.md)，上游根许可证与 NOTICE 分别保存在 [UPSTREAM-LICENSE](../hsc_backend/UPSTREAM-LICENSE)、[UPSTREAM-NOTICE](../hsc_backend/UPSTREAM-NOTICE)。构建脚本下载完整的固定版本源码到本机构建目录。

发布或转交 Chrono 衍生资产时，应同时保留本文件与 Chrono 许可证；其他第三方文件随各自来源保留许可和声明。
