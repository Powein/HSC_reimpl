# 仓库内容与本机环境

本仓库保存 HSC 复现代码、固定版本的第三方参考文件、Chrono 衍生车辆资产，以及两组完整行程的正式结果。结果入口为 [result/vehicle_full_pass/report.md](result/vehicle_full_pass/report.md)。每组的参数、逐帧轨迹、原生 HSC 日志、网格快照和源码副本均随仓库保存，可直接查看或重新绘图。

原始论文 PDF、重复 ZIP、Python/CUDA 二进制依赖、编译缓存和本机运行路径不进入 Git。论文与技术附录可从[作者项目页](https://sites.google.com/view/multibody)获取；已经解包的作者 BSD-3-Clause 源码保存在 `Guo-2026-HSC/`。历史尝试在 `result/vehicle_full_pass/attempts/` 中只提交参数和终止状态，完整中间数据仍保留在原实验机器上。

`.gitattributes` 禁用自动换行转换，以保留下载清单和实验源码副本的 SHA-256。结果中的绝对路径是运行来源记录，不代表克隆仓库后会存在相同的磁盘目录。第三方代码和资产保留各自许可证，见[来源说明](reproduction/vehicle_research/THIRD_PARTY_NOTICES.md)。

## 在新机器上使用

查看报告、图表和动画不需要安装求解器。需要重新绘图时，在 Windows 的 Python 3.12 环境中安装项目绘图依赖，然后从项目根目录运行：

```powershell
./reproduction/setup.ps1 -Python python
python reproduction/experiments/vehicle_full_pass/render.py --animate
```

重新运行物理实验需要编译带补丁的 libuipc，不能用普通 pyuipc 包替代 HSC。当前构建脚本针对 Windows、Visual Studio 2022 Community 和 CUDA `sm_86`，具体过程见[后端构建说明](reproduction/hsc_backend/README.md)。成功构建会在本机生成 `reproduction/hsc_backend/runtime.json`；该文件不提交到仓库。若要指定构建磁盘，可在运行准备脚本前创建同目录的 `build_location.json`，填写本机绝对路径，例如 `{"root":"E:/HSC-build"}`，并按实际磁盘修改。

```powershell
Push-Location reproduction/hsc_backend
python prepare_build.py
python prepare_sdk.py
python prepare_dependencies.py
python build_backend.py --jobs 4
Pop-Location
./reproduction/experiments/vehicle_full_pass/run.ps1
```

已完成的同配置结果会跳过计算。开展新实验时，应修改配置中的 `experiment` 和 `result_directory`，保留本次结果。Git 中的 `checkpoint.json` 仅记录原实验断点；原生求解器缓存不随仓库分发，不能仅凭这个 JSON 在另一台机器上续算。

