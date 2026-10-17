# HC / HSC 论文复现

两组汽车完整行程均已完成，全程使用接入 Newton/IPC 循环的原生 HSC。车载胎固定 `E=1e10 Pa`，地面胎 `E=1e8`、`1e6 Pa` 两组分别完成 213、209 帧，模拟时长为 7.100、6.967 秒，结束时整车均已驶过全部障碍。

本轮数据独立保存在 `result/vehicle_full_pass/`。查看[结果报告](result/vehicle_full_pass/report.md)、[完整行程动画](result/vehicle_full_pass/preview/passage.gif)和[数值对照](result/vehicle_full_pass/comparison.csv)；运行入口和参数说明见[完整行程实验](reproduction/experiments/vehicle_full_pass/README.md)。每组 `status.json` 中的 `COMPLETED_FULL_PASS` 对应四轮越过障碍、整车留出至少 0.15 m 间距并保持五帧的实际判定。

Git 版本包含正式结果和必要资产；本机依赖、编译缓存及原始 PDF 的处理方式，以及克隆后的环境准备，见[仓库说明](REPOSITORY.md)。

```powershell
./reproduction/experiments/vehicle_full_pass/run.ps1
```

| 路径 | 内容 |
|---|---|
| `HC.pdf` / `Guo-2026-HSC.zip` | 用户提供的论文与作者源码包 |
| `Guo-2026-HSC/` | 原始源码解包，保留原文件 |
| `reproduction/experiments/vehicle_full_pass/` | 本轮两组完整行程实验的配置、启动、统计和绘图入口 |
| `result/vehicle_full_pass/` | 本轮实际结果、运行参数副本、求解日志和预览 |
| `reproduction/vehicle_assembly/` | 可复用车辆场景、碰撞代理、悬挂和胎圈连接 |
| `reproduction/hsc_backend/` | libuipc 原生 HSC 补丁、构建入口和本机环境记录 |
| `reproduction/vehicle_research/` | 开源资产、来源与许可证、预处理网格 |
| `reproduction/results/` | 初期算法和原始 CUDA 函数验证的历史结果 |
| `reproduction/vehicle_assembly/results/` | 使用库自带求解器的早期装配短测 |
| `reproduction/hsc_backend/results/` | 本轮之前的 HSC 接触初期结果 |

各阶段结果保留原位置，避免改动历史引用。新增实验按 `reproduction/experiments/<实验名>/` 放代码，按根目录 `result/<实验名>/` 放数据。第三方许可见 [来源与许可说明](reproduction/vehicle_research/THIRD_PARTY_NOTICES.md)。

公开包未包含作者完整车辆场景。当前使用 Chrono 替代资产、显式胎圈连接，以及部分 CPU 准备和 ABD 自适应多项式延长的 HSC 实现；具体差别见 [HSC 接入说明](reproduction/hsc_backend/README.md)。实验完成与论文原资产、固定阶算法及性能复现分别记录。

完整行程统一使用对数 Neo-Hookean 胎体，并在原生线搜索中约束四面体体积；首次软胎翻转的记录与修正方法见[防翻转说明](reproduction/hsc_backend/NONINVERSION.md)。最终报告仅从两组已完成且体积有效的轨迹生成，文档文字按 `humanizer-zh` 技能整理。
