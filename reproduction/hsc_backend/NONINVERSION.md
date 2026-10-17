# 轮胎材料与单元体积约束

完整行程采用压缩性、各向同性的对数 Neo-Hookean 胎体。材料注册名为 `LogNeoHookean`，本地 UID 为 `10010`；每次 Newton 线搜索检查整条试探路径的四面体体积，保证接受的状态处于正体积可行域。

两组实验共用该材料实现，车载胎 E 为 `1e10 Pa`，地面胎 E 分别为 `1e8 Pa`、`1e6 Pa`，泊松比为 0.3，密度为 1000 kg/m³。

## 能量与导数

能量密度为：

\[
\psi(F)=\frac{\mu}{2}(\|F\|_F^2-3)-\mu\ln J+\frac{\lambda}{2}(\ln J)^2,
\qquad J=\det F>0.
\]

Lamé 参数由杨氏模量 E、泊松比 ν 计算：`μ=E/[2(1+ν)]`，`λ=Eν/[(1+ν)(1−2ν)]`。CUDA 内核计算梯度和 Hessian；Hessian 使用最多 32 轮循环 Jacobi 分解，将负特征值截断后，与接触、ABD 和关节项共同装配给 HSC。

能量、导数和小矩阵运算集中在 [log_neo_hookean_function.h](overlays/src/backends/cuda/finite_element/constitutions/log_neo_hookean_function.h)，三维单元装配位于 [log_neo_hookean_3d.cu](overlays/src/backends/cuda/finite_element/constitutions/log_neo_hookean_3d.cu)。

## 线搜索路径检查

对试探步检查 `F(s)=F0+s(Ftrial−F0)`，其中 `s∈[0,1]`。行列式 `det(F(s))` 是三次多项式，因此检查区间两端及区间内导数为零的位置，即可取得整条路径的最小体积比。

可行域阈值为 `J>1e-8`。越过阈值的试探步返回正无穷能量，回溯线搜索随之缩短步长；该检查也适用于满足微小步终止条件的候选方向。初始能量及接受状态需要通过有限性检查。

[run_case.py](../experiments/vehicle_full_pass/run_case.py) 还会逐帧从实际顶点计算单元体积。非有限或非正体积触发异常快照与失败状态。原生路径检查约束连续的试探步，逐帧测量记录最终接受的几何，两者共同保存在运行数据中。

## 数值记录

[check_log_nh.py](check_log_nh.py) 调用与 CUDA 内核共用的函数头，核对能量梯度、解析 Hessian、Jacobi 投影、五种路径情况及 GPU 设备调用。[log_nh_check.json](log_nh_check.json) 保存具体样本与结果。

| 检查 | 已保存结果 |
|---|---:|
| 随机变形梯度样本 | 32 个 |
| 最大归一化梯度误差 | 3.45333e-10 |
| 最大归一化 Hessian 误差 | 8.69671e-11 |
| 地面胎 1e8 Pa 完整行程的最小体积比 | 0.68947 |
| 地面胎 1e6 Pa 完整行程的最小体积比 | 0.16173 |

材料比较与初始化诊断的参数、终止状态见[诊断记录目录](../../result/vehicle_full_pass/attempts/README.md)。完整行程的逐帧体积与收敛记录见[结果报告](../../result/vehicle_full_pass/report.md)。

## 接入与来源

接口和单元装配结构基于 libuipc 提交 `9c748a759bb9952a7aeedd59076a70591dc61d6c`，新增材料位于 `overlays/`，线搜索改动由 `apply_backend.py` 应用并汇总到 `integration.patch`。上游原文件位于 `pristine/`，构建入口为 `build_backend.py`。

对数 Neo-Hookean 及上述体积路径检查是本项目的材料实现，作为弹性与可行性约束接入 [HSC 后端](README.md)。接触流程参考 [IPC 论文与实现](https://github.com/ipc-sim/IPC)及 libuipc。上游根许可证和声明保存在 [UPSTREAM-LICENSE](UPSTREAM-LICENSE)、[UPSTREAM-NOTICE](UPSTREAM-NOTICE)，各组件的来源见[第三方说明](../vehicle_research/THIRD_PARTY_NOTICES.md)。
