# 轮胎单元的防翻转处理

完整行程的第一轮计算使用 libuipc 的 `StableNeoHookean`。较软地面胎在第 20–24 帧出现负体积，最小体积比为 −0.0186021438605995；两组均已停止，数据保留在 `result/vehicle_full_pass/attempts/stable_neo_hookean/`。这批轨迹不作为正式结果。

该模型的能量允许翻转，表面 IPC 接触也不能单独约束内部四面体。修正后的两组统一使用新增的 `LogNeoHookean`（本地 UID 10010），保持车载胎 `E=1e10 Pa`，只改变地面胎 `E=1e8/1e6 Pa`。

## 能量及线搜索

采用压缩性对数 Neo-Hookean 能量密度：

\[
\psi(F)=\frac{\mu}{2}(\|F\|_F^2-3)-\mu\ln J+\frac{\lambda}{2}(\ln J)^2,
\quad J=\det F>0.
\]

`mu`、`lambda` 由给定 E、ν 计算，不使用 Stable Neo-Hookean 的系数重标定。梯度和 Hessian 在 CUDA 中计算，Hessian 通过最多 32 轮的循环 Jacobi 分解截断负特征值，再与接触、ABD 和关节项共同交给 HSC。小矩阵使用显式计算，避免此次 Windows/CUDA 环境中通用 Eigen 运算的设备端异常。

每次线搜索还检查 `F(s)=F0+s(Ftrial−F0)`，`s∈[0,1]` 上的最小行列式。行列式是三次多项式，检查两端和区间内导数为零的位置即可。只检查两端会漏掉中间经过两次零体积的情况。当前数值域要求 `J>1e-8`；试探步不满足时返回正无穷能量，由回溯线搜索缩短步长。初始迭代能量、NaN 和负无穷仍视为错误，已满足收敛阈值的微小步也不能跳过可行性检查。

`run_case.py` 每帧额外读取实际顶点体积；非有限或非正体积会保存异常快照并立即失败。该监测不能代替原生路径检查，两者检查的是不同层面。

## 来源和可追溯性

接口、装配和 CUDA 内核结构改编自固定提交 `9c748a759bb9952a7aeedd59076a70591dc61d6c` 的 libuipc，根许可证为 Apache-2.0，随附 [UPSTREAM-LICENSE](UPSTREAM-LICENSE) 和 [UPSTREAM-NOTICE](UPSTREAM-NOTICE)。新增文件保存在 `overlays/`，所有后端改动由 `apply_backend.py` 写入 `integration.patch`，原始文件保存在 `pristine/`。构建入口仍是 `build_backend.py`。

[IPC 论文和参考实现](https://github.com/ipc-sim/IPC)将无交叉、无翻转作为时间推进要求。本次对数 NH 和单元路径检查是本地补齐的实现；没有证据表明作者 HSC 车辆场景使用了完全相同的本构参数与检查代码。HSC 算法没有因这次材料修正而改成其他求解器。

`check_log_nh.py` 编译并调用与 CUDA 内核共用的函数头，对能量梯度、未投影 Hessian 作有限差分检查，比较 Jacobi 投影与 CPU 特征分解结果，并检查五种路径情况及实际设备调用。32 个变形梯度样本的最大归一化梯度误差为 3.45333e-10，Hessian 误差为 8.69671e-11，详见 [log_nh_check.json](log_nh_check.json)。这是本次数值实现的定向检查，不重复运行车辆装配测试。
