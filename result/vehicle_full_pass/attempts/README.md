# 历史尝试

这里的尝试不计入正式两组对照。Git 版本保留各组 `parameters.json` 和 `status.json`，用于说明参数和终止原因；网格、中间日志及源码副本保留在原实验机器上。

- `table1_interpretation/`：在确定单变量方案之前改变车载胎刚度的探索，随后终止。
- `stable_neo_hookean/`：较软地面胎出现负体积，两组均停止，改用带路径可行性检查的对数 Neo-Hookean 模型重跑。
- `log_nh_device_initialization/`：新材料初次接入时的设备端异常，没有完成时间步。

正式结果为上一级目录的 `ground_1e8/` 和 `ground_1e6/`，两组均已完成整车通过。

