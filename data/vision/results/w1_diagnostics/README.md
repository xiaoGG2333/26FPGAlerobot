# results/w1_diagnostics/

W1 D1 只读诊断基线数据目录。由 `sw/diag/run_w1_d1.py` 自动按时间戳创建子目录写入。

每个子目录:
- `static/summary.json` — ID1-6 逐样本反馈(位置/电压/温度/负载)
- `w1_d1_summary.json` — 汇总 + ID2 漂移统计 + IK 单解计时

规则(计划书红线 7):实测数据必须注明条件(板卡、固件版本、接线、日期),不引用历史数字冒充新验证。
