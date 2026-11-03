# 测试数据

| 文件 | 内容 |
|---|---|
| [samples.h](benchmark/samples.h) | 视频性能测试使用的 234 个输入，每个样本依次为目标 XYZ（m）、腕旋转（rad）、五个初始关节角（rad）。样本编号从 0 开始。 |
| [board_results.csv](../board/results/board_results.csv) | 板端原始逐次测量。性能对比选 mode=1，通过 rep 和 target 与电脑记录配对。 |
| [paired_results.csv](benchmark/paired_results.csv) | 234 个目标、3 轮，共 702 条 Python 与板端配对记录。时间单位为微秒。 |
| [summary.json](benchmark/summary.json) | 视频中性能对比的环境与汇总。每个目标先取三轮中位数，再统计共同收敛且需要迭代的目标。 |
| [regression/](regression/) | 原工程离线回归输入及参考记录。 |

来源为既有视频实验的 board_benchmark_20261103 与 python_vs_board_20261103 两组记录。此次目录整理未重跑板端实验。
