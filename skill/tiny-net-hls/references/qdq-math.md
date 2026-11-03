# int8 QDQ 的定点语义

一份能跑通的 numpy/C++ 定点参考模型，全部依赖下面这些规则。任何一条搞错，现象都不是「小误差」，而是整体饱和到 0 或 255。

## 1. 先看激活的 dtype

QDQ 图的激活多半是 **QUInt8（uint8, 0..255）**，不是 int8。判据：zero_point 出现 131/128/125 这类 >127 的值，或者某个输出恒定在 127/255。

把 uint8 强转 `np.int8`，131 会变成 -125，然后整条链全饱和。写参考模型前先 `print` 一次 dtype、zero_point 分布。

对比：权重是 **QInt8，zero_point 恒为 0**，per-output-channel scale。

## 2. 每个张量的三个量怎么取

在 QDQ 图里**不能按 tensor 名字找 scale**，量化器会把 Q/DQ 插进来并改名：

- 某层的输入 key = 喂给它的那个 `DequantizeLinear` 的 **input[0]**（那是个 `QuantizeLinear` 的输出名，不是 initializer）。
- scale / zero_point 永远从 **DQ 的 scale initializer** 读；只有当那个 int8 张量本身是 initializer 时才知道它的形状。
- 值会经过 `Flatten/Reshape/Squeeze/Unsqueeze/Identity`，名字跟着变。做法：`qmap[Q_output] = 数组`，再建 `alias[q_of(node.output)] = xkey(node.input)`，取不到时沿 alias 链走。
- 一个 `GAP -> Q -> DQ -> Flatten -> Q` 的双重量化会把 FC 的输入藏起来，最容易漏。

## 3. 一层卷积的完整公式

```
acc_int32 = sum( (x_uint8[i] - x_zp) * w_int8[i] )      # 逐输出通道
y_uint8   = clamp( round(acc * M) + y_zp , 0, 255 )
M         = x_scale * w_scale[c] / y_scale
```

要点：

- **零填充在量化域里写 `x_zp`，不是 0**。写 0 会让每层 max diff 变成 255、median 是 0。
- **bias initializer 已经是 int32 且按 `x_scale*w_scale` 预缩放过的**，直接加，别再除一次 scale。
- 累加用 int32 就够：最坏 `C * 131 * 127`，即使 C=432 也只有 7.2e6。只有 `acc*mult` 这一步（~3e16）需要 int64。
- 硬件里 `x_zp != 0` 不花代价：`sum((x-x_zp)w) = sum(x*w) - x_zp*sum(w)`，后者折进 per-channel 常量（TFLite 方案）。所以不用为了对称性重新量化。

## 4. TFLite 式 requant（推荐给硬件）

```
M = mult / 2^(31+shift),  M ∈ [0.5, 1)   # 不断乘 2 直到落进区间，再拆出 2^31
y = clamp( (acc*mult + (1<<(30+shift))) >> (31+shift) + y_zp , 0, 255 )
```

`acc*mult` 用 int64。这套编码比浮点乘快，也比「近似成移位」准得多——实测 per-layer 的 `M` 大多落在 0.001~0.025，纯移位近似会直接废掉精度。

## 5. 残差 add 和 GAP 是两个特殊层

**add**（`a` 是 skip 分支，`b` 是本块输出）：

```
out = clamp( round( ((a - a_zp)*a_scale + (b - b_zp)*b_scale) / y_scale ) + y_zp , 0, 255 )
```

- 两条分支各减自己的 zero_point，漏掉任何一个都会产生 ~128 LSB 的整体偏移并往后传。
- **ONNX 里 add 的 input[0] 是 skip、input[1] 是本块输出**，参数按这个顺序绑；反了会在 12 LSB 那一档卡住（真实案例：修对之后 12.90 → 0.87 LSB）。
- 硬件实现用 Q20 定点：`am = round(a_scale/y_scale*2**20)`、`bm = round(b_scale/y_scale*2**20)`，`v = (a-a_zp)*am + (b-b_zp)*bm`，再带符号右移（负数要 `-(((-v)+rnd)>>S)` 才能和「四舍五入远离零」一致）。

**GAP**：`y = clamp( round( mean(x - x_zp) * x_scale / y_scale ) + y_zp , 0, 255)`。
漏掉 `- x_zp` 会让 131 左右的均值乘上 ~3.4 的比值，输出直接钉在 255，后面 FC 全废——而热图分支完全正常，所以热图对得上不能证明图是对的。

## 6. ReLU6 不用单独的 Clip 节点

量化器会把 ReLU6 折进量化区间：输出 `zp=0, scale=6/255`，正好覆盖 [0,6]。所以

- 图里看不到任何 `Clip`/`Relu` 节点是**正常**的；
- 硬件只要按 uint8 范围 `clamp(0,255)`，就等于同时实现了 ReLU6 的上下界。

判据：`zp == 0` 的那些层就是 ReLU6 输出。

## 7. 逐层指纹（回归用）

第一次跑通后，把某一帧的每层输出 `shape/min/max/mean` 存成基线。之后任何改动先对指纹，比重新调参快一个数量级。示例（51 层 = 40 conv + 8 add + GAP + 3 FC）：

```
L00 conv 24x96x96  [0,255]   13.5
L02 conv 16x96x96  [28,227] 130.9
L09 add  24x48x48  [32,216] 127.8
L47 conv 21x12x12  [17,249]  97.6   <- 热图输出
L48 gap  72        [..]     ...
L49 fc   256       [0,255]   41.6
L50 fc   1         231
L51 fc   2         [76,180] 128.0
```

## 8. 校准集

用**域内**图像（和部署场景同源），256 张就够，正负样本按真实比例混。换房间/换灯光后重采。MinMax 在样本少时容易压不住，可先试 Percentile/Entropy 再决定要不要 QAT——先量化再判断，别一上来就 QAT。
