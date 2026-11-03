"""手部检测网络: 输入 144x144x3 -> (conf, u, v)。

结构约束(计划书 §4, HLS 化前置):
  - 全卷积, 无 FC 大矩阵乘
  - 静态 shape, 无动态控制流, 无 NMS
  - 算子白名单: conv / depthwise-conv / BN(推理时融合) / ReLU / hard-sigmoid / avgpool
    (+ soft-argmax 头的 softmax: 见下方 HLS 备注)
  - INT8 QAT 友好

头设计（2026-10-14 修订，红线 3 经主人授权）:
  - **conf 头**：s6(18x18,96) -> GAP -> 1x1 conv -> 1（原方案，实测召回 95.7%，保留）
  - **u/v 头**：soft-argmax 热图。s4(36x36,64) -> 1x1(64->24) -> dw3x3 -> 1x1(24->1)
    -> softmax(β) -> 空间期望坐标。stride 4 => 4px/格，配亚格期望可达 <4px。
  - 旧头（GAP + 1x1 出 3 通道）**定位能力被证死**：96 维 GAP 特征线性探针天花板 26.9px，
    见 `results/w1_detect_train/README.md` §6。故默认换 soft-argmax；`head='gap'` 仅供对照复现。

HLS 备注: softmax 需要 exp(1296 点)。若不想要 exp，可换等价的无 exp 版本
`w = relu(h); p = w / sum(w)`（归一化 ReLU 质心），精度待测。ASIC/PL 侧 exp 通常用 LUT。

MACs 预算: 总预算 30M（检测+分类）；本版检测 21.1M @144x144，分类 4.78M -> 25.8M ✅
"""
import torch
import torch.nn as nn


def conv_bn_relu(in_c, out_c, k, s=1, groups=1):
    return nn.Sequential(
        nn.Conv2d(in_c, out_c, k, stride=s, padding=k // 2, groups=groups, bias=False),
        nn.BatchNorm2d(out_c),
        nn.ReLU(inplace=True))


class DetectNet(nn.Module):
    def __init__(self, head='softargmax'):
        super().__init__()
        assert head in ('softargmax', 'gap'), head
        self.head_kind = head
        # 144 -> 72 -> 36 -> 18 （stride 2/2/2：stem, s3, s5）
        self.stem = conv_bn_relu(3, 12, 3, s=2)
        self.s2 = nn.Sequential(conv_bn_relu(12, 12, 3, groups=12), conv_bn_relu(12, 32, 1))
        self.s3 = nn.Sequential(conv_bn_relu(32, 32, 3, s=2, groups=32), conv_bn_relu(32, 64, 1))
        self.s4 = nn.Sequential(conv_bn_relu(64, 64, 3, groups=64), conv_bn_relu(64, 64, 1))   # 36x36
        self.s5 = nn.Sequential(conv_bn_relu(64, 64, 3, s=2, groups=64), conv_bn_relu(64, 96, 1))
        self.s6 = nn.Sequential(conv_bn_relu(96, 96, 3, groups=96), conv_bn_relu(96, 96, 1))   # 18x18

        if head == 'softargmax':
            # conf: 全局特征（GAP）
            self.conf_head = nn.Conv2d(96, 1, 1)
            # u/v: stride-4 热图 -> 空间期望
            self.uv_head = nn.Sequential(
                conv_bn_relu(64, 24, 1),
                conv_bn_relu(24, 24, 3, groups=24),
                nn.Conv2d(24, 1, 1),
            )
            self.log_beta = nn.Parameter(torch.tensor(1.5))   # β = exp(1.5) ≈ 4.5
        else:
            self.head = nn.Conv2d(96, 3, 1)                   # 旧结构: conf,u,v 同一读出

    # ---------- 前向 ----------
    def _backbone(self, x):
        x = self.stem(x)
        x = self.s2(x)
        x = self.s3(x)
        f4 = self.s4(x)
        x = self.s5(f4)
        f6 = self.s6(x)
        return f4, f6

    def forward(self, x, return_hm=False):
        """返回 (conf_logit, uv)。uv 已是归一化 0~1 坐标。return_hm=True 时额外返回 36x36 热图 logits。"""
        f4, f6 = self._backbone(x)
        if self.head_kind == 'gap':
            out = self.head(f6.mean(dim=(2, 3), keepdim=True)).view(x.size(0), 3)
            if return_hm:
                raise ValueError('gap 头没有热图')
            return out[:, 0], torch.sigmoid(out[:, 1:])
        conf = self.conf_head(f6.mean(dim=(2, 3), keepdim=True)).view(x.size(0))
        hm = self.uv_head(f4)
        uv = self.soft_argmax(hm)
        return (conf, uv, hm) if return_hm else (conf, uv)

    def soft_argmax(self, logits):
        """热图 -> 坐标期望（格中心约定，避免边缘偏置）。"""
        b, _, h, w = logits.shape
        beta = self.log_beta.clamp(-2.0, 4.0).exp()
        p = torch.softmax((logits * beta).view(b, -1), dim=-1)
        iy = torch.arange(h, dtype=p.dtype, device=p.device).view(1, h, 1).expand(b, h, w)
        ix = torch.arange(w, dtype=p.dtype, device=p.device).view(1, 1, w).expand(b, h, w)
        u = (p * (ix.reshape(b, -1) + 0.5)).sum(1) / w
        v = (p * (iy.reshape(b, -1) + 0.5)).sum(1) / h
        return torch.stack([u, v], dim=1)

    def predict(self, x):
        """推理: (conf 0~1, u, v 归一化 0~1)。"""
        conf, uv = self.forward(x)
        return torch.cat([torch.sigmoid(conf).unsqueeze(1), uv], dim=1)

    def heatmap(self, x):
        """可视化用: 返回 (conf 0~1, uv, 36x36 热图概率)。"""
        f4, f6 = self._backbone(x)
        logits = self.uv_head(f4)
        b = logits.size(0)
        beta = self.log_beta.clamp(-2.0, 4.0).exp()
        p = torch.softmax((logits * beta).view(b, -1), dim=-1).view(logits.shape)
        conf = torch.sigmoid(self.conf_head(f6.mean(dim=(2, 3), keepdim=True))).view(b)
        return conf, self.soft_argmax(logits), p


def count_macs(net, size=144):
    """粗算 MACs(乘加数), 用于预算核对。"""
    total = 0
    x = torch.zeros(1, 3, size, size)
    hooks = []

    def hook(module, inp, out):
        nonlocal total
        if isinstance(module, nn.Conv2d):
            out_c, in_c_per_g, kh, kw = module.weight.shape
            n, h, w = inp[0].shape[0], out[0].shape[-2], out[0].shape[-1]
            total += int(n * h * w * kh * kw * in_c_per_g * out_c)

    for m in net.modules():
        hooks.append(m.register_forward_hook(hook))
    net.eval()
    with torch.no_grad():
        net(x)
    for h in hooks:
        h.remove()
    return total / 1e6  # 百万 MACs


if __name__ == '__main__':
    for kind in ('softargmax', 'gap'):
        net = DetectNet(kind)
        print(f'DetectNet[{kind}] MACs: {count_macs(net):.2f}M @144x144  '
              f'参数 {sum(p.numel() for p in net.parameters())/1e3:.1f}K')
