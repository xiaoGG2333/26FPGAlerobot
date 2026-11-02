"""三分类网络: 输入 80x80x3 -> 3 类(fist 闭合 / palm 张开 / other 其它)。

标签语义见 dataset.py（主人 2026-10-14 定义：张开=五指张开、闭合=握拳、其它=夹爪不动）。

结构约束同 detect(计划书 §4 白名单)。MACs ~4.78M @80x80(三分类头只比二分类多 96 个 MACs)。
通道: stem12 -> 32 -> 64 -> 64,三次下采样至 10x10。

v1->v2: 头 96->2 改为 96->3；n_classes 参数化以便复现 v1 的二分类权重。
"""
import torch
import torch.nn as nn

NUM_CLASSES = 3


def conv_bn_relu(in_c, out_c, k, s=1, groups=1):
    return nn.Sequential(
        nn.Conv2d(in_c, out_c, k, stride=s, padding=k // 2, groups=groups, bias=False),
        nn.BatchNorm2d(out_c),
        nn.ReLU(inplace=True))


class ClassifyNet(nn.Module):
    def __init__(self, n_classes=NUM_CLASSES):
        super().__init__()
        self.n_classes = n_classes
        # 80 -> 40 -> 20 -> 10
        self.stem = conv_bn_relu(3, 12, 3, s=2)
        self.s2 = nn.Sequential(conv_bn_relu(12, 12, 3, groups=12), conv_bn_relu(12, 32, 1))
        self.s3 = nn.Sequential(conv_bn_relu(32, 32, 3, s=2, groups=32), conv_bn_relu(32, 64, 1))
        self.s4 = nn.Sequential(conv_bn_relu(64, 64, 3, groups=64), conv_bn_relu(64, 64, 1))
        self.s5 = nn.Sequential(conv_bn_relu(64, 64, 3, s=2, groups=64), conv_bn_relu(64, 96, 1))
        self.head = nn.Conv2d(96, n_classes, 1)  # logits: fist / palm / other

    def forward(self, x):
        x = self.stem(x)
        x = self.s2(x)
        x = self.s3(x)
        x = self.s4(x)
        x = self.s5(x)
        x = x.mean(dim=(2, 3), keepdim=True)
        return self.head(x).view(x.size(0), self.n_classes)


def count_macs(net, size=80):
    total = 0
    x = torch.zeros(1, 3, size, size)

    def hook(module, inp, out):
        nonlocal total
        if isinstance(module, nn.Conv2d):
            out_c, in_c_per_g, kh, kw = module.weight.shape
            n, h, w = inp[0].shape[0], out[0].shape[-2], out[0].shape[-1]
            total += int(n * h * w * kh * kw * in_c_per_g * out_c)

    for m in net.modules():
        m.register_forward_hook(hook)
    net.eval()
    with torch.no_grad():
        net(x)
    return total / 1e6  # 一次性统计, 不清理 hook


if __name__ == '__main__':
    net = ClassifyNet()
    print(f'ClassifyNet[{net.n_classes}类] MACs: {count_macs(net):.2f}M @80x80')
    print(f'参数: {sum(p.numel() for p in net.parameters())/1e3:.1f}K')
