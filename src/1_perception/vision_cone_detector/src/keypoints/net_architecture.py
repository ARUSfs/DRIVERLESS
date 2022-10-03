"""
Script to implement net architecture of detector.

@author: Mariano del Río
@date: 20211204
"""

import torch.nn as nn
import torch
import torch.nn.functional

from cross_ratio_loss import CrossRatioLoss


class ResNet(nn.Module):
    def __init__(self, in_channels, out_channels):

        super(ResNet, self).__init__()

        self.conv1 = nn.Conv2d(in_channels=in_channels,
                               out_channels=out_channels, kernel_size=3,
                               stride=1, padding=2, dilation=2)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(in_channels=out_channels,
                               out_channels=out_channels, kernel_size=3,
                               stride=1, padding=1)
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.relu2 = nn.ReLU()

        self.shortcut_conv = nn.Conv2d(in_channels=in_channels,
                                       out_channels=out_channels,
                                       kernel_size=1, stride=1)
        self.shortcut_bn = nn.BatchNorm2d(out_channels) #Capas de atajo

    def forward(self, x):
        c1 = self.conv1(x)
        b1 = self.bn1(c1)
        act1 = self.relu1(b1)
        out = self.relu2(self.shortcut_bn(self.shortcut_conv(x)) + self.bn2(self.conv2(act1)))
        return out


class KeypointNet(nn.Module):
    def __init__(self, image_size=(80, 80), num_kpt=7, init_weight=True):

        super(KeypointNet, self).__init__()
        net_size = 16
        self.conv = nn.Conv2d(in_channels=3, out_channels=net_size,
                              kernel_size=7, stride=1, padding=3)
        self.bn = nn.BatchNorm2d(net_size)
        self.relu = nn.ReLU()
        self.res1 = ResNet(net_size, net_size)
        self.res2 = ResNet(net_size, net_size * 2)
        self.res3 = ResNet(net_size * 2, net_size * 4)
        self.res4 = ResNet(net_size * 4, net_size * 8)
        self.out = nn.Conv2d(in_channels=net_size*8, out_channels=7,
                             kernel_size=1)

        if init_weight:
            self._initialize_weights()

        self.image_size = image_size
        self.num_kpt = num_kpt

    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out',
                                        nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, (nn.BatchNorm2d, nn.GroupNorm)):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.Linear):
                nn.init.normal_(m.weight, 0, 0.01)
                nn.init.constant_(m.bias, 0)

    def flat_softmax(self, inp):
        flat = inp.view(-1, self.image_size[0] * self.image_size[1])
        flat = torch.nn.functional.softmax(flat, 1)
        return flat.view(-1, self.num_kpt, self.image_size[0], self.image_size[1])

    def soft_argmax(self, inp):
        values_y = torch.linspace(0, (self.image_size[0] - 1.) / self.image_size[0],
                                  self.image_size[0],
                                  dtype=inp.dtype,
                                  device=inp.device)
        values_x = torch.linspace(0, (self.image_size[1] - 1.) / self.image_size[1],
                                  self.image_size[1],
                                  dtype=inp.dtype,
                                  device=inp.device)

        exp_y = (inp.sum(3) * values_y).sum(-1)
        exp_x = (inp.sum(2) * values_x).sum(-1)
        return torch.stack([exp_x, exp_y], -1)

    def forward(self, x):
        act1 = self.relu(self.bn(self.conv(x)))
        act2 = self.res1(act1)
        act3 = self.res2(act2)
        act4 = self.res3(act3)
        act5 = self.res4(act4)
        hm = self.out(act5)
        hm = self.flat_softmax(self.out(act5))
        out = self.soft_argmax(hm)
        return hm, out.view(-1, self.num_kpt, 2)
