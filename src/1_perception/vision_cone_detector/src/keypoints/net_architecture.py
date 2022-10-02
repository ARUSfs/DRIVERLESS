# -*- coding: utf-8 -*-

import torch.nn as nn
import torch
import torchvision


# Class for a neural net to detect keypoints
class KeypointNet(nn.Module):
    def __init__(self, image_size, num_kpt, pretrained=True, freeze=False):

        super(KeypointNet, self).__init__()

        # Obtain NN resnet34
        rN = torchvision.models.resnet34(pretrained=pretrained)

        # Remove last layer
        self.resNet = torch.nn.Sequential(*list(rN.children())[:-1])

        # Add output layer
        self.out = torch.nn.Linear(512, num_kpt*2)

        if freeze:
            for param in self.resnet.parameters():
                param.requires_grad = False

        # Init output layer weigths
        nn.init.normal_(self.out.weight, 0, 0.01)
        nn.init.constant_(self.out.bias, 0)

        self.image_size = image_size
        self.num_kpt = num_kpt

    def forward(self, x):
        x = self.resNet(x)

        # Reshape
        x = x.view(x.shape[0], -1)
        x = self.out(x)
        return x
