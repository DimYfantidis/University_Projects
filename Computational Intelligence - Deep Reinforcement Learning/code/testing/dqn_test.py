import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as opt
import numpy as np
import math

from torch.nn import Conv2d
from torch.nn import Linear
from torch.nn import MaxPool2d
from torch.nn import LazyLinear


INPUT_DIMS = (144, 256, 3)
n_actions = 5

w, h, c = INPUT_DIMS
x = T.empty(1, w, h, c)
x = x.permute(0, 3, 1, 2)
y = T.tensor(x)
print(x.shape)
# build the cnn
conv1 = Conv2d(in_channels=3, out_channels=30, kernel_size=(3, 3), stride=3)
x = conv1(x)
print(x.shape)
maxPool1 = MaxPool2d(kernel_size=(2, 2), stride=1)
x = maxPool1(x)
print(x.shape)
conv2 = Conv2d(in_channels=30, out_channels=60, kernel_size=(3, 3), stride=3)
x = conv2(x)
print(x.shape)
maxPool2 = MaxPool2d(kernel_size=(2, 2))
x = maxPool2(x)
print(x.shape)
conv3 = Conv2d(in_channels=60, out_channels=90, kernel_size=(3, 3), stride=3)
x = conv3(x)
print(x.shape)
maxPool3 = MaxPool2d(kernel_size=(2, 2))
x = maxPool3(x)
print(x.shape)

flat = nn.Flatten()
x = flat(x)
print(x.shape)

fc1 = Linear(x.shape[1], 128)
x = fc1(x)
print(x.shape)
fc2 = Linear(128, 64)
x = fc2(x)
print(x.shape)
fc3 = Linear(64, n_actions)
x = fc3(x)
print(x.shape)
print("\033[31m{}\033[0m".format("WARNING: DQN input passed as `PIL.Image.Image` - Consider inferring data as `torch.Tensor`"))
