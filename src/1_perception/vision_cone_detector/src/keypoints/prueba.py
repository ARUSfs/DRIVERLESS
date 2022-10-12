
from utils import calculate_distance,euclidean_distance_loss
from utils import scale_labels,unscale_labels
import numpy as np


p1 = [[1,2],[3,4],[5,6],[9,9]]

s = scale_labels(p1,0.5,0.5)
u = unscale_labels(s,0.5,0.5)
print(s)
print(u)

ss = np.array(p1)* np.array([0.5,0.5])
ss = np.array(ss,dtype=int)
uu = np.array(ss)/ np.array([0.5,0.5])
uu = np.array(uu,dtype=int)
print(ss)
print(uu)

