from delaunay_detector import delaunay_triangulation
from delaunay_detector import build_path_tree
from delaunay_detector import find_best_path
import numpy as np
from utils import spline

def calculate_path():

    path = [[0.0,0.0]]
    y = [[0,2], [2,2],[4,2]]
    b = [[0,-2], [2,-2],[4,-2]]
    points = np.array(y+b)

    midpoints = delaunay_triangulation(points, y,b)
    tree = build_path_tree(path, midpoints)
    path, weight = find_best_path(tree)
    print(path)
    path = spline(path)

    return path

print(calculate_path())
