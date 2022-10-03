'''
Script to implement delaunay triangulation and
search path.

@author: Mariano del Río
@date: 20220902
'''


from scipy.spatial import Delaunay
from treelib import Tree
import numpy as np

from utils import distance2D, midpoint, get_cos

# Constants
W_DISTANCE = 1
W_ANGLE = 1
MAX_COST1 = 5
MAX_COST2 = 1
MAX_DISTANCE = 10


def contains(p: np.ndarray, points: list):

    value = False
    for c in points:
        if c[0] == p[0] and c[1] == p[1]:
            value = True
            break

    return value


def correct_colors(p1: np.ndarray, p2: np.ndarray, y_cones: list,
                   b_cones: list):

    value = True
    condition1 = contains(p1, y_cones) and contains(p2, y_cones)
    condition2 = contains(p1, b_cones) and contains(p2, b_cones)

    if condition1 or condition2:
        value = False

    return value


def filter_edge(points: np.ndarray, i1: int, i2: int, y_cones: list,
                b_cones: list):
    """
    Filter edges larger than a maximum distance and edges with
    cones of the same color.
    """

    v = False
    if distance2D(points[i1], points[i2]) < MAX_DISTANCE:
        if correct_colors(points[i1], points[i2], y_cones, b_cones):
            v = True
    return v


def delaunay_triangulation(points: np.ndarray, y_cones: list,
                           b_cones: list):

    triangles = Delaunay(points)
    edges = []
    for t in triangles.simplices:
        if filter_edge(points, t[0], t[1], y_cones, b_cones):
            edges.append((points[t[0]], points[t[1]]))

        if filter_edge(points, t[1], t[2], y_cones, b_cones):
            edges.append((points[t[1]], points[t[2]]))

        if filter_edge(points, t[0], t[2], y_cones, b_cones):
            edges.append((points[t[0]], points[t[2]]))

    return edges


def calculate_midpoints(edges: list):
    midpoints = []
    for p1, p2 in edges:
        p = midpoint(p1, p2)
        if not contains(p, midpoints):
            midpoints.append(p)

    return midpoints


def cost_function(o_path: list, new_point: np.ndarray):
    """
    Cost function to build best path. It takes in account distance
    and angles between midpoints.
    """

    len_path = len(o_path)
    path = o_path.copy()
    path.append(new_point)

    cost_dist = 0
    cost_angle = 0

    cost_dist += distance2D(path[0], path[1])

    if len_path > 2:
        for i in range(1, len(path)-1):
            d = distance2D(path[i], path[i+1])
            a = get_cos(path[i-1], path[i], path[i+1])
            if a > 0:
                a = 10000
            if d > 10:
                d = 10000
            cost_dist += d
            cost_angle += a

    else:
        for i in range(1, len(path)-1):
            d = distance2D(path[i], path[i+1])
            cost_dist += d

    return (cost_dist*W_DISTANCE + cost_angle*W_ANGLE)/len(o_path)


def build_path_tree(path: list, points: np.ndarray):
    """
    Recursive function (use of build_tree_children) to build a tree
    being nodes next point of path and choose best path according
    to a defined cost function.
    """

    assert len(path) > 0

    path_tree = Tree()
    counter = 0  # Counter to identify every node
    path_tree.create_node(tag=0, identifier=counter, data=path)

    # Sorted list of pair (point, cost of path with this point)
    c_points = sorted([[cost_function(path.copy(), p), p] for p in points])

    # To take in account that it is possible to get less than
    # two elements in list c_points
    t1, t2 = None, None
    c1, c2 = 0, 0
    if len(c_points) > 1:
        t1, t2 = c_points[:2]
        c2, p2 = t2  # Cost and point associated
        c1, p1 = t1
    elif len(c_points) == 1:
        t1 = c_points[0]
        c1, p1 = t1

    if c1 < MAX_COST1 and t1 is not None:
        # If cost is higher than a max, prune tree
        new_path = path.copy()
        new_path.append(p1)
        new_points = points
        new_points.remove(p1)
        path_tree = build_tree_children(new_path, new_points, counter,
                                        path_tree, c1)

    if c2 < MAX_COST1 and t2 is not None:
        new_path = path.copy()
        new_path.append(p2)
        new_points = points
        new_points.remove(p2)
        path_tree = build_tree_children(new_path, new_points,
                                        counter, path_tree, c2)

    return path_tree


def build_tree_children(path: list, points: np.ndarray, id_parent: int,
                        tree: Tree, cost: float):
    """
    Auxiliar recursive function of build_path_tree.
    """

    counter = id_parent+1
    while tree.contains(counter):
        counter = counter+1  # To identify every node

    tree.create_node(tag=cost, identifier=counter, data=path,
                     parent=id_parent)

    # Sorted list of pair (point, cost of path with this point)
    c_points = sorted([[cost_function(path.copy(), p), p] for p in points])

    # To take in account that it is possible to get less than
    # two elements in list c_points
    t1, t2 = None, None
    c1, c2 = 0, 0
    if len(c_points) > 1:
        t1, t2 = c_points[:2]
        c2, p2 = t2  # Cost and point associated
        c1, p1 = t1
    elif len(c_points) == 1:
        t1 = c_points[0]
        c1, p1 = t1

    if c1 < MAX_COST2 and t1 is not None:
        # If cost is higher than a max, prune tree
        new_path = path.copy()
        new_path.append(p1)
        new_points = points.copy()
        new_points.remove(p1)
        tree = build_tree_children(new_path, new_points, counter, tree, c1)

    if c2 < MAX_COST2 and t2 is not None:
        new_path = path.copy()
        new_path.append(p2)
        new_points = points.copy()
        new_points.remove(p2)
        tree = build_tree_children(new_path, new_points, counter, tree, c2)

    return tree


def find_best_path(tree: Tree):
    """
    Function to choose best path of tree taking in account costs
    of every path represented in every leave.
    Cost is accumulated in attribute tag of node.
    """

    leaves = tree.leaves()
    min_node = leaves[0]

    for node in leaves[1:]:
        if node.tag < min_node.tag:
            min_node = node

    return np.array(min_node.data), min_node.tag
