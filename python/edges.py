#!/usr/bin/env python3
"""
    find edges in wall segmented point cloud
"""

from math import sqrt
import glob
import argparse
import numpy as np
import open3d as o3d

def line2d(pp):
    """ find the normalized equation of 2d line through two points

        :param pp: numpy array of points
        :returns: numpy array of a,b,c parameters of line equation ax+by+c=0
    """
    x = pp[:, 0]    # x coordinates
    y = pp[:, 1]    # y coordinates
    l = np.array([y[0] - y[1], x[1] - x[0],
                  x[0] * y[1] - x[1] * y[0]])
    l = l / sqrt(l[0]**2 + l[1]**2)     # normalize
    return l

def intersection(line1, line2):
    """ calculate intersection point of two lines

        :param line1: parameters of first line
        :param line2: parameters of second line
        :returns: x,y or None if error
    """
    ph = [line1[1] * line2[2] - line2[1] * line1[2],
          line2[0] * line1[2] - line1[0] * line2[2],
          line1[0] * line2[1] - line2[0] * line1[1]]
    try:
        x, y = ph[0] / ph[2], ph[1] / ph[2]
    except:
        return None
    return x, y

def ransac_line(points, tolerance=0.1, rep=0):
    """ find RANSAC line/edge in a point cloud

        :param points: numpy array of points (xyz)
        :param rep: repetition of random sample
        :param tolerance: tolerance for RANSAC
        :returns: TODO
    """
    if rep < 1:
        rep = points.shape[0] // 2
    n = points.shape[0]
    points[:, 2] = 1            # homogenouos 2D coordinates
    best_n = 0
    for _ in range(rep):
        # select two random points
        p = []  # list of random indices for points
        while len(p) != 2:
            p = list(set(np.random.randint(n, size=2))) # remove repeated random integers
        p1 = points[p]  # randomly selected points
        # line equation from the two points using homogenouos coordinates
        l1 = line2d(p1)
        # select close points
        inl = np.abs(np.dot(points, l1)) < tolerance
        inliers = points[inl]
        if inliers.shape[0] > best_n:
            # better solution found
            best_n = inliers.shape[0]
            best_inl = inl.copy()
            best_line = l1.copy()
    return best_inl, best_line, best_n

def get_minmax(pc, ext=10.0):
    """ get exteded minimax of a point cloud

        :param pc: point cloud
        :param ext: extension of boundary
        :returns: xmin, ymin, zmin, xmax, ymax, zmax
    """
    points = np.asarray(pc.points)
    return np.r_[points.min(axis=0) - ext, points.max(axis=0) + ext]

def get_lines(pc, tol=0.1, lim=30, rep=0):
    """
        Get line equations of a building outline

        :param pc: point cloud
        :param tol: tolerance for RANSAC
        :param lim: minimal number of point on a plane
    """
    lines = []
    xyz = np.asarray(pc.points)
    while True:
        inliers, line, n = ransac_line(xyz, tolerance=tol, rep=rep)
        if n < limit:
            break
        lines.append(line)
        xyz = xyz[np.logical_not(inliers)]
    return lines

def get_intersecs(line_eq, mima):
    """ calculate intersection points of lines

        :param line_eq: list of line equations
        :param mima: limits for instersection area
        :returns: intersection points
    """
    corners = []
    for i, l1 in enumerate(line_eq):
        for l2 in line_eq[i+1:]:
            p = intersection(l1, l2)
            if p is not None and mima[0] < p[0] < mima[3] and mima[1] < p[1] < mima[4]:
                corners.append(p)
    return corners

def get_edges(pc, ext=5, threshold=0.1, limit=30):
    """ get 2D edges of building walls

        :param pc: open3d point cloud
        :param ext: extension to min-max to limit intersection points
        :param threshold: tolerance for RANSAC
        :param limit: minimal number of point for RANSAC
        :returns: TODO
    """
    min_max = get_minmax(pc, ext)
    print(f'{min_max[0]:.3f}, {min_max[1]:.3f}, {min_max[3]:.3f}, {min_max[4]:.3f}')
    lines = get_lines(pc, threshold, limit)
    corners = get_intersecs(lines, min_max)
    for corner in corners:
        print(f'{corner[0]:.3f}, {corner[1]:.3f}')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('names', metavar='file_names', type=str, nargs='+',
                        help='point clouds to process')
    parser.add_argument('-t', '--threshold', type=float, default=0.1,
                        help='Threshold distance to RANSAC plane')
    parser.add_argument('-e', '--extend', type=float, default=10.0,
                        help='Extent the size of minimax to limit corners')
    parser.add_argument('-l', '--limit', type=int, default=90,
                        help='number of points limit for RANSAC')
    args = parser.parse_args()
    for name in args.names:
        names1 = glob.glob(name)
        for name1 in names1:
            print(name1)
            pc = o3d.io.read_point_cloud(name1)
            get_edges(pc, args.extend, args.threshold, args.limit)
