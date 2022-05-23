#!/usr/bin/env python3
"""
    find edges in wall segmented point cloud
"""

from math import sqrt, cos, sin, atan2, hypot
import glob
import os.path
import argparse
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
import ezdxf

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
        if n < lim:
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

def get_edges(pc, ext=5, threshold=0.1, limit=30, edge_limit=90, d1=0.2, d2=0.2, debug=False):
    """ get 2D edges of building walls

        :param pc: open3d point cloud
        :param ext: extension to min-max to limit intersection points
        :param threshold: tolerance for RANSAC
        :param limit: minimal number of point for RANSAC
        :param edge_limit: minimalnumber of points in 1 m^2
        :param d1: offset from the corner to find points on edge
        :param d2: offset from the line to find points on edge
        :returns: edges as pair of points
    """
    min_max = get_minmax(pc, ext)
    if debug:
        print(f'{min_max[0]:.3f}, {min_max[1]:.3f}, {min_max[3]:.3f}, {min_max[4]:.3f}')
    lines = get_lines(pc, threshold, limit)
    corners = get_intersecs(lines, min_max)
    edges = []
    if len(corners) == 0:
        return corners, edges    # no corners found
    # create point cloud of possible corners
    corner_pc = o3d.geometry.PointCloud()
    corner_pc.points = o3d.utility.Vector3dVector(np.array(np.c_[corners, np.full(len(corners), 0.0)]))
    if debug:
        for corner in corners:
            print(f'{corner[0]:.3f}, {corner[1]:.3f}')
    # try to create edge for any pair of corners
    for i, corner1 in enumerate(corners):
        for j, corner2 in enumerate(corners[i+1:]):
            # bearing and distance from corner1 to corner2
            bearing = atan2(corner2[1] - corner1[1], corner2[0] - corner1[0])
            dist = hypot(corner2[0] - corner1[0], corner2[1] - corner1[1])
            area = abs((dist-(2*d1))*d2*2)
            # rectangular buffer for internal part of edge
            poly = np.array(
                [[corner1[0] + d1 * cos(bearing) - d2 * sin(bearing),
                  corner1[1] + d1 * sin(bearing) + d2 * cos(bearing), 0.0],
                 [corner1[0] + (dist - d1) * cos(bearing) - d2 * sin(bearing),
                  corner1[1] + (dist - d1) * sin(bearing) + d2 * cos(bearing), 0.0],
                 [corner1[0] + (dist - d1) * cos(bearing) + d2 * sin(bearing),
                  corner1[1] + (dist - d1) * sin(bearing) - d2 * cos(bearing), 0.0],
                 [corner1[0] + d1 * cos(bearing) + d2 * sin(bearing),
                  corner1[1] + d1 * sin(bearing) - d2 * cos(bearing), 0.0]])
            # set cutting polygon for open3d
            vol = o3d.visualization.SelectionPolygonVolume()
            vol.orthogonal_axis = 'Z'
            vol.axis_max = 50
            vol.axis_min = -1
            vol.bounding_polygon = o3d.utility.Vector3dVector(poly)
            # cut point cloud of walls
            edge_pc = vol.crop_point_cloud(pc)
            # cut point cloud of corners
            c_pc = vol.crop_point_cloud(corner_pc)
            if debug:
                plt.plot([corner1[0], corner2[0]], [corner1[1], corner2[1]])
                plt.plot(poly[:, 0], poly[:, 1])
                plt.scatter(np.asarray(pc.points)[:, 0], np.asarray(pc.points)[:, 1])
                plt.axis('equal')
                plt.show()
            n = np.asarray(edge_pc.points).shape[0]     # number of points from walls in poly
            m = np.asarray(c_pc.points).shape[0]        # number of corner points in poly
            #print(n/area)
#            if n > edge_limit and m == 0:             # number of points in the edge
            if n/area > edge_limit and m == 0:         # number or points in 1 m^2
                edges.append([corner1, corner2])
    return corners, edges

class to_dxf(object):
    """ create dxf output from building corners and edges

        :param fn_out: DXF output filename
    """
    def __init__(self, fn_out):
        """ create dxff file and layers """
        self.dxf = ezdxf.new(dxfversion='R2018')   # create a new empty dxf
        self.dxf.layers.add('POINTS', color=2)     # create new layer for points
        self.dxf.layers.add('LINES', color=3)      # create new layer for lines
        self.fn_out = fn_out

    def add(self, points, lines):
        ''' add corners and edged to dxf

            :param points: corners of walls
            :param lines: edges as pair of points
        '''
        msp = self.dxf.modelspace()
        for point in points:
            msp.add_point(point, dxfattribs={'layer': 'POINTS'})
        for line in lines:
            msp.add_line(line[0], line[1], dxfattribs={'layer': 'LINES'})

    def save(self):
        self.dxf.saveas(self.fn_out)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('names', metavar='file_names', type=str, nargs='+',
                        help='point clouds to process')
    parser.add_argument('-t', '--threshold', type=float, default=0.1,
                        help='Threshold distance to RANSAC plane')
    parser.add_argument('-e', '--extend', type=float, default=10.0,
                        help='Extent the size of minimax to limit corners')
    parser.add_argument('-l', '--limit', type=int, default=90,
                        help='minimal number of points limit for edges')
    parser.add_argument('-i', '--edgelimit', type=int, default=90,
                        help='minimal number of points limit for RANSAC line')
    parser.add_argument('-c', '--cornerdist', type=float, default=0.2,
                        help='Threshold distance from corners in edge detection')
    parser.add_argument('-g', '--edgedist', type=float, default=0.2,
                        help='Threshold distance from edges in edge detection')
    parser.add_argument('-d', '--debug', action="store_true",
                        help='generate debug output')

    args = parser.parse_args()
    dxf_name = os.path.splitext(args.names[0])[0] + '.dxf'
    doc = to_dxf(dxf_name)
    for name in args.names:
        names1 = glob.glob(name)
        for name1 in names1:
            if args.debug:
                print(name1)
            pc = o3d.io.read_point_cloud(name1)
            cor, edg = get_edges(pc, args.extend, args.threshold, args.limit,
                                 args.edgelimit, args.cornerdist, args.edgedist,
                                 args.debug)
            doc.add(cor, edg)
            if args.debug:
                for e in edg:
                    print(f'{e[0][0]:.3f} {e[0][1]:.3f}  {e[1][0]:.3f} {e[1][1]:.3f}')
    doc.save()
