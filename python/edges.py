#!/usr/bin/env python3
"""
    find edges in wall segmented point cloud
"""

#from math import sqrt, cos, sin, atan2, hypot
import glob
import re
import os.path
import argparse
import numpy as np
import open3d as o3d
#from matplotlib import pyplot as plt
import ezdxf
from ransac import Ransac
from regression import LinearReg

def get_lines(xyz, tol=0.1, lim=30):
    """
        Get line equations of a building outline

        :param pc: point cloud
        :param tol: tolerance for RANSAC
        :param lim: minimal number of point on a line
    """
    lines = []
    xy = xyz[:, 0:2]
    while True:
        if xy.shape[0] < lim:
            break
        lr = LinearReg(xy)
        r = Ransac(lr)
        xy_fit, _ = r.ransac_filter(tolerance=tol)
        xy_line = lr.get_pnts_by_index(xy_fit)
        if xy_line.shape[0] < lim:
            break
        lr1 = LinearReg(xy_line)
        params = lr1.lkn_reg(limits=True)
        line = [(params[3], params[4]), (params[5], params[6])]
        lines.append(line)
        xy = lr.get_pnts_by_index(np.logical_not(xy_fit))
    return lines

class to_dxf():
    """ create dxf output from building corners and edges

        :param fn_out: DXF output filename
    """
    def __init__(self, fn_out):
        """ create dxff file and layers """
        self.dxf = ezdxf.new(dxfversion='R12')     # create a new empty dxf
        self.dxf.layers.add('POINTS', color=2)     # create new layer for points
        self.dxf.layers.add('LINES', color=3)      # create new layer for lines
        self.fn_out = fn_out

    def add(self, lines):
        ''' add corners and edged to dxf

            :param lines: line segments to draw
        '''
        msp = self.dxf.modelspace()
        for line in lines:
            msp.add_line(line[0], line[1], dxfattribs={'layer': 'LINES'})

    def save(self):
        """ save edges and corners to dxf file """
        self.dxf.saveas(self.fn_out)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('names', metavar='file_names', type=str, nargs='+',
                        help='point clouds to process')
    parser.add_argument('-t', '--threshold', type=float, default=0.1,
                        help='Threshold distance to RANSAC line, default: 0.1')
    parser.add_argument('-e', '--eps', type=float, default=0.4,
                        help='maximum distance between two samples for one to be considered as in the neighborhood of the other,default: 0.4')

    parser.add_argument('-m', '--min_points', type=int, default=100,
                        help='number of points for a cluster, default: 100')
    parser.add_argument('-d', '--debug', action="store_true",
                        help='generate debug output')

    args = parser.parse_args()
    dxf_name = re.sub(r'_\d+$', '', os.path.splitext(args.names[0])[0], flags=re.ASCII) + '.dxf'
    doc = to_dxf(dxf_name)
    for name in args.names:
        names1 = glob.glob(name)
        for name1 in names1:
            if args.debug:
                print(name1)
            pc = o3d.io.read_point_cloud(name1)
            labels = np.array(pc.cluster_dbscan(args.eps, args.min_points, print_progress=False))
            # cluster ids
            clusters = np.unique(labels)
            xyz = np.asarray(pc.points)

            if args.debug:
                print(f"clusters: {clusters}")
            for cluster in clusters:
                if cluster > -1:    # skip noise
                    # Get row indexes for points in this cluster
                    row_ix = np.where(labels == cluster)

                    # find lines in cluster
                    xyz_cluster = xyz[row_ix]
                    lines = get_lines(xyz_cluster, args.threshold, args.min_points)
                    if args.debug:
                        print(f"*** cluster {cluster} {len(lines)} lines ")
                    doc.add(lines)
    doc.save()
