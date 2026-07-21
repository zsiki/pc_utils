#!/usr/bin/env python3
"""
    find edges in wall segmented point cloud
"""

from math import sqrt, cos, sin, atan2, hypot
import glob
import re
import os.path
import argparse
import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
import alphashape
import shapely
import ezdxf

class to_dxf():
    """ create dxf output from building corners and edges

        :param fn_out: DXF output filename
    """
    def __init__(self, fn_out):
        """ create dxff file and layers """
        self.dxf = ezdxf.new(dxfversion='R2018')   # create a new empty dxf
        self.dxf.layers.add('POINTS', color=2)     # create new layer for points
        self.dxf.layers.add('LINES', color=3)      # create new layer for lines
        self.fn_out = fn_out

    def add(self, points):
        ''' add corners and edged to dxf

            :param points: corners of polygon
        '''
        msp = self.dxf.modelspace()
        msp.add_lwpolyline(points)

    def save(self):
        """ save edges and corners to dxf file """
        self.dxf.saveas(self.fn_out)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('names', metavar='file_names', type=str, nargs='+',
                        help='point clouds to process')
    parser.add_argument('-a', '--alpha', type=float, default=0,
                        help='alpha parameter, default: 0=convex hul')
    parser.add_argument('-t', '--tolerance', type=float, default=0.2,
                        help='tolerance for Douglas-Pecker, default: 0.2')
    parser.add_argument('-d', '--debug', action="store_true",
                        help='generate debug output')

    args = parser.parse_args()
    dxf_name = re.sub('_\d+$', '', os.path.splitext(args.names[0])[0], flags=re.ASCII) + '.dxf'
    doc = to_dxf(dxf_name)
    for name in args.names:
        names1 = glob.glob(name)
        for name1 in names1:
            if args.debug:
                print(name1)
            pc = o3d.io.read_point_cloud(name1)
            if np.array(pc.points).shape[0] < 10:
                continue    # skip nearly empty clouds
            xy = np.array(pc.points)[:,0:2]
            # get alphashape boundary
            shape = alphashape.alphashape(xy, args.alpha)
            # simplify border
            shape_simpl = shapely.simplify(shape, args.tolerance,
                                           preserve_topology=True)
            # add to dxf
            if isinstance(shape_simpl, shapely.MultiPolygon):
                for p in shape_simpl.geoms:
                    doc.add(list(p.exterior.coords))
            else:
                doc.add(list(shape_simpl.exterior.coords))
            if args.debug:
                print(f"poly: {type(shape)}")
                print(type(list(shape.exterior.coords)))
                for p in shape_simpl.exterior.coords:
                    print(p, type(p))
    doc.save()
