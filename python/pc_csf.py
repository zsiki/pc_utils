#! /bin/env python3
"""
    Classify input point cloud into ground and non-ground points with Cloth Simulation Filter (CSF).
    The result are saved into .PLY format.

    W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan,
    “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,”
    Remote Sens., vol. 8, no. 6, p. 501, 2016. (http://www.mdpi.com/2072-4292/8/6/501/htm)

"""
import sys
import os
import argparse
import CSF
import numpy as np
import open3d as o3d

parser = argparse.ArgumentParser()
parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                    help='point cloud to process')
parser.add_argument('-r', '--resolution', type=float, default = 1.0,
                    help='resolution for CSF')
parser.add_argument('-o', '--output', type=str, default='output',
                    help='output base name for ground and non-ground points')
parser.add_argument('--rigidness', type=int, default=3,
                    help='rigidness of cloth 1,2,3: mountain with desen vegetation(1) OR complex scenes(2) OR flat terrain with high-rise buildings(3)')
parser.add_argument('--smooth', action='store_true',
                    help='postprocess to smooth')
parser.add_argument('--iterations', type=int, default=1000,
                    help='number of iterations')
parser.add_argument('--classification', type=int, default=0.5,
                    help='classification threshold')
args = parser.parse_args()

# load PC
pc = o3d.io.read_point_cloud(args.name[0])
if not pc.has_points():
    print(f'Point cloud file not found or empty {args.name[0]}')
    sys.exit(1)
pc_xyz = np.asarray(pc.points)

# CSF parameters TODO all paramaters to set from user input
csf = CSF.CSF()
csf.params.bSloopSmooth = args.smooth
csf.params.cloth_resolution = args.resolution
csf.params.rigidness = args.rigidness
csf.params.time_step = 0.65
csf.params.class_threshold = args.classification
csf.params.interations = args.iterations

csf.setPointCloud(pc_xyz)
ground = CSF.VecInt()
non_ground = CSF.VecInt()
csf.do_filtering(ground, non_ground, exportCloth=False)

# write non-ground points into binary
non_ground = pc.select_by_index(non_ground)
o3d.io.write_point_cloud(os.path.join(args.output + '_non_ground.ply'), non_ground, True)

# write ground points into binary
pc_ground = pc.select_by_index(ground)
o3d.io.write_point_cloud(os.path.join(args.output + '_ground.ply'), pc_ground, True)

