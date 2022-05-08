#! /bin/env python3
"""
    generate Digital Elevation Model (DEM) from a point cloud using Cloth Simulation Filter (CSF) and GDAL

    W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan,
    “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,”
    Remote Sens., vol. 8, no. 6, p. 501, 2016. (http://www.mdpi.com/2072-4292/8/6/501/htm)

"""
import sys
from os import path, remove
import argparse
import CSF
import gdal
import numpy as np
import open3d as o3d
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                    help='point cloud to process')
parser.add_argument('-r', '--resolution', type=float, default=2.0,
                    help='resolution for dem')
parser.add_argument('-o', '--output', type=str, default='output',
                    help='output base name for DEM file and non-ground points')
parser.add_argument('--rigidness', type=int, default=3,
                    help='rigidness of cloth 1,2,3: mountain with desen vegetation(1) OR complex scenes(2) OR flat terrain with high-rise buildings(3)')
parser.add_argument('--smooth', action='store_true',
                    help='postprocess to smooth')
parser.add_argument('--iterations', type=int, default=500,
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
csf.params.cloth_resolution = args.resolution / 2.0
csf.params.rigidness = args.rigidness
csf.params.time_step = 0.65
csf.params.class_threshold = args.classification
csf.params.interations = args.iterations

csf.setPointCloud(pc_xyz)
ground = CSF.VecInt()
non_ground = CSF.VecInt()
csf.do_filtering(ground, non_ground, exportCloth=False)

# write ground points into ascii with vrt
base_name = path.splitext(args.output)[0]

name = path.basename(base_name)
pc_ground = pc.select_by_index(ground)

xyz = np.asarray(pc_ground.points)
df = pd.DataFrame(xyz, columns=['x', 'y', 'z'])
df.to_csv(base_name + '.csv', index=False, header=True)
vrt = base_name + '.vrt'
with open(vrt, "w") as fvrt:
    fvrt.write(f"""<OGRVRTDataSource>
<OGRVRTLayer name="{name}">
    <SrcDataSource relativeToVRT="1">{name}.csv</SrcDataSource>
    <GeometryType>wkbPoint</GeometryType>
    <GeometryField encoding="PointFromColumns" x="x" y="y" z="z"/>
</OGRVRTLayer>
</OGRVRTDataSource>""")

# NOTE ground and non-ground point coud resolution change to CFS resolution!
#      output removed
# write non-ground points into binary
#non_ground = pc.select_by_index(non_ground)
#o3d.io.write_point_cloud(base_name + '_non_ground.ply', non_ground, True)
# write ground points into binary
#o3d.io.write_point_cloud(base_name + '_ground.ply', pc_ground, True)

# create DEM
gdal.Grid(base_name + '.tif', vrt, algorithm='linear', zfield='z')
# remove temperary files
remove(vrt)
remove(base_name + '.csv')
