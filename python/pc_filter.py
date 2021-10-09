#!/usr/bin/env python3

""" Downsample point cloud and filter

    Input parameters can be specified in a JSON file
"""

import sys
import json
import open3d as o3d

if len(sys.argv) < 4:
    print("Usage: {} input_point_cloud output_point_cloud param.json".format(sys.argv[0]))
    sys.exit()
IN_PC = sys.argv[1]
OUT_PC = sys.argv[2]
JNAME = sys.argv[3]
VOXEL_DOWNSAMPLE = 0
STATISTICAL_OUTLIERS = 0
RADIUS_OUTLIERS = 0
VOXEL_SIZE = 0.1
NB_NEIGHBORS = 20
STD_RATIO = 2.0
NB_POINTS = 16
RADIUS = 0.4
with open(JNAME) as jfile:
    JDATA = json.load(jfile)
    if "voxel_downsample" in JDATA:
        VOXEL_DOWNSAMPLE = JDATA["voxel_downsample"]
        if VOXEL_DOWNSAMPLE > 0:
            if "voxel_size" in JDATA:
                VOXEL_SIZE = JDATA["voxel_size"]
    if "statistical_outliers" in JDATA:
        STATISTICAL_OUTLIERS = JDATA["statistical_outliers"]
        if STATISTICAL_OUTLIERS > 0:
            if "nb_neighbors" in JDATA:
                NB_NEIGHBORS = JDATA["nb_neighbors"]
            if "std_ratio" in JDATA:
                STD_RATIO = JDATA["std_ratio"]
    if "radius_outliers" in JDATA:
        RADIUS_OUTLIERS = JDATA["radius_outliers"]
        if RADIUS_OUTLIERS > 0:
            if "nb_points" in JDATA:
                NB_POINTS = JDATA["nb_points"]
            if "radius" in JDATA:
                RADIUS = JDATA["radius"]

# read point cloud
pc = o3d.io.read_point_cloud(IN_PC)
for i in range(1, max(VOXEL_DOWNSAMPLE, STATISTICAL_OUTLIERS, RADIUS_OUTLIERS) + 1):
    if STATISTICAL_OUTLIERS == i:
        pc, _ = pc.remove_statistical_outlier(nb_neighbors=NB_NEIGHBORS,
                                          std_ratio=STD_RATIO)
    if RADIUS_OUTLIERS == i:
        pc, _ = pc.remove_radius_outlier(nb_points=NB_POINTS,
                                     radius=RADIUS)
    if VOXEL_DOWNSAMPLE == i:
        pc = pc.voxel_down_sample(voxel_size=VOXEL_SIZE)

o3d.io.write_point_cloud(OUT_PC, pc)
o3d.visualization.draw_geometries([pc])
