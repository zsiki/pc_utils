#!/usr/bin/env python3

""" Downsample point cloud and filter """

import sys
import json
import open3d as o3d

if len(sys.argv) < 4:
    print("Usage: {} input_point_cloud output_point_cloud param.json".format(sys.argv[0]))
    sys.exit()
IN_PC = sys.argv[1]
OUT_PC = sys.argv[2]
JNAME = sys.argv[3]
with open(JNAME) as jfile:
    JDATA = json.load(jfile)
    VOXEL_DOWNSAMPLE = JDATA["voxel_downsample"].upper()
    VOXEL_SIZE = -1
    if VOXEL_DOWNSAMPLE in ["TRUE", "YES", "T", "Y"]:
        VOXEL_SIZE = JDATA["voxel_size"]

    STATISTICAL_OUTLIERS = JDATA["statistical_outliers"].upper()
    NB_NEIGHBORS = -1
    STD_RATIO = -1
    if STATISTICAL_OUTLIERS in ["TRUE", "YES", "T", "Y"]:
        NB_NEIGHBORS = JDATA["nb_neighbors"]
        STD_RATIO = JDATA["std_ratio"]
    RADIUS_OUTLIERS = JDATA["radius_outliers"].upper()
    NB_POINTS = -1
    RADIUS = -1
    if RADIUS_OUTLIERS in ["TRUE", "YES", "T", "Y"]:
        NB_POINTS = JDATA["nb_points"]
        RADIUS = JDATA["radius"]

# read point cloud
pc = o3d.io.read_point_cloud(IN_PC)
if VOXEL_DOWNSAMPLE in ["TRUE", "YES", "T", "Y"]:
    pc = pc.voxel_down_sample(voxel_size=VOXEL_SIZE)
if STATISTICAL_OUTLIERS in ["TRUE", "YES", "T", "Y"]:
    pc, _ = pc.remove_statistical_outlier(nb_neighbors=NB_NEIGHBORS,
                                          std_ratio=STD_RATIO)
if RADIUS_OUTLIERS in ["TRUE", "YES", "T", "Y"]:
    pc, _ = pc.remove_radius_outlier(nb_points=NB_POINTS,
                                     radius=RADIUS)
o3d.io.write_point_cloud(OUT_PC, pc)
o3d.visualization.draw_geometries([pc])
