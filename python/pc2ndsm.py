#!/bin/env python3
""" convert point cloud elevations to relative heights above DEM """
import sys
import os.path
from osgeo import gdal
import open3d as o3d
import numpy as np

def ndsm(src_ds, pc, ndsm_min):
    """ Create nDSM from DEM and point cloud, points outside the DEM or
        in a NODATA cell are dropped, the height of original point clouds
        points are changed

        :param src_ds: DEM (GDAL)
        :param pc: point cloud
        :param ndsm_min: points below this are dropped
    """
    dem_max_col = src_ds.RasterXSize - 1
    dem_max_row = src_ds.RasterYSize - 1
    gt = src_ds.GetGeoTransform()
    rb = src_ds.GetRasterBand(1)
    no_data = rb.GetNoDataValue()
    dem = rb.ReadAsArray()
    # get point cloud
    pc_xyz = np.asarray(pc.points)
    outlier = []
    for i in range(pc_xyz.shape[0]):
        # Convert from map coordinates to grid indices
        px = min(max(int((pc_xyz[i, 0] - gt[0]) / gt[1]), 0), dem_max_col) #x pixel
        py = min(max(int((pc_xyz[i, 1] - gt[3]) / gt[5]), 0), dem_max_row) #y pixel
        dz = pc_xyz[i, 2] - dem[py, px]   # relative height above DEM
        pc_xyz[i, 2] = dz
        # filter NODATA and low vegetation
        if dem[py, px] == no_data or dz < ndsm_min:
            outlier.append(i)
            continue    # skip nodata
    pc.points = o3d.utility.Vector3dVector(pc_xyz)
    if len(outlier) > 0:
        pc = pc.select_by_index(outlier, invert=True)
    return pc

if __name__ == "__main__":
    # check command line parameters
    if len(sys.argv) < 3:
        print("usage: {} dem_file point_cloud min". format(sys.argv[0]))
        sys.exit(3)
    dem_filename = sys.argv[1]
    pc_filename = sys.argv[2]
    ndsm_min = -9999.           # default min value, no points removed
    if len(sys.argv) > 3:
        ndsm_min = float(sys.argv[3])
    pc_split = os.path.splitext(pc_filename)
    out_filename = pc_split[0] + '_ndsm' + pc_split[1]
    # get DEM
    dem_data = gdal.Open(dem_filename)
    if not dem_data:
        print(f'Cannot open DEM {dem_filename}')
        sys.exit(1)
    # get point cloud
    point_cloud = o3d.io.read_point_cloud(pc_filename)
    if not point_cloud.has_points():
        print(f'Point cloud file not found or empty {pc_filename}')
        sys.exit(2)
    pc = ndsm(dem_data, point_cloud, ndsm_min)
    # write out normalized point cloud (elevations from ground)
    o3d.io.write_point_cloud(out_filename, pc)
