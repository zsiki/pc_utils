#!/usr/bin/env python3

""" Extract wall points from a point cloud by the direction of normals
"""
import os.path
import argparse
import numpy as np
import open3d as o3d

def height_range(pc, max_z):
    """ cut arange of points in z direction

        :param pc: point cloud to cut
        :param min_z: lower limit
        :param max_z: upper limit
        :returns: cutted point cloud
    """
    points_z = np.asarray(pc.points)[:, 2]
    inliers = np.argwhere(points_z < max_z)
    return pc.select_by_index(inliers)

def set_normals(pc, search_type=2,
                knn=20, radius=0.2):
    """ add normals to the point cloud

     :   :params pc: the input point cloud, the normals will be added to it
        :param search_type: KNNSearch/RadiusSearch/HybridSearch 0/1/2
        :param knn: maximum number of neighbours to search
        :param radius: search radius
    """
    if search_type == 2:
        pc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=knn))
    elif search_type == 1:
        pc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamRadius(radius=radius))
    elif search_type == 0:
        pc.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn))
    else:
        pc.estimate_normals()

def wall_points(pc, tolerance=0.35):
    """ select wall points by normals

        :param pc: point cloud with normals
        :param tolerance: angle tolerance for normal in radians, default ~2 degree
        :returns: point cloud with suggested wall points only
    """
    if not pc.has_normals():
        return None
    normals = np.asarray(pc.normals)
    # normal angle to horizontal direction
    angles = np.abs(np.arctan2(normals[:, 2], np.hypot(normals[:, 0], normals[:, 1])))
    inliers = np.argwhere(angles < tolerance)
    return pc.select_by_index(inliers)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                        help='point cloud to process')
    parser.add_argument('-z', '--maxz', type=float, default=2.5,
                        help='Maximal z value for walls')
    parser.add_argument('-n', '--normals', type=int, default=2,
                        help='Method for normals 0/1/2 KNN/Radius/Hybrid')
    parser.add_argument('-k', '--knn', type=int, default=20,
                        help='Number of points for KNN')
    parser.add_argument('-r', '--radius', type=float, default=0.2,
                        help='Radius for normals')
    parser.add_argument('-a', '--angle', type=float, default=0.35,
                        help='Angle tolerance in radians')
    parser.add_argument('-o', '--out_dir', type=str, default=".",
                        help='Path to output directory')
    args = parser.parse_args()
    pc = o3d.io.read_point_cloud(args.name[0])
    # remove high points
    pc = height_range(pc, args.maxz)
    #o3d.visualization.draw_geometries([pc])
    if not pc.has_normals():
        set_normals(pc, -1, knn=args.knn, radius=args.radius)
    wall = wall_points(pc, tolerance=args.angle)
    # remove normals and colors
    wall1 = o3d.geometry.PointCloud()
    wall1.points = o3d.utility.Vector3dVector(wall.points)
    o3d.visualization.draw_geometries([wall1])
    base_name = os.path.splitext(os.path.basename(args.name[0]))[0]
    fname = os.path.join(args.out_dir, base_name + '_wall.ply')
    o3d.io.write_point_cloud(fname, wall1)
