#!/usr/bin/env python3

'''
Using the DBSCAN (machine learning based) clustering to separete group of points from each other.

Source:
http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html?highlight=dbscan

usage: dbscan_clsutering.py file_name -e 0.15 -m 50 -f folder

positional arguments:
  pc_file_name          point cloud of the segmented points (.PLY)

optional arguments:
  -h, --help            show this help message and exit
  -e EPS, --eps EPS     distance to neighbors in a cluster
  -m MIN_POINTS, --min_points MIN_POINTS
                        minimum number of points required to form a cluster
  -f FOLDER, --folder FOLDER
                        output folder
  -d DEBUG, --debug DEBUG
                        to switch debug mode (displaying the results) use: 1

'''

import os
import shutil
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# Command windows parameters
parser = argparse.ArgumentParser()
parser.add_argument('name', metavar='pc_file_name', type=str, nargs=1,
                    help='point cloud of the segmented points (.PLY)')
parser.add_argument('-e', '--eps', type=float, default=0.15,
                    help='defines the distance to neighbors in a cluster')
parser.add_argument('-m', '--min_points', type=int, default=50,
                    help=' minimum number of points required to form a cluster')
parser.add_argument('-f', '--folder', type=str, default='clusters',
                    help='output folder')
parser.add_argument('-d', '--debug', type=int, default=0,
                    help='to switch debug mode (displaying the results) use: 1 ')
args = parser.parse_args()

# Parameters into variables
pc_filename = args.name[0]
new_folder = args.folder
eps = args.eps
min_points = args.min_points #TODO: check the parameters --> too much clusters, but the result is promising
debug = args.debug

'''
# Parameters
pc_filename = 'barnag_ndsm_roofs_clusters.ply'
new_folder = 'clusters'
eps = 0.15
min_points = 50  #TODO: check the parameters --> too much clusters, but the result is promising
debug = 1
'''

# Check the existance of the folder and create if it need
if os.path.isdir(new_folder):
    shutil.rmtree(new_folder, ignore_errors=True)
    os.mkdir(new_folder)
else:
    os.mkdir(new_folder)

# Import .PLY format point cloud and create a numpy array
pcd = o3d.io.read_point_cloud(pc_filename)
xyz = np.asarray(pcd.points)

# Labelling with the DBSCAN
labels = np.array(pcd.cluster_dbscan(eps, min_points))

# Add clusters for loop
clusters = np.unique(labels)

# Save clusters into point clouds TODO: condition aon the min. number of points
for cluster in clusters:

    # Get row indexes for samples with this cluster
    row_ix = np.where(labels == cluster)

    # Export the clusters as a point cloud
    xyz_cluster = xyz[row_ix]
    pc_cluster = o3d.geometry.PointCloud()
    pc_cluster.points = o3d.utility.Vector3dVector(xyz_cluster)
    if cluster >= 0:
        if not o3d.io.write_point_cloud(os.path.join(new_folder, f'cluster_{str(cluster)}.ply'), pc_cluster):  # export .ply format
            print('Failed to save cluster {cluster}')
    else:
        if not o3d.io.write_point_cloud(os.path.join(new_folder, 'noise.ply'), pc_cluster): # export noise
            print('Failed to save cluster {cluster}')

    # Create scatter of these samples
    if debug == 1:
        plt.scatter(xyz_cluster[0], xyz_cluster[1], label=str(cluster)+' cluster')

# Debug switch to display the results in a 3D viewer
if debug == 1:

    # Show plot
    plt.title('Point cloud clusters')
    plt.xlabel('y_EOV [m]')
    plt.ylabel('x_EOV [m]')
    plt.axis('equal')
    plt.show()

    # Display point cloud
    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    max_label = labels.max()
    print(f"Point cloud has {max_label + 1} clusters")

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

    # Display point cloud
    o3d.visualization.draw_geometries([pcd])
