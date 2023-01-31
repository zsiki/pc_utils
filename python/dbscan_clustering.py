#!/usr/bin/env python3

'''
Using the DBSCAN (machine learning based) clustering to separete group of points from each other.

Sources:
http://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html?highlight=dbscan
https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html

usage: dbscan_clsutering.py file_name -u 1 -e 0.15 -m 50 -f folder

positional arguments:
  pc_file_name          point cloud of the segmented points (.PLY)

optional arguments:
  -h, --help            show this help message and exit
  -u MODUL, --modul MODUL
                        to use Open3D DBSCAN: 0, to use scikit-learn DBSCAN: 1, to use HDBSCAN: 2, default is 0
  -e EPS, --eps EPS     maximum distance between two samples for one to be considered as in the neighborhood of the other
  -m MIN_POINTS, --min_points MIN_POINTS
                        at Open3D method (0): number of samples (or total weight) in a neighborhood for a point to be considered as a core      
                        point, at scikit-learn method (1): minimum number of points required to form a cluster
  -f FOLDER, --folder FOLDER
                        output folder
  -d DEBUG, --debug DEBUG
                        to switch debug mode (displaying the results) use: 1
'''

import sys
import os
import shutil
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn.datasets import make_classification
from sklearn.cluster import DBSCAN
import hdbscan
from joblib import Memory

# Command windows parameters
parser = argparse.ArgumentParser()
parser.add_argument('name', metavar='pc_file_name', type=str, nargs=1,
                    help='point cloud of the segmented points (.PLY)')
parser.add_argument('-u', '--modul', type=int, default=0,
                    help='to use Open3D DBSCAN: 0, to use scikit-learn DBSCAN: 1, default is 0')                    
parser.add_argument('-e', '--eps', type=float, default=0.4,  
                    help='maximum distance between two samples for one to be considered as in the neighborhood of the other')
parser.add_argument('-m', '--min_points', type=int, default=100,
                    help='at Open3D method (0): number of samples (or total weight) in a neighborhood for a point to be considered as a core point,'\
                          ' at scikit-learn method (1): minimum number of points required to form a cluster,'\
                            'at HDBSCAN (2): ...')                                                                             #TODO: new clustering
parser.add_argument('-f', '--folder', type=str, default='clusters',
                    help='output folder')
parser.add_argument('-d', '--debug', action='store_true',
                    help='to switch debug mode (displaying the results)')
args = parser.parse_args()

# Parameters into variables
pc_filename = args.name[0]
modul = args.modul
new_folder = args.folder
eps = args.eps
min_points = args.min_points                        #TODO: check the parameters --> too much clusters, but the result is promising
debug = args.debug

'''
# Parameters
pc_filename = 'barnag_ndsm_roofs_clusters.ply'
new_folder = 'clusters'
eps = 0.15
min_points = 50                                     #TODO: check the parameters --> too much clusters, but the result is promising
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
col = np.asarray(pcd.colors)

# Choose modul
if modul == 0:
    # Labelling with the Open3D DBSCAN
    labels = np.array(pcd.cluster_dbscan(eps, min_points, print_progress = True))
    # Add clusters for loop
    clusters = np.unique(labels)
elif modul == 1:
    # Labelling with scikit-learning DBSCAN
    model = DBSCAN(eps=eps, min_samples=min_points, algorithm='auto', n_jobs=-1) # algorithm: defines the method of find neareast neighbors,
                                                              #kd_tree           # n_jobs: number of paralell job, -1 means all processors
    # Fit model and predict clusters
    labels = model.fit_predict(xyz[:,:])                                                                   #TODO: xyz + colors?
    
    # Retrieve unique clusters
    clusters = np.unique(labels)

elif modul == 2:
    # Create cache folder
    if os.path.isdir('./cache'):
        shutil.rmtree('./cache', ignore_errors=True)
        os.mkdir('./cache')
    else:
        os.mkdir('./cache')

    # Define model parameters                                                                               #TODO: testing
    model = hdbscan.HDBSCAN(algorithm='best', alpha=1.0, approx_min_span_tree=True,
    gen_min_span_tree=False, leaf_size=40, memory = Memory('./cache'), p=None, min_samples=None,            #TODO: min_samples = 1?
    metric='euclidean', min_cluster_size=min_points, cluster_selection_epsilon = eps)

    # For help: https://hdbscan.readthedocs.io/en/latest/parameter_selection.html
    
    # Fit model and predict clusters
    hdbscan_result = model.fit(xyz[:,:])                                                                    #TODO: xyz + colors?
    labels = hdbscan_result.labels_

    # Retrieve unique clusters
    clusters = np.unique(labels)
else:
    print('Please add values of 0 (Opend3D DBSCAN) or 1 (scikit-learn DBSCAN) or 2 (HDBSCAN)!')

# If there is no any cluster:
if len(clusters) == 0:
    print('Roof clusters have not been found!')
    sys.exit(1)

# Save clusters into point clouds TODO: condition on the min. number of points
for cluster in clusters:

    # Get row indexes for samples with this cluster
    row_ix = np.where(labels == cluster)

    # Export the clusters as a point cloud
    xyz_cluster = xyz[row_ix]
    col_cluster = col[row_ix]
    pc_cluster = o3d.geometry.PointCloud()
    pc_cluster.points = o3d.utility.Vector3dVector(xyz_cluster)
    pc_cluster.colors = o3d.utility.Vector3dVector(col_cluster)
    if cluster >= 0:
        if not o3d.io.write_point_cloud(os.path.join(new_folder, f'cluster_{str(cluster)}.ply'), pc_cluster):  # export .ply format
            print('Failed to save cluster {cluster}')
    else:
        if not o3d.io.write_point_cloud(os.path.join(new_folder, 'noise.ply'), pc_cluster): # export noise
            print('Failed to save cluster {cluster}')

    # Create scatter plot from these samples
    if debug:
        plt.scatter(xyz_cluster[0], xyz_cluster[1], label=str(cluster)+' cluster')

# Debug switch to display the results in a 3D viewer
if debug:
    # Show plot
    plt.title('Point cloud clusters')
    plt.xlabel('y [m]')
    plt.ylabel('x [m]')
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
