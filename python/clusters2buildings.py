'''
Using the dbscan_clustering.py result the folder, that contains the clusters to segment wall points.
Using the roof clusters a convex bounding polygon created and scaled.
With the scaled polygon the wall points are cropped and segmented by buildings.

usage: clusters2building.py [-h] [-c CLUSTER_PATH] [-w WALLS_PATH] [-b BUILDINGS_PATH] [-a AREA] [-s SCALE] [-d DEBUG] pc_file_name

positional arguments:
  pc_file_name          point cloud of the wall points (.PLY)

optional arguments:
  -h, --help            show this help message and exit
  -c CLUSTER_PATH, --cluster_path CLUSTER_PATH
                        folder that contains the DBSCAN roof clusters
  -w WALLS_PATH, --walls_path WALLS_PATH
                        folder to save walls
  -b BUILDINGS_PATH, --buildings_path BUILDINGS_PATH
                        folder to save buildings
  -a AREA, --area AREA  minimum base area to from a building segment
  -s SCALE, --scale SCALE
                        scaling the roofs area to crop wall points
  -d DEBUG, --debug DEBUG
                        to switch debug mode (displaying the results) use: 1
'''

import os
import random
import glob
import shutil
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull #, convex_hull_plot_2da not used

# Default variables
#wall_points = 'barnag_ndsm_walls.ply'
def_cluster_path = 'clusters'
def_wall_path = 'walls'
def_buildings_path = 'buildings'
def_area = 10
def_scale = 1.3
def_debug = 0

# Command line parameters
parser = argparse.ArgumentParser()
parser.add_argument('wall_points', metavar='pc_file_name', type=str, nargs=1,
                    help='point cloud of the wall points (.PLY)')
parser.add_argument('-c', '--cluster_path', type=str, default=def_cluster_path,
                    help='folder that contains the DBSCAN roof clusters')
parser.add_argument('-w', '--walls_path', type=str,
                    default=def_wall_path,
                    help='folder to save walls')
parser.add_argument('-b', '--buildings_path', type=str,
                    default=def_buildings_path,
                    help='folder to save buildings')
parser.add_argument('-a', '--area', type=float, default=def_area,
                    help=' minimum base area to from a building segment')
parser.add_argument('-s', '--scale', type=float, default=def_scale,
                    help='scaling the roofs area to crop wall points')
parser.add_argument('-d', '--debug', type=int, default=def_debug,
                    help='to switch debug mode (displaying the results) use: 1')
args = parser.parse_args()

# Parameters into variables
#pc_filename = args.wall_points[0]

# Checking the existance of the folders
if os.path.isdir(args.buildings_path):
    shutil.rmtree(args.buildings_path, ignore_errors=True)
os.mkdir(args.buildings_path)

if os.path.isdir(args.walls_path):
    shutil.rmtree(args.walls_path, ignore_errors=True)
os.mkdir(args.walls_path)

# Path of the clusters
clusters = glob.glob(os.path.join(args.cluster_path, 'cluster_*.ply'))

# Import .PLY format wall points
pcd_walls = o3d.io.read_point_cloud(args.wall_points[0])
pcd_walls_points = np.asarray(pcd_walls.points)

# The main loop
i = 0

for cluster in clusters:
    # Import a roof cluster
    pcd_cluster = o3d.io.read_point_cloud(cluster)
    # Coordinates of the point clouds
    pcd_cluster_xyz = np.asarray(pcd_cluster.points)

    # Convex hull
    pcd_cluster_convhull = ConvexHull(pcd_cluster_xyz[:, :2])

    # collect vertices by index
    pcd_cluster_convhull_xyz = pcd_cluster_xyz[pcd_cluster_convhull.vertices]
    # print(pcd_cluster_convhull_xyz[1,:])

    if pcd_cluster_convhull.area >= args.area:
        
        # Print base area of the roofs
        if args.debug == 1:
            print(f'Roof cluster {i} area: {pcd_cluster_convhull.area:.2f} m^2')
 #           print('cluster_'+str(i)+' area: '+str(pcd_cluster_convhull.area)) 
        
        # Scale it with Open3D                                                  TODO: scaling is not perfect along the building sides -> other solutions?
        pcd_cluster_convhull = o3d.geometry.PointCloud()
        pcd_cluster_convhull.points = o3d.utility.Vector3dVector(pcd_cluster_convhull_xyz)
        pcd_cluster_convhull_sc = pcd_cluster_convhull.scale(args.scale,
                center=pcd_cluster_convhull.get_center())

        #print(np.array(pcd_cluster_convhull_sc.points))
        pcd_cluster_convhull_sc_xyz = np.array(pcd_cluster_convhull_sc.points)
        
        if args.debug == 1:
            plt.figure()                                        
            plt.scatter(pcd_cluster_convhull_xyz[:, 0],             # Show the original bounding points 
                        pcd_cluster_convhull_xyz[:, 1])          
            plt.scatter(pcd_cluster_convhull_sc_xyz[:, 0],
                        pcd_cluster_convhull_sc_xyz[:, 1], color='red') # Show the scaled bounding points
            plt.title('Point cloud of the cluster_'+str(i)+' and wall points')  
            plt.xlabel('y_EOV [m]')
            plt.ylabel('x_EOV [m]')
            plt.axis('equal')
            plt.show()

        # Create a SelectionPolygonVolume
        cluster_volume = o3d.visualization.SelectionPolygonVolume()

        # Specify what axis to orient the polygon and min/max of the the polygon vertices
        cluster_volume.orthogonal_axis = "Z"
        cluster_volume.axis_max = 1000
        cluster_volume.axis_min = -1000

        # Convert the np.array to a Vector3dVector
        cluster_volume.bounding_polygon = o3d.utility.Vector3dVector(pcd_cluster_convhull_xyz)

        # Crop the point cloud using the Vector3dVector
        pcd_cropped = cluster_volume.crop_point_cloud(pcd_walls)

        # Append with the roof points
        pcd_cropped_walls_xyz = np.array(pcd_cropped.points)
        pcd_cluster_stack = np.concatenate((pcd_cropped_walls_xyz, pcd_cluster_xyz), axis=0)

        # Create a point cloud with a random color
        pcd_cropped_buildings = o3d.geometry.PointCloud()
        pcd_cropped_buildings.points = o3d.utility.Vector3dVector(pcd_cluster_stack)

        pcd_cropped_walls = o3d.geometry.PointCloud()
        pcd_cropped_walls.points = o3d.utility.Vector3dVector(pcd_cropped_walls_xyz)

        # Add a color to the point cloud
        col_r = np.full((np.shape(pcd_cluster_stack)[0], 1), random.uniform(0, 1))      # Create random colors for point clouds
        col_g = np.full((np.shape(pcd_cluster_stack)[0], 1), random.uniform(0, 1))
        col_b = np.full((np.shape(pcd_cluster_stack)[0], 1), random.uniform(0, 1))
        col = np.concatenate((col_r, col_g, col_b), axis=1)                             # Merge RGB values
        pcd_cropped_buildings.colors = o3d.utility.Vector3dVector(col)                  # Colorize building point clouds
        pcd_cropped_walls.colors = o3d.utility.Vector3dVector(col[:np.shape(pcd_cropped_walls_xyz)[0],:]) # Colorize wall points

        # Visualize the results of the building points
        if args.debug == 1:
            o3d.visualization.draw_geometries([pcd_cropped_buildings])

        # Export point clouds into a folder                                                                   
        o3d.io.write_point_cloud(os.path.join(args.walls_path, f'walls_{i}.ply'), pcd_cropped_walls)                # TODO: add the number from the used cluster filename! 
        o3d.io.write_point_cloud(os.path.join(args.buildings_path, f'building_{i}.ply'), pcd_cropped_buildings)     # Save separeted buildings

        '''
        if args.debug == 1:

            # For testing
            plt.figure()
            plt.scatter(pcd_cluster_xyz[:, 0],pcd_cluster_xyz[:, 1])
            plt.scatter(pcd_walls_points[:, 0],pcd_walls_points[:, 1], color='red')
            plt.title('Point cloud cluster and wall points')
            plt.xlabel('y_EOV [m]')
            plt.ylabel('x_EOV [m]')
            plt.axis('equal')
            plt.show()
        '''
        i += 1
