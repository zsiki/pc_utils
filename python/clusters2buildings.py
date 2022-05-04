'''
The script using the dbscan_clustering.py result folder to segment wall points.
The roof clusters are used to create scaled bounding polygons.
With the polygons the wall points are cropped, segmented and saved by buildings.  

Usage:

positional arguments:
  pc_file_name          point cloud of the wall points (.PLY)

optional arguments:
  -h, --help            show this help message and exit
  -f FOLDER, --folder FOLDER
                        folder that contains the DBSCAN roof clusters
  -fr FOLDER_RES, --folder_res FOLDER_RES
                        folder to save buildings
  -a AREA, --area AREA  minimum base area to from a building segment
  -s SCALE, --scale SCALE
                        scaling the roofs area to crop wall points
  -d DEBUG, --debug DEBUG
                        to switch debug mode (displaying the results) use: 1

'''
# Not yet
import os
import shutil
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import random
from scipy.spatial import ConvexHull, convex_hull_plot_2d

# Variables
wall_points = 'barnag_ndsm_walls.ply'
cluster_folder  = 'clusters'
buildings_folder = 'buildings'
area = 10 
scale = 1.3 

# Command windows parameters
parser = argparse.ArgumentParser()
parser.add_argument('wall_points', metavar='pc_file_name', type=str, nargs=1,
                    help='point cloud of the wall points (.PLY)')                    
parser.add_argument('-f', '--folder', type=str, default='clusters',
                    help='folder that contains the DBSCAN roof clusters')
parser.add_argument('-fr', '--folder_res', type=str, default='buildings',
                    help='folder to save buildings')
parser.add_argument('-a', '--area', type=float, default=10,
                    help=' minimum base area to from a building segment')
parser.add_argument('-s', '--scale', type=float, default=1.3,
                    help='scaling the roofs area to crop wall points')
parser.add_argument('-d', '--debug', type=int, default=0,
                    help='to switch debug mode (displaying the results) use: 1 ')
args = parser.parse_args()

# Parameters into variables
pc_filename = args.wall_points[0]
cluster_folder = args.folder
buildings_folder = args.folder_res
area = args.area
scale = args.scale
debug = args.debug

# Check the existance of the folder and create if it need
if os.path.isdir(buildings_folder):
    shutil.rmtree(buildings_folder, ignore_errors=True)
    os.mkdir(buildings_folder)
else:
    os.mkdir(buildings_folder)

# Path of the clusters
clusters = os.listdir('./'+cluster_folder)

# Last is the noise file                    TODO: search it and delete from the list on a direct way ('noise.ply')
clusters = clusters[:-1]

# Import .PLY format wall points
pcd_walls = o3d.io.read_point_cloud(wall_points)
pcd_walls_points = np.asarray(pcd_walls.points)

# The main program
i = 0

for cluster in clusters:
    # Import .PLY format point cloud and create a numpy array
    pcd_cluster = o3d.io.read_point_cloud(cluster_folder+'/'+cluster)
    pcd_cluster_xyz = np.asarray(pcd_cluster.points)

    # Coordinates of the point clouds
    pcd_cluster_points = np.asarray(pcd_cluster.points)

    # Convex hull
    pcd_cluster_convhull = ConvexHull(pcd_cluster_xyz[:,:2])

    # collect vertices by index
    pcd_cluster_convhull_xyz = pcd_cluster_xyz[pcd_cluster_convhull.vertices]
    # print(pcd_cluster_convhull_xyz[1,:])

    # Check base area of the roofs
    if debug == 1:
        print('cluster_'+str(i)+' area: '+str(pcd_cluster_convhull.area))

    if pcd_cluster_convhull.area >= area:
        
        # Scale it with Open3D                                                  TODO: scaling is not perfect along the building sides -> other solutions?
        pcd_cluster_convhull_xyz_sc = o3d.geometry.PointCloud()
        pcd_cluster_convhull_xyz_sc.points = o3d.utility.Vector3dVector(pcd_cluster_convhull_xyz)
        pcd_cluster_convhull_xyz_sc = pcd_cluster_convhull_xyz_sc.scale(scale,center=pcd_cluster_convhull_xyz_sc.get_center())

        #print(np.array(pcd_cluster_convhull_xyz_sc.points))

        pcd_cluster_convhull_xyz_sc = np.array(pcd_cluster_convhull_xyz_sc.points)

        if debug == 1:
            plt.figure()
            plt.scatter(pcd_cluster_convhull_xyz[:, 0],pcd_cluster_convhull_xyz[:, 1])
            plt.scatter(pcd_cluster_convhull_xyz_sc[:, 0],pcd_cluster_convhull_xyz_sc[:, 1], color='red')
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
        pcd_cropped_walls = np.array(pcd_cropped.points)
        pcd_cluster_stack = np.concatenate((pcd_cropped_walls, pcd_cluster_points), axis=0)

        # Create a point cloud with a random color 
        pcd_cropped_buildings = o3d.geometry.PointCloud()
        pcd_cropped_buildings.points = o3d.utility.Vector3dVector(pcd_cluster_stack) 

        # Add a color to the point cloud
        col_r = np.full((np.shape(pcd_cluster_stack)[0], 1), random.uniform(0, 1))
        col_g = np.full((np.shape(pcd_cluster_stack)[0], 1), random.uniform(0, 1)) 
        col_b = np.full((np.shape(pcd_cluster_stack)[0], 1), random.uniform(0, 1))  
        col = np.concatenate((col_r, col_g, col_b), axis=1)
        pcd_cropped_buildings.colors = o3d.utility.Vector3dVector(col)

        # Visualize it
        if debug == 1:
            o3d.visualization.draw_geometries([pcd_cropped_buildings])

        # Export point cloud
        o3d.io.write_point_cloud(buildings_folder+'/'+'building_'+str(i)+'.ply', pcd_cropped_buildings)

        '''if debug == 1:

            # For testing
            plt.figure()
            plt.scatter(pcd_cluster_points[:, 0],pcd_cluster_points[:, 1])
            plt.scatter(pcd_walls_points[:, 0],pcd_walls_points[:, 1], color='red')
            plt.title('Point cloud cluster and wall points')
            plt.xlabel('y_EOV [m]')
            plt.ylabel('x_EOV [m]')
            plt.axis('equal')
            plt.show()'''
            
        i += 1
        
    else:
        i = i


