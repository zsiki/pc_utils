'''
Using the dbscan_clustering.py result folder, that contains the clusters to segment wall points.
Using the roof clusters a convex bounding polygon created and scaled.
With the scaled polygon the wall points are cropped and segmented by buildings.
'''
# Not yet
import os
import glob
import shutil
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull #, convex_hull_plot_2da not used

# Variables
#wall_points = 'barnag_ndsm_walls.ply'
DEF_CLUSTER_PATH = 'clusters'
DEF_BUILDINGS_PATH = 'buildings'
DEF_AREA = 10
DEF_SCALE = 1.3
DEF_LIMIT_Z = 1000

# Command line parameters
parser = argparse.ArgumentParser()
parser.add_argument('wall_points', metavar='pc_file_name', type=str, nargs=1,
                    help='point cloud of the wall points (.PLY)')
parser.add_argument('-c', '--cluster_path', type=str, default=DEF_CLUSTER_PATH,
                    help=f'folder that contains the DBSCAN roof clusters, default: {DEF_CLUSTER_PATH}')
parser.add_argument('-b', '--buildings_path', type=str,
                    default=DEF_BUILDINGS_PATH,
                    help=f'folder to save buildings, default: {DEF_BUILDINGS_PATH}')
parser.add_argument('-a', '--area', type=float, default=DEF_AREA,
                    help=f'minimum base area to form a building segment, default: {DEF_AREA}')
parser.add_argument('-s', '--scale', type=float, default=DEF_SCALE,
                    help=f'scaling the roofs area to crop wall pointsi, default: default={DEF_SCALE}')
parser.add_argument('-w', '--walls_only', action="store_true",
                    help='Do not add roof points to the building pc (use it for edge.py)')
parser.add_argument('-l', '--limit_wall_z', type=float, default=DEF_LIMIT_Z,
                    help='Do not add points to output above this height (use it for edge.py), default: no limit')
parser.add_argument('-d', '--debug', action="store_true",
                    help='to switch debug mode (displaying the results)')
args = parser.parse_args()
# Parameters into variables
pc_filename = args.wall_points[0]

# Check the existance of the folder and delete content
if os.path.isdir(args.buildings_path):
    shutil.rmtree(args.buildings_path, ignore_errors=True)
os.mkdir(args.buildings_path)

# Path of the clusters
clusters = glob.glob(os.path.join(args.cluster_path, 'cluster_*.ply'))

# Import .PLY format wall points
pcd_walls = o3d.io.read_point_cloud(pc_filename)
pcd_walls_points = np.asarray(pcd_walls.points)

# The main loop
for cluster in clusters:
    # get building ID
    name_id, _ = os.path.splitext(cluster)
    b_id = name_id.split('_')[-1]
    # Import a roof cluster
    pcd_cluster = o3d.io.read_point_cloud(cluster)
    # Coordinates of the point clouds
    pcd_cluster_xyz = np.asarray(pcd_cluster.points)

    # Convex hull TODO concave hull would be better (shapely.concavehull)
    pcd_cluster_convhull = ConvexHull(pcd_cluster_xyz[:, :2])   # 2D!

    # collect vertices by index
    pcd_cluster_convhull_xyz = pcd_cluster_xyz[pcd_cluster_convhull.vertices]
    # print(pcd_cluster_convhull_xyz[1,:])

    if args.debug:
        print(f'cluster_{b_id} kerület: {pcd_cluster_convhull.area:.2f} terület: {pcd_cluster_convhull.volume:-2f}')

    # Check base area of the roofs
    if pcd_cluster_convhull.volume >= args.area:
        # Scale it with Open3D                                                  TODO: scaling is not perfect along the building sides -> other solutions?
        pcd_cluster_convhull = o3d.geometry.PointCloud()
        pcd_cluster_convhull.points = o3d.utility.Vector3dVector(pcd_cluster_convhull_xyz)
        pcd_cluster_convhull_sc = pcd_cluster_convhull.scale(args.scale,
                center=pcd_cluster_convhull.get_center())

        #print(np.array(pcd_cluster_convhull_sc.points))
        pcd_cluster_convhull_sc_xyz = np.array(pcd_cluster_convhull_sc.points)

        if args.debug:
            plt.figure()
            plt.scatter(pcd_cluster_convhull_xyz[:, 0],
                        pcd_cluster_convhull_xyz[:, 1])
            plt.scatter(pcd_cluster_convhull_sc_xyz[:, 0],
                        pcd_cluster_convhull_sc_xyz[:, 1], color='red')
            plt.title('Point cloud of the cluster_'+b_id+' and wall points')
            plt.xlabel('y_EOV [m]')
            plt.ylabel('x_EOV [m]')
            plt.axis('equal')
            plt.show()

        # Create a SelectionPolygonVolume
        cluster_volume = o3d.visualization.SelectionPolygonVolume()

        # Specify what axis to orient the polygon and min/max of the the polygon vertices
        cluster_volume.orthogonal_axis = "Z"
        cluster_volume.axis_max = args.limit_wall_z
        cluster_volume.axis_min = 0

        # Convert the np.array to a Vector3dVector
        cluster_volume.bounding_polygon = o3d.utility.Vector3dVector(pcd_cluster_convhull_xyz)

        # Crop the point cloud using the Vector3dVector
        pcd_cropped = cluster_volume.crop_point_cloud(pcd_walls)

        if args.walls_only:
            pcd_cluster_stack = pcd_cropped
        else:
            # Append with the roof points
            pcd_cluster_stack = pcd_cropped + pcd_cluster

        # check for empty pc
        if len(pcd_cluster_stack.points) > 0:
            # Visualize it
            if args.debug:
                #o3d.visualization.draw_geometries([pcd_cropped_buildings])
                o3d.visualization.draw_geometries([pcd_cluster_stack])

            # Export point cloud
            o3d.io.write_point_cloud(os.path.join(args.buildings_path, f'building_{b_id}.ply'), pcd_cluster_stack)

        else:
            if args.debug:
                print(f'Empty PC skipped {cluster}')
