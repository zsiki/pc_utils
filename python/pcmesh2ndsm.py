'''
Convert point cloud elevations to relative heights above DEM.
The script using the Cloth Simulation Filter (CSF) plugin's results 
the mesh of the ground points (.PLY) and non-ground points (.PLY).

The script using the following modules: https://pypi.org/project/pyCloudCompareCLI/
                                        http://www.open3d.org/docs/release/index.html

'''

# Import modules
import argparse
import pyCloudCompare as cc
import numpy as np
import open3d as o3d

# Command windows parameters
parser = argparse.ArgumentParser()
parser.add_argument('name1', metavar='pc_file_name', type=str, nargs=1,
                    help='point cloud of the non-ground (.PLY)')
parser.add_argument('name2', metavar='mesh_file_name', type=str, nargs=1,
                    help='mesh of the ground (.PLY)')
parser.add_argument('-d', '--debug', type=int, default=0,
                    help='to switch debug mode use: 1 ')
parser.add_argument('-o', '--output', type=str, default='pc_file_name_ndsm.ply',
                    help='output normalized point cloud filename (.PLY)')
args = parser.parse_args()

# paramaters into variables
fnm_buildings = args.name1[0]
fnm_mesh = args.name2[0]
fnm_output = args.output
if fnm_output == 'pc_file_name_ndsm.ply':
    fnm_output = fnm_buildings[:-4]+'_ndsm.ply'
debug_val = args.debug
col = 6 # the column of the cloud-mesh distance values 

# Initalize CC
cli = cc.CloudCompareCLI()
cmd = cli.new_command()

# Debug mode
if debug_val == 0:             
    cmd.silent()

# Switch auto save off
cmd.auto_save(on_off=0)                                     

# Read a file
cmd.open(fnm_buildings)
cmd.open(fnm_mesh)

# Cloud mesh distance
cmd.c2m_dist()

# Export the result in ASCII
cmd.cloud_export_format(cc.CLOUD_EXPORT_FORMAT.ASCII, precision=3, separator=';', extension='TXT')

# Export the C2M txt
cmd.save_clouds(fnm_output[:-4]+'_c2m_result.txt')

# Print the command line 
print(cmd)

# Run the merged commands in cmd
cmd.execute()

# Import the C2M result and convert to .PLY format 
xydrgb = np.loadtxt(fnm_output[:-4]+'_c2m_result.txt',delimiter=';', skiprows=0, usecols=(0,1,col,3,4,5))
pcd = o3d.geometry.PointCloud()

# Add coordinates - x,y,distance 
pcd.points = o3d.utility.Vector3dVector(xydrgb[:,0:3])

# Add R,G,B values to the coordinates 
pcd.colors = o3d.utility.Vector3dVector(np.divide(xydrgb[:,3:6],255))

# Export to .PLY
o3d.io.write_point_cloud(fnm_output, pcd)

