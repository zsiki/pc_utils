'''
Create a mesh from the ground points separeted by Cloth Simulation Filter (CSF) plugin.
The script uses CloudCompare(CC) built-in functions.

The script using the following module: https://pypi.org/project/pyCloudCompareCLI/

'''
# Import modules
import argparse
import pyCloudCompare as cc


# Command windows parameters
parser = argparse.ArgumentParser()
parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                    help='point cloud of the ground (.PLY)')
parser.add_argument('-r', '--resolution', type=float, default=2.0,
                    help='resolution of the GRID')
parser.add_argument('-d', '--debug', type=int, default=0,
                    help='to switch debug mode use: 1 ')
parser.add_argument('-o', '--output', type=str, default='file_name_mesh.ply',
                    help='filename of the output mesh (.PLY)')
args = parser.parse_args()

# paramaters into variables
fnm = args.name[0]
res = str(args.resolution)
ofn = args.output
if ofn == 'file_name_mesh.ply':
    ofn = fnm[:-4]+'_mesh.ply'
debug_val = args.debug

# Initalize CC
cli = cc.CloudCompareCLI()
cmd = cli.new_command()

# Debug mode
if debug_val == 0:             
    cmd.silent()

# Switch auto save off
cmd.auto_save(on_off=0)                                     #TODO: auto save off, but the CC   

# Read a file
cmd.open(fnm)

# Export parameters
cmd.mesh_export_format(cc.MESH_EXPORT_FORMAT.PLY, extension='PLY')

# Rasterize command 
cmd.rasterize(grid_step=res, vert_dir=2, proj='AVG', sf_proj='AVG', empty_fill='INTERP', output_mesh=True)

# Export mesh in ply
cmd.save_meshes(ofn)

# Print the command line 
print(cmd)

# Run the merged commands in cmd
cmd.execute()