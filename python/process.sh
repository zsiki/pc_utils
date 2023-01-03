#!/bin/bash
# Point cloud to 2D buildings lines (CAD)

# parameters
RIGIDNESS=3     # rigidness for CSF (pc2dem)
MIN_HEIGHT=0.3  # remove points below MIN_HEIGHT from nDSM
MIN_POINTS=200  # minimum number of points in a cluster for DBSCAN
DBSCAN_MODE=1   # 0/1 open3d/scikit learn
DBSCAN_EPS=3    # distance parameter for DBSCAN
A=20            # min area for a building
# end of changable parameters

if [[ $# -eq 0 ]]
then
    echo "Usage: $0 point_cloud [working_directory] [segment_method]"
    echo "  point_cloud ply/xyz/xyzn/xyzrgb/pts/pcd formats are supported"
    echo "  working_directory all temperary and output files are sent to this dir"
    echo "  segment_method 0/1 RANSAC/NORMALS, default 0"
    exit
fi
pc=$1
bn=$(basename "$1")
bn1=${bn%%.*}
if [[ $# -gt 1 ]]
then
    dir=$2
else
    dir=$(dirname "$1")
fi
if [[ $# -gt 2 ]]
then
    SM=$2
else
    SM=0
fi
echo "Point cloud: $pc"
echo "Base name: $bn1"
echo "Working dir: $dir"
echo "Segment method: $SM"

# create DEM using Cloth Simulation Filter (CSF) algorithm
echo ---- pc2dem ----
echo ./pc2dem.py -r 1 -o $dir/$bn1 --rigidness $RIGIDNESS $pc
./pc2dem.py -r 1 -o $dir/$bn1 --rigidness $RIGIDNESS $pc
if [[ $? -ne 0 ]]
then
    echo pc2dem.py fails
    exit
fi
# create nDSM
echo ---- pc2ndsm ----
echo ./pc2ndsm.py $dir/$bn1.tif $pc $MIN_HEIGHT
./pc2ndsm.py $dir/$bn1.tif $pc $MIN_HEIGHT
if [[ $? -ne 0 ]]
then
    echo pc2ndsm.py fails
    exit
fi
# separate walls and roofs
echo ---- plane_segment ----
echo ./plane_segment.py -c $dir/plane_segment.json $dir/${bn1}_ndsm.ply
./plane_segment.py -c $dir/plane_segment.json $dir/${bn1}_ndsm.ply
if [[ $? -ne 0 ]]
then
    echo plane_segment.py fails
    exit
fi
if [[ $SM -eq 1 ]]
then
    # alternate wall separation by normals OVERWRITES WALLS CREATED BY plane_segment
    echo ./wall_segment.py -o $dir $dir/${bn1}_ndsm.ply
    ./wall_segment.py -o $dir $dir/${bn1}_ndsm.ply 
fi
# cluster roofs, make groups of continuous roof points
echo ---- dbscan_clustering ----
echo ./dbscan_clustering.py --modul $DBSCAN_MODE --eps $DBSCAN_EPS --min_points $MIN_POINTS --folder $dir/roofs $dir/${bn1}_ndsm_roof.ply
./dbscan_clustering.py --modul $DBSCAN_MODE --eps $DBSCAN_EPS --min_points $MIN_POINTS --folder $dir/roofs $dir/${bn1}_ndsm_roof.ply
if [[ $? -ne 0 ]]
then
    echo dbscan_clustering.py fails
    exit
fi
# find corners and edges of buildings in 2D
echo ---- clusters2buildings ----
echo ./clusters2buildings.py -c $dir/roofs -w $dir/walls -a $A $dir/${bn1}_ndsm_wall.ply
./clusters2buildings.py -c $dir/roofs -w $dir/walls -b $dir/buildings -a $A $dir/${bn1}_ndsm_wall.ply
echo ---- edges ----
echo ./edges.py $dir/walls/walls_*.ply
./edges.py --debug $dir/walls/walls_*.ply
