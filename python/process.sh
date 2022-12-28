#!/bin/bash
# parameters for plane segment
if [[ $# -eq 0 ]]
then
    echo Usage: $0 point_cloud [working_directory]
    exit
fi
pc=$1
bn=$(basename "$1")
bn1=${bn%%.*}
if [[ $# -eq 1 ]]
then
    dir=$(dirname "$1")
else
    dir=$2
fi
echo $pc
echo $bn1
echo $dir
# create DEM
echo ---- pc2dem ----
RIGIDNESS=3
echo ./pc2dem.py -r 1 -o $dir/$bn1 --rigidness $RIGIDNESS $pc
./pc2dem.py -r 1 -o $dir/$bn1 --rigidness $RIGIDNESS $pc
if [[ $? -ne 0 ]]
then
    echo pc2dem.py fails
    exit
fi
# create nDSM
echo ---- pc2ndsm ----
MIN_HEIGHT=0.3
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
# alternate wall separation by normals OVERWRITES WALLS CREATED BY plane_segment
echo ./wall_segment.py -o $dir $dir/${bn1}_ndsm.ply
./wall_segment.py -o $dir $dir/${bn1}_ndsm.ply 
# cluster roofs
echo ---- dbscan_clustering ----
MIN_POINTS=200
echo ./dbscan_clustering.py --eps 2 --min_points $MIN_POINTS --folder $dir/roofs $dir/${bn1}_ndsm_roof.ply
./dbscan_clustering.py --eps 2 --min_points $MIN_POINTS --folder $dir/roofs $dir/${bn1}_ndsm_roof.ply
if [[ $? -ne 0 ]]
then
    echo dbscan_clustering.py fails
    exit
fi
echo ---- clusters2buildings ----
A=20
echo ./clusters2buildings.py -c $dir/roofs -w $dir/walls -a $A $dir/${bn1}_ndsm_wall.ply
./clusters2buildings.py -c $dir/roofs -w $dir/walls -a $A $dir/${bn1}_ndsm_wall.ply
echo ---- edges ----
echo ./edges.py $dir/walls/walls_*.ply
./edges.py --debug $dir/walls/walls_*.ply
