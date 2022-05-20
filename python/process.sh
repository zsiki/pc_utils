#!/bin/bash
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
echo ./pc2dem.py -r 1 -o $dir/$bn1 --rigidness 3 $pc
./pc2dem.py -r 1 -o $dir/$bn1 --rigidness 3 $pc
if [[ $? -ne 0 ]]
then
    echo pc2dem.py fails
    exit
fi
# create nDSM
echo ---- pc2ndsm ----
echo ./pc2ndsm.py $dir/$bn1.tif $pc 0.3
./pc2ndsm.py $dir/$bn1.tif $pc 0.3
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
# cluster roofs
echo ---- dbscan_clustering ----
echo ./dbscan_clustering.py --eps 2 --min_points 100 --folder $dir/roofs $dir/${bn1}_ndsm_roof.ply
./dbscan_clustering.py --eps 2 --min_points 100 --folder $dir/roofs $dir/${bn1}_ndsm_roof.ply
if [[ $? -ne 0 ]]
then
    echo dbscan_clustering.py fails
    exit
fi
echo ---- clusters2buildings ----
echo ./clusters2buildings.py -c $dir/roofs -w $dir/walls -a 12 $dir/${bn1}_ndsm_wall.ply
./clusters2buildings.py -c $dir/roofs -w $dir/walls -a 12 $dir/${bn1}_ndsm_wall.ply
echo ---- edges ----
echo ./edges.py $dir/walls/walls_*.ply
./edges.py --debug $dir/walls/walls_*.ply
