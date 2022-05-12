# Python utilities

Processing steps to find building footprints in point clouds. These small utilities use open3d, gdal, numpy.

1. Outliers handling, filtering by *pc_filter.py*
2. Separate ground and non-ground point and DEM generation from ground points *pc2dem.py* using CSF algorithm
3. Creating nDSM with low vegetation removed *pc2ndsm.py*
4. Generatating spare point cloud using voxels, only significant planes are preserved with a point in the in the voxel and normal *building.py*
5. segment spare point cloud int o wall, roof and other points *building.py*
6. group roof points with clustering *dbscan_clustering.py*
7. segment wall points by the roof polygons
8. find 2D concave hull of wall segments

## Flowchart of processing:

![image](https://user-images.githubusercontent.com/48557905/167934260-c4d37565-fd11-4745-bf47-738d2986636f.png)

## Data pre-processing

### pc_filter.py

Downsample and noise reduction of point cloud. The parameters can be given
in a json configuration file or in the command line. An ordinal number is
assigned to the method name (statistical_outliers, radius_outliers,
voxel_downsample) if it is larger or equal to zero the method is applied in
increasing order of the numbers.

Usage:

```
./pc_filter.py input_point_cloud output_point_cloud param.json
```

Three different downsample/filter methods are selectable from the Open3D library.

Statistical outlier removal is a noise reduction method. It removes points 
that are further away from their neighbors compared to the average for the 
point cloud. There are two parameters the number of neighbors to be taken 
account, and the threshold for the standard deviation ratio. 
The lower this threshold the more aggressive the filter will be.

Radius outlier removal algorithm removes points wich have fewer neighbors in a 
given radius (isolited points). There are two parameters the minimum number of
points the sphere should contain and the radius of the sphere.

Voxel downsample keeps one point in each occupied voxel calculating as an
average of points in the voxel. There is only one parameter the voxel size.

One, two or all filters can be applied in a session.
The applying order of the filters is defined in the json parameter file.

Sample json parameter file

```
{ "voxel_downsample": 2,
  "voxel_size": 0.05,
  "statistical_outliers": 0,
  "nb_neighbors": 20,
  "std_ratio": 2.0,
  "radius_outliers": 1,
  "nb_points": 16,
  "radius": 2
}

```

The above sample first applies statistical outliers filter (0), then radius
outliers filter (1) and finally voxel downsample (2). If the value is less than
zero for the filter or the filter is not in the parameter file, it is not used.

### pc2dem.py

Separate ground and non-ground points using cloth filter simulation. (CSF) 
Save non-ground points and a DEM generated from ground points using GDAL.

```
usage: pc2dem.py [-h] [-r RESOLUTION] [-o OUTPUT] [--rigidness RIGIDNESS]
                 [--smooth] [--iterations ITERATIONS]
                 file_name

positional arguments:
  file_name             point cloud to process

optional arguments:
  -h, --help            show this help message and exit
  -r RESOLUTION, --resolution RESOLUTION
                        resolution for dem
  -o OUTPUT, --output OUTPUT
                        output DEM file (TIF) and non-ground points
  --rigidness RIGIDNESS
                        rigidness of cloth 1 or 2 or 3
  --smooth              postprocess to smooth
  --iterations ITERATIONS
                        number of iterations
```

### pc2ndsm.py

Point cloud to normalized Digital Surface Model (nDSM). The height differencess
are calculated from a DTM (any GDAL compatible DTM can be used). The low
vegetation and terrain can also be filtered adding a minimum elevation for the
nDSM. If no minimum elevation is given all points are preserved.

Usage:

```
./pc2ndsm.py dem_file point_cloud min_elev
```

## Alternative way of the pre-processing with CloudCompare

![](https://github.com/zsiki/pc_utils/blob/main/python/images/pre_procc_alt.png)

### pc_csf.py

Classify input point cloud into ground and non-ground points with Cloth Simulation Filter (CSF).
The result are saved into .PLY format.

```
usage: pc_csf.py [-h] [-r RESOLUTION] [-o OUTPUT] [--rigidness RIGIDNESS] [--smooth]
       [--iterations ITERATIONS] [--classification CLASSIFICATION] file_name

positional arguments:
  file_name             point cloud to process

optional arguments:
  -h, --help            show this help message and exit
  -r RESOLUTION, --resolution RESOLUTION
                        resolution for CSF
  -o OUTPUT, --output OUTPUT
                        output base name for ground and non-ground points
  --rigidness RIGIDNESS
                        rigidness of cloth 1,2,3: mountain with desen vegetation(1) OR complex scenes(2)
                        OR flat terrain with high-rise buildings(3)
  --smooth              postprocess to smooth
  --iterations ITERATIONS
                        number of iterations
  --classification CLASSIFICATION
                        classification threshold
```

### pc2mesh.py

Using the *pc2dem.py* result (separeted ground points) create a .PLY format mesh,
which can be considered as a DTM.

```
usage: pc2mesh.py [-h] [-r RESOLUTION] [-d DEBUG] [-o OUTPUT] file_name

positional arguments:
  file_name             point cloud of the ground (.PLY)

optional arguments:
  -h, --help            show this help message and exit
  -r RESOLUTION, --resolution RESOLUTION
                        resolution of the GRID
  -d DEBUG, --debug DEBUG
                        to switch debug mode use: 1
  -o OUTPUT, --output OUTPUT
                        filename of the output mesh (.PLY)
```

### pcmesh2ndsm.py
 
 Create a normalized Digital Surface Model (nDSM) using the mesh of the
 ground points (DTM) and the off-ground points.  
 
``` 
usage: pcmesh2ndsm.py [-h] [-d DEBUG] [-o OUTPUT] pc_file_name mesh_file_name

positional arguments:
  pc_file_name          point cloud of the non-ground (.PLY)
  mesh_file_name        mesh of the ground (.PLY)

optional arguments:
  -h, --help            show this help message and exit
  -d DEBUG, --debug DEBUG
                        to switch debug mode use: 1
  -o OUTPUT, --output OUTPUT
                        output normalized point cloud filename (.PLY)
```

## Data segmentation

### plane_segment.py

Find building footprints in a point cloud using a divide and conquer algorithm.
First the points are divided into voxels. For points in each voxel one or more
planes are fitted using RANSAC method. If enough points fit to the RANSAC plane, the
voxel is substituted by a point on the planes and the normal directions
These points are collected into a new spare point cloud. This spare pount cloud
is segmented into roof, wall and other categories finally.
This is under development partial results are available only.


```
usage: building.py [-h] [-v VOXEL_SIZE] [-t THRESHOLD] [-l LIMIT]
                   [-n RANSAC_N] [-i ITERATIONS] [-a ANGLES ANGLES]
                   [-r RATES [RATES ...]] [-o OUT_DIR] [-c CONFIG] [-d] [-m]
                   [-s] [-z ROOF_Z]
                   file_name

positional arguments:
  file_name             point cloud to process

optional arguments:
  -h, --help            show this help message and exit
  -v VOXEL_SIZE, --voxel_size VOXEL_SIZE
                        Voxel size
  -t THRESHOLD, --threshold THRESHOLD
                        Threshold distance to RANSAC plane
  -l LIMIT, --limit LIMIT
                        Minimal number of points for ransac
  -n RANSAC_N, --ransac_n RANSAC_N
                        Number of random points for ransac plane
  -i ITERATIONS, --iterations ITERATIONS
                        Number of iterations for ransac plane
  -a ANGLES ANGLES, --angles ANGLES ANGLES
                        Angle borders for walls, others and roofs
  -r RATES [RATES ...], --rates RATES [RATES ...]
                        Rates for points on the plane
  -o OUT_DIR, --out_dir OUT_DIR
                        Path to output directory
  -c CONFIG, --config CONFIG
                        Path to config file (json)
  -d, --debug           Save ascii point cloud for each plane and print plane
                        data
  -m, --multi           Use multi processing
  -s, --skip_spare      Do not generate spare cloud, only normals for the
                        original
  -z ROOF_Z, --roof_z ROOF_Z
                        Minimal height for roof segment, use with nDSM only
```

sample json configuration:

```
{ "voxel_size": 1,
  "threshold" : 0.1,
  "limit": 100,
  "n": 10,
  "iteration": 25,
  "angle_limits": [0.087, 0.698],
  "rate": [0.20, 0.45, 0.65, 0.8],
  "n_plane": 4,
  "out_dir": "./temp",
  "skip_spare": 0,
  "roof_z": 3,
  "multi": 1,
  "debug": 0
}
```

Angle limits means deflection from vertical 0 - limit1 is wall limit1 - limit2 is other
limit2 to pi/2 roof. Angles are in radians. Rates vector is used if more than one RANSAC planes are searched in a voxel and this values are the precentes of points on a
plane to accept.


### dbscan_clustering.py

The scripts using a Machine Learning (ML) based algorithm, the DBSCAN clustering to separete group of points in a point cloud.
During the usage the distance to neighbors in a cluster (eps) and the minimum number of points required to form a cluster (min_points) have to be defined.

```
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
                        
   ```                     
### clusters2buildings.py 

The script using the dbscan_clustering.py result folder to segment wall points.
The roof clusters are used to create scaled bounding polygons.
With the polygons the wall points are cropped, segmented and saved by buildings.  

   ``` 
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
   ``` 
