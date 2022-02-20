# Python utilities

## Open3D

### pc_filter.py

Downsample and noise reduction of point cloud. The parameters can be given
in a json configuration file.

sample json configuration:

```json
{ "statistical_outliers": "False",
  "nb_neighbors": 20,
  "std_ratio": 2.0,
  "radius_outliers": "True",
  "nb_points": 16,
  "radius": 0.25
  "voxel_downsample": "False",
  "voxel_size": 0.05,
}
```

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

One, two or all filters can be applied in on session.
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

### pc2ndsm.py

Point cloud to normalized Digital Surface Model (nDSM). The height differencess
are calculated from a DTM (any gdal compatible DTM can be used). The low
vegetation and terrain can also be filtered adding a minimum elevation for the
nDSM.

Usage:

```
./pc2ndsm.py dem_file point_cloud min_elev
```

### building.py

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
