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

The applying order of the filters is statistical outliers, radius outlier and
voxel downsampling if selected.

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
First the points are divided into voxels. For points in each voxel a plane is 
fitted using RANSAC method. If most of the points fit to the RANSAC plane, the
voxel is substituted by a point on the plane and the normal direction is
saved in the color fields. These points are collected into a new spare point 
cloud. This spare pount cloud is segmented into roof and wall finally.
This is under development partial results are available only.

Usage:

```
./building.py point_cloud json_config
```

sample json configuration:

{ "voxel_size": [0.5],
  "threshold" : [0.05, 0.1],
  "limit": [25, 50],
  "n": [5, 10]
}

For all parameters a vector can be given, for each vector element the algorithm 
is excuted. The threshold, limit and n parameters are the parameters of the
Open3D RANSAC algorithms.
