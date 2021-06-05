# Python utilities

## Open3D

### pc_filter.py

Downsample and noise reduction of point cloud. The parameters can be given
in a json configuration file.

sample json configuration:

```json
{ "voxel_downsample": "False",
  "voxel_size": 0.05,
  "statistical_outliers": "False",
  "nb_neighbors": 20,
  "std_ratio": 2.0,
  "radius_outliers": "True",
  "nb_points": 16,
  "radius": 0.25
}
```

Usage:

```
./pc_filter.py input_point_cloud output_point_cloud param.json
```

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

Find building footprints in a point cloud.
This is under development partial results are available only

Usage:

```
./building.py point_cloud json_config
```

