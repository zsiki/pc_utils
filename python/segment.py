#!/usr/bin/env python3

"""
... module:: segment.py
    :platform: Linux, Windows

    Separate continuous segment from a point cloud
"""
# https://adioshun.gitbooks.io/pcl/content/Tutorial/Segmentation/euclidean-cluster-extraction-pcl-python.html
import argparse
import os
import json
import pcl

modes = (pcl.SACMODEL_PLANE, pcl.SACMODEL_SPHERE,
         pcl.SACMODEL_PERPENDICULAR_PLANE, pcl.SACMODEL_PARALLEL_PLANE,
         pcl.SACMODEL_STICK)
methods = (pcl.SAC_RANSAC, pcl.SAC_LMEDS, pcl.SAC_MSAC, pcl.SAC_RRANSAC,
           pcl.SAC_RMSAC, pcl.SAC_MLESAC, pcl.SAC_PROSAC)
# RandomSampleConsensus, LeastMedianSquare, MEstimatorSampleConsensus, RandomizedMEstimatorSampleConsensus, MaximumLikelihoodSampleConsensus, ProgressiveSampleConsensus
parser = argparse.ArgumentParser()
parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                    help='point cloud to process')
parser.add_argument('-t', '--dist_thres', type=float, default=0.02,
                    help='Distance threshold')
parser.add_argument('-i', '--iterations', type=int, default=100,
                    help='Max RANSAC iteration')
parser.add_argument('-u', '--clust_tol', type=float, default=0.02,
                    help='Cluster tolerance')
parser.add_argument('-m', '--min_clust', type=int, default=10000,
                    help='Min cluster size')
parser.add_argument('-x', '--max_clust', type=int, default=1000000,
                    help='Max cluster size')
parser.add_argument('-l', '--model_type', type=int, default=pcl.SACMODEL_PLANE,
                    help='Model type')
parser.add_argument('-e', '--method_type', type=int, default=pcl.SAC_RANSAC,
                    help='Method type')
parser.add_argument('-c', '--config', type=str,
                    help='Path to config file (json)')
parser.add_argument('-o', '--out_dir', type=str, default=".",
                    help='Path to output directory')
args = parser.parse_args()
# if json config given other parameters are ignored
if args.config is not None:
    JNAME = args.config
    with open(JNAME) as jfile:
        JDATA = json.load(jfile)
        DIST_THRES = JDATA["dist_thres"]
        ITERATIONS = JDATA["iterations"]
        CLUS_TOL = JDATA["clust_tol"]
        MIN_CLUST = JDATA["min_clust"]
        MAX_CLUST = JDATA["max_clust"]
        MODEL_TYPE = JDATA["model_type"]
        METHOD_TYPE = JDATA["method_type"]
        OUT_DIR = JDATA["out_dir"]
else:
    DIST_THRES = args.dist_thres
    ITERATIONS = args.iterations
    CLUS_TOL = args.clust_tol
    MIN_CLUST = args.min_clust
    MAX_CLUST = args.max_clust
    MODEL_TYPE = args.model_type
    METHOD_TYPE = args.method_type
    OUT_DIR = args.out_dir
if MODEL_TYPE not in modes:
    print(f'Mode error, available modes: {modes}')
    print(f'SACMODEL_PLANE: {pcl.SACMODEL_PLANE}, SACMODEL_SPHERE: {pcl.SACMODEL_SPHERE}, SACMODEL_PERPENDICULAR_PLANE: {pcl.SACMODEL_PERPENDICULAR_PLANE}, SACMODEL_PARALLEL_PLANE: {pcl.SACMODEL_PARALLEL_PLANE} , SACMODEL_STICK: {pcl.SACMODEL_STICK}')
    exit(-1)
if METHOD_TYPE not in methods:
    print(f'Method error, available methods: {methods}')
    print(f'SAC_PROSAC: {pcl.SAC_RANSAC}, SAC_LMEDS: {pcl.SAC_LMEDS}, SAC_MSAC: {pcl.SAC_MSAC}, SAC_RRANSAC: {pcl.SAC_RRANSAC}, SAC_RMSAC: {pcl.SAC_RMSAC}, SAC_MLESAC: {pcl.SAC_MLESAC}, SAC_PROSAC: {pcl.SAC_PROSAC}')
    exit(-1)
cloud = pcl.load(args.name[0])

# Create the segmentation object for the planar model and set all the parameters
seg = cloud.make_segmenter()
seg.set_optimize_coefficients(True)
# model types:  pcl.SACMODEL_CIRCLE2D, pcl.SACMODEL_CIRCLE3D, pcl.SACMODEL_LINE, pcl.SACMODEL_PARALLEL_LINE, pcl.SACMODEL_PARALLEL_PLANE, pcl.SACMODEL_PERPENDICULAR_PLANE, pcl.SACMODEL_PLANE, pcl.SACMODEL_SPHERE, pcl.SACMODEL_STICK
seg.set_model_type(MODEL_TYPE)
seg.set_method_type(METHOD_TYPE)
seg.set_MaxIterations(ITERATIONS)
seg.set_distance_threshold(DIST_THRES)

nr_points = cloud.size
tree = cloud.make_kdtree()
ec = cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(CLUS_TOL)
ec.set_MinClusterSize(MIN_CLUST)
ec.set_MaxClusterSize(MAX_CLUST)
ec.set_SearchMethod(tree)
cluster_indices = ec.Extract()

cloud_cluster = pcl.PointCloud()
for j, indices in enumerate(cluster_indices):
    cloud_cluster = cloud.extract(indices)
    ss = os.path.join(OUT_DIR, f"cloud_cluster_{j}.pcd")
    pcl.save(cloud_cluster, ss)
