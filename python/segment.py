# https://adioshun.gitbooks.io/pcl/content/Tutorial/Segmentation/euclidean-cluster-extraction-pcl-python.html
import argparse
import numpy as np
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
parser.add_argument('-c', '--clus_tol', type=float, default=0.02,
                    help='Cluster tolerance')
parser.add_argument('-m', '--min_clust', type=int, default=10000,
                    help='Min cluster size')
parser.add_argument('-x', '--max_clust', type=int, default=1000000,
                    help='Max cluster size')
parser.add_argument('-l', '--model_type', type=int, default=pcl.SACMODEL_PLANE,
                    help='Model type')
parser.add_argument('-e', '--method_type', type=int, default=pcl.SAC_RANSAC,
                    help='Method type')
# TODO pcl.SACMODEL_PLANE pcl.SAC_RANSAC as parameter???
args = parser.parse_args()
if args.model_type not in modes:
    print(f'Mode error, available modes: {modes}')
    print(f'SACMODEL_PLANE: {pcl.SACMODEL_PLANE}, SACMODEL_SPHERE: {pcl.SACMODEL_SPHERE}, SACMODEL_PERPENDICULAR_PLANE: {pcl.SACMODEL_PERPENDICULAR_PLANE}, SACMODEL_PARALLEL_PLANE: {pcl.SACMODEL_PARALLEL_PLANE} , SACMODEL_STICK: {pcl.SACMODEL_STICK}')
    exit(-1)
if args.method_type not in methods:
    print(f'Method error, available methods: {methods}')
    print(f'SAC_PROSAC: {pcl.SAC_RANSAC}, SAC_LMEDS: {pcl.SAC_LMEDS}, SAC_MSAC: {pcl.SAC_MSAC}, SAC_RRANSAC: {pcl.SAC_RRANSAC}, SAC_RMSAC: {pcl.SAC_RMSAC}, SAC_MLESAC: {pcl.SAC_MLESAC}, SAC_PROSAC: {pcl.SAC_PROSAC}')
    exit(-1)
cloud = pcl.load(args.name[0])

# Create the segmentation object for the planar model and set all the parameters
seg = cloud.make_segmenter()
seg.set_optimize_coefficients(True)
# model types:  pcl.SACMODEL_CIRCLE2D, pcl.SACMODEL_CIRCLE3D, pcl.SACMODEL_LINE, pcl.SACMODEL_PARALLEL_LINE, pcl.SACMODEL_PARALLEL_PLANE, pcl.SACMODEL_PERPENDICULAR_PLANE, pcl.SACMODEL_PLANE, pcl.SACMODEL_SPHERE, pcl.SACMODEL_STICK
seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_MaxIterations(args.iterations)
seg.set_distance_threshold(args.dist_thres)

nr_points = cloud.size
tree = cloud.make_kdtree()
ec = cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(args.clus_tol)
ec.set_MinClusterSize(args.min_clust)
ec.set_MaxClusterSize(args.max_clust)
ec.set_SearchMethod(tree)
# TODO pcl.SACMODEL_PLANE pcl.SAC_RANSAC as parameter???
args = parser.parse_args()
cloud = pcl.load(args.name[0])

# Create the segmentation object for the planar model and set all the parameters
seg = cloud.make_segmenter()
seg.set_optimize_coefficients (True)
# model types:  pcl.SACMODEL_CIRCLE2D, pcl.SACMODEL_CIRCLE3D, pcl.SACMODEL_LINE, pcl.SACMODEL_PARALLEL_LINE, pcl.SACMODEL_PARALLEL_PLANE, pcl.SACMODEL_PERPENDICULAR_PLANE, pcl.SACMODEL_PLANE, pcl.SACMODEL_SPHERE, pcl.SACMODEL_STICK
seg.set_model_type (pcl.SACMODEL_PERPENDICULAR_PLANE)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_MaxIterations (args.iterations)
seg.set_distance_threshold (args.dist_thres)

nr_points = cloud.size
tree = cloud.make_kdtree()
ec = cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance (args.clus_tol)
ec.set_MinClusterSize (args.min_clust)
ec.set_MaxClusterSize (args.max_clust)
ec.set_SearchMethod (tree)
cluster_indices = ec.Extract()

cloud_cluster = pcl.PointCloud()
for j, indices in enumerate(cluster_indices):
    # cloudsize = indices
    print('indices = ' + str(len(indices)))
    # cloudsize = len(indices)
    points = np.zeros((len(indices), 3), dtype=np.float32)

    for i, index in enumerate(indices):
        # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud[indice][0]) + ' ' + str(cloud[indice][1]) + ' ' + str(cloud[indice][2]))
        # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
        points[i][0] = cloud[index][0]
        points[i][1] = cloud[index][1]
        points[i][2] = cloud[index][2]
    # TODO point = cloud[indices] ?????

    cloud_cluster.from_array(points)
    ss = "cloud_cluster_" + str(j) + ".pcd"
    pcl.save(cloud_cluster, ss)
