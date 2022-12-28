#!/usr/bin/env python3

"""
    generate sample plane data for CNN ML
"""
# TODO vertical corner generation
# TODO roof shapes
import argparse
from math import pi, sin, cos
import numpy as np

def eigen_based_features(pnts):
    """ calculate eigen based feautures of pont cloud
        :param pnts: numpy array of points
        :returns: list of params lambda_x
    """
    p_mean = np.mean(pnts, axis=0)
    # differences from mean
    diffs = pnts - p_mean
    cov = diffs.T.dot(diffs)
    # standard deviations
    sigma = np.sqrt(np.diagonal(cov) / pnts.shape[0])
    # eigen vvalues/vectors
    e_val, e_vec = np.linalg.eig(cov)
    e_val_sorted = np.sort(e_val)[::-1]   # sort eigen values decreasing
    # eigen based features
    # anisotropy
    lambda_a = (e_val_sorted[0] - e_val_sorted[2]) / e_val_sorted[0]
    # linearity
    lambda_l = (e_val_sorted[0] - e_val_sorted[1]) / e_val_sorted[0]
    # planarity
    lambda_p = (e_val_sorted[1] - e_val_sorted[2]) / e_val_sorted[0]
    # surface variation
    lambda_v = e_val_sorted[2] / np.sum(e_val_sorted)
    # omnivariance
    lambda_o = np.prod(e_val_sorted)**(1/3)
    # sphericity
    lambda_s = e_val_sorted[2] / e_val_sorted[0]
    # eigen entropy
    lambda_e = - np.sum(e_val_sorted * np.log(e_val_sorted))

    return lambda_a, lambda_l, lambda_p, lambda_v, lambda_o, lambda_s, lambda_e

def gen_plane(n, size, direction, slope, rng=0.02):
    """ generate random points on vertical plane
        :param n: number of points
        :param size: size of including cube
        :param direction: direction in xy plane radians
        :param slope: slope angle around radians
        :param rng: random range for points on the plane
    """
    # points in xz plane
    points = np.zeros((n, 3))
    points[:,0] = size * np.random.rand(n) - size / 2 + rng * np.random.rand(n) - rng / 2
    points[:,1] = rng * np.random.rand(n) - rng / 2
    points[:,2] = size * np.random.rand(n) - size / 2 + rng * np.random.rand(n) - rng / 2
    # rotation matrix
    r_mat = np.array([[cos(direction), -sin(direction), 0.0],
                      [sin(direction), cos(direction), 0.0],
                      [0.0, 0.0, 1.0]]).dot(
            np.array([[1.0, 0.0, 0.0],
                      [0.0, cos(slope), sin(slope)],
                      [0.0, -sin(slope), cos(slope)]]))
    return points.dot(r_mat)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--num_points', type=int, default=1000,
                        help='number of points in voxel, default 1000')
    parser.add_argument('-v', '--voxel_size', type=float, default=1.0,
                        help='voxel size, default 1')
    parser.add_argument('-r', '--range', type=float, default=0.02,
                        help='random range from the plane, default 0.02')
    parser.add_argument('-a', '--angle_step', type=int, default=30,
                        help='angle step rotating plane, default 30')
    parser.add_argument('-s', '--save_path', type=str, default=None,
                        help='path to save point clouds, default None/do not save')
    parser.add_argument('-d', '--header', action="store_true",
                        help='add header to output')
    args = parser.parse_args()

    HEAD = "range angle1 angle2 anisotropy linearity planarity surf_vari omni_vari spher entropy"
    if args.header:
        print(HEAD)
    for angle1 in range(0, 360, args.angle_step):
        for angle2 in range(0, 180, args.angle_step):
            pts = gen_plane(args.num_points, args.voxel_size,
                             angle1 / 180 * pi, angle2 / 180 * pi, args.range)
            if args.save_path is not None:
                np.savetxt(f'{args.save_path}/p_{args.range:.2f}_{angle1}_{angle2}.xyz', pts)
            ebf = eigen_based_features(pts)
            ebf_str = [f"{i:.5f}" for i in ebf]
            print(args.range, angle1, angle2, " ".join(ebf_str))
