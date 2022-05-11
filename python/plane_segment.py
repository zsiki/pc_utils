#!/usr/bin/env python3

"""
    Segment the point cloud (nDSM) to wall and roof parts
    Input point cloud should be a nDSM, ground and low vegetation removed
"""

import sys
import math
import os.path
import time
import json
import argparse
from multiprocessing import Process, Queue, cpu_count
import numpy as np
import open3d as o3d

def voxel_angle(plane):
    """ calculate angle of normal to the vertical direction

        :param plane: vector of plane equation coefficients
        :return: angle of normal direction from vertical in radians in 0-pi/2 range
    """
    return math.atan2(abs(plane[2]), math.hypot(plane[0], plane[1]))

def voxel_segment_multi(voxel, args, que):
    """ separate wall and roof part of voxel

        :param voxel: voxel point cloud
        :param args: parameters for ransac
        :param que: queue for results
    """
    for i in range(args.ransac_n_plane):
        n = np.asarray(voxel.points).shape[0]
        if n > args.ransac_limit:
            # fit ransac plane
            plane_model, inliers = voxel.segment_plane(args.ransac_threshold,
                                                       args.ransac_n,
                                                       args.ransac_iterations)
            m = len(inliers)    # number of inliers
            if m / n > args.rate[i]:
                tmp = voxel.select_by_index(inliers)
                angle = abs(voxel_angle(plane_model))
                xyz = np.asarray(tmp.points)
                print(f'*** {i} inliers: {m} angle: {angle * 180 / math.pi:.1f}')
                if angle < args.angle_limits[0]:    # wall
                    que.put(('w', xyz))
                elif angle < args.angle_limits[1] or \
                     np.max(xyz[:, 2]) < args.roof_z:   # other
                    pass
                else:   # roof
                    que.put(('r', xyz))

                # reduce pc to outliers
                voxel = voxel.select_by_index(inliers, invert=True)
        else:
            break

def voxel_segment(voxel, args):
    """ separate wall and roof part of voxel

        :param voxel: voxel point cloud
        :param args: parameters for ransac
        :param que: queue for results
    """
    res = []
    a = np.mean(np.asarray(voxel.points), axis=0)
    if 4.5 < a[0] < 5.5 and 0 < a[1] < 1.5:
        print(a)
    for i in range(args.ransac_n_plane):
        n = np.asarray(voxel.points).shape[0]
        if n > args.ransac_limit:
            # fit ransac plane
            plane_model, inliers = voxel.segment_plane(args.ransac_threshold,
                                                       args.ransac_n,
                                                       args.ransac_iterations)
            m = len(inliers)    # number of inliers
            if m / n > args.rate[i]:
                tmp = voxel.select_by_index(inliers)
                angle = abs(voxel_angle(plane_model))
                xyz = np.asarray(tmp.points)
                if angle < args.angle_limits[0]:    # wall
                    res.append(['w', xyz])
                    print(f'*** wall {i} inliers: {m} angle: {angle * 180 / math.pi:.1f}')
                elif angle < args.angle_limits[1] or \
                     np.max(xyz[:, 2]) < args.roof_z: # other
                    pass
                else:   # roof
                    res.append(['r', xyz])
                    print(f'*** roof {i} inliers: {m} angle: {angle * 180 / math.pi:.1f}')

                # reduce pc to outliers
                voxel = voxel.select_by_index(inliers, invert=True)
        else:
            break
    return res

class MultiPar():
    """ Collect necessary parameters for worker

        :param source: source class for paramters
    """
    def __init__(self, source, counter=0):

        self.ransac_n_plane = source.ransac_n_plane
        self.ransac_limit = source.ransac_limit
        self.ransac_threshold = source.ransac_threshold
        self.ransac_n = source.ransac_n
        self.ransac_iterations = source.ransac_iterations
        self.rate = source.rate
        self.angle_limits = source.angle_limits
        self.roof_z = source.roof_z
        self.counter = counter

class PointCloud():
    """
        :param file_name: open3d compatible input point cloud file
        :param voxel_size: size of voxels to fit plane
        :param ransac_limit: minimal number of points for ransac
        :param ransac_threshold: max distance from ransac plane
        :param ransac_n: number of random points for ransac plane
        :param ransac_iteration: number of iterations for ransac
        :param angle_limits: threshold to separate wall, roof and other planes
        :param rate: list of percents of points fit on ransac plane
        :param ransac_n_plane: maximal number of planes to search in a voxel
        :param out_dir: ouput directory for results
    """

    def __init__(self, file_name, voxel_size=0.5, ransac_limit=100,
                 ransac_threshold=0.025, ransac_n=10, ransac_iterations=100,
                 angle_limits=[0.087, 0.698], rate=[0.2, 0.45, 0.65, 0.8],
                 ransac_n_plane=4, roof_z=3.0):
        """ Initialize instance
        """
        self.file_name = file_name
        self.voxel_size = voxel_size
        self.ransac_limit = ransac_limit
        self.ransac_threshold = ransac_threshold
        self.ransac_n = ransac_n
        self.ransac_iterations = ransac_iterations
        self.angle_limits = angle_limits
        self.rate = rate
        self.ransac_n_plane = ransac_n_plane
        self.roof_z = roof_z
        self.pc = o3d.io.read_point_cloud(file_name)
        pc_xyz = np.asarray(self.pc.points)
        if pc_xyz.shape[0] < 1:    # empty point cloud?
            self.pc_mi = None
            self.pc_index = None
            self.rng = None
        else:
            self.pc_mi = np.min(pc_xyz, axis=0)  # get min values for indexing
            self.pc_ma = np.max(pc_xyz, axis=0)  # get max values for indexing
            self.pc_index = ((pc_xyz - self.pc_mi) / self.voxel_size).astype(int)
            self.rng = np.max(self.pc_index, axis=0)  # range of indices
        self.counter = 0    # counter for parts output

    def get_voxel(self, i, j, k):
        """ get points in voxel at index i, j, k

            :param i, j, k: indices of voxel
            :returns: point cloud with points in voxel
        """
        x = self.pc_mi[0] + i * self.voxel_size
        y = self.pc_mi[1] + j * self.voxel_size
        z = self.pc_mi[2] + k * self.voxel_size
        x1 = x + self.voxel_size
        y1 = y + self.voxel_size
        z1 = z + self.voxel_size
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x, y, z),
                                                   max_bound=(x1, y1, z1))
        return self.pc.crop(bbox)

    def segment_pc_multi(self):
        """ segment point cloud into wall and roof part using multiprocessing
        """
        n_cpu = cpu_count() # number of cpu cores
        # collect parameters for multi processing
        args = MultiPar(self)
        res = Queue()   # queue for results from multiprocessing
        w_list = []     # list for wall segments by planes
        r_list = []     # list for roof segments by planes
        procs = []
        for k in range(self.rng[2]+1):
            # select voxels in the same elevation
            z = self.pc_mi[2] + k * self.voxel_size
            zbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(self.pc_mi[0], self.pc_mi[1], z),
                                                       max_bound=(self.pc_ma[0], self.pc_ma[1], z + self.voxel_size))
            zvoxels = self.pc.crop(zbox)
            for i in range(self.rng[0]+1):
                # select voxels paralel to x
                x = self.pc_mi[0] + i * self.voxel_size
                zxbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x, self.pc_mi[1], z),
                                                            max_bound=(x + self.voxel_size, self.pc_ma[1], z + self.voxel_size))
                zxvoxels = zvoxels.crop(zxbox)
                for j in range(self.rng[1]+1):
                    # select single voxel
                    y = self.pc_mi[1] + j * self.voxel_size  # y
                    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x, y, z),
                                                               max_bound=(x+self.voxel_size, y+self.voxel_size, z+self.voxel_size))
                    voxel = zxvoxels.crop(bbox)
                    voxel_xyz = np.asarray(voxel.points)
                    if voxel_xyz.shape[0] > self.ransac_limit:  # are there enough points in voxel
                        print(k, i, j, voxel_xyz.shape)
                        if len(procs) >= n_cpu:     # all available cores used?
                            for proc in procs:
                                proc.join()         # wait for processes
                            procs = []
                            print(f'procs len: {len(procs)}')
                            while not res.empty():  # get results from queue
                                t, part = res.get()
                                if t == 'w':
                                    w_list.append(part)
                                elif t == 'r':
                                    r_list.append(part)
                        else:
                            # start paralel process
                            pr = Process(target=voxel_segment_multi, args=(voxel, args, res))
                            procs.append(pr)
                            pr.start()
        if len(procs) > 0:          # are there process running yet?
            for proc in procs:
                proc.join()
            while not res.empty():
                t, part = res.get()
                if t == 'w':
                    w_list.append(part)
                elif t == 'r':
                    r_list.append(part)

        w_xyz = np.concatenate(w_list)
        r_xyz = np.concatenate(r_list)
        return (w_xyz, r_xyz)

    def segment_pc(self):
        """ segment point cloud into wall and roof part
        """
        w_list = []     # list for wall segments by planes
        r_list = []     # list for roof segments by planes
        args = MultiPar(self)
        for k in range(self.rng[2]+1):
            # select voxels in the same elevation
            z = self.pc_mi[2] + k * self.voxel_size
            zbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(self.pc_mi[0], self.pc_mi[1], z),
                                                       max_bound=(self.pc_ma[0], self.pc_ma[1], z + self.voxel_size))
            zvoxels = self.pc.crop(zbox)
            for i in range(self.rng[0]+1):
                # select voxels paralel to x
                x = self.pc_mi[0] + i * self.voxel_size
                zxbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x, self.pc_mi[1], z),
                                                            max_bound=(x + self.voxel_size, self.pc_ma[1], z + self.voxel_size))
                zxvoxels = zvoxels.crop(zxbox)
                for j in range(self.rng[1]+1):
                    # select single voxel
                    y = self.pc_mi[1] + j * self.voxel_size  # y
                    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x, y, z),
                                                               max_bound=(x+self.voxel_size, y+self.voxel_size, z+self.voxel_size))
                    voxel = zxvoxels.crop(bbox)
                    voxel_xyz = np.asarray(voxel.points)
                    if voxel_xyz.shape[0] > 0:
                        print(k, i, j, voxel_xyz.shape)
                    if voxel_xyz.shape[0] > self.ransac_limit:  # are there enough points in voxel
                        l_parts = voxel_segment(voxel, args)
                        for l in l_parts:
                            if l[0] == 'w':
                                w_list.append(l[1])
                            elif l[0] == 'r':
                                r_list.append(l[1])
        w_xyz = None
        r_xyz = None
        if len(w_list) > 0:
            w_xyz = np.concatenate(w_list)
        if len(r_list) > 0:
            r_xyz = np.concatenate(r_list)

        return (w_xyz, r_xyz)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('name', metavar='file_name', type=str, nargs=1,
                        help='point cloud to process')
    parser.add_argument('-v', '--voxel_size', type=float, default=1.0,
                        help='Voxel size')
    parser.add_argument('-t', '--threshold', type=float, default=0.1,
                        help='Threshold distance to RANSAC plane')
    parser.add_argument('-l', '--limit', type=int, default=25,
                        help='Minimal number of points for ransac')
    parser.add_argument('-n', '--ransac_n', type=int, default=3,
                        help='Number of random points for ransac plane')
    parser.add_argument('-i', '--iterations', type=int, default=20,
                        help='Number of iterations for ransac plane')
    parser.add_argument('-a', '--angles', type=float, nargs=2,
                        help='Angle borders for walls, others and roofs')
    parser.add_argument('-r', '--rates', type=float, nargs='+',
                        help='Rates for points on the plane')
    parser.add_argument('-o', '--out_dir', type=str, default=".",
                        help='Path to output directory')
    parser.add_argument('-c', '--config', type=str,
                        help='Path to config file (json)')
    parser.add_argument('-m', '--multi', action="store_true",
                        help='Use multi processing')
    parser.add_argument('-z', '--roof_z', type=float, default=0.0,
                        help='Minimal height for roof segment, use with nDSM only')
    args = parser.parse_args()
    FNAME = args.name[0]
    # if json config given other parameters are ignored
    if args.config is not None:
        JNAME = args.config
        try:
            with open(JNAME) as jfile:
                JDATA = json.load(jfile)
                VOXEL = JDATA["voxel_size"]
                THRES = JDATA["threshold"]
                LIM = JDATA["limit"]
                N = JDATA["n"]
                ITERATION = JDATA["iteration"]
                ANG = JDATA["angle_limits"]
                RATE = JDATA["rate"]
                NP = JDATA["n_plane"]
                OUT_DIR = JDATA["out_dir"]
                ROOF_Z = JDATA["roof_z"]
                MULTI = JDATA["multi"]
        except:
            print(f'File not found or invalid {JNAME}')
            sys.exit(2)
    else:
        # command line parameters
        VOXEL = args.voxel_size
        THRES = args.threshold
        LIM = args.limit
        N = args.ransac_n
        ITERATION = args.iterations
        if args.rates is None:
            RATE = [0.20, 0.45, 0.65, 0.8]
        else:
            RATE = args.rates
        if args.angles is None:
            ANG = [0.087, 0.698]
        else:
            ANG = args.angles
        NP = args.ransac_n
        OUT_DIR = args.out_dir
        ROOF_Z = args.roof_z
        MULTI = args.multi

    base_name = os.path.splitext(os.path.basename(FNAME))[0]
    PC = PointCloud(FNAME, voxel_size=VOXEL, ransac_threshold=THRES,
                    ransac_limit=LIM, ransac_n=N, rate=RATE,
                    angle_limits=ANG, ransac_n_plane=NP, roof_z=ROOF_Z)
    if PC.pc_mi is None:
        print("Unable to load {}".format(FNAME))
        sys.exit(1)
    t1 = time.perf_counter()
    if MULTI:
        w, r = PC.segment_pc_multi()
    else:
        w, r = PC.segment_pc()
    n_w = n_r = 0
    if w is not None:
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(w)       # walls
        fname = os.path.join(OUT_DIR, base_name + '_wall.ply')
        o3d.io.write_point_cloud(fname, pc)
        n_w = w.shape[0]
    if r is not None:
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(r)       # roofs
        fname = os.path.join(OUT_DIR, base_name + '_roof.ply')
        o3d.io.write_point_cloud(fname, pc)
        n_r = r.shape[0]
    t2 = time.perf_counter()
    print(VOXEL, THRES, LIM, N, n_w, n_r, t2-t1)
