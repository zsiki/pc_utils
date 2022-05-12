#!/usr/bin/env python3

"""
    Create spare point cloud using ransac planes and
    separate wall, roof and other groups by normal angles of
    spare point cloud.
    Input point cloud should be a nDSM, ground and low vegetation removed
"""
#   TODO list
#   colors lost in output?

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

def voxel_ransac(voxel, args, que):
    """ fit planes to voxel for multiprocessing

        :param voxel: voxel point cloud
        :param args: parameters for ransac
        :param que: queue for results
    """
    args.counter += 1
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
                if args.debug:
                    pid = args.counter * 10 + i
                    angle = abs(voxel_angle(plane_model))
                    xyz = np.asarray(tmp.points)
                    if angle < args.angle_limits[0]:    # wall
                        t = 'w'
                    elif angle < args.angle_limits[1] or \
                         np.max(xyz[:, 2]) < args.roof_z:  # other
                        t = 'o'
                    else:
                        t = 'r'                         # roof

                    np.savetxt(os.path.join(args.out_dir, f'temp{pid}_{t}.txt'),
                               np.c_[xyz, np.full(m, pid)])
                que.put((plane_model, np.asarray(tmp.points)[m // 2], np.asarray(tmp.colors)[m // 2]))
                # reduce pc to outliers
                voxel = voxel.select_by_index(inliers, invert=True)
        else:
            break

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
        self.debug = source.debug
        self.angle_limits = source.angle_limits
        self.out_dir = source.out_dir
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
                 ransac_n_plane=4, roof_z=3.0, out_dir='.', debug=False):
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
        self.out_dir = out_dir
        self.debug = debug
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
        self.spare_pc = None    # down sampled point cloud
        self.spare_voxel = None # voxels with normal direction
        self.counter = 0    # counter for parts output

    def get_voxel_old(self, i, j, k):
        """ get voxel at index i, j, k

            :returns: point cloud with points in voxel
        """
        pc_xyz = np.asarray(self.pc.points)
        colors = np.asarray(self.pc.colors)
        cond = np.logical_and(np.logical_and(self.pc_index[:, 0] == i,
                                             self.pc_index[:, 1] == j),
                              self.pc_index[:, 2] == k)
        voxel = o3d.geometry.PointCloud()
        voxel.points = o3d.utility.Vector3dVector(pc_xyz[cond, :])
        voxel.colors = o3d.utility.Vector3dVector(colors[cond, :])
        return voxel

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

    def voxel_ransac(self, voxel):
        """ fit planes to voxel

            :param voxel: voxel point cloud
            :returns: list of plane parameters a,b,c,d
        """
        res = []
        self.counter += 1
        for i in range(self.ransac_n_plane):
            n = np.asarray(voxel.points).shape[0]
            if n > self.ransac_limit:
                # fit ransac plane
                plane_model, inliers = voxel.segment_plane(self.ransac_threshold,
                                                           self.ransac_n,
                                                           self.ransac_iterations)
                m = len(inliers)    # number of inliers
                #if self.debug:
                #    print("{:4d} {:4d} {:6d} {:6d}".format(self.counter,
                #                                           i, n, m))
                #    print("{:.6f} {:.6f} {:.6} {:.6}".format(plane_model[0],
                #                                             plane_model[1],
                #                                             plane_model[2],
                #                                             plane_model[3]))
                if m / n > self.rate[i]:
                    tmp = voxel.select_by_index(inliers)
                    xyz = np.asarray(tmp.points)
                    if self.debug:
                        pid = self.counter * 10 + i
                        angle = abs(voxel_angle(plane_model))
                        if angle < self.angle_limits[0]:    # wall
                            t = 'w'
                        elif angle < self.angle_limits[1] or \
                                np.max(xyz[:, 2]) < self.roof_z:  # other
                            t = 'o'
                        else:
                            t = 'r'                         # roof

                        np.savetxt(os.path.join(self.out_dir,
                                                f'temp{pid}_{t}.txt'),
                                   np.c_[xyz, np.full(m, pid)])
                    res.append([plane_model, xyz[m // 2], np.asarray(tmp.colors)[m // 2]])
                    # reduce pc to outliers
                    voxel = voxel.select_by_index(inliers, invert=True)
                else:
                    return res
            else:
                return res
        return res

    def create_spare_pc_multi(self):
        """ create spare point cloud for plane voxels only using multiprocessing
        """
        n_max = (self.rng[0] + 1) * (self.rng[1] + 1) * (self.rng[2] + 1) * self.ransac_n_plane
        xyz = np.zeros((n_max, 3))
        normal = np.zeros((n_max, 3)).astype(np.single)
        color = np.zeros((n_max, 3)).astype(np.single)
        p = np.array([0, 0, 0, 1], dtype=float)
        n_voxel = 0
        n_cpu = cpu_count() # number of cpu cores
        # collect parameters for multi processing
        args = MultiPar(self)
        res = Queue()   # queue for results from multiprocessing
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
                        if len(procs) >= n_cpu:     # all available cores used?
                            for proc in procs:
                                proc.join()         # wait for processes
                            procs = []
                            while not res.empty():  # get results from queue
                                plane, pp, c = res.get()
                                # x, y, z projected to the plane
                                p[0:3] = pp
                                t = np.dot(p, plane)    # point - plane distance
                                xyz[n_voxel, 0] = p[0] - t * plane[0]
                                xyz[n_voxel, 1] = p[1] - t * plane[1]
                                xyz[n_voxel, 2] = p[2] - t * plane[2]
                                normal[n_voxel] = plane[0:3]
                                try:
                                    color[n_voxel] = c  # TODO
                                except IndexError:
                                    color[n_voxel] = np.array([1, 1, 1])
                                n_voxel += 1
                        else:
                            # start paralel process
                            pr = Process(target=voxel_ransac, args=(voxel, args, res))
                            procs.append(pr)
                            pr.start()
        if len(procs) > 0:          # are there process running yet?
            for proc in procs:
                proc.join()
            while not res.empty():
                plane, index = res.get()
                # x, y, z projected to the plane
                p[0:3] = voxel_xyz[index]
                t = np.dot(p, plane)    # point - plane distance
                xyz[n_voxel, 0] = p[0] - t * plane[0]
                xyz[n_voxel, 1] = p[1] - t * plane[1]
                xyz[n_voxel, 2] = p[2] - t * plane[2]
                normal[n_voxel] = plane[0:3]
                try:
                    color[n_voxel] = np.asarray(voxel.colors)[index]
                except IndexError:
                    color[n_voxel] = np.array([1, 1, 1])
                n_voxel += 1

        xyz = np.resize(xyz, (n_voxel, 3))              # free unused memory
        normal = np.resize(normal, (n_voxel, 3))
        color = np.resize(normal, (n_voxel, 3))
        self.spare_pc = o3d.geometry.PointCloud()       # create new point cloud for spare point cloud
        self.spare_pc.points = o3d.utility.Vector3dVector(xyz)
        self.spare_pc.normals = o3d.utility.Vector3dVector(normal)
        self.spare_pc.colors = o3d.utility.Vector3dVector(color)

    def create_spare_pc(self):
        """ create spare point cloud for plane voxels only
        """
        n_max = (self.rng[0] + 1) * (self.rng[1] + 1) * (self.rng[2] + 1) * self.ransac_n_plane
        xyz = np.zeros((n_max, 3))
        normal = np.zeros((n_max, 3)).astype(np.single)
        color = np.zeros((n_max, 3)).astype(np.single)
        p = np.array([0, 0, 0, 1], dtype=float)
        n_voxel = 0
        for k in range(self.rng[2]+1):
            z = self.pc_mi[2] + k * self.voxel_size
            zbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(self.pc_mi[0], self.pc_mi[1], z),
                                                       max_bound=(self.pc_ma[0], self.pc_ma[1], z + self.voxel_size))
            zvoxels = self.pc.crop(zbox)
            for i in range(self.rng[0]+1):
                x = self.pc_mi[0] + i * self.voxel_size
                zxbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x, self.pc_mi[1], z),
                                                            max_bound=(x + self.voxel_size, self.pc_ma[1], z + self.voxel_size))
                zxvoxels = zvoxels.crop(zxbox)
                for j in range(self.rng[1]+1):
                    y = self.pc_mi[1] + j * self.voxel_size  # y
                    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(x, y, z),
                                                               max_bound=(x+self.voxel_size, y+self.voxel_size, z+self.voxel_size))
                    voxel = zxvoxels.crop(bbox)
                    voxel_xyz = np.asarray(voxel.points)
                    if voxel_xyz.shape[0] > self.ransac_limit:
                        res = self.voxel_ransac(voxel)
                        for plane, pp, cc in res:
                            # x, y, z projected to the plane
                            p[0:3] = pp
                            t = np.dot(p, plane)    # point - plane distance
                            xyz[n_voxel, 0] = p[0] - t * plane[0]
                            xyz[n_voxel, 1] = p[1] - t * plane[1]
                            xyz[n_voxel, 2] = p[2] - t * plane[2]
                            normal[n_voxel] = plane[0:3]
                            try:
                                color[n_voxel] = cc
                            except IndexError:
                                color[n_voxel] = np.array([1, 1, 1])
                            n_voxel += 1
        xyz = np.resize(xyz, (n_voxel, 3))
        normal = np.resize(normal, (n_voxel, 3))
        color = np.resize(normal, (n_voxel, 3))
        self.spare_pc = o3d.geometry.PointCloud()
        self.spare_pc.points = o3d.utility.Vector3dVector(xyz)
        self.spare_pc.normals = o3d.utility.Vector3dVector(normal)
        self.spare_pc.colors = o3d.utility.Vector3dVector(color)

    def create_spare_pc_old(self):
        """ create spare point cloud for plane voxels only
        """
        n_max = (self.rng[0] + 1) * (self.rng[1] + 1) * (self.rng[2] + 1) * 4
        xyz = np.zeros((n_max, 3))
        normal = np.zeros((n_max, 3)).astype(np.single)
        n_voxel = 0
        p = np.array([0, 0, 0, 1], dtype=float)  # homogenous coordinate for center of voxel
        for k in range(self.rng[2]+1):
            for i in range(self.rng[0]+1):
                for j in range(self.rng[1]+1):
                    # collect points in voxel
                    voxel = self.get_voxel(i, j, k)
                    voxel_xyz = np.asarray(voxel.points)
                    if voxel_xyz.shape[0] > self.ransac_limit:
                        # find best fitting RANSAC plane
                        res = self.voxel_ransac(voxel)
                        for plane, index in res:
                            # store
                            if plane is not None:
                                # x, y, z projected to the plane
                                p[0:3] = voxel_xyz[index]
                                t = np.dot(p, plane)    # point - plane distance
                                xyz[n_voxel, 0] = p[0] - t * plane[0]
                                xyz[n_voxel, 1] = p[1] - t * plane[1]
                                xyz[n_voxel, 2] = p[2] - t * plane[2]
                                normal[n_voxel, 0] = plane[0]
                                normal[n_voxel, 1] = plane[1]
                                normal[n_voxel, 2] = plane[2]
                                n_voxel += 1
        xyz = np.resize(xyz, (n_voxel, 3))
        normal = np.resize(normal, (n_voxel, 3))
        self.spare_pc = o3d.geometry.PointCloud()
        self.spare_pc.points = o3d.utility.Vector3dVector(xyz)
        self.spare_pc.normals = o3d.utility.Vector3dVector(normal)

    def spare_export(self, fname=None, typ='.ply'):
        """ output spare cloud

            :param fname: name of output file, default same as loaded
            :param typ: type of point cloud o3d supported extension .xyz, .pcd, .ply, pts, xyzn, xyzrgb
        """
        if fname is None:
            fname = os.path.join(self.out_dir, os.path.basename(os.path.splitext(self.file_name)[0]) + '_spare' + typ)
        else:
            fname = os.path.join(self.out_dir, os.path.basename(os.path.splitext(fname)[0]) + '_spare' + typ)
        if self.spare_pc is None:
            self.create_spare_pc()
        o3d.io.write_point_cloud(fname, self.spare_pc)

    def spare_import(self, fname=None):
        """ load saved spare point cloud

            :param fname: name of input file
        """
        if fname is None:
            fname = os.path.splitext(self.file_name)[0] + '_spare.ply'
        self.spare_pc = o3d.io.read_point_cloud(fname)

    def segmentation(self):
        """ create three point cloud roof, wall and other category
        """
        if self.spare_pc is None:
            raise ValueError('No space point cloud')
        roof = []
        wall = []
        other = []
        normals = np.asarray(self.spare_pc.normals)
        points = np.asarray(self.spare_pc.points)
        for i in range(normals.shape[0]):
            # angle from horizontal
            angle = abs(voxel_angle(normals[i]))
            if angle < self.angle_limits[0]:    # near vertical (wall)
                wall.append(i)
            elif angle < self.angle_limits[1] or \
                 points[i][2] < self.roof_z:    # other
                other.append(i)
            else:
                roof.append(i)                  # roof
        return roof, wall, other

    def segment_export(self, inliers, segment, typ='.ply', fname=None):
        """ export inliers points

            :param inliers: list of point indices to export
            :param segment: tag for segment
            :param fname: path to file
            :param typ: file type
        """
        if len(inliers) == 0:
            return
        if fname is None:
            fname = os.path.join(self.out_dir, os.path.basename(os.path.splitext(self.file_name)[0]) + segment + typ)
        else:
            fname = os.path.join(self.out_dir, os.path.basename(os.path.splitext(fname)[0]) + segment + typ)
        if self.spare_pc is None:
            raise ValueError("No spare point cloud")
        s = self.spare_pc.select_by_index(inliers)
        o3d.io.write_point_cloud(fname, s)

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
    parser.add_argument('-d', '--debug', action="store_true",
                        help='Save ascii point cloud for each plane and print plane data')
    parser.add_argument('-m', '--multi', action="store_true",
                        help='Use multi processing')
    parser.add_argument('-s', '--skip_spare', action="store_true",
                        help='Do not generate spare cloud, only normals for the original')
    parser.add_argument('-z', '--roof_z', type=float, default=0.0,
                        help='Minimal height for roof segment, use with nDSM only')
    args = parser.parse_args()
    FNAME = args.name[0]
    # if json config given other parameters are ignored
    if args.config is not None:
        JNAME = args.config
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
            SKIP_SPARE = JDATA["skip_spare"]
            ROOF_Z = JDATA["roof_z"]
            MULTI = JDATA["multi"]
            DEBUG = JDATA["debug"]
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
        SKIP_SPARE = args.skip_spare
        ROOF_Z = args.roof_z
        MULTI = args.multi
        DEBUG = args.debug

    PC = PointCloud(FNAME, voxel_size=VOXEL, ransac_threshold=THRES,
                    ransac_limit=LIM, ransac_n=N, rate=RATE,
                    angle_limits=ANG, ransac_n_plane=NP, roof_z=ROOF_Z,
                    out_dir=OUT_DIR, debug=DEBUG)
    if PC.pc_mi is None:
        print("Unable to load {}".format(FNAME))
        sys.exit()
    t1 = time.perf_counter()
    if SKIP_SPARE:
        # skip spare creation, use original but normals should be created
        PC.spare_pc = PC.pc
        PC.spare_pc.estimate_normals()
    else:
        if MULTI:
            PC.create_spare_pc_multi()
        else:
            PC.create_spare_pc()
    PC.spare_export()
    r, w, o = PC.segmentation()
    PC.segment_export(r, '_roof', '.pcd')
    PC.segment_export(w, '_wall', '.pcd')
    PC.segment_export(o, '_other', '.pcd')
    t2 = time.perf_counter()
    print(VOXEL, THRES, LIM, N,
          np.asarray(PC.spare_pc.points).shape[0], t2-t1)
