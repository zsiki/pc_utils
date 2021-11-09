#!/usr/bin/env python3

"""
    Search for buildings in a point cloud, ground and vegetation should be
    filtered out before.
"""
#   TODO list
#   colors in o3d are 0-1 range voxel indices are in 0-n range
#   fast selection for empty/non-empty voxels, creating set from voxel indices?
#   voxel.add((i, j, k))???
#   detect corners (two/three/four dominant plane in a voxel) READY
#   paralel processing of voxels and segmentation of huge clouds
#   segmented point cloud (original points on accepted planes) should be collected
#   use octree for voxel subset

import sys
import math
import os.path
import time
import json
import numpy as np
import open3d as o3d

class PointCloud():
    """
        :param file_name: open3d compatible input point cloud file
        :param voxel_size: size of voxels to fit plane
        :param ransac_limit: minimal number of point for ransac
        :param ransac_threshold: max distance from ransac plane
        :param ransac_n: number of random points for ransac plane
        :param ransac_iteration: number of iterations for ransac
        :param angle_limits: threshold to separate wall, roof and other planes
        :param rate: list of percents of points fit on ransac plane
        :param ransac_n_plane: maximal number of planes to search in a voxel
    """
    NODATA = -9999.0

    def __init__(self, file_name, voxel_size=0.5, ransac_limit=100,
                 ransac_threshold=0.025, ransac_n=10, ransac_iterations=100,
                 angle_limits=[0.087, 0.698], rate=[0.2, 0.45, 0.65, 0.8],
                 ransac_n_plane=4):
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
        for i in range(self.ransac_n_plane):
            n = np.asarray(voxel.points).shape[0]
            if n > self.ransac_limit:
                # fit ransac plane
                plane_model, inliers = voxel.segment_plane(self.ransac_threshold,
                                                           self.ransac_n,
                                                           self.ransac_iterations)
                m = len(inliers)    # number of inliers
                sys.stderr.write("{:4d} {:4d} {:6d} {:6d}\n".format(self.counter, i, n, m))
                sys.stderr.write("{:.6f} {:.6f} {:.6} {:.6}\n".format(plane_model[0], plane_model[1], plane_model[2], plane_model[3]))
                if m / n > self.rate[i]:
                    tmp = voxel.select_by_index(inliers)
                    np.savetxt('temp{}.txt'.format(self.counter), np.asarray(tmp.points))

                    self.counter += 1
                    res.append([plane_model, m // 2])
                    # reduce pc to outliers
                    voxel = voxel.select_by_index(inliers, invert=True)
                else:
                    return res
            else:
                return res
        return res

    @staticmethod
    def voxel_angle(plane):
        """ calculate angle of normal to the vertical direction

            :param plane: vector of plane equation coefficients
            :return: angle of normal direction from vertical in radians in 0-pi/2 range
        """
        return math.atan2(abs(plane[2]), math.hypot(plane[0], plane[1]))

    def create_spare_pc(self):
        """ create spare point cloud for plane voxels only
        """
        n_max = (self.rng[0] + 1) * (self.rng[1] + 1) * (self.rng[2] + 1)
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
                        for plane, index in res:
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
        n_max = (self.rng[0] + 1) * (self.rng[1] + 1) * (self.rng[2] + 1)
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
            fname = os.path.splitext(self.file_name)[0] + '_spare' + typ
        else:
            fname = os.path.splitext(fname)[0] + '_spare' + typ
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
        for i in range(normals.shape[0]):
            # angle from horizontal
            angle = abs(self.voxel_angle(normals[i]))
            if angle < self.angle_limits[0]:    # 0-5 degree
                wall.append(i)
            elif angle < self.angle_limits[1]:  # 5-40 degree
                other.append(i)
            else:
                roof.append(i)                  # 40-90 degree
        return roof, wall, other

    def segment_export(self, inliers, segment, fname=None, typ='.ply'):
        """ export inliers points

            :param inliers: list of point indices to export
            :param segment: tag for segment
            :param fname: path to file
            :param typ: file type
        """
        if fname is None:
            fname = os.path.splitext(self.file_name)[0] + segment + typ
        else:
            fname = os.path.splitext(fname)[0] + segment + typ
        if self.spare_pc is None:
            raise ValueError("No spare point cloud")
        s = self.spare_pc.select_by_index(inliers)
        o3d.io.write_point_cloud(fname, s)

if __name__ == "__main__":

    if len(sys.argv) > 1:
        FNAME = sys.argv[1]
    else:
        FNAME = 'barnag_dtm_test_ndsm.ply'
    if len(sys.argv) > 2:
        JNAME = sys.argv[2]
        with open(JNAME) as jfile:
            JDATA = json.load(jfile)
            VOXEL = JDATA["voxel_size"]
            THRES = JDATA["threshold"]
            LIM = JDATA["limit"]
            N = JDATA["n"]
            ITERATION = JDATA["iteration"]
            RATE = JDATA["rate"]
            NP = JDATA["n_plane"]
    else:
        VOXEL = 1.0
        THRES = 0.1
        LIM = 25
        N = 5
        ITERATION = 20
        RATE = [0.20, 0.45, 0.65, 0.8]
        NP = 4

    print(FNAME)
    print('voxel_size threshold limit n n_voxel time')
    PC = PointCloud(FNAME, voxel_size=VOXEL, ransac_threshold=THRES,
                    ransac_limit=LIM, ransac_n=N, rate=RATE,
                    ransac_n_plane=NP)
    if PC.pc_mi is None:
        print("Unable to load {}".format(FNAME))
        sys.exit()
    t1 = time.perf_counter()
    PC.create_spare_pc()
    PC.spare_export()
    r, w, o = PC.segmentation()
    PC.segment_export(r, '_roof')
    PC.segment_export(w, '_wall')
    PC.segment_export(o, '_other')
    #PC.spare_import()
    #r, ri = PC.roof_segmentation()
    t2 = time.perf_counter()
    print(VOXEL, THRES, LIM, N,
          np.asarray(PC.spare_pc.points).shape[0], t2-t1)
