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
#import pyransac3d as pyrsc      # just for checking open3D

class PointCloud():
    """
        :param file_name: open3d compatible input point cloud file
        :param voxel_size: size of voxels to fit plane
        :param ransac_limit: minimal number of point for ransac
        :param ransac_threshold: max distance from ransac plane
        :param ransac_n: number of random points for ransac plane
        :param ransac_iteration: number of iterations for ransac
        :param roof_angle_limit: threshold to separate wall and roof planes
        :param rate: percent of points fit on ransac plane to set voxel as plane
        :param ransac_n_plane: maximal number of planes to search in a voxel
    """
    NODATA = -9999.0

    def __init__(self, file_name, voxel_size=0.5, ransac_limit=100,
                 ransac_threshold=0.025, ransac_n=10, ransac_iterations=100,
                 roof_angle_limit=0.873, rate=0.25, ransac_n_plane=4):
        """ Initialize instance
        """
        self.file_name = file_name
        self.voxel_size = voxel_size
        self.ransac_limit = ransac_limit
        self.ransac_threshold = ransac_threshold
        self.ransac_n = ransac_n
        self.ransac_iterations = ransac_iterations
        self.roof_angle_limit = roof_angle_limit
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
        """ get voxel at index i, j, k

            :returns: point cloud with points in voxel
        """
        x = self.pc_mi[0] + i * self.voxel_size
        y = self.pc_mi[1] + j * self.voxel_size
        x = self.pc_mi[2] + k * self.voxel_size
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
                tmp = voxel.select_by_index(inliers)
                np.savetxt('temp.txt', np.asarray(tmp.points))
                if m / n > self.rate:
                    res.append([plane_model, m // 2])
                    voxel = voxel.select_by_index(inliers, invert=True)   # reduce pc to outliers
                else:
                    return res
            else:
                return res
        return res

    @staticmethod
    def voxel_angle(plane):
        """ classify voxel by normal direction

            :return: angle of normal direction from vertical in radians in 0-pi/2 range
        """
        h = math.hypot(plane[0], plane[1])
        return math.atan2(abs(plane[2]), h)

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
                zxvoxels = self.pc.crop(zxbox)
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
                            color[n_voxel] = np.asarray(voxel.colors)[index]
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

    def voxel_export(self, fname=None):
        """ output grass 3D raster
        """
        # TODO voxel_down_sample() can do similar thing
        if fname is None:
            fname = os.path.splitext(self.file_name)[0] + '.ascii'
        v = self.spare_to_voxel()
        with open(fname, 'w') as f:
            f.write('version: grass7\n')
            f.write('order: snbt\n')
            f.write('north: {:.3f}\n'.format(PC.pc_mi[1] + self.rng[1] * PC.voxel_size))
            f.write('south: {:.3f}\n'.format(PC.pc_mi[1]))
            f.write('east: {:.3f}\n'.format(PC.pc_mi[0] + self.rng[0] * PC.voxel_size))
            f.write('west: {:.3f}\n'.format(PC.pc_mi[0]))
            f.write('top: {:.3f}\n'.format(PC.pc_mi[2] + self.rng[2] * PC.voxel_size))
            f.write('bottom: {:.3f}\n'.format(PC.pc_mi[2]))
            f.write('rows: {}\n'.format(self.rng[1]))
            f.write('cols: {}\n'.format(self.rng[0]))
            f.write('levels: {}\n'.format(self.rng[2]))
            for k in range(self.rng[2]+1):
                for i in range(self.rng[0]+1):
                    for j in range(self.rng[1]+1):
                        f.write('{:.3f} '.format(v[i, j, k]))
                    f.write('\n')

    def spare_export(self, fname=None, typ='.ply'):
        """ output spare cloud

            :param fname: name of output file, default same as loaded
            :param typ: type of point cloud o3d supported extension .asc, .pcd, .ply
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

    def get_neighbour_index(self, i, j, k):
        """ get list of sheet neighbour voxel indices (max 6)

            :param i: row index of actual voxel
            :param j: column index of actual voxel
            :param k: storey index of actual voxel
            :return: row indices of available
        """
        dif = np.abs(col - np.array([i, j, k]))
        sdif = np.sum(dif, axis=1)
        inds = np.where(sdif == 1)
        return inds

    def voxel_to_index(self, i, j, k):
        """ change voxel index to point cloud index

            :param i: row index of actual voxel
            :param j: column index of actual voxel
            :param k: storey index of actual voxel
            :return: row indices of voxel or None if not exists
        """
        col = np.asarray(self.spare_pc.colors)  # TODO does not work
        try:
            ind = np.where(np.all(col == (i, j, k), axis=1))[0][0]
        except Exception:
            ind = None
        return ind

    def spare_index_to_voxel(self, ind):
        """ calculate voxel indices (i, j, k) from point index

            :param ind: index of point in point cloud
            :return: tuple of voxel indices or None
        """
        # TODO does not work
        try:
            col = np.asarray(self.spare_pc.colors)[ind]
        except Exception:
            return None
        return (int(col[0]), int(col[1]), int(col[2]))

    def spare_to_voxel(self):
        """ convert spare point cloud to voxels with normal angle
        """
        if self.spare_pc is None:
            return None
        col = np.asarray(self.spare_pc.colors)
        norm = np.asarray(self.spare_pc.normals)
        v = np.full((self.rng[0], self.rng[1], self.rng[2]), self.NODATA)
        for i in range(col.shape[0]):
            v[int(col[i, 0]), int(col[i, 1]), int(col[i, 2])] = self.voxel_angle(norm[i])
        return v

    def roof_segmentation(self):
        """ find roof voxels in spare point cloud and save it into 2d
            array """
        if self.spare_pc is None:
            return
        v = self.spare_to_voxel()
        roofs = np.empty((self.rng[0], self.rng[1]))
        roof_indices = np.empty((self.rng[0], self.rng[1]), dtype=int)
        for i in range(self.rng[0]+1):
            for j in range(self.rng[1]+1):
                # find first non empty voxel
                k = self.rng[2]
                while k >= 0:
                    if v[j, j, k] is not None:
                        # can be roof
                        if v[i, j, k] < self.roof_angle_limit:
                            roofs[i, j] = v[i, j, k]
                            roof_indices[i, j] = k
                        else:
                            roofs[i, j] = None
                            roof_indices[i, j] = None
                        break
                    k -= 1
        # find continouos roof areas
        #TODO
        return roofs, roof_indices

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
            RATE = JDATA["rate"]
            NP = JDATA["n_plane"]
    else:
        VOXEL = 1.0
        THRES = 0.1
        LIM = 25
        N = 5
        RATE = 0.25
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
    #PC.spare_import()
    #r, ri = PC.roof_segmentation()
    t2 = time.perf_counter()
    print(VOXEL, THRES, LIM, N,
          np.asarray(PC.spare_pc.points).shape[0], t2-t1)
