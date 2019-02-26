"""This file defines a class to interact with KITTI dataset. """

import os
import time
from os.path import isfile, join
import sys

import numpy as np
import open3d
import cv2
from collections import namedtuple
import yaml

Points = namedtuple('Points', ['xyz', 'attr'])

class KittiDataset(object):
    """A class to interact with KITTI dataset."""

    def __init__(self, image_dir, point_dir, calib_dir, label_dir,
        index_filename=None, is_training=True, is_raw=False):
        """
        Args:
            image_dir: a string of the path to image folder.
            point_dir: a string of the path to point cloud data folder.
            calib_dir: a string of the path to the calibration matrices.
            label_dir: a string of the path to the label folder.
            index_filename: a string containing a path an index file.
        """

        self._image_dir = image_dir
        self._point_dir = point_dir
        self._calib_dir = calib_dir
        self._label_dir = label_dir
        self._index_filename = index_filename
        if index_filename:
            self._file_list = self._read_index_file(index_filename)
        else:
            self._file_list = self._get_file_list(self._image_dir)
        # self._verify_file_list(
        #     self._image_dir, self._point_dir, self._label_dir, self._calib_dir,
        #     self._file_list, is_training, is_raw)
        self._is_training = is_training
        self._is_raw = is_raw

    def __str__(self):
        """Generate a string summary of the dataset"""

        summary_string = ('Dataset Summary:\n'
            +'image_dir=%s\n' % self._image_dir
            +'point_dir=%s\n' % self._point_dir
            +'calib_dir=%s\n' % self._calib_dir
            +'label_dir=%s\n' % self._label_dir
            +'index_filename=%s\n' % self._index_filename
            +'Total number of sampels: %d' % self.num_files)
        return summary_string

    @property
    def num_files(self):
        return len(self._file_list)

    def _read_index_file(self, index_filename):
        """Read an index file containing the filenames.

        Args:
            index_filename: a string containing the path to an index file.

        Returns: a list of filenames.
        """

        file_list = []
        with open(index_filename, 'r') as f:
            for line in f:
                file_list.append(line.rstrip('\n').split('.')[0])
        return file_list

    def _get_file_list(self, image_dir):
        """Load all filenames from image_dir.

        Args:
            image_dir: a string of path to the image folder.

        Returns: a list of filenames.
        """

        file_list = [f.split('.')[0]
            for f in os.listdir(image_dir) if isfile(join(image_dir, f))]
        file_list.sort()
        return file_list
    def _verify_file_list(
        self, image_dir, point_dir, label_dir, calib_dir, file_list,
        is_training, is_raw):
        """Varify the files in file_list exist.

        Args:
            image_dir: a string of the path to image folder.
            point_dir: a string of the path to point cloud data folder.
            label_dir: a string of the path to the label folder.
            calib_dir: a string of the path to the calibration folder.
            file_list: a list of filenames.
            is_training: if False, label_dir is not verified.

        Raise: assertion error when file in file_list is not complete.
        """

        for f in file_list:
            image_file = join(image_dir, f)+'.png'
            point_file = join(point_dir, f)+'.bin'
            label_file = join(label_dir, f)+'.txt'
            calib_file = join(calib_dir, f)+'.txt'
            assert isfile(image_file), "Image %s does not exist" % image_file
            assert isfile(point_file), "Point %s does not exist" % point_file
            if not is_raw:
                assert isfile(calib_file), "Calib %s does not exist" % calib_file
            if is_training:
                assert isfile(label_file), "Label %s does not exist" % label_file

    def downsample_by_average_voxel(self, points, voxel_size):
        """Voxel downsampling using average function.

        points: a Points namedtuple containing "xyz" and "attr".
        voxel_size: the size of voxel cells used for downsampling.
        """
        # create voxel grid
        xmax, ymax, zmax = np.amax(points.xyz, axis=0)
        xmin, ymin, zmin = np.amin(points.xyz, axis=0)
        dim_x = int((xmax - xmin) / voxel_size + 1)
        dim_y = int((ymax - ymin) / voxel_size + 1)
        dim_z = int((zmax - zmin) / voxel_size + 1)
        xyz_offset = np.asarray([[xmin, ymin, zmin]])
        xyz_zeros = np.asarray([0, 0, 0], dtype=np.float32)
        xyz_idx = (points.xyz - xyz_offset) // voxel_size
        xyz_idx = xyz_idx.astype(np.int32)
        keys = xyz_idx[:, 0] + xyz_idx[:, 1]*dim_x + xyz_idx[:, 2]*dim_y*dim_x
        num_points = xyz_idx.shape[0]
        voxels = {}
        voxels_ctr = {}
        include_attr = points.attr is not None
        if include_attr:
            voxels_attr = {}
            attr_zeros = np.zeros(points.attr.shape[1], dtype=np.float32)
        for pidx in range(num_points):
            key = keys[pidx]
            voxel_xyz = voxels.get(key, xyz_zeros)
            voxel_ctr = voxels_ctr.get(key, 0)
            voxels[key] = voxel_xyz + points.xyz[pidx, :]
            voxels_ctr[key] = voxel_ctr + 1
            if include_attr:
                voxel_attr = voxels_attr.get(key, attr_zeros)
                voxels_attr[key] = voxel_attr + points.attr[pidx, :]
        downsampled_xyz = [voxels[key]/voxels_ctr[key] for key in voxels_ctr]
        if include_attr:
            downsampled_attr = [
                voxels_attr[key]/voxels_ctr[key] for key in voxels_ctr]
            return Points(xyz=np.vstack(downsampled_xyz),
                    attr=np.vstack(downsampled_attr))
        else:
            return Points(xyz=np.vstack(downsampled_xyz),
                    attr=None)

    def downsample_by_voxel(self, points, voxel_size, method='AVERAGE'):
        """Downsample point cloud by voxel.

        points: a Points namedtuple containing "xyz" and "attr".
        voxel_size: the size of voxel cells used for downsampling.
        method: 'AVERAGE', all points inside a voxel cell are averaged
        including xyz and attr.
        """
        # create voxel grid
        xmax, ymax, zmax = np.amax(points.xyz, axis=0)
        xmin, ymin, zmin = np.amin(points.xyz, axis=0)
        dim_x = int((xmax - xmin) / voxel_size + 1)
        dim_y = int((ymax - ymin) / voxel_size + 1)
        dim_z = int((zmax - zmin) / voxel_size + 1)
        # use List consumes too much memory
        # voxel_account = []
        # for i in range(int((xmax - xmin) / voxel_size + 1)):
        #     voxel_jk = []
        #     for j in range(int((ymax - ymin) / voxel_size + 1)):
        #         voxel_k = []
        #         for k in range(int((zmax - zmin) / voxel_size + 1)):
        #             voxel_k.append([])
        #         voxel_jk.append(voxel_k)
        #     voxel_account.append(voxel_jk)
        # map points into the grid
        voxel_account = {}
        xyz_idx = np.int32(
            (points.xyz - np.asarray([[xmin, ymin, zmin]])) / voxel_size)
        for pidx in range(xyz_idx.shape[0]):
            x_idx = xyz_idx[pidx, 0]
            y_idx = xyz_idx[pidx, 1]
            z_idx = xyz_idx[pidx, 2]
            # TODO check bug impact
            key = x_idx + y_idx*dim_x + z_idx*dim_y*dim_x
            if key in voxel_account:
                voxel_account[key].append(pidx)
            else:
                voxel_account[key] = [pidx]
        # compute voxel points
        downsampled_xyz_list = []
        if points.attr is not None:
            downsampled_attr_list = []
        if method == 'AVERAGE':
            for idx, pidx_list in voxel_account.iteritems():
                if len(pidx_list) > 0:
                    downsampled_xyz_list.append(
                        np.mean(points.xyz[pidx_list, :],
                            axis=0, keepdims=True))
                    if points.attr is not None:
                        downsampled_attr_list.append(
                            np.mean(points.attr[pidx_list, :],
                                axis=0, keepdims=True))
        if points.attr is not None:
            return Points(xyz=np.vstack(downsampled_xyz_list),
                attr=np.vstack(downsampled_attr_list))
        else:
            return Points(xyz=np.vstack(downsampled_xyz_list),
                attr=None)

    def get_calib(self, frame_idx):
        """Load calibration matrices and compute calibrations.

        Args:
            frame_idx: the index of the frame to read.

        Returns: a dictionary of calibrations.
        """

        calib_file = join(self._calib_dir, self._file_list[frame_idx])+'.txt'
        with open(calib_file, 'r') as f:
            calib = {}
            for line in f:
                fields = line.split(' ')
                matrix_name = fields[0].rstrip(':')
                matrix = np.array(fields[1:], dtype=np.float32)
                calib[matrix_name] = matrix
        calib['P2'] = calib['P2'].reshape(3, 4)
        calib['R0_rect'] = calib['R0_rect'].reshape(3,3)
        calib['Tr_velo_to_cam'] = calib['Tr_velo_to_cam'].reshape(3,4)
        R0_rect = np.eye(4)
        R0_rect[:3, :3] = calib['R0_rect']
        calib['velo_to_rect'] = np.vstack([calib['Tr_velo_to_cam'],[0,0,0,1]])
        calib['cam_to_image'] = np.hstack([calib['P2'][:, 0:3], [[0],[0],[0]]])
        calib['rect_to_cam'] = np.hstack([
            calib['R0_rect'],
            np.matmul(
                np.linalg.inv(calib['P2'][:, 0:3]), calib['P2'][:, [3]])])
        calib['rect_to_cam'] = np.vstack([calib['rect_to_cam'],
            [0,0,0,1]])
        calib['velo_to_cam'] = np.matmul(calib['rect_to_cam'],
            calib['velo_to_rect'])
        calib['cam_to_velo'] = np.linalg.inv(calib['velo_to_cam'])

        # senity check
        calib['velo_to_image'] = np.matmul(calib['cam_to_image'],
            calib['velo_to_cam'])
        assert np.isclose(calib['velo_to_image'],
            np.matmul(np.matmul(calib['P2'], R0_rect),
            calib['velo_to_rect'])).all()

        return calib
        
    def get_raw_calib(self, calib_velo_to_cam_path, calib_cam_to_cam_path):
        """Read calibrations in kitti raw dataset."""
        with open(calib_cam_to_cam_path, 'r') as f:
            calib = {}
            for line in f:
                line = line.rstrip('\n')
                fields = line.split(':')
                calib[fields[0]] = fields[1]
        calib['corner_dist'] = np.array(
            calib['corner_dist'], dtype=np.float32)
        for i in range(4):
            calib['S_0%d'%i] = np.array(
                calib['S_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(1,2)
            calib['K_0%d'%i] = np.array(
                calib['K_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(3,3)
            calib['D_0%d'%i] = np.array(
                calib['D_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(1,5)
            calib['R_0%d'%i] = np.array(
                calib['R_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(3,3)
            calib['T_0%d'%i] = np.array(
                calib['T_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(3,1)
            calib['S_rect_0%d'%i] = np.array(
                calib['S_rect_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(1,2)
            calib['R_rect_0%d'%i] = np.array(
                calib['R_rect_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(3,3)
            calib['P_rect_0%d'%i] = np.array(
                calib['P_rect_0%d'%i].split(' ')[1:], dtype=np.float32).reshape(3,4)

        with open(calib_velo_to_cam_path, 'r') as f:
            for line in f:
                line = line.rstrip('\n')
                fields = line.split(':')
                calib[fields[0]] = fields[1]
        calib['R'] = np.array(
            calib['R'].split(' ')[1:], dtype=np.float32).reshape(3,3)
        calib['T'] = np.array(
            calib['T'].split(' ')[1:], dtype=np.float32).reshape(3,1)
        calib['Tr_velo_to_cam'] = np.vstack(
            [np.hstack([calib['R'], calib['T']]),[0,0,0,1]])

        R0_rect = np.eye(4)
        R0_rect[:3, :3] = calib['R_rect_00']
        T2 = np.eye(4)
        T2[0, 3] = calib['P_rect_02'][0, 3]/calib['P_rect_02'][0, 0]
        calib['velo_to_cam'] = T2.dot(R0_rect.dot(calib['Tr_velo_to_cam']))
        calib['cam_to_image'] = np.hstack([calib['P_rect_02'][:, 0:3], [[0],[0],[0]]])
        calib['velo_to_image'] = np.matmul(calib['cam_to_image'],
            calib['velo_to_cam'])

        return calib

    def get_autoware_calib(self, calib_path):
        """Read calibrations in kitti raw dataset."""
        calib = {}
        with open(calib_path, 'r') as f:
            calib = yaml.load(f)
        calib['cam_to_velo'] =  np.reshape(calib['CameraExtrinsicMat']['data'],
            (calib['CameraExtrinsicMat']['rows'], calib['CameraExtrinsicMat']['cols']))
        calib['velo_to_cam'] = np.linalg.inv(calib['cam_to_velo'])
        calib['CameraMat'] = np.reshape(calib['CameraMat']['data'],
            (calib['CameraMat']['rows'], calib['CameraMat']['cols']))
        calib['cam_to_image'] = np.hstack([calib['CameraMat'], [[0],[0],[0]]])
        calib['velo_to_image'] = np.matmul(calib['cam_to_image'],
            calib['velo_to_cam'])
        calib['DistCoeff'] =  np.reshape(calib['DistCoeff']['data'],
            (calib['DistCoeff']['rows'], calib['DistCoeff']['cols']))

        return calib
    def get_filename(self, frame_idx):
        """Get the filename based on frame_idx.

        Args:
            frame_idx: the index of the frame to get.

        Returns: a string containing the filename.
        """

        return self._file_list[frame_idx]

    def get_velo_points(self, frame_idx):
        """Load velo points from frame_idx.

        Args:
            frame_idx: the index of the frame to read.

        Returns: Points.
        """

        point_file = join(self._point_dir, self._file_list[frame_idx])+'.bin'
        velo_data = np.fromfile(point_file, dtype=np.float32).reshape(-1, 4)
        velo_points = velo_data[:,:3]
        reflections = velo_data[:,[3]]
        return Points(xyz = velo_points, attr = reflections)

    def get_cam_points(self, frame_idx, downsample_voxel_size=None, calib=None):
        """Load velo points and convert them to camera coordinates.

        Args:
            frame_idx: the index of the frame to read.

        Returns: Points.
        """

        velo_points = self.get_velo_points(frame_idx)
        if calib is None:
            calib = self.get_calib(frame_idx)
        cam_points = self.velo_points_to_cam(velo_points, calib)
        if downsample_voxel_size is not None:
            # start_time = time.time()
            # cam_points_old = self.downsample_by_voxel(cam_points,
            #     downsample_voxel_size)
            # print(time.time() - start_time)
            # start_time = time.time()
            cam_points = self.downsample_by_average_voxel(cam_points,
                downsample_voxel_size)
            # print(time.time() - start_time)
            # assert np.isclose(cam_points_old.xyz, cam_points.xyz).all(), 'must be the same'
            # assert np.isclose(cam_points_old.attr, cam_points.attr).all(), 'must be the same'
        return cam_points

    def get_cam_points_in_image(self, frame_idx, downsample_voxel_size=None,
        calib=None):
        """Load velo points and remove points that are not observed by camera.
        """
        if calib is None:
            calib = self.get_calib(frame_idx)
        cam_points = self.get_cam_points(frame_idx, downsample_voxel_size, calib=calib)
        image = self.get_image(frame_idx, calib=calib)
        height = image.shape[0]
        width = image.shape[1]
        front_cam_points_idx = cam_points.xyz[:,2] > 0.1
        front_cam_points = Points(cam_points.xyz[front_cam_points_idx, :],
            cam_points.attr[front_cam_points_idx, :])
        img_points = self.cam_points_to_image(front_cam_points, calib)
        img_points_in_image_idx = np.logical_and.reduce(
            [img_points.xyz[:,0]>0, img_points.xyz[:,0]<width,
             img_points.xyz[:,1]>0, img_points.xyz[:,1]<height])
        cam_points_in_img = Points(
            xyz = front_cam_points.xyz[img_points_in_image_idx,:],
            attr = front_cam_points.attr[img_points_in_image_idx,:])
        return cam_points_in_img

    def get_cam_points_in_image_with_rgb(self, frame_idx,
        downsample_voxel_size=None, calib=None):
        """Get camera points that are visible in image and append image color
        to the points as attributes."""
        if calib is None:
            calib = self.get_calib(frame_idx)
        cam_points = self.get_cam_points(frame_idx, downsample_voxel_size,
            calib = calib)
        image = self.get_image(frame_idx, calib=calib)
        height = image.shape[0]
        width = image.shape[1]
        front_cam_points_idx = cam_points.xyz[:,2] > 0.1
        front_cam_points = Points(cam_points.xyz[front_cam_points_idx, :],
            cam_points.attr[front_cam_points_idx, :])
        img_points = self.cam_points_to_image(front_cam_points, calib)
        img_points_in_image_idx = np.logical_and.reduce(
            [img_points.xyz[:,0]>0, img_points.xyz[:,0]<width,
             img_points.xyz[:,1]>0, img_points.xyz[:,1]<height])
        cam_points_in_img = Points(
            xyz = front_cam_points.xyz[img_points_in_image_idx,:],
            attr = front_cam_points.attr[img_points_in_image_idx,:])
        cam_points_in_img_with_rgb = self.rgb_to_cam_points(cam_points_in_img,
            image, calib)
        return cam_points_in_img_with_rgb

    def get_image(self, frame_idx, calib=None):
        """Load the image from frame_idx.

        Args:
            frame_idx: the index of the frame to read.

        Returns: cv2.matrix
        """
        image_file = join(self._image_dir, self._file_list[frame_idx])+'.png'
        image = cv2.imread(image_file)
        if calib is not None and 'DistCoeff' in calib:
            image = cv2.undistort(image, calib['CameraMat'], calib['DistCoeff'])
        return image

    def get_label(self, frame_idx):
        """Load bbox labels from frame_idx frame.

        Args:
            frame_idx: the index of the frame to read.

        Returns: a list of object label dictionaries.
        """

        label_file = join(self._label_dir, self._file_list[frame_idx])+'.txt'
        label_list = []
        with open(label_file, 'r') as f:
            for line in f:
                label={}
                fields = line.rstrip('\n').split(' ')
                label['name'] = fields[0]
                # 0=visible 1=partly occluded, 2=fully occluded, 3=unknown
                label['truncation'] = float(fields[1])
                label['occlusion'] = int(fields[2])
                label['alpha'] =  float(fields[3])
                label['xmin'] =  float(fields[4])
                label['ymin'] =  float(fields[5])
                label['xmax'] =  float(fields[6])
                label['ymax'] =  float(fields[7])
                label['height'] =  float(fields[8])
                label['width'] =  float(fields[9])
                label['length'] =  float(fields[10])
                label['x3d'] =  float(fields[11])
                label['y3d'] =  float(fields[12])
                label['z3d'] =  float(fields[13])
                label['yaw'] =  float(fields[14])
                label_list.append(label)
        return label_list

    def box3d_to_cam_points(self, label, expend_factor=(1.0, 1.0, 1.0)):
        """Project 3D box into camera coordinates.
        Args:
            label: a dictionary containing "x3d", "y3d", "z3d", "yaw", "height"
                "width", "length".

        Returns: a numpy array [8, 3] representing the corners of the 3d box in
            camera coordinates.
        """

        yaw = label['yaw']
        R = np.array([[np.cos(yaw),  0,  np.sin(yaw)],
                      [0,            1,  0          ],
                      [-np.sin(yaw), 0,  np.cos(yaw)]]);
        h = label['height']
        delta_h = h*(expend_factor[0]-1)
        w = label['width']*expend_factor[1]
        l = label['length']*expend_factor[2]
        corners = np.array([[ l/2,  delta_h/2,  w/2],  # front up right
                            [ l/2,  delta_h/2, -w/2],  # front up left
                            [-l/2,  delta_h/2, -w/2],  # back up left
                            [-l/2,  delta_h/2,  w/2],  # back up right
                            [ l/2, -h-delta_h/2,  w/2],  # front down right
                            [ l/2, -h-delta_h/2, -w/2],  # front down left
                            [-l/2, -h-delta_h/2, -w/2],  # back down left
                            [-l/2, -h-delta_h/2,  w/2]]) # back down right
        r_corners = corners.dot(np.transpose(R))
        tx = label['x3d']
        ty = label['y3d']
        tz = label['z3d']
        cam_points_xyz = r_corners+np.array([tx, ty, tz])
        return Points(xyz = cam_points_xyz, attr = None)

    def draw_open3D_box(self, label, expend_factor=(1.0, 1.0, 1.0)):
        """Draw a 3d box using open3d.

        Args:
            label: a dictionary containing "x3d", "y3d", "z3d", "yaw",
            "height", "width", "lenth".

        returns: a open3d mesh object.
        """
        yaw = label['yaw']
        R = np.array([[np.cos(yaw),  0,  np.sin(yaw)],
                      [0,            1,  0          ],
                      [-np.sin(yaw), 0,  np.cos(yaw)]]);
        Rh = np.array([ [1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])

        Rl = np.array([ [0, 0, 1],
                        [0, 1, 0],
                        [1, 0, 0]])

        h = label['height']
        delta_h = h*(expend_factor[0]-1)
        w = label['width']*expend_factor[1]
        l = label['length']*expend_factor[2]
        tx = label['x3d']
        ty = label['y3d']
        tz = label['z3d']

        box_offset = np.array([ [ l/2,  -h/2-delta_h/2,  w/2],
                                [ l/2,  -h/2-delta_h/2, -w/2],
                                [-l/2,  -h/2-delta_h/2, -w/2],
                                [-l/2,  -h/2-delta_h/2,  w/2],

                                [ l/2, delta_h/2, 0],
                                [ -l/2, delta_h/2, 0],
                                [l/2, -h-delta_h/2, 0],
                                [-l/2, -h-delta_h/2, 0],

                                [0, delta_h/2, w/2],
                                [0, delta_h/2, -w/2],
                                [0, -h-delta_h/2, w/2],
                                [0, -h-delta_h/2, -w/2]])

        transform = np.matmul(R, np.transpose(box_offset))
        transform = transform + np.array([[tx], [ty], [tz]])
        transform = np.vstack((transform, np.ones((1, 12))))
        hrotation = np.vstack((R.dot(Rh), np.zeros((1,3))))
        lrotation = np.vstack((R.dot(Rl), np.zeros((1,3))))
        wrotation = np.vstack((R, np.zeros((1,3))))

        h1_cylinder = open3d.create_mesh_cylinder(radius = h/100, height = h)
        h1_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
        h1_cylinder.transform(np.hstack((hrotation, transform[:, [0]])))

        h2_cylinder = open3d.create_mesh_cylinder(radius = h/100, height = h)
        h2_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
        h2_cylinder.transform(np.hstack((hrotation, transform[:, [1]])))

        h3_cylinder = open3d.create_mesh_cylinder(radius = h/100, height = h)
        h3_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
        h3_cylinder.transform(np.hstack((hrotation, transform[:, [2]])))

        h4_cylinder = open3d.create_mesh_cylinder(radius = h/100, height = h)
        h4_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
        h4_cylinder.transform(np.hstack((hrotation, transform[:, [3]])))

        w1_cylinder = open3d.create_mesh_cylinder(radius = w/100, height = w)
        w1_cylinder.paint_uniform_color([0.9, 0.1, 0.1])
        w1_cylinder.transform(np.hstack((wrotation, transform[:, [4]])))

        w2_cylinder = open3d.create_mesh_cylinder(radius = w/100, height = w)
        w2_cylinder.paint_uniform_color([0.9, 0.1, 0.1])
        w2_cylinder.transform(np.hstack((wrotation, transform[:, [5]])))

        w3_cylinder = open3d.create_mesh_cylinder(radius = w/100, height = w)
        w3_cylinder.paint_uniform_color([0.9, 0.1, 0.1])
        w3_cylinder.transform(np.hstack((wrotation, transform[:, [6]])))

        w4_cylinder = open3d.create_mesh_cylinder(radius = w/100, height = w)
        w4_cylinder.paint_uniform_color([0.9, 0.1, 0.1])
        w4_cylinder.transform(np.hstack((wrotation, transform[:, [7]])))

        l1_cylinder = open3d.create_mesh_cylinder(radius = l/100, height = l)
        l1_cylinder.paint_uniform_color([0.1, 0.1, 0.9])
        l1_cylinder.transform(np.hstack((lrotation, transform[:, [8]])))

        l2_cylinder = open3d.create_mesh_cylinder(radius = l/100, height = l)
        l2_cylinder.paint_uniform_color([0.1, 0.1, 0.9])
        l2_cylinder.transform(np.hstack((lrotation, transform[:, [9]])))

        l3_cylinder = open3d.create_mesh_cylinder(radius = l/100, height = l)
        l3_cylinder.paint_uniform_color([0.1, 0.1, 0.9])
        l3_cylinder.transform(np.hstack((lrotation, transform[:, [10]])))

        l4_cylinder = open3d.create_mesh_cylinder(radius = l/100, height = l)
        l4_cylinder.paint_uniform_color([0.1, 0.1, 0.9])
        l4_cylinder.transform(np.hstack((lrotation, transform[:, [11]])))

        return [h1_cylinder, h2_cylinder, h3_cylinder, h4_cylinder,
                w1_cylinder, w2_cylinder, w3_cylinder, w4_cylinder,
                l1_cylinder, l2_cylinder, l3_cylinder, l4_cylinder]

    def box3d_to_normals(self, label, expend_factor=(1.0, 1.0, 1.0)):
        """Project a 3D box into camera coordinates, compute the center
        of the box and normals.

        Args:
            label: a dictionary containing "x3d", "y3d", "z3d", "yaw",
            "height", "width", "lenth".

        Returns: a numpy array [3, 3] containing [wx, wy, wz]^T, a [3] lower
            bound and a [3] upper bound.
        """
        box3d_points = self.box3d_to_cam_points(label, expend_factor)
        box3d_points_xyz = box3d_points.xyz
        wx = box3d_points_xyz[[0], :] - box3d_points_xyz[[4], :]
        lx = np.matmul(wx, box3d_points_xyz[4, :])
        ux = np.matmul(wx, box3d_points_xyz[0, :])
        wy = box3d_points_xyz[[0], :] - box3d_points_xyz[[1], :]
        ly = np.matmul(wy, box3d_points_xyz[1, :])
        uy = np.matmul(wy, box3d_points_xyz[0, :])
        wz = box3d_points_xyz[[0], :] - box3d_points_xyz[[3], :]
        lz = np.matmul(wz, box3d_points_xyz[3, :])
        uz = np.matmul(wz, box3d_points_xyz[0, :])
        return(np.concatenate([wx, wy, wz], axis=0),
            np.concatenate([lx, ly, lz]), np.concatenate([ux, uy, uz]))

    def sel_points_in_box3d(self, label, points, expend_factor=(1.0, 1.0, 1.0)):
        """Select points in a 3D box.

        Args:
            label: a dictionary containing "x3d", "y3d", "z3d", "yaw",
            "height", "width", "lenth".

        Returns: a bool mask indicating points inside a 3D box.
        """

        normals, lower, upper = self.box3d_to_normals(label, expend_factor)
        projected = np.matmul(points.xyz, np.transpose(normals))
        points_in_x = np.logical_and(projected[:, 0] > lower[0],
            projected[:, 0] < upper[0])
        points_in_y = np.logical_and(projected[:, 1] > lower[1],
            projected[:, 1] < upper[1])
        points_in_z = np.logical_and(projected[:, 2] > lower[2],
            projected[:, 2] < upper[2])
        mask = np.logical_and.reduce((points_in_x, points_in_y, points_in_z))
        return mask

    def sel_xyz_in_box3d(self, label, xyz):
        """Select points in a 3D box.

        Args:
            label: a dictionary containing "x3d", "y3d", "z3d", "yaw",
            "height", "width", "lenth".

        Returns: a bool mask indicating points inside a 3D box.
        """

        normals, lower, upper = self.box3d_to_normals(label)
        projected = np.matmul(xyz, np.transpose(normals))
        points_in_x = np.logical_and(projected[:, 0] > lower[0],
            projected[:, 0] < upper[0])
        points_in_y = np.logical_and(projected[:, 1] > lower[1],
            projected[:, 1] < upper[1])
        points_in_z = np.logical_and(projected[:, 2] > lower[2],
            projected[:, 2] < upper[2])
        mask = np.logical_and.reduce((points_in_x, points_in_y, points_in_z))
        return mask
    def rgb_to_cam_points(self, points, image, calib):
        """Append rgb info to camera points"""

        img_points = self.cam_points_to_image(points, calib)
        rgb = image[np.int32(img_points.xyz[:,1]),
            np.int32(img_points.xyz[:,0]),::-1].astype(np.float32)/255
        return Points(points.xyz, np.hstack([points.attr, rgb]))


    def velo_points_to_cam(self, points, calib):
        """Convert points in velodyne coordinates to camera coordinates.

        """

        velo_xyz1 = np.hstack([points.xyz, np.ones([points.xyz.shape[0],1])])
        cam_xyz = np.matmul(velo_xyz1, np.transpose(calib['velo_to_cam']))[:,:3]
        return Points(xyz = cam_xyz, attr = points.attr)

    def cam_points_to_velo(self, points, calib):
        """Convert points from camera coordinates to velodyne coordinates.

        Args:
            points: a [N, 3] float32 numpy array.

        Returns: a [N, 3] float32 numpy array.
        """

        cam_xyz1 = np.hstack([points.xyz, np.ones([points.xyz.shape[0],1])])
        velo_xyz = np.matmul(cam_xyz1, np.transpose(calib['cam_to_velo']))[:,:3]
        return Points(xyz = velo_xyz, attr = points.attr)

    def cam_points_to_image(self, points, calib):
        """Convert camera points to image plane.

        Args:
            points: a [N, 3] float32 numpy array.

        Returns: points on image plane: a [M, 2] float32 numpy array,
                  a mask indicating points: a [N, 1] boolean numpy array.
        """

        cam_points_xyz1 = np.hstack(
            [points.xyz, np.ones([points.xyz.shape[0],1])])
        img_points_xyz = np.matmul(
            cam_points_xyz1, np.transpose(calib['cam_to_image']))
        img_points_xy1 = img_points_xyz/img_points_xyz[:,[2]]
        img_points = Points(img_points_xy1, points.attr)
        return img_points

    def velo_points_to_image(self, points, calib):
        """Convert points from velodyne coordinates to image coordinates. Points
        that behind the camera is removed.

        Args:
            points: a [N, 3] float32 numpy array.

        Returns: points on image plane: a [M, 2] float32 numpy array,
                 a mask indicating points: a [N, 1] boolean numpy array.
        """

        cam_points = self.velo_points_to_cam(points, calib)
        img_points = self.cam_points_to_image(cam_points, calib)
        # The following cause points behind camera to be projected onto image.
        # points = np.hstack([points, np.ones([points.shape[0],1])])
        # img_points = np.matmul(points, np.transpose(calib['velo_to_image']))
        # img_points = img_points[:,:2]/img_points[:,[2]]
        return img_points

    def vis_draw_2d_box(self, image, label_list):
        """Draw 2D bounding boxes on the image.
        """
        color_list = [(0, 128, 0), (0, 255, 255), (0, 0, 128), (255, 255, 255)]
        for label in label_list:
            if label['name'] == 'DontCare':
                color = (255,191,0)
            else:
                color = color_list[label['occlusion']]
            xmin = int(label['xmin'])
            ymin = int(label['ymin'])
            xmax = int(label['xmax'])
            ymax = int(label['ymax'])
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 2)
            cv2.putText(image, '{:s}'.format(label['name']),
                (xmin, ymin), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color,2)

    def vis_draw_3d_box(self, image, label_list, calib):
        """Draw 3D bounding boxes on the image.
        """
        color_list = [(0, 128, 0), (0, 255, 255), (0, 0, 128), (255, 255, 255)]
        for label in label_list:
            cam_points = self.box3d_to_cam_points(label)
            if any(cam_points.xyz[:, 2]<0.1):
                # only draw 3D bounding box for objects in front of the camera
                continue
            img_points = self.cam_points_to_image(cam_points, calib)
            img_points_xy = img_points.xyz[:, 0:2].astype(np.int)
            color = color_list[label['occlusion']]
            cv2.line(image, tuple(img_points_xy[0,:]),
                tuple(img_points_xy[1,:]),color,2)
            cv2.line(image, tuple(img_points_xy[1,:]),
                tuple(img_points_xy[5,:]),color,2)
            cv2.line(image, tuple(img_points_xy[5,:]),
                tuple(img_points_xy[4,:]),color,2)
            cv2.line(image, tuple(img_points_xy[4,:]),
                tuple(img_points_xy[0,:]),color,2)
            cv2.line(image, tuple(img_points_xy[1,:]),
                tuple(img_points_xy[2,:]),color,2)
            cv2.line(image, tuple(img_points_xy[2,:]),
                tuple(img_points_xy[6,:]),color,2)
            cv2.line(image, tuple(img_points_xy[6,:]),
                tuple(img_points_xy[5,:]),color,2)
            cv2.line(image, tuple(img_points_xy[2,:]),
                tuple(img_points_xy[3,:]),color,2)
            cv2.line(image, tuple(img_points_xy[3,:]),
                tuple(img_points_xy[7,:]),color,2)
            cv2.line(image, tuple(img_points_xy[7,:]),
                tuple(img_points_xy[6,:]),color,2)
            cv2.line(image, tuple(img_points_xy[3,:]),
                tuple(img_points_xy[0,:]),color,2)
            cv2.line(image, tuple(img_points_xy[4,:]),
                tuple(img_points_xy[7,:]),color,2)
            cv2.putText(image, '{:s}'.format(label['name']),
                tuple(img_points_xy[0,:]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                color,2)

    def get_birdview_image(self, frame_idx, downsample_voxel_size=None):
        """TODO"""

    def inspect_points(self, frame_idx,
        downsample_voxel_size=None, calib=None, expend_factor=(1.0, 1.0, 1.0)):
        """Inspect points inside dataset"""
        cam_points_in_img_with_rgb = self.get_cam_points_in_image_with_rgb(
            frame_idx, downsample_voxel_size=downsample_voxel_size, calib=calib)
        print("#(points)="+str(cam_points_in_img_with_rgb.xyz.shape))
        label_list = self.get_label(frame_idx)
        self.vis_points(cam_points_in_img_with_rgb,
            label_list, expend_factor=expend_factor)

def test_raw(root_dir, yamlfile=None):
    kitti = KittiDataset(
        os.path.join(root_dir, 'image_00/data/'),
        os.path.join(root_dir, 'velodyne_points/data/'),
        '', '', is_training=False, is_raw=True)
    #calib = kitti.get_autoware_calib(
    #os.path.join(root_dir, '0622_calibration2.yaml'))
    #calib = kitti.get_autoware_calib(    
    #    os.path.join(root_dir, yamlfile))
    # If you are using kitti
    calib = kitti.get_raw_calib(
        '../../kitti_data/2011_09_26/calib_velo_to_cam.txt',
        '../../kitti_data/2011_09_26/calib_cam_to_cam.txt'
    )
    vis = open3d.Visualizer()
    vis.create_window()
    cam_points = kitti.get_cam_points(0, calib=calib)
    pcd = open3d.PointCloud()
    pcd.points = open3d.Vector3dVector(cam_points.xyz)
    vis.add_geometry(pcd)
    # ctr = vis.get_view_control()
    # ctr.rotate(0.0, 3141.0, 0)
    # vis.run()
    print(kitti.num_files)
    for frame_idx in range(0, kitti.num_files):
        # Get an image
        image = kitti.get_image(frame_idx,  calib=calib)
        # Get lidar points in camera coordinates
        cam_points = kitti.get_cam_points(frame_idx, calib=calib)
        # Get lidar points in camera coordinates and are visible in image
        cam_points_in_img = kitti.get_cam_points_in_image(frame_idx, calib=calib)
        # Project lidar points to image
        img_points = kitti.cam_points_to_image(cam_points_in_img, calib=calib)
        # Visualizing lidar point on image
        min_distance = np.min(cam_points_in_img.xyz[:,2])
        max_distance = np.max(cam_points_in_img.xyz[:,2])
        scale = 255/(max_distance - min_distance)
        point_and_img = image.copy()
        depth = cam_points_in_img.xyz[:,2]
        depth_color = cv2.applyColorMap(np.uint8(scale*(depth-min_distance)),
            cv2.COLORMAP_JET)
        for i, img_point in enumerate(img_points.xyz):
            color = depth_color[i,0, :].astype(np.uint8).tolist()
            cv2.circle(point_and_img,
                (int(img_point[0]), int(img_point[1])), 2, color, -1, 4, 0)
        cv2.imshow('point_img', point_and_img)
        cv2.waitKey(1)
        # Visualizing lidar points in camera coordinates
        pcd.points = open3d.Vector3dVector(cam_points.xyz)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()


if __name__ == '__main__':
    
    if len(sys.argv) > 1:
        root_dir = '../../data_raw/' + sys.argv[1]
        yamlname = sys.argv[1] + '.yaml'
        test_raw(root_dir, yamlname)
    else: 
        # root_dir = '../autoware-20180928131539/'
        # root_dir = '../0216_4/'
        # root_dir = '../0217_sync/'
        #root_dir = '../../kitti_data/2011_09_26/2011_09_26_drive_0009_sync/'
        root_dir = '../../kitti_data/2011_09_26/2011_09_26_drive_0057_sync/'
        test_raw(root_dir)
