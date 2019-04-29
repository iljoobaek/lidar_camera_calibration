import os
import time
from os.path import isfile, join
import sys
import glob

import numpy as np
import open3d
import cv2
import sensor_msgs.point_cloud2 as pc2 

parser_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),'..')) + '/ros_velodyne/'
sys.path.insert(0, parser_dir)
from parser import RosbagParser

debug_print = False

def data_path_loader(path='../../../data/data_bag/20190424_pointgrey/'):
    path_horizontal = path + 'horizontal/*.bag'
    path_tilted = path + 'tilted/*.bag'

    horizontal = sorted(glob.glob(path_horizontal))
    tilted = sorted(glob.glob(path_tilted))
    return {'horizontal': horizontal, 'tilted': tilted}

'''
get_points_from_laser_number(pointcloud, num): get lidar points from laser scan "num"
    pointcloud.shape = (n, 5), num in range (0, 16)
'''
def get_points_from_laser_number(pointcloud, num):
    idx = pointcloud[:,4] == num
    # print idx.shape
    return pointcloud[idx,:]    

'''
rearrange_pointcloud_by_ring(pointcloud): rearrange the pointcloud by the order "laser scan 0 to 15"
    pointcloud.shape = (n, 5)
'''
def rearrange_pointcloud_by_ring(pointcloud):
    pc_rearrange = np.empty((0, 5), dtype=float)
    for i in range(0, 16):
        pc_i = get_points_from_laser_number(pointcloud, float(i))
        pc_rearrange = np.vstack((pc_rearrange, pc_i))
    return pc_rearrange

'''
color map for laser scan 0 to 15
'''
c_map = np.zeros((16, 3), dtype='float')
c_map[0] = np.array([1., 0., 0.])
c_map[1] = np.array([0., 1., 0.])
c_map[2] = np.array([0., 0., 1.])
c_map[3] = np.array([0.5, 0.5, 0.])
c_map[4] = np.array([0.5, 0., 0.5])
c_map[5] = np.array([0., 0.5, 0.5])
c_map[6] = np.array([1.0, 0.5, 0.])
c_map[7] = np.array([1.0, 0., 0.5])
c_map[8] = np.array([0, 1., 0.5])
c_map[9] = np.array([0.5, 1., 0.])
c_map[10] = np.array([0., 0.5, 1.])
c_map[11] = np.array([0.5, 0.5, 1.])
c_map[12] = np.array([0.5, 1., 0.5])
c_map[13] = np.array([1., 0.5, 0.5])
c_map[14] = np.array([0., 0., 1.])
c_map[15] = np.array([0.5, 0.5, 0.])

'''
get_color(pointcloud): get the color for each point from hardcoded color map for each scan line [0, 15]
'''
def get_color(pointcloud):
    n, c = pointcloud.shape
    color = np.zeros((n, 3), dtype='float')
    for i in range(0, 16):
        idx = pointcloud[:,4] == float(i)
        color[idx] = c_map[i] 
    return color

'''
get_color_elevation(elevation, value): get the color for each point by elevation value
'''
def get_color_elevation(elevation, value=.005):
    n, c = elevation.shape
    color = np.ones((n, 3), dtype='float') / 2.
    idx = (elevation[:, 0] > value) + (elevation[:, 0] < -value) 
    color[idx] = np.array([1., 0., 0.])
    print "Points marked:", np.sum(idx) 
    return color

'''
max_height_filter(pointcloud, max_height): filter the pointcloud by maximun height
'''
def max_height_filter(pointcloud, max_height):
    idx = pointcloud[:,2] < max_height
    return pointcloud[idx,:]    

'''
get_slope(p1, p2): calculate slope of two points p1 and p2
'''
def get_slope(p1, p2):
    dist = np.linalg.norm(p2[0:2]-p1[0:2])
    if debug_print:
        print p1, p2, dist
    return (p2[2] - p1[2])

'''
elevation_map(pointcloud): return a (n, 1) evevation map for each point in the pointcloud
'''
def elevation_map(pointcloud):
    n, c = pointcloud.shape
    elevation = np.zeros((n, 1), dtype='float')
    curr_layer = pointcloud[0,4]
    first = 0
    for i in range(0, n):
        if i == n-1:
            elevation[i] = get_slope(pointcloud[i,:3], pointcloud[first, :3])
        elif pointcloud[i+1,4] > curr_layer:
            elevation[i] = get_slope(pointcloud[i,:3], pointcloud[first, :3])
            curr_layer = pointcloud[i+1,4]
            first = i+1
        else:
            elevation[i] = get_slope(pointcloud[i,:3], pointcloud[i+1, :3])
    return elevation            

''' 
test_visualize(lidar_data): visualize the pointcloud data from the RosbagParser object "lidar_data" in open3d visualizer
    data : np array with shape = (n, 5), for each row: (x, y, z, i, r)
'''
def test_visualize(data):
    # data_xyz = data[:,:3]
    # data_ir = data[:,3:]
    # print data_ir.shape
    pc_list = []
    for i in range(0, 16):
        pc_i = get_points_from_laser_number(data, float(i))
        pc_list.append(pc_i)
    
    pc_rearrange = rearrange_pointcloud_by_ring(data)
    # pc_rearrange = max_height_filter(pc_rearrange, -0.9)
    
    data_xyz = pc_rearrange[:,:3]
    color_map = get_color(pc_rearrange)

    vis = open3d.Visualizer()
    vis.create_window()
    pcd = open3d.PointCloud()
    pcd.points = open3d.Vector3dVector(data_xyz)
    vis.add_geometry(pcd)
    
    n = data_xyz.shape[0] # n = number of points
    for idx in range(0, n): 
        # Visualizing lidar points in camera coordinates
        pcd.points = open3d.Vector3dVector(data_xyz[0:idx,:])
        pcd.colors = open3d.Vector3dVector(color_map[0:idx,:])
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()

'''
update_vis(vis, pcd, pointcloud, color_map): update the open3d visualizer in the loop
'''
def update_vis(vis, pcd, pointcloud, color_map):
    pcd.points = open3d.Vector3dVector(pointcloud)
    pcd.colors = open3d.Vector3dVector(color_map)
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()

'''
add_origin_axis(vis, z=0.9): draw the xyz coordinates axis at the origin / (0, 0, z)
'''
def add_origin_axis(vis, z=0.9):
    points = [[0,0,0-z],[1,0,0-z],[0,1,0-z],[0,0,1-z]]
    lines = [[0,1],[0,2],[0,3]]
    colors = [[1,0,0],[0,1,0],[0,0,1]]
    line_set = open3d.LineSet()
    line_set.points = open3d.Vector3dVector(points)
    line_set.lines = open3d.Vector2iVector(lines)
    line_set.colors = open3d.Vector3dVector(colors)
    vis.add_geometry(line_set)

def get_pointcloud_from_msg(msg):
    pointcloud = np.empty((0,5), 'float32')
    for p in pc2.read_points(msg):
        arr = np.array(p[:],'float32') # x y z intensity ring 
        pointcloud = np.vstack((pointcloud, arr))
    return pointcloud

'''
rotation_matrix(theta=90.): return a rotation matrix which rotata CCW along y axis
'''
def rotation_matrix(theta=90.):
    theta = theta * np.pi / 180.
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]], 'float32')


def rotate_pc(pointcloud, rot):
    pc_trans = np.transpose(pointcloud[:,:3])
    pc_rotated = np.matmul(rot, pc_trans)
    pc_rotated = np.transpose(pc_rotated)
    pointcloud[:,:3] = pc_rotated
    return pointcloud

''' 
visualize(lidar_data): visualize the pointcloud data from the RosbagParser object "lidar_data" in open3d visualizer
'''
def visualize_from_bag(lidar_data, config='horizontal'):
    if config not in ['horizontal', 'tilted']:
        print 'Invalid config input, should be horizontal or tilted'
        return

    # initialize visualizer
    vis = open3d.Visualizer()
    vis.create_window(window_name='point cloud', width=1280, height=960)
    pcd = open3d.PointCloud()
    ctr = vis.get_view_control()

    # draw coodinate axis at (0, 0, -0.9)
    add_origin_axis(vis)

    # get rotation matrix
    rot = rotation_matrix(20.)

    idx = 0
    for topic_1, msg_1, t_1 in lidar_data.topic_1:
        print 'frame', idx, '/', lidar_data.len_1
        # get pointcloud from current frame
        pointcloud = get_pointcloud_from_msg(msg_1)
        pc_rearrange = rearrange_pointcloud_by_ring(pointcloud)
        pc_rearrange = rotate_pc(pc_rearrange, rot) 
        pc_rearrange = max_height_filter(pc_rearrange, -0.5)

        # calculate elevation    
        elevation =  elevation_map(pc_rearrange)
        color_map = get_color_elevation(elevation, 0.01)
        
        # visualizing lidar points in camera coordinates
        if idx == 0:
            pcd.points = open3d.Vector3dVector(pc_rearrange[:,:3])
            vis.add_geometry(pcd)
        update_vis(vis, pcd, pc_rearrange[:,:3], color_map)
        idx += 1
    vis.destroy_window()

topics = ['/camera/image_raw', '/points_raw']
if __name__ == '__main__':
    data_path = data_path_loader()
    print rotation_matrix()

    lidar_data = RosbagParser(data_path['tilted'][2], topics)
    visualize_from_bag(lidar_data, 'tilted')

