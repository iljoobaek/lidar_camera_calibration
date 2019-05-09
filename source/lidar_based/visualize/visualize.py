import os
import time
from os.path import isfile, join
import sys
import glob

import numpy as np
import open3d
import cv2
import rosbag
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, PointField


import matplotlib.pyplot as plt

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

def get_points_from_laser_number(pointcloud, num):
    """
    Get lidar points from laser scan "num"
    
    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @num: number in range(0, 16)
    @type: float
    @return: output pointcloud
    @rtype: numpy array with shape (n, 5)
    """
    idx = pointcloud[:,4] == num
    return pointcloud[idx,:]    

def rearrange_pointcloud_by_ring(pointcloud):
    """
    Rearrange the pointcloud by the order "laser scan 0 to 15"
    
    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @return: output pointcloud
    @rtype: numpy array with shape (n, 5)
    """
    pc_rearrange = np.empty((0, 5), dtype=float)
    index = np.empty((16, 2), dtype=int)
    curr = 0
    for i in range(0, 16):
        pc_i = get_points_from_laser_number(pointcloud, float(i))
        pc_rearrange = np.vstack((pc_rearrange, pc_i))
        index[i] = [curr, pc_rearrange.shape[0]]
        curr = pc_rearrange.shape[0]
    if debug_print:
        for i in range(0, 16):
            print index[i]
    return pc_rearrange, index

def get_pointcloud_list_by_ring_from_pointcloud(pointcloud):
    """
    Return a pointcloud list by the order "laser scan 0 to 15"
    
    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @return: output list of pointcloud 
    @rtype: list of numpy array 
    """
    pc_list = []
    for i in range(0, 16):
        pc_i = get_points_from_laser_number(pointcloud, float(i))
        curb = np.zeros((pc_i.shape[0],1), 'float')
        pc_i = np.hstack((pc_i, curb))
        left_idx = pc_i[:,1] > 0.
        right_idx = pc_i[:,1] <= 0.
        left = pc_i[left_idx]
        right = pc_i[right_idx]
        pc_list.append({'left': left, 'right': right})
    return pc_list

"""
color map for laser scan 0 to 15
"""
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

def get_color(pointcloud):
    """ 
    Get the color for each point from hardcoded color map for each scan line [0, 15]

    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @return: output color map for the pointcloud
    @rtype: numpy array with shape (n, 3)
    """ 
    n, c = pointcloud.shape
    color = np.zeros((n, 3), dtype='float')
    for i in range(0, 16):
        idx = pointcloud[:,4] == float(i)
        color[idx] = c_map[i] 
    return color

def get_color_elevation(elevation, value=.005):
    """ 
    Get the color for each point by elevation value

    @param elevation: input elevation value for each point
    @type: numpy array with shape (n, 1)
    @param value: threshold
    @type: float
    @return: output color map for the pointcloud
    @rtype: numpy array with shape (n, 3)
    """ 
    n, c = elevation.shape
    color = np.ones((n, 3), dtype='float') / 2.
    idx = (elevation[:, 0] > value) + (elevation[:, 0] < -value) 
    color[idx] = np.array([1., 0., 0.])
    print "Points marked:", np.sum(idx) 
    return color

def max_height_filter(pointcloud, max_height):
    """ 
    Filter the pointcloud by maximun height

    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @param max_height: threshold for maximum height
    @type: float
    @return: output filtered pointcloud
    @rtype: numpy array with shape (n, 5)
    """ 
    idx = pointcloud[:,2] < max_height
    return pointcloud[idx,:]    

def FOV_positive_x_filter(pointcloud):
    """ 
    Filter the pointcloud by x value, return only positive x

    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @param max_height: threshold for maximum height
    @type: float
    @return: output filtered pointcloud
    @rtype: numpy array with shape (n, 5)
    """ 
    idx = pointcloud[:,0] > 0.
    return pointcloud[idx,:]    

def get_slope(p1, p2):
    """ 
    Calculate slope of two points p1 and p2

    @param p1: input point
    @type: numpy array with shape (1, 3)
    @param p1: input point
    @type: numpy array with shape (1, 3)
    @return: output slope of vector p1p2
    @rtype: float
    """ 
    dist = np.linalg.norm(p2[0:2]-p1[0:2])
    if debug_print:
        print p1, p2, dist
    return (p2[2] - p1[2]) / dist

def get_z_diff(p1, p2):
    """ 
    Calculate z direction difference of two points p1 and p2

    @param p1: input point
    @type: numpy array with shape (1, 3)
    @param p1: input point
    @type: numpy array with shape (1, 3)
    @return: output z value of vector p1p2
    @rtype: float
    """ 
    if debug_print:
        print p1, p2, dist
    return p2[2] - p1[2]

def elevation_map(pointcloud):
    """ 
    Return a (n, 1) evevation map for each point in the pointcloud

    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @return: output elevation map
    @rtype: numpy array with shape (n, 1)
    """ 
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

def add_curb_column(pointcloud, elevation, value=.005):
    """ 
    Get the color for each point by elevation value

    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @param elevation: input elevation values
    @type: numpy array with shape (n, 1)
    @param value: threshold
    @type: float
    @return: output pointcloud with one more column "curb"
    @rtype: numpy array with shape (n, 6)
    """ 
    n, c = elevation.shape
    curb = np.zeros_like(elevation)
    idx = (elevation[:, 0] > value) + (elevation[:, 0] < -value) 
    curb[idx] = np.array([1.])
    return np.hstack((pointcloud, curb)) 

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

def update_vis(vis, pcd, pointcloud, color_map):
    """
    Update the open3d visualizer in the loop
    """
    pcd.points = open3d.Vector3dVector(pointcloud)
    pcd.colors = open3d.Vector3dVector(color_map)
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()

def add_origin_axis(vis, z=0.9):
    """
    Draw the xyz coordinates axis in open3d visualizer at the origin / (0, 0, z)
    """
    points = [[0,0,0-z],[1,0,0-z],[0,1,0-z],[0,0,1-z]]
    lines = [[0,1],[0,2],[0,3]]
    colors = [[1,0,0],[0,1,0],[0,0,1]]
    line_set = open3d.LineSet()
    line_set.points = open3d.Vector3dVector(points)
    line_set.lines = open3d.Vector2iVector(lines)
    line_set.colors = open3d.Vector3dVector(colors)
    vis.add_geometry(line_set)

def get_pointcloud_from_msg(msg):
    """
    Get pointcloud from pointcloud2 ros message

    @param msg: ros message
    @type: pointcloud2
    @return: output pointcloud 
    @rtype: numpy array with shape (n, 5)
    """
    # pointcloud = np.empty((0,5), 'float32')
    # for p in pc2.read_points(msg):
    #     arr = np.array(p[:],'float32') # x y z intensity ring 
    #     pointcloud = np.vstack((pointcloud, arr))
    pc_list = list(pc2.read_points(msg))
    return np.array(pc_list, 'float32')

def get_pointcloud_list_from_msg(msg):
    """
    Get pointcloud list from pointcloud2 ros message

    @param msg: ros message
    @type: pointcloud2
    @return: output pointcloud list 
    @rtype: list of numpy array 
    """
    pc_list = list(pc2.read_points(msg))
    pointcloud = np.array(pc_list, 'float32')
    pointcloud_list = []
    for i in range(0, 16):
        pointcloud_list.append(get_points_from_laser_number(pointcloud, float(i)))
    return pointcloud_list

def rotation_matrix(theta=90.):
    """
    Return a rotation matrix which rotata CCW along y axis

    @param theta: theta value in degree
    @type: float
    @return: output rotation matrix
    @rtype: numpy array with shape (3, 3)
    """
    theta = theta * np.pi / 180.
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]], 'float32')


def rotate_pc(pointcloud, rot):
    """
    Return pointcloud rotated with rotation matrix rot

    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @param rot: input rotation matrix
    @type: numpy array with shape (3, 3)        get_points_from_laser_number()
    @return: output rotated pointcloud
    @rtype: numpy array with shape (n, 5        get_points_from_laser_number()
    """
    pc_trans = np.transpose(pointcloud[:,:3])
    pc_rotated = np.matmul(rot, pc_trans)
    pc_rotated = np.transpose(pc_rotated)
    pointcloud[:,:3] = pc_rotated
    return pointcloud

def translate_z_pc(pointcloud, z):
    """
    Return pointcloud traslated with z

    @param pointcloud: input pointcloud
    @type: numpy array with shape (n, 5)
    @param z: input traslation value in z direction
    @type: float
    @return: output translated pointcloud
    @rtype: numpy array with shape (n, 5)
    """
    pointcloud[:,2] += z
    return pointcloud

def find_matrix(lidar_data):
    z = 0.
    rot = rotation_matrix()
    for topic_1, msg_1, t_1 in lidar_data.topic_1:
        pointcloud = get_pointcloud_from_msg(msg_1)
        pc_i = get_points_from_laser_number(pointcloud, 0)
        for i in range(0, 20):
            theta = 15. + 0.5 * i
            print theta
            rot = rotation_matrix(theta)
            pc_new = rotate_pc(pc_i, rot) 
            print pc_new[:,2] 
            plt.hist(pc_new[:,2], bins=50)    
            plt.show() 
        break

    return rot, z

def visualize_from_bag(lidar_data, config='horizontal'):
    """
    Visualize the pointcloud data from the RosbagParser object "lidar_data" in open3d visualizer

    @param lidar_data: input lidar data  
    @type: RosbagParser object
    @param config: input config of the lidar sensor  
    @type: string
    @return: 
    @rtype: 
    """
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
    rot = rotation_matrix(18.)

    idx = 0
    for topic_1, msg_1, t_1 in lidar_data.topic_1:
        print 'frame', idx, '/', lidar_data.len_1
        # get pointcloud from current frame
        pointcloud = get_pointcloud_from_msg(msg_1)
        # pc_rearrange = get_points_from_laser_number(pointcloud, 0)
        pc_rearrange = rearrange_pointcloud_by_ring(pointcloud)
        if config == 'tilted': 
            pc_rearrange = rotate_pc(pc_rearrange, rot)
        pc_rearrange = translate_z_pc(pc_rearrange, 0.3)
        pc_rearrange = max_height_filter(pc_rearrange, .3)

        # calculate elevation    
        elevation =  elevation_map(pc_rearrange)

        if config == 'tilted': 
            color_map = get_color_elevation(elevation, 0.01)
        else:
            color_map = get_color_elevation(elevation, 0.003)

        # visualizing lidar points in camera coordinates
        if idx == 0:
            pcd.points = open3d.Vector3dVector(pc_rearrange[:,:3])
            vis.add_geometry(pcd)
        update_vis(vis, pcd, pc_rearrange[:,:3], color_map)
        idx += 1
    vis.destroy_window()

def pc2_message(msg, pc_data):
    header = Header()
    header.frame_id = msg.header.frame_id
    header.stamp = msg.header.stamp

    # add one more field 'curb' in the pointcloud2 message
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('ring', 16, PointField.FLOAT32, 1),
            PointField('curb', 20, PointField.FLOAT32, 1)]
    return pc2.create_cloud(header, fields, pc_data)

def edge_filter(pointcloud_z, index, k=4):
    """
    Calculate z difference from current point to k points left and k points right
    If one side close to zero and the other is not, mark it as edge point

    @param pointcloud: input pointcloud with only z 
    @type: numpy array with shape (n, 1)
    @param index: index recording start and end position of each ring from 0 to 15
    @type: numpy array with shape (16, 2)
    @return:  
    @rtype: 
    """
    thres = 0.05
    edges = np.zeros((pointcloud_z.shape[0], 1), 'float')
    for r in range(0,16):
        start, end = index[r,0], index[r,1]
        pc_i = pointcloud_z[start:end]   #               shape = (n, 1)
        diff1 = pc_i[1:]-pc_i[:-1]        # (i-1) - i    shape = (n-1, 1)
        diff2 = pc_i[2:]-pc_i[:-2]        # (i-2) - i    shape = (n-2, 1)
        diff3 = pc_i[3:]-pc_i[:-3]        # (i-3) - i    shape = (n-3, 1)
        diff4 = pc_i[4:]-pc_i[:-4]        # (i-4) - i    shape = (n-4, 1)
        # diff5 = pc_i[5:]-pc_i[:-5]        # (i-5) - i    shape = (n-5, 1)
        right_sum = diff1[k:-3] + diff2[k:-2] + diff3[k:-1] + diff4[k:]
        left_sum = (-diff1[3:-k]) + (-diff2[2:-k]) + (-diff3[1:-k]) + (-diff4[:-k])
        right_sum = np.abs(right_sum) 
        left_sum = np.abs(left_sum)
        is_edge = (right_sum < thres) * (left_sum > thres) + (right_sum > thres) * (left_sum < thres)
        edges[start+k: end-k][is_edge] = 1.
    return edges

def curb_detection_v1(msg, config, rot, height):
    """
    Detect and return ros message with additional "curb" information  
    Version one

    @param msg: input ros message
    @type: pointcloud2
    @param config: input config of the lidar sensor  
    @type: string
    @param rot: input rotation matrix
    @type: numpy array with shape (3, 3)
    @param height: input translation value in z direction
    @type: float
    @return: output ros message 
    @rtype: ros pointcloud2 message
    """
    pointcloud = get_pointcloud_from_msg(msg)

    if config == 'tilted': 
        pointcloud = rotate_pc(pointcloud, rot)
        pointcloud = translate_z_pc(pointcloud, height)
    else:
        pointcloud = translate_z_pc(pointcloud, height-0.2)
    pointcloud = max_height_filter(pointcloud, .3)
    pointcloud = FOV_positive_x_filter(pointcloud)
    pc_rearrange, index = rearrange_pointcloud_by_ring(pointcloud)

    # calculate elevation    
    elevation =  elevation_map(pc_rearrange)

    # calculate z difference for 5 points left and 5 points right at point i
    edges = edge_filter(pc_rearrange[:,2], index)
    pc_data = add_curb_column(pc_rearrange, edges, 0.5)

    # if config == 'tilted': 
    #     pc_data = add_curb_column(pc_rearrange, elevation, 0.5)
    # else:
    #     pc_data = add_curb_column(pc_rearrange, elevation, 0.003)
    return pc2_message(msg, pc_data)

# looks fine with thres_slope=0.07 thres_flat=0.04 (fails when curb is not flat)
def find_edge_from_right_half_v02(pointcloud_right, k=6, thres_slope=0.07,thres_flat=0.04):
    """
    Detect and return edge points index from single laser group at right side of vehicle

    @param pointcloud: input pointcloud (right & front in CW order) 
    @type: numpy array with shape (n, 6)
    @return: index of curb points 
    @rtype: numpy array with shape (n, 6)
    """
    n =  pointcloud_right.shape[0]
    if n - 2 * k < 0:
        return pointcloud_right

    # reorder the points
    theta = np.zeros(n, 'float') 
    for i in range(n):
        theta[i] = -np.arctan2(pointcloud_right[i,1], pointcloud_right[i,0]) * 180 / np.pi 
    order = np.argsort(theta)
    pointcloud_right = pointcloud_right[order,:]

    # calculate left_sum and right_sum
    right_sum, left_sum = np.zeros((n-2*k), 'float'), np.zeros((n-2*k), 'float')
    diff = []
    pointcloud_z = pointcloud_right[:,2]
    for i in range(1,k+1):
        diff1 = pointcloud_z[i:]-pointcloud_z[:-i] 
        diff.append(diff1)
    for i in range(k):
        if i == k - 1:
            right_sum += diff[i][k:]
            left_sum += (-diff[i][:-k])
        else:
            right_sum += diff[i][k:(i-k+1)] 
            left_sum += (-diff[i][k-i-1:-k])

    # parameters
    curr_start, curr_end = 0, 0

    # find start points    
    is_edge_start = (right_sum > thres_slope) * (np.abs(left_sum) < thres_flat)
    is_edge_start = np.pad(is_edge_start, (k,k), 'constant',constant_values=False)
    pointcloud_right[:,5][is_edge_start] = 1.
    
    # find end points    
    is_edge_end = (left_sum < -thres_slope) * (np.abs(right_sum) < thres_flat)
    is_edge_end = np.pad(is_edge_end, (k,k), 'constant',constant_values=False)
    pointcloud_right[:,5][is_edge_end] = 0.4

    curb_list = []
    for i in range(n):
        if is_edge_start[i]:
            curr_start = i
        if is_edge_end[i] and curr_start != 0:
            h =  pointcloud_z[i] - pointcloud_z[curr_start]
            if h > 0.02 and h < 0.3:
                curb_list.append([curr_start, i])
                curr_start, curr_end = 0, 0
                curb_height = pointcloud_z[i]

    for c in curb_list:
        if c[0] < c[1] and c[0] > 0 and c[1] > 0 and c[1]-c[0] > 4:
            pointcloud_right[c[0]+1:c[1],5] = 0.7  
    
    # first_start =  np.argmax(pointcloud_right[:,5] > 0.5) # !!! would return 0 if nothing found
    # first_end =  np.argmax((pointcloud_right[:,5] > 0.3) * (pointcloud_right[:,5] < 0.5)) # !!! would return 0 if nothing found
    # if first_start < first_end:
    #     pointcloud_right[first_start+1:first_end,5] = 0.7  
    return pointcloud_right

def find_edge_from_left_half_v02(pointcloud_left, k=4, thres_slope=0.05,thres_flat=0.02):
    """
    Detect and return edge points index from single laser group at left side of vehicle

    @param pointcloud: input pointcloud (left & front in CW order) 
    @type: numpy array with shape (n, 6)
    @return: index of curb points 
    @rtype: numpy array with shape (n, 6)
    """
    n =  pointcloud_left.shape[0]
    if n - 2 * k < 0:
        return pointcloud_left
    
    # calculate left_sum and right_sum
    right_sum, left_sum = np.zeros((n-2*k), 'float'), np.zeros((n-2*k), 'float')
    diff = []
    pointcloud_z = pointcloud_left[:,2]
    for i in range(1,k+1):
        diff1 = pointcloud_z[i:]-pointcloud_z[:-i] 
        diff.append(diff1)
    for i in range(k):
        if i == k - 1:
            right_sum += diff[i][k:]
            left_sum += (-diff[i][:-k])
        else:
            right_sum += diff[i][k:(i-k+1)] 
            left_sum += (-diff[i][k-i-1:-k])
    
    # find start points    
    is_edge_start = (left_sum > thres_slope) * (np.abs(right_sum) < thres_flat)
    is_edge_start = np.pad(is_edge_start, (k,k), 'constant',constant_values=False)
    
    pointcloud_left[:,5][is_edge_start] = 1.
    
    # find end points    
    is_edge_end = (right_sum < -thres_slope) * (np.abs(left_sum) < thres_flat)
    is_edge_end = np.pad(is_edge_end, (k,k), 'constant',constant_values=False)
    
    pointcloud_left[:,5][is_edge_end] = 0.4

    # parameters
    max_curb = 0.3
    curb_height = 0
    curr_start, curr_end = 0, 0
    curb_list = []
    for i in range(n-1,-1,-1):
        if is_edge_start[i]:
            if curr_start == 0:
                curr_start = i
            else:
                if curr_end == 0:
                    curr_start = i
                else:
                    pass
        
        if is_edge_end[i] and curr_start != 0:
            h =  pointcloud_z[i] - pointcloud_z[curr_start]
            if h > 0.05 and h < 0.3:
                curb_list.append([curr_start, i])
            curr_start, curr_end = 0, 0
            curb_height = pointcloud_z[i]

    for c in curb_list:
        if c[1] < c[0] and c[0] > 0 and c[1] > 0 and c[0]-c[1] > 6:
            pointcloud_left[c[1]+1:c[0],5] = 0.7    

    return pointcloud_left

def unit_vec(vec):
    return vec / np.linalg.norm(vec)

def angle_between(v1, v2):
    v1_unit = unit_vec(v1) 
    v2_unit = unit_vec(v2) 
    return np.arccos(np.clip(np.dot(v1_unit, v2_unit), -1., 1.))

# looks fine with thres_slope=0.07 thres_flat=0.04 (fails when curb is not flat)
def find_edge_from_right_half_v03(pointcloud_right, k=6, thres_slope=0.07,thres_flat=0.04):
    """
    Detect and return edge points index from single laser group at right side of vehicle

    @param pointcloud: input pointcloud (right & front in CW order) 
    @type: numpy array with shape (n, 6)
    @return: index of curb points 
    @rtype: numpy array with shape (n, 6)
    """
    n =  pointcloud_right.shape[0]
    if n - 2 * k < 0:
        return pointcloud_right

    # reorder the points
    theta = np.zeros(n, 'float') 
    for i in range(n):
        theta[i] = -np.arctan2(pointcloud_right[i,1], pointcloud_right[i,0]) * 180 / np.pi 
    order = np.argsort(theta)
    pointcloud_right = pointcloud_right[order,:]

    # calculate left_sum and right_sum
    right_sum, left_sum = np.zeros((n-2*k), 'float'), np.zeros((n-2*k), 'float')
    diff = []
    pointcloud_z = pointcloud_right[:,2]
    for i in range(1,k+1):
        diff1 = pointcloud_z[i:]-pointcloud_z[:-i] 
        diff.append(diff1)
    for i in range(k):
        if i == k - 1:
            right_sum += diff[i][k:]
            left_sum += (-diff[i][:-k])
        else:
            right_sum += diff[i][k:(i-k+1)] 
            left_sum += (-diff[i][k-i-1:-k])

    # parameters
    curr_start, curr_end = 0, 0

    # find start points    
    is_edge_start = (right_sum > thres_slope) * (np.abs(left_sum) < thres_flat)
    is_edge_start = np.pad(is_edge_start, (k,k), 'constant',constant_values=False)
    pointcloud_right[:,5][is_edge_start] = 1.
    
    # find end points    
    is_edge_end = (left_sum < -thres_slope) * (np.abs(right_sum) < thres_flat)
    is_edge_end = np.pad(is_edge_end, (k,k), 'constant',constant_values=False)
    pointcloud_right[:,5][is_edge_end] = 0.4

    curb_list = []
    for i in range(n):
        if is_edge_start[i]:
            curr_start = i
        if is_edge_end[i] and curr_start != 0:
            h =  pointcloud_z[i] - pointcloud_z[curr_start]
            if h > 0.02 and h < 0.3:
                curb_list.append([curr_start, i])
                curr_start, curr_end = 0, 0
                curb_height = pointcloud_z[i]

    # add angle thres 0.15 rad about 9 degree
    angle_thres = 0.45
    for c in curb_list:
        if c[0] < c[1] and c[0] > 0 and c[1] > 0 and c[1]-c[0] > 4:
            start = pointcloud_right[c[0]+1,:3]
            v2 = pointcloud_right[c[1],:3] - pointcloud_right[c[0],:3]
            isline = True
            for i in range(c[0]+1, c[1]):
                # print angle_between(pointcloud_right[i,:3]-start, v2)
                if angle_between(pointcloud_right[i,:3]-start, v2) > angle_thres:
                    isline = False
                    break
            if isline:
                pointcloud_right[c[0]+1:c[1],5] = 0.7  
    
    # first_start =  np.argmax(pointcloud_right[:,5] > 0.5) # !!! would return 0 if nothing found
    # first_end =  np.argmax((pointcloud_right[:,5] > 0.3) * (pointcloud_right[:,5] < 0.5)) # !!! would return 0 if nothing found
    # if first_start < first_end:
    #     pointcloud_right[first_start+1:first_end,5] = 0.7  
    return pointcloud_right

def find_edge_from_left_half_v03(pointcloud_left, k=4, thres_slope=0.05,thres_flat=0.02):
    """
    Detect and return edge points index from single laser group at left side of vehicle

    @param pointcloud: input pointcloud (left & front in CW order) 
    @type: numpy array with shape (n, 6)
    @return: index of curb points 
    @rtype: numpy array with shape (n, 6)
    """
    n =  pointcloud_left.shape[0]
    if n - 2 * k < 0:
        return pointcloud_left
    
    # calculate left_sum and right_sum
    right_sum, left_sum = np.zeros((n-2*k), 'float'), np.zeros((n-2*k), 'float')
    diff = []
    pointcloud_z = pointcloud_left[:,2]
    for i in range(1,k+1):
        diff1 = pointcloud_z[i:]-pointcloud_z[:-i] 
        diff.append(diff1)
    for i in range(k):
        if i == k - 1:
            right_sum += diff[i][k:]
            left_sum += (-diff[i][:-k])
        else:
            right_sum += diff[i][k:(i-k+1)] 
            left_sum += (-diff[i][k-i-1:-k])
    
    # find start points    
    is_edge_start = (left_sum > thres_slope) * (np.abs(right_sum) < thres_flat)
    is_edge_start = np.pad(is_edge_start, (k,k), 'constant',constant_values=False)
    
    pointcloud_left[:,5][is_edge_start] = 1.
    
    # find end points    
    is_edge_end = (right_sum < -thres_slope) * (np.abs(left_sum) < thres_flat)
    is_edge_end = np.pad(is_edge_end, (k,k), 'constant',constant_values=False)
    
    pointcloud_left[:,5][is_edge_end] = 0.4

    # parameters
    max_curb = 0.3
    curb_height = 0
    curr_start, curr_end = 0, 0
    curb_list = []
    for i in range(n-1,-1,-1):
        if is_edge_start[i]:
            if curr_start == 0:
                curr_start = i
            else:
                if curr_end == 0:
                    curr_start = i
                else:
                    pass
        
        if is_edge_end[i] and curr_start != 0:
            h =  pointcloud_z[i] - pointcloud_z[curr_start]
            if h > 0.05 and h < 0.3:
                curb_list.append([curr_start, i])
            curr_start, curr_end = 0, 0
            curb_height = pointcloud_z[i]

    for c in curb_list:
        if c[1] < c[0] and c[0] > 0 and c[1] > 0 and c[0]-c[1] > 6:
            pointcloud_left[c[1]+1:c[0],5] = 0.7    

    return pointcloud_left

def curb_detection_v2(msg, config, rot, height):
    """
    Detect and return ros message with additional "curb" information  
    Version two

    @param msg: input ros message
    @type: pointcloud2
    @param config: input config of the lidar sensor  
    @type: string
    @param rot: input rotation matrix
    @type: numpy array with shape (3, 3)
    @param height: input translation value in z direction
    @type: float
    @return: output ros message 
    @rtype: ros pointcloud2 message
    """
    pointcloud = get_pointcloud_from_msg(msg)
    if config == 'tilted': 
        pointcloud = rotate_pc(pointcloud, rot)
        pointcloud = translate_z_pc(pointcloud, height)
    else:
        pointcloud = translate_z_pc(pointcloud, height-0.2)
    pointcloud = max_height_filter(pointcloud, .45)
    pointcloud = FOV_positive_x_filter(pointcloud)

    pc_rearrange, index = rearrange_pointcloud_by_ring(pointcloud)
    
    # get pointcloud list
    pointcloud_list = get_pointcloud_list_by_ring_from_pointcloud(pointcloud)

    pc_data = np.empty((0,6),'float') 
    for i in range(16):
        pc_l = find_edge_from_left_half_v02(pointcloud_list[i]['left'])
        pc_r = find_edge_from_right_half_v02(pointcloud_list[i]['right'])
        pc_i = np.vstack((pc_l, pc_r))
        pc_data = np.vstack((pc_data, pc_i))
    
    return pc2_message(msg, pc_data)

def curb_detection_v3(msg, config, rot, height):
    """
    Detect and return ros message with additional "curb" information  
    Version two

    @param msg: input ros message
    @type: pointcloud2
    @param config: input config of the lidar sensor  
    @type: string
    @param rot: input rotation matrix
    @type: numpy array with shape (3, 3)
    @param height: input translation value in z direction
    @type: float
    @return: output ros message 
    @rtype: ros pointcloud2 message
    """
    pointcloud = get_pointcloud_from_msg(msg)
    if config == 'tilted': 
        pointcloud = rotate_pc(pointcloud, rot)
        pointcloud = translate_z_pc(pointcloud, height)
    else:
        pointcloud = translate_z_pc(pointcloud, height-0.2)
    pointcloud = max_height_filter(pointcloud, .45)
    pointcloud = FOV_positive_x_filter(pointcloud)

    pc_rearrange, index = rearrange_pointcloud_by_ring(pointcloud)
    
    # get pointcloud list
    pointcloud_list = get_pointcloud_list_by_ring_from_pointcloud(pointcloud)

    pc_data = np.empty((0,6),'float') 
    for i in range(16):
        pc_l = find_edge_from_left_half_v03(pointcloud_list[i]['left'])
        pc_r = find_edge_from_right_half_v03(pointcloud_list[i]['right'])
        pc_i = np.vstack((pc_l, pc_r))
        pc_data = np.vstack((pc_data, pc_i))
    
    return pc2_message(msg, pc_data)

def run_detection_and_save(data_name, datta, config, tilted_angle=19.2, height=1.195):
    """t
    Run curb detection algorithm throught all messages in data and store as new rosbag file
    pointcloud = get_pointcloud_from_msg(msg)

    @param data_name: input name of the rosbag file
    @type: string
    @param data: input lidar data
    @type: RosbagParser object
    @param config: input config of the lidar sensor  
    @type: string
    @param tilted_angle: input tilted angle of lidar sensor in degree
    @type: float
    @param height: input translation value in z direction
    @type: float
    @return:  
    @rtype: 
    """
    if config not in ['horizontal', 'tilted']:
        print 'Invalid config input, should be horizontal or tilted'
        return
    
    path = '/home/rtml/LiDAR_camera_calibration_work/source/lidar_based/results/'
    bag_name = path + data_name.split('/')[-1].split('.')[0] + '_processed.bag'
    output_bag = rosbag.Bag(bag_name, 'w')

    # /image_raw
    for topic_0, msg_0, t_0 in lidar_data.topic_0:
        output_bag.write(topic_0, msg_0, t=t_0)

    rot = rotation_matrix(tilted_angle)
    # /points_raw
    idx = 0
    for topic_1, msg_1, t_1 in lidar_data.topic_1:
        print 'frame', idx, '/', lidar_data.len_1
        msg_1_processed = curb_detection_v3(msg_1, config, rot, height) # run curb detection algorithm 
        output_bag.write(topic_1, msg_1_processed, t=t_1)
        idx += 1
    output_bag.close()

topics = ['/camera/image_raw', '/points_raw']
if __name__ == '__main__':
    data_path = data_path_loader()

    # change the number to read different rosbag file
    # tilted: 0 to 9 
    # horizontal: 0 to 5 
    data_name = data_path['tilted'][2]
    # data_name = data_path['horizontal'][2]
    print data_name
    lidar_data = RosbagParser(data_name, topics)
    run_detection_and_save(data_name, lidar_data, 'tilted')
    # run_detection_and_save(data_name, lidar_data, 'horizontal)

    # visualize_from_bag(lidar_data, 'tilted')