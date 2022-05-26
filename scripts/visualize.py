import open3d as o3d
import numpy as np
import os
import glob


pcd = o3d.io.read_point_cloud('/home/tony/Documents/University/hiwi/catkin/data_2/pointclouds/test.ply')

o3d.visualization.draw_geometries([pcd])