import open3d as o3d
import numpy as np
import os
import glob

def load_files(path):
    cams = sorted(os.listdir(path))
    current_path = os.path.join(path, cams[0])
    files = sorted(glob.glob(current_path + '/*.ply'))
    result = [[] for i in range(len(cams))]

    for file in files:
        basename = os.path.basename(file)
        exists = np.full((len(cams)), False)
        for i, cam in enumerate(cams):
            file = os.path.join(path, cam, basename)
            if os.path.exists(file):
                exists[i] = True
        if sum(exists) == len(exists):
            for i, cam in enumerate(cams):
                result[i].append(os.path.join(path, cam, basename))

    return result

def concat(files):
    files = np.vstack(files)
    
    for i in range(files.shape[1]):
    # for i in range(1):
        pointclouds_files = files[:, i]
        base_name = os.path.basename(pointclouds_files[0])
        pointclouds_load = []
        pointclouds_color = []
        for pointcloud_file in pointclouds_files:
            pcd = o3d.io.read_point_cloud(pointcloud_file)
            pointclouds_load.append(np.asarray(pcd.points))
            pointclouds_color.append(np.asarray(pcd.colors))
        
        pcd_load = np.concatenate((pointclouds_load), axis = 0)
        pcd_color = np.concatenate((pointclouds_color), axis = 0)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcd_load)
        pcd.colors = o3d.utility.Vector3dVector(pcd_color)

        o3d.io.write_point_cloud(os.path.join('../../../data/pointclouds/', base_name), pcd)
        print(f'Written: {base_name}')


def main():
    path = '/home/tony/Downloads/holistic_or_export/'
    files = load_files(path)
    concat(files)

if __name__=='__main__':
    main()

        



