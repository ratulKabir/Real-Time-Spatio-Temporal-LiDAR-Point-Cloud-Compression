import numpy as np
import open3d as o3d

if __name__ == "__main__":
    # Load binary point cloud
    bin_pcd = np.fromfile("/home/ratul-motorai/data/dataset/KITTI/training/velodyne/000001.bin", dtype=np.float32)
    # bin_pcd = np.fromfile("/home/ratul-motorai/data/dataset/KITTI/training/velodyne-orig/000001.bin", dtype=np.float32)

    # Reshape and drop reflection values
    points = bin_pcd.reshape((-1, 4))[:, 0:3]

    # Convert to Open3D point cloud
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    print(pcd)
    # print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

