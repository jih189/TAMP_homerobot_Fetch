import numpy as np
# import open3d as o3d

def read_ply(filename, colors  = False):
    """Reads a PLY file representing EuRoc's point cloud.
    Args:
        filename
    Returns:
        (xyz, colors) Both np arrays of (n, 3) size
        if colors = False, returns (xyz, None)
    """
    with open(filename) as f:
        lines = f.readlines()

    if colors:
        lines = lines[10:]
    else:
        lines = lines[7:]
    xyz = []
    c = []  # The color channel (just one, it's greyscale)
    for l in lines:
        tokens = l.split(' ')
        xyz.append([float(t) for t in tokens[:3]])
        c.append([float(t) for t in tokens[3:]])
    
    # xyz = farthest_point_sample(np.array(xyz), 1024)
    points_fps = normalize_pc(xyz)
    return points_fps, None


def normalize_pc(pc):
    centroid = np.mean(pc, axis=0)
    pc = pc - centroid
    m = np.max(np.sqrt(np.sum(pc**2, axis=1)))
    pc = pc / m
    return pc



def farthest_point_sample(point, npoint):
    """
    Input:
        xyz: pointcloud data, [N, D]
        npoint: number of samples
    Return:
        centroids: sampled pointcloud index, [npoint, D]
    """
    N, D = point.shape
    xyz = point[:,:3]
    centroids = np.zeros((npoint,))
    distance = np.ones((N,)) * 1e10
    farthest = np.random.randint(0, N)
    for i in range(npoint):
        centroids[i] = farthest
        centroid = xyz[farthest, :]
        dist = np.sum((xyz - centroid) ** 2, -1)
        mask = dist < distance
        distance[mask] = dist[mask]
        farthest = np.argmax(distance, -1)
    point = point[centroids.astype(np.int32)]
    return point


def display_point_cloud(pc):
    """
    pc is of shape (N, 3)
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc)
    o3d.visualization.draw_geometries([pcd])
