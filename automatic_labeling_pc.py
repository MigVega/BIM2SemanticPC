import open3d as o3d
import numpy as np
from sklearn.neighbors import NearestNeighbors

# Load the first PLY point cloud
ply_cloud = o3d.io.read_point_cloud("/13.BIM2SCD/v2_private_github/BIM2SCD/Ifc2SegmentedPc"
                                    "/Outputs_pcd/All_unified_BIM_pc_v3.ply")

# Load the second PCD point cloud
pcd_cloud = o3d.io.read_point_cloud("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd"
                                    "/05_LiDAR_in_TLSCoords_after_ICP/single_scans/1_1650963683860383.pcd")

# Extract XYZ coordinates from the first PLY point cloud
ply_points = np.asarray(ply_cloud.points)

# Extract RGB colors from the first PLY point cloud
ply_colors = np.asarray(ply_cloud.colors)

# Create a nearest neighbors model using the XYZ coordinates from the first PLY point cloud
nn_model = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(ply_points)

# Find the closest points in the first PLY point cloud for each point in the second PCD point cloud
pcd_points = np.asarray(pcd_cloud.points)
distances, indices = nn_model.kneighbors(pcd_points)

# Threshold distance (9 cm)
distance_threshold = 0.09  # in meters

# Initialize colors and scalar values for the second PCD point cloud
pcd_colors = np.zeros_like(pcd_points)
scalar_values = np.zeros(len(pcd_points))

# Assign colors and scalar values from the first PLY point cloud to the second PCD point cloud
for i, (distance, index) in enumerate(zip(distances, indices)):
    if distance[0] <= distance_threshold:
        pcd_colors[i] = ply_colors[index[0]]
        # Set scalar value to distance for visualization
        scalar_values[i] = distance[0]
    else:
        # Set black color (R=0, G=0, B=0) for points farther than 5 cm
        pcd_colors[i] = [0, 0, 0]
        # Set scalar value to 0 for points farther than 5 cm
        scalar_values[i] = 0

# Set colors to the second PCD point cloud
pcd_cloud.colors = o3d.utility.Vector3dVector(pcd_colors)

# Create a new array for scalar values as a channel in the point cloud
# scalar_channel = o3d.utility.DoubleVector(scalar_values)
# pcd_cloud.point["scalar"] = scalar_channel

# Save the resulting point cloud in PLY format
o3d.io.write_point_cloud("/BIM2SCD/Ifc2SegmentedPc"
                         "/Outputs_pcd_labeled/resulting_point_cloud.ply", pcd_cloud, write_ascii=True)



