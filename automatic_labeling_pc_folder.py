import os
import open3d as o3d
import numpy as np
from sklearn.neighbors import NearestNeighbors
from whole_pipline.utils.point_clouds.point_cloud_cropping import crop_point_cloud_with_sphere

# V1 Distorted (good scans) +
# # Directory containing the input PCD files
input_folder_scans = ("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd/07.from_bag_file/04"
                ".scans_in_BIM_only_good/All")

input_poses_dir = ("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd/07.from_bag_file/"
                   "04.poses_in_BIM_coords_only_good/All_LiDAR_poses")

# Directory to save labeled PCD files
output_folder = ("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd"
                 "/07.from_bag_file/04.scans_in_BIM_only_good/All_cropped_and_labeled_6cm_v2")

# Load the BIM semantically segmented PLY point cloud (non accurate and non-dense point cloud from BIM)
# bim_ply_cloud = o3d.io.read_point_cloud("/Repos/13.BIM2SCD/v2_private_github/BIM2SCD/"
#                                         "Ifc2SegmentedPc/Outputs_pcd/
#                                         ConSLAM_BIM_semantic_pcd_No_doors.ply")

# V2 undistorted (good scans after DirectLIO) + Accurate and dense point cloud from BIM!
# Directory containing the input PCD files
# input_folder_scans = ("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd/"
#                       "08.from_bag_undistorted/02.single_scans_in_BIM_coords_v2_OnlyGood")
#
# input_poses_dir = ("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd/08.from_bag_undistorted"
#                    "/03.poses_in_BIM_coords_after_icp_v2_OnlyGood/Good")
#
# # Directory to save labeled PCD files
# output_folder = ("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd/08.from_bag_undistorted/02"
#                  ".single_scans_in_BIM_coords_v2_OnlyGood_labeled/labeled_6cm/") # end with /

# Threshold distance (6 cm)
distance_threshold = 0.06  # in meters
# Load the BIM semantically segmented PLY point cloud (more accurate and dense point cloud from BIM)
bim_ply_cloud = o3d.io.read_point_cloud("/"
                                        "Ifc2SegmentedPc/Outputs_pcd/"
                                        "V9_All_merged_ROI_0_05_target_BIM.ply")
# very dense -> 5 cm point cloud for better labeling


# Extract XYZ coordinates from the BIM PLY point cloud
ply_points = np.asarray(bim_ply_cloud.points)

# Extract RGB colors from the BIM PLY point cloud
ply_colors = np.asarray(bim_ply_cloud.colors)

# Create a nearest neighbors model using the XYZ coordinates from the BIM PLY point cloud
nn_model = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(ply_points)
print("nearest neighbors model created! ")

# List all files in the input folder
pcd_files = [file for file in os.listdir(input_folder_scans) if file.endswith('.pcd')]
pcd_files = sorted(pcd_files)

sphere_radius = 12  # Points outside this radius will be removed

# Loop through each PCD file in the input folder
for pcd_file in pcd_files:
    number = pcd_file.split("_")[0]
    print("Processing file Nr. " + number)

    # Find corresponding pose file -> This will only be used to crop the point cloud to leave only the points in the sphere
    scan_name = os.path.basename(pcd_file).split('_')[1].split('.')[0]
    pose_file = number + "_" + scan_name + "__transformation_final.txt"

    # Read 4x4 matrix from the pose file
    pose_file_path = os.path.join(input_poses_dir, pose_file)
    with open(pose_file_path, 'r') as f:
        lines = f.readlines()
    pose_matrix = np.loadtxt(lines)

    # Extracting translations along x, y, and z axes
    tx = pose_matrix[0][3]  # Translation along x-axis
    ty = pose_matrix[1][3]  # Translation along y-axis
    tz = pose_matrix[2][3]  # Translation along z-axis

    sphere_center = np.array([tx, ty, tz])

    # Read the PCD file
    pcd_cloud = o3d.io.read_point_cloud(os.path.join(input_folder_scans, pcd_file))

    pcd_cloud = crop_point_cloud_with_sphere(pcd_cloud, sphere_center, sphere_radius)

    # Extract XYZ coordinates from the current PCD point cloud
    pcd_points = np.asarray(pcd_cloud.points)

    # Find the closest points in the first PLY point cloud for each point in the current PCD point cloud
    distances, indices = nn_model.kneighbors(pcd_points)

    # Initialize colors and scalar values for the current PCD point cloud
    pcd_colors = np.zeros_like(pcd_points)

    # Assign colors from the first PLY point cloud to the current PCD point cloud
    for i, (distance, index) in enumerate(zip(distances, indices)):
        if distance[0] <= distance_threshold:
            pcd_colors[i] = ply_colors[index[0]]
        else:
            # Set black color (R=0, G=0, B=0) for points farther than 9 cm
            pcd_colors[i] = [0, 0, 0]

    # Set colors to the current PCD point cloud
    pcd_cloud.colors = o3d.utility.Vector3dVector(pcd_colors)

    # Save the resulting labeled point cloud in PCD format with the same name as the original but with '_label' suffix
    output_file = os.path.splitext(pcd_file)[0] + "_label.pcd"
    o3d.io.write_point_cloud(os.path.join(output_folder, output_file), pcd_cloud, write_ascii=True)
