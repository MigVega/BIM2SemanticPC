import os
import open3d as o3d
import numpy as np
from sklearn.neighbors import NearestNeighbors
from whole_pipline.utils.point_clouds.point_cloud_cropping import crop_point_cloud_with_sphere
import concurrent.futures

# V1 Distorted (good scans) +
# # Directory containing the input PCD files
input_folder_scans = ("/03_session_data_blensor/blensor_scans")

# input_poses_dir = ("/01.Datasets/03.ConSLAM/01.Sequences/Sequence2/02.lidar-pcd/07.from_bag_file/"
#                    "04.poses_in_BIM_coords_only_good/All_LiDAR_poses")

# Directory to save labeled PCD files
output_folder = "/03_session_data_blensor/blensor_scans_v4_crop_labeled"
output_folder2 = "/03_session_data_blensor/blensor_scans_v4_crop_OnlyWalls_columns"

# MV: One can select one of the following BIM point clouds, the V9 is denser -> however is only in the ROI, the other
# is not that dense -> therefore requires a larger distance_threshold and will be able to label all the scans
# Threshold distance (6 cm)
# sampled

### OPTION 1:
# distance_threshold = 0.06  # in meters
# # # Load the BIM semantically segmented PLY point cloud (more accurate and dense point cloud from BIM)
# bim_ply_cloud = o3d.io.read_point_cloud("/Repos/13.BIM2SCD/v2_private_github/BIM2SCD/"
#                                         "Ifc2SegmentedPc/Outputs_pcd/"
#
#                                         "V9_All_merged_ROI_0_05_target_BIM.ply")
### OPTION 2:
# Load the BIM semantically segmented PLY point cloud (non accurate and non-dense point cloud from BIM)
distance_threshold = 0.15  # in meters # Here it needs to be large because the point cloud from the entire bim is heavily down
bim_ply_cloud = o3d.io.read_point_cloud("/Ifc2SegmentedPc/Outputs_pcd/Archive/"
                                        "All_unified_BIM_pc.ply")

# very dense -> 5 cm point cloud for better labeling

pose_file_path_CC = "/02_scan_positions/optimized_poses_for_CC.txt"
with open(pose_file_path_CC, 'r') as f:
    lines = f.readlines()

entity_colors = {
    "Wall": np.array([162, 173, 0]) / 255.0,
    "Column": np.array([227, 114, 34]) / 255.0,
    "Door": np.array([134, 94, 60]) / 255.0,
    "Window": np.array([153, 193, 241]) / 255.0,
    "Floor": np.array([0, 101, 189]) / 255.0,
    "Ceiling": np.array([153, 153, 153]) / 255.0
}


# Function to extract points with specific colors from a point cloud
def extract_points_by_color(pcd, color, tolerance=0.01):
    color_array = np.asarray(pcd.colors)
    mask = np.all(np.abs(color_array - color) < tolerance, axis=1)
    extracted_pcd = pcd.select_by_index(np.where(mask)[0])
    return extracted_pcd


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
sphere_center = np.array([0, 0, 0])

# Loop through each PCD file in the input folder
def process_pcd_file(pcd_file, j, lines, input_folder_scans, nn_model, ply_colors, entity_colors, output_folder, output_folder2, sphere_center, sphere_radius):
    number = pcd_file.split(".")[0]
    print("Processing file Nr. " + number)


    # Read the PCD file
    pcd_cloud = o3d.io.read_point_cloud(os.path.join(input_folder_scans, pcd_file))

    pcd_cloud = crop_point_cloud_with_sphere(pcd_cloud, sphere_center, sphere_radius)

    # Extracting x, y coordinates
    tx, ty = map(float, lines[j].strip().split(','))
    # Set z translation to 0
    tz = 12
    translation_vector = np.array([tx, ty, tz])

    pcd_cloud.translate(translation_vector)

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
    output_file = os.path.splitext(pcd_file)[0] + "_labeled.pcd"
    o3d.io.write_point_cloud(os.path.join(output_folder, output_file), pcd_cloud, write_ascii=True)

    # Extract points with wall color
    wall_color = entity_colors["Wall"]
    wall_points = extract_points_by_color(pcd_cloud, wall_color)

    # Extract points with column color
    column_color = entity_colors["Column"]
    column_points = extract_points_by_color(pcd_cloud, column_color)

    # Concatenate the two extracted point clouds
    extracted_points = wall_points + column_points

    # Translate the extracted points back to the origin
    extracted_points.translate(-translation_vector)

    # Save the resulting labeled and translated point cloud in PCD format
    output_file = os.path.splitext(pcd_file)[0] + "_Walls_Columns.pcd"
    o3d.io.write_point_cloud(os.path.join(output_folder2, output_file), extracted_points, write_ascii=True)



with concurrent.futures.ProcessPoolExecutor() as executor:
    futures = [executor.submit(process_pcd_file, pcd_file, j, lines, input_folder_scans, nn_model, ply_colors, entity_colors, output_folder, output_folder2, sphere_center, sphere_radius)
               for j, pcd_file in enumerate(pcd_files)]

# for future in concurrent.futures.as_completed(futures):
#     print(future.result())
