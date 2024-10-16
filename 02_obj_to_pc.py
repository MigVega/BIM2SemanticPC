import os
import open3d as o3d
import numpy as np
import random

def generate_point_clouds_from_obj(obj_files, output_folder):
    entity_colors = {
        "Wall": np.array([162, 173, 0]) / 255.0,
        "Door": np.array([134, 94, 60]) / 255.0,
        "Window": np.array([153, 193, 241]) / 255.0,
        "Floor": np.array([0, 101, 189]) / 255.0,
        "Ceiling": np.array([153, 153, 153]) / 255.0,
        "Column": np.array([227, 114, 34]) / 255.0
    }
    #
    # mesh = o3d.geometry.TriangleMesh.create_sphere()
    # mesh.compute_vertex_normals()
    # o3d.visualization.draw_geometries([mesh])
    # pcd = mesh.sample_points_uniformly(number_of_points=500)
    # o3d.visualization.draw_geometries([pcd])

    for entity, obj_file in obj_files.items():
        mesh = o3d.io.read_triangle_mesh(obj_file)
        # vertices = np.asarray(mesh.vertices)
        # triangles = np.asarray(mesh.triangles)

        num_points_to_sample = 100000000

        # Sample points uniformly with the specified minimum distance
        sampled_points = mesh.sample_points_uniformly(number_of_points=num_points_to_sample,
                                                                       use_triangle_normal=True)
        # Downsample the point cloud using VoxelGrid
        # voxel_size = 0.005  # Adjust this value according to your needs #0.5 cm = 5 mm # takes too long here/ does not work -> better do it manually in CC
        # downsampled_cloud = sampled_points.voxel_down_sample(voxel_size)


        # # Filter points based on minimum distance
        # filtered_points = [sampled_points[0]]
        # for i in range(1, len(sampled_points)):
        #     if np.min(np.linalg.norm(np.array(filtered_points) - sampled_points[i], axis=1)) >= min_distance:
        #         filtered_points.append(sampled_points[i])

        color = entity_colors.get(entity, [1, 1, 1])

        # point_cloud = o3d.geometry.PointCloud()
        # point_cloud.points = o3d.utility.Vector3dVector(np.array(sampled_points))

        sampled_points.colors = o3d.utility.Vector3dVector(np.tile(color, (len(sampled_points.points), 1)))

        output_filename = os.path.join(output_folder, f"V9_{entity}_pcd.ply")
        o3d.io.write_point_cloud(output_filename, sampled_points)


if __name__ == "__main__":
    # Assuming obj_files is a dictionary containing entity names as keys and their corresponding obj file paths as values
    pwd = os.path.dirname(os.path.realpath(__file__))
    input_dir = os.path.join(pwd, '../../../Ifc2SegmentedPc/Outputs_obj_per_category/01_ROI/ConSLAM_Building_00_GroundTruth_V9')
    output_folder = os.path.join(pwd, '../../../Ifc2SegmentedPc/Outputs_pcd')

    obj_files = {
        "Wall": input_dir + "_walls.obj",
        "Floor": input_dir + "_floor.obj",
        "Ceiling": input_dir + "_ceiling.obj", # WARNING: for the celing the obj has to be created manually -> separate from the floor obj
        "Column": input_dir + "_columns.obj"
    }
    # "Door": input_dir + "_doors.obj",
    # "Window": input_dir + "_windows.obj",
    generate_point_clouds_from_obj(obj_files, output_folder)

    # The following is another approach that did not work that well, since it produces more points close to the
    # edges of the triangles of the mesh

    # for entity, obj_file in obj_files.items():
    #     mesh = o3d.io.read_triangle_mesh(obj_file)
    #     vertices = np.asarray(mesh.vertices)
    #     triangles = np.asarray(mesh.triangles)
    #
    #     num_points_to_sample = 1000000
    #     sampled_points = []
    #
    #     while len(sampled_points) < num_points_to_sample:
    #         triangle_index = random.randint(0, len(triangles) - 1)
    #         triangle = triangles[triangle_index]
    #
    #         u = random.uniform(0, 1)
    #         v = random.uniform(0, 1 - u)
    #
    #         w = 1 - u - v
    #
    #         sampled_point = u * vertices[triangle[0]] + v * vertices[triangle[1]] + w * vertices[triangle[2]]
    #         sampled_points.append(sampled_point)
    #
    #     color = entity_colors.get(entity, [1, 1, 1])
    #
    #     point_cloud = o3d.geometry.PointCloud()
    #     point_cloud.points = o3d.utility.Vector3dVector(np.array(sampled_points))
    #     point_cloud.colors = o3d.utility.Vector3dVector(np.tile(color, (len(sampled_points), 1)))
    #
    #     output_filename = os.path.join(output_folder, f"{entity}_point_cloud.ply")
    #     o3d.io.write_point_cloud(output_filename, point_cloud)
