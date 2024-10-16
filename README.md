# BIM2SemanticPC
This code enables the conversion of BIM models in IFC format into semantically enriched 3D point clouds. It is divided into two parts, which can also be used independently:

## 1. Creating OBJ Meshes
The code uses **IfcConvert** to generate OBJ meshes from different selected entities of the IFC model.

## 2. Sampling Point Clouds
It generates uniformly sampled point clouds from the OBJ meshes (or any mesh). These point clouds, which are labeled with different colors for easy distinction, can be merged later using tools like **CloudCompare**.

## Additional Functionality: Automatic Labeling
The code also includes functionality for automatically labeling a point cloud based on a semantically labeled reference point cloud (such as the one created from a BIM model). 

The labeling is done by associating each point in the target cloud with the nearest neighbor in the reference point cloud, assigning the corresponding semantic label.

