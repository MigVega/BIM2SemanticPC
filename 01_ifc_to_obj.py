import os

def ifc_to_stp(input_folder: str, output_folder: str, ifc_converter_path: str):
# Convert Columns to .obj
    column_command = ifc_converter_path + " " \
                     + os.path.join(input_folder, file) + " " \
                     + os.path.join(output_folder, os.path.splitext(file)[0]) + "_columns.stp" \
                     + " --include entities IfcColumn"
    os.system(column_command)

def ifc_to_dae(input_folder: str, output_folder: str, ifc_converter_path: str):
# Convert Columns to .obj
    column_command = ifc_converter_path + " " \
                     + os.path.join(input_folder, file) + " " \
                     + os.path.join(output_folder, os.path.splitext(file)[0]) + "_columns.dae" \
                     + " --include entities IfcColumn"
    os.system(column_command)

def ifc_to_obj(input_folder: str, output_folder: str, ifc_converter_path: str):

    # Convert Walls to .obj
    wall_command = ifc_converter_path + " " \
        + os.path.join(input_folder, file) + " " \
        + os.path.join(output_folder, os.path.splitext(file)[0]) + "_walls.obj" \
        + " --include entities IfcWall"         # + " --exclude entities IfcDoor IfcWindow IfcSpace IfcFloor IfcColumn IfcRoof"
    os.system(wall_command)

    # Convert Columns to .obj
    column_command = ifc_converter_path + " " \
        + os.path.join(input_folder, file) + " " \
        + os.path.join(output_folder, os.path.splitext(file)[0]) + "_columns.obj" \
        + " --include entities IfcColumn"
    os.system(column_command)

    # Convert Windows to .obj
    window_command = ifc_converter_path + " " \
        + os.path.join(input_folder, file) + " " \
        + os.path.join(output_folder, os.path.splitext(file)[0]) + "_windows.obj" \
        + " --include entities IfcWindow"
    os.system(window_command)

    # Convert Doors to .obj
    door_command = ifc_converter_path + " " \
        + os.path.join(input_folder, file) + " " \
        + os.path.join(output_folder, os.path.splitext(file)[0]) + "_doors.obj" \
        + " --include entities IfcDoor"
    os.system(door_command)

    # Convert Floor to .obj
    floor_command = ifc_converter_path + " " \
        + os.path.join(input_folder, file) + " " \
        + os.path.join(output_folder, os.path.splitext(file)[0]) + "_floor.obj" \
        + " --include entities IfcFloor IfcSlab"
    os.system(floor_command)

    # Convert Ceiling to .obj # WARNING: ceiling will not work directly with the conSLAM model since it is modeled as a slab
    ceiling_command = ifc_converter_path + " " \
        + os.path.join(input_folder, file) + " " \
        + os.path.join(output_folder, os.path.splitext(file)[0]) + "_ceiling.obj" \
        + " --include entities IfcRoof"
    os.system(ceiling_command)


if __name__ == "__main__":
    # Set the input/output paths
    pwd = os.path.dirname(os.path.realpath(__file__))
    input_folder = os.path.join(pwd, '../../../Ifc2SegmentedPc/Inputs')
    output_folder = os.path.join(pwd, '../../../Ifc2SegmentedPc/Outputs_obj_per_category')
    ifc_converter_path = os.path.join(pwd, 'IfcConvert')

    for file in os.listdir(input_folder):
        if file.endswith(".ifc"):
            ifc_to_obj(input_folder, output_folder, ifc_converter_path)
            # ifc_to_dae(input_folder, output_folder, ifc_converter_path)
            # ifc_to_stp(input_folder, output_folder, ifc_converter_path)