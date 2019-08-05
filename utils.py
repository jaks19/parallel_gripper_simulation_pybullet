import random
from paths import *

# Given an original urdf file, create a new one in the same directory, with a random name
def get_new_urdf_path_tmp(urdf_path_orig):
    split = urdf_path_orig.split('/')
    urdf_name = split[-1]
    urdf_name_with_idx = "".join([str(x) for x in [random.randint(0,9) for i in range(10)]]) + urdf_name
    return os.path.join(TMP_FOLDER_PATH_URDFS, urdf_name_with_idx)

# To change object in the pybullet simulation, we are manually changing the object name in the original 
# object urdf file and saving the new urdf file which will then be read
def edit_object_name_in_urdf_file(local_object_path, urdf_path):
    # .obj file
    assert(len(local_object_path.split('.'))>1 and 'obj' in local_object_path.split('.'))

    # .urdf file
    assert(len(urdf_path.split('.'))>1 and 'urdf' in urdf_path.split('.'))
    urdf_new_path = get_new_urdf_path_tmp(urdf_path_orig=urdf_path)

    import xml.etree.ElementTree as ET

    # A tree is a virtual urdf file
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Change the name of the object in the urdf tree
    for mesh in root.iter("mesh"):
        mesh.set('filename', local_object_path)

    # Write the new urdf file
    tree.write(urdf_new_path)
    return urdf_new_path