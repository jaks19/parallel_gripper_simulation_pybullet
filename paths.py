import os

# paths
_root_dir = os.path.dirname(os.path.realpath(__file__))
TMP_FOLDER_PATH_URDFS = os.path.join(_root_dir, './tmp/urdfs/')
ROOT_URDFS_FOLDER = os.path.join(_root_dir, './urdfs/')
ORIGINAL_OBJECTS_URDF_PATH = os.path.join(ROOT_URDFS_FOLDER, 'object.urdf')
TMP_FOLDER_PATH_OBJECTS = os.path.join(_root_dir, './tmp/objects/')

TMP_FOLDERS = [TMP_FOLDER_PATH_OBJECTS, TMP_FOLDER_PATH_URDFS]
for f in TMP_FOLDERS:
        if not os.path.exists(f): 
            os.makedirs(f)