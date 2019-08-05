#### The Simulation class creates a pybullet environment
#### Loads the gripper, object and plane urdf files
#### Sets quantities for referencing by any wrapper designed at a higher level of abstraction

from paths import *

# Useful quantities dashboard

OBJECT_SCALE = 0.3
OBJECT_MASS = 0.1
SUSPENSION_HEIGHT = 0.5

INIT_OBJECT_POS = [0,0,0.04]
INIT_OBJECT_ORN = [0,0,0,1]

INIT_CAM_DISTANCE = 0.063
INIT_CAM_YAW = -90
INIT_CAM_PITCH = 270.001
INIT_CAM_POSITION = [0,0,0]

class Simulation(object):
    def __init__(self, p, arm_urdf=None, debug=True):
        self.p = p

        if not debug: self.p.connect(self.p.DIRECT)
        else: self.p.connect(self.p.GUI)

        self.p.configureDebugVisualizer(self.p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
        self.p.configureDebugVisualizer(self.p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)

        self.set_constants()
        
        return

    def reset_camera(self):
        self.p.resetDebugVisualizerCamera(cameraDistance=INIT_CAM_DISTANCE, cameraYaw=INIT_CAM_YAW, cameraPitch=INIT_CAM_PITCH, cameraTargetPosition=INIT_CAM_POSITION)
        self.cam_distance_yaw_pitch = [INIT_CAM_DISTANCE, INIT_CAM_YAW, INIT_CAM_PITCH]
        self.cam_pos = INIT_CAM_POSITION
        return

    def set_constants(self):
        # Joints tally
        self.num_joints = 9
        self.gripper_joint_indices = [5, 7]
        self.controlled_joints = [0, 1, 2, 3] + self.gripper_joint_indices

        # Limits
        self.ll = [-10, -10, -10, -10, 0, -100, 0, -100, 0]
        self.ul = [10, 10, 10, 10, 0, 100, 0, 100, 0]
        return

    def reset_state(self, include=['gripper', 'plane', 'object'], object_pos=INIT_OBJECT_POS, object_orn=INIT_OBJECT_ORN, object_mass=OBJECT_MASS, object_scale=OBJECT_SCALE, gripper_h=SUSPENSION_HEIGHT, object_urdf_path=None):
        # Restart the simulation engine
        self.p.resetSimulation()

        # Camera
        self.reset_camera()

        # Init simulation variables
        self.p.setRealTimeSimulation(False)
        self.gravity = -10
        self.p.setGravity(0, 0, self.gravity)

        # Objects
        self.load_designs(include, object_pos, object_orn, object_mass, object_scale, gripper_h, object_urdf_path)

        # Torques
        self.gripper_torque_vec = [0]*2
        return

    def load_designs(self, include, object_pos, object_orn, object_mass, object_scale, gripper_h, object_urdf_path):
        if 'gripper' in include:
            # Weiss SG-32 hand
            self.gripper_susp_h = gripper_h
            self.arm_id = self.p.loadURDF(fileName=os.path.join(ROOT_URDFS_FOLDER, "wsg_32.urdf"), basePosition=[0, 0, gripper_h], useFixedBase=True)
            # Initial position of gripper, needed for debugger sliders to work
            self.pos = [0,0,gripper_h,0]

        if 'plane' in include:
            # Surface to land on
            self.plane_id = self.p.loadURDF(fileName=os.path.join(ROOT_URDFS_FOLDER, "plane.urdf"), basePosition=[0, 0, 0], useFixedBase=True)

        if 'object' in include:
            # Object to be grasped
            self.object_id = self.p.loadURDF(fileName=object_urdf_path, basePosition=object_pos, baseOrientation=object_orn, 
            useFixedBase=False, globalScaling=object_scale)
            self.p.changeDynamics(self.object_id, -1, mass=object_mass)
        return