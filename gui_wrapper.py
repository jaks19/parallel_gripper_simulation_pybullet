### A wrapper that takes in a Simulation object and adds controls to the GUI, allowing realtime features such as:
### - Moving the gripper
### - Opening/closing the gripper
### - Manipulating the camera

from math import pi
import time
import argparse

from simulation import Simulation
from paths import *
from utils import *

class GUI_Wrapper():

    def __init__(self, sim):
        self.sim = sim
        return
    
    def run(self):
        sliders_names_to_ids = self.add_sliders_to_sim()

        for _ in range(int(1e9)):
            self.sim.p.stepSimulation()
            time.sleep(1/240)

            self.camera_refresh(sliders_names_to_ids)
            self.run_position_mover(sliders_names_to_ids)
            self.run_torque_mover(sliders_names_to_ids)

    def add_sliders_to_sim(self):
        sliders_names_to_ids = {}

        ### Movement to coordinates (inverese dynamics)
        sliders_names_to_ids['x'] = self.sim.p.addUserDebugParameter(paramName='end_effector_x', rangeMin=-2, rangeMax=2, startValue=self.sim.pos[0])
        sliders_names_to_ids['y'] = self.sim.p.addUserDebugParameter(paramName='end_effector_y', rangeMin=-2, rangeMax=2, startValue=self.sim.pos[1])
        sliders_names_to_ids['z'] = self.sim.p.addUserDebugParameter(paramName='end_effector_z', rangeMin=0, rangeMax=0.4, startValue=self.sim.pos[2])
        sliders_names_to_ids['rot'] = self.sim.p.addUserDebugParameter(paramName='rotation', rangeMin=-pi, rangeMax=+pi, startValue=self.sim.pos[3])

        ### Torques (torque control)
        sliders_names_to_ids['gripper_open_close'] = self.sim.p.addUserDebugParameter(paramName='gripper_on_off', rangeMin=0, rangeMax=1, startValue=0)
        sliders_names_to_ids['gripper_torque_close'] = self.sim.p.addUserDebugParameter(paramName='gripper_torque_close', rangeMin=0, rangeMax=250, startValue=5)

        ### Debug camera
        dist, yaw, pitch = self.sim.cam_distance_yaw_pitch
        sliders_names_to_ids['cam_distance'] = self.sim.p.addUserDebugParameter(paramName='cam_distance', rangeMin=0, rangeMax=5, startValue=0.737)
        sliders_names_to_ids['cam_yaw'] = self.sim.p.addUserDebugParameter(paramName='cam_yaw', rangeMin=0, rangeMax=360, startValue=89.0)
        sliders_names_to_ids['cam_pitch'] = self.sim.p.addUserDebugParameter(paramName='cam_pitch', rangeMin=0, rangeMax=360, startValue=193.0)
        sliders_names_to_ids['cam_x'] = self.sim.p.addUserDebugParameter(paramName='cam_x', rangeMin=-5, rangeMax=5, startValue=0)
        sliders_names_to_ids['cam_y'] = self.sim.p.addUserDebugParameter(paramName='cam_y', rangeMin=-5, rangeMax=5, startValue=0)
        sliders_names_to_ids['cam_z'] = self.sim.p.addUserDebugParameter(paramName='cam_z', rangeMin=-5, rangeMax=5, startValue=0.158) 

        return sliders_names_to_ids

    def camera_refresh(self, sliders_names_to_ids):
        all_sliders_cam = ['cam_distance', 'cam_yaw', 'cam_pitch', 'cam_x', 'cam_y', 'cam_z']
        slider_cam = [self.sim.p.readUserDebugParameter(sliders_names_to_ids[x]) for x in all_sliders_cam]

        assert(len(slider_cam) == len(self.sim.cam_distance_yaw_pitch)+len(self.sim.cam_pos))
        if slider_cam != self.sim.cam_distance_yaw_pitch+self.sim.cam_pos:
            self.sim.cam_distance_yaw_pitch = slider_cam[:3]
            self.sim.cam_pos = slider_cam[3:]

            dist, yaw, pitch = self.sim.cam_distance_yaw_pitch
            self.sim.p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=yaw, cameraPitch=pitch, cameraTargetPosition=self.sim.cam_pos)
        return

    def run_position_mover(self, sliders_names_to_ids):
        all_sliders_pos = ['x', 'y', 'z', 'rot']
        slider_pos = [self.sim.p.readUserDebugParameter(sliders_names_to_ids[x]) for x in all_sliders_pos]
        slider_pos[2] -= self.sim.gripper_susp_h

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                    jointIndices=[0,1,2,3],
                                    controlMode=self.sim.p.POSITION_CONTROL,
                                    targetPositions=slider_pos)
        return

    def run_torque_mover(self, sliders_names_to_ids):
        torque_on_off = self.sim.p.readUserDebugParameter(sliders_names_to_ids['gripper_open_close'])
        torque_close = self.sim.p.readUserDebugParameter(sliders_names_to_ids['gripper_torque_close'])

        if torque_on_off < 0.5:
            self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id, jointIndices=self.sim.gripper_joint_indices, controlMode=self.sim.p.VELOCITY_CONTROL, forces=[0]*len(self.sim.gripper_joint_indices))
            self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id, jointIndices=self.sim.gripper_joint_indices, controlMode=self.sim.p.TORQUE_CONTROL, forces=[-10, 10])

        else: 
            self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id, jointIndices=self.sim.gripper_joint_indices, controlMode=self.sim.p.VELOCITY_CONTROL, forces=[0]*len(self.sim.gripper_joint_indices))
            self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id, jointIndices=self.sim.gripper_joint_indices, controlMode=self.sim.p.TORQUE_CONTROL, forces=[torque_close, -torque_close])    
        return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Debug the simulation or run a basic routine')
    parser.add_argument('--obj_path', type=str, required=True)
    args = parser.parse_args()

    # A new urdf file for whichever object was picked at args.obj_path
    object_urdf_path = edit_object_name_in_urdf_file(local_object_path=args.obj_path, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)

    # Create a fresh sim
    import pybullet as p
    sim = Simulation(p, debug=True)
    sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path)

    # Wrap sim in the GUI wrapper and launch the wrapper
    gui_wrapper = GUI_Wrapper(sim=sim)
    gui_wrapper.run()