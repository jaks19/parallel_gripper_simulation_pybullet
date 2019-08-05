### An example wrapper that takes in a Simulation object and implements methods that allow controlling
### the gripper modularly
### We provide primitives for performing the following sub-routines:
### - opening/closing the gripper
### - moving the gripper to a specific point in space
### - shaking the gripper

### We combine those primitives into a grasping routine where the gripper starts high above the table,
#### Then it is lowered till it reached the surface, closes on the object and goes back up, before shaking to test the grasp goodness

### Notw:
### We also show, in the __main__ section, how to stabilize any loaded object at the center of the table
### The strategy used is to load the object and record the pos and orn at which it stabililizes
### Then exploit this info in a relaunch

import numpy as np
import time
import argparse

from simulation import Simulation
from paths import *
from utils import *

GRASP_TIMEOUT = 5 

class Dummy_Grasp_Routine_Wrapper:

    def __init__(self, sim):
        # Provided sim must be reset to any desired state before passing it to here
        self.sim = sim
        self.step_to_sec = 1/240
        self.torques = [0]*self.sim.num_joints

        # No debug sliders (unlike GUI) so no need to keep track of currnt location, camera info etc
        self.sim.p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=90, cameraPitch=187, cameraTargetPosition=[0,0,0.158])
        return


    def advance_time(self, seconds):
        assert(seconds >= self.step_to_sec)
        num_steps = int((1 / (self.step_to_sec))*seconds)
        
        for i in range(num_steps): 
            self.apply_torques()
            self.sim.p.stepSimulation()
            time.sleep(self.step_to_sec)
        return

    def apply_torques(self):
        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id, 
                                    jointIndices=list(range(self.sim.num_joints)[4:]), 
                                    controlMode=self.sim.p.VELOCITY_CONTROL, 
                                    forces=([0]*self.sim.num_joints)[4:])

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                    jointIndices=list(range(self.sim.num_joints)),
                                    controlMode=self.sim.p.TORQUE_CONTROL,
                                    forces=self.torques)
        return

    #### Setters change the sim wrapper state but do not move time
    def set_gripper_open(self, force):
        self.torques[self.sim.gripper_joint_indices[0]] = -force
        self.torques[self.sim.gripper_joint_indices[1]] = force
        return

    def set_gripper_close(self, force=100):
        self.torques[self.sim.gripper_joint_indices[0]] = force
        self.torques[self.sim.gripper_joint_indices[1]] = -force
        return


    #### sub-soutines:
    #### see R_grasp_object for an example of a routine that uses sub-routines as primitives
    def R_open_gripper(self):
        self.set_gripper_open(force=100)
        self.advance_time(0.05)
        pass

    def R_close_gripper(self):
        force = 40
        for i in np.arange(0,force,1):
            self.set_gripper_close(force=i)
            self.advance_time(self.step_to_sec)
        return

    def R_force_move_to_xyz(self, dst_xyz, timeout=None):
        start_time = time.time()

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                    jointIndices=[0,1,2],
                                    controlMode=self.sim.p.POSITION_CONTROL,
                                    targetPositions=np.array(dst_xyz)-np.array([0,0,self.sim.gripper_susp_h]),
                                    targetVelocities=[1e-8]*3)

        while True: 
            self.advance_time(self.step_to_sec)
            if np.allclose(self.get_live_position_gripper(), dst_xyz, rtol=1e-3, atol=1e-3): break
            elif time.time()-start_time > GRASP_TIMEOUT: raise TimeoutError
        return

    def R_shake_gripper(self, amp=0.5):

        for i in range(200):
            dirn = 1 if (i%40<10 or (i%40>=30 and i%40<40)) else -1
            self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                                 jointIndices=[0,1,2],
                                                 controlMode=self.sim.p.VELOCITY_CONTROL,
                                                 targetVelocities=[0,dirn*amp,0])
            self.advance_time(self.step_to_sec)

        self.sim.p.setJointMotorControlArray(bodyIndex=self.sim.arm_id,
                                                 jointIndices=[0,1,2],
                                                 controlMode=self.sim.p.VELOCITY_CONTROL,
                                                 targetVelocities=[0,0,0])
        self.advance_time(self.step_to_sec)
        return


    #### Example routine
    #### Here is an example of a routine (uses sub-routines)
    def R_grasp_object(self):
        self.advance_time(1)
        self.R_open_gripper()
        self.R_force_move_to_xyz(dst_xyz=[0,0,0.137])
        self.R_close_gripper()
        self.R_force_move_to_xyz(dst_xyz=[0,0,0.250])
        self.R_shake_gripper()
        return

    ##### Helpers
    def get_live_position_gripper(self):
        return [self.sim.p.getLinkState(bodyUniqueId=self.sim.arm_id, linkIndex=i)[0][i] for i in range(3)]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run a basic routine')
    parser.add_argument('--obj_path', type=str, required=True)
    args = parser.parse_args()

    # A new urdf file for whichever object was picked at args.obj_path
    object_urdf_path = edit_object_name_in_urdf_file(local_object_path=args.obj_path, urdf_path=ORIGINAL_OBJECTS_URDF_PATH)

    # Create a fresh sim, wrap it in the GUI wrapper and launch the wrapper
    import pybullet as p
    sim = Simulation(p, debug=True)

    ### Stabilize the object being loaded by loading it in an unstable pos and orn first, letting it stabilize, 
    ### then read off the stable pos and orn and use that to reset the sim
    
    # Unstable
    sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path)
    # Step for 3s to let stabilize and read off stable pos and orn 
    for i in range(int(3/(1/240))): sim.p.stepSimulation()
    pos, orn = sim.p.getBasePositionAndOrientation(bodyUniqueId=sim.object_id)
    # Start at origin, and right on the plane surface
    stable_pos = [0,0,pos[1]]
    # Start at last measured stable orn
    stable_orn = orn
    # Stable
    sim.reset_state(include=['object', 'gripper', 'plane'], object_urdf_path=object_urdf_path, object_pos=stable_pos, object_orn=stable_orn, gripper_h=0.5)
    # Wrap then launch wrapper
    routine_wrapper = Dummy_Grasp_Routine_Wrapper(sim=sim)
    routine_wrapper.R_grasp_object()