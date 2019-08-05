## Parallel WSG-32 Gripper Simulation with a GUI wrapper and a Grasping-Routine wrapper

A pybullet simulation that loads the [WSG-32 parallel-finger gripper](https://www.weiss-robotics.com/en/produkte/gripping-systems/performance-line-en/wsg-32-en/). 

The gripper parts are taken from [this](https://code.google.com/archive/p/wsg50-ros-pkg/) repo. 

We package the gripper parts in a simulation with a flat table, and the possibility of loading any desired object in '.obj' format.

### How to run

We provide two wrappers that take in the provided simulation and implement abstract functionality.

Several example meshes are available for you to use in the './sample_raw_meshes' directory.

#### 1) The GUI wrapper
A wrapper that takes in a Simulation object and adds slider controls to the GUI, allowing realtime features such as:
  * Moving the gripper
  * Opening/closing the gripper
  * Manipulating the camera
  
Example command:
python gui_wrapper.py --obj_path ./sample_raw_meshes/elephant.obj

![GUI wrapper preview](https://github.mit.edu/jaks19/parallel_gripper_simulation_pybullet/blob/master/pics/pic_debug.png)

#### 2) The dummy grasp routine wrapper
One might want to implement a wrapper that performs a specific routine with the gripper, like grasping an object. 

This is an example of such a wrapper that takes in a Simulation object and implements primitive methods that allow controlling
the gripper modularly.

We provide primitives for performing the following sub-routines:
* Opening/closing the gripper
* Moving the gripper to a specific point in space
* Shaking the gripper

We combine those primitives into a grasping routine where the gripper starts high above the table, then it is lowered till it reached the surface, closes on the object and goes back up, before shaking to test the grasp goodness.

Note:
We also show, in the __main__ section for this wrapper, how to stabilize any loaded object at the center of the table. The strategy used is to load the object, record the pos and orn at which it stabililizes, then re-launch it at the stable pos and orn.

Example command:
python dummy_grasp_routine_wrapper.py --obj_path ./sample_raw_meshes/elephant.obj

![Dummy_grasp_routine_wrapper wrapper preview](https://github.mit.edu/jaks19/parallel_gripper_simulation_pybullet/blob/master/pics/pic_routine.png)


