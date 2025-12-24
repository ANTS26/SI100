###
# MAC version of Inverse Kinematics practice with MuJoCo
# SI100B Robotics Programming
# This code is modified based on the MuJoCo template code at https://github.com/pab47/pab47.github.io/tree/master.
# Date: Dec., 2025
###

import mujoco as mj
from mujoco.glfw import glfw
import mujoco.viewer
import numpy as np
import os
import scipy as sp
import time

xml_path = 'scene.xml' #xml file (assumes this is in the same folder as this file)
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Helper function
def random_cube_apex(
    cx, cy, cz,
    cube_size=0.1
):
    """
    Randomly returns the position of one apex of a cube in 3D space.

    Parameters
    ----------
    cube_size : float
        Edge length of the cube.
    center_range : tuple of 3 tuples
        Sampling range for cube center in x, y, z.

    Returns
    -------
    apex : np.ndarray, shape (3,)
        3D position of a randomly chosen cube apex.
    """
    # Random cube center
    center = np.array([cx, cy, cz])

    # Half edge length
    h = cube_size / 2.0

    # All 8 apex offsets
    offsets = np.array([
        [ h,  h,  h],
        [ h,  h, -h],
        [ h, -h,  h],
        [ h, -h, -h],
        [-h,  h,  h],
        [-h,  h, -h],
        [-h, -h,  h],
        [-h, -h, -h],
    ])

    # Randomly select one apex
    apex_offset = offsets[np.random.randint(0, 7)]
    apex = center + apex_offset

    return apex

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

# Get the full path
dirname = os.path.dirname(__file__)
abspath = xml_path = os.path.join(dirname, "..", "..", "models", "universal_robots_ur5e", xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Initialize the controller
init_controller(model,data)

# Set the controller
mj.set_mjcb_control(controller)

# Initialize home position
key_frame_index = model.key("home").id
mujoco.mj_resetDataKeyframe(model, data, key_frame_index)
mujoco.mj_forward(model, data)
q_home = data.qpos.copy()

############################################################
##  TRY TO CHANGE THESE VALUES TO SEE DIFFERENT BEHAVIORS ##
##  REMEMBER TO KEEP WITHIN THE ROBOT'S WORKSPACE!        ##
############################################################
x_ref = 0.5
y_ref = 0.2
z_ref = 0.3
phi_ref = 3.14
theta_ref = 0
psi_ref = 0
############################################################

apex_ref = np.array([x_ref,y_ref,z_ref])
alter_flag = True
start_time = time.time()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        # Get current end-effector position and orientation
        position_Q = data.site_xpos[0]
        mat_Q = data.site_xmat[0]
        quat_Q = np.zeros(4)
        mj.mju_mat2Quat(quat_Q, mat_Q)
        r_Q = sp.spatial.transform.Rotation.from_quat([quat_Q[1],quat_Q[2],quat_Q[3],quat_Q[0]])
        euler_Q = r_Q.as_euler('xyz')
        
        # Compute Jacobian J
        # void mj_jac(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr,
        #     const mjtNum point[3], int body);
        jacp = np.zeros((3, 6))
        jacr = np.zeros((3, 6))
        mj.mj_jac(model, data, jacp, jacr, position_Q, 7)

        # Compute inverse Jacobian Jinv
        J = np.vstack((jacp, jacr))
        Jinv = np.linalg.pinv(J)

        # Compute dX
        if (int(start_time - time.time())%5==0) and (alter_flag==True):
            apex_ref = random_cube_apex(x_ref,y_ref,z_ref)
            alter_flag = False
        if (int(start_time - time.time())%5!=0):
            alter_flag = True
            
        X_ref = np.array([apex_ref[0],apex_ref[1],apex_ref[2],phi_ref,theta_ref,psi_ref])
        X = np.concatenate((position_Q, euler_Q))
        dX = X_ref - X

        # Compute dq = Jinv * dX
        dq = Jinv.dot(dX)

        q_home += dq
        data.qpos = q_home.copy()
        mj.mj_forward(model, data)
        viewer.sync()
        
        time_until_next_step = 1/60 - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
            