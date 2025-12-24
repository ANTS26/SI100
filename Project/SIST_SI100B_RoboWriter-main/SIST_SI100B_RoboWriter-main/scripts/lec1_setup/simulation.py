import time

import mujoco.viewer
import mujoco
import numpy as np
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(script_dir, "..", "..", "models", "universal_robots_ur5e", "scene.xml")

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    key_frame_index = model.key("home").id
    mujoco.mj_resetDataKeyframe(model, data, key_frame_index)
    mujoco.mj_forward(model, data)
    default_ctrl = data.ctrl.copy()

    print("info about the model:")
    print("nq:", model.nq)
    print("nv:", model.nv)
    print("nu:", model.nu)
    print("na:", model.na)
    print("timestep:", model.opt.timestep)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # ctrl = np.zeros(model.nu)
            data.ctrl[:] = default_ctrl
            mujoco.mj_step(model, data)
            # print("State pos:", data.qpos[:])
            # print("State vel:", data.qvel[:])
            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("Simulation ended.")