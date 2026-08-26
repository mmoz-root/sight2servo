from pathlib import Path

import mujoco


MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "models"
    / "arm.xml"
)

model = mujoco.MjModel.from_xml_path(
    str(MODEL_PATH)
)

data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

end_effector_world_m = data.site(
    "end_effector"
).xpos

print(
    f"End-effector world position: "
    f"{end_effector_world_m}"
)

print(f"Joints: {model.njnt}")
print(f"Actuators: {model.nu}")
print(f"qpos: {data.qpos}")
print(f"qvel: {data.qvel}")
print(f"ctrl: {data.ctrl}")


data.ctrl[0] = 0.01

for _ in range(100):
    mujoco.mj_step(model, data)

print()
print("After stepping:")
print(f"time: {data.time}")
print(f"qpos: {data.qpos}")
print(f"qvel: {data.qvel}")
print(f"ctrl: {data.ctrl}")