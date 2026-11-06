"""用 pybullet GUI 查看 URDF，每个可动关节一个滑条。

用法（系统 python3，pybullet 装在 ~/.local，不需要 ROS）:
    python3 scripts/view_urdf.py jduck_description/urdf/jduck_description.urdf
"""
import os
import sys
import time

import pybullet as p

urdf = os.path.abspath(sys.argv[1])
# 让 package://<pkg>/meshes/... 能按 URDF 所在目录的上级解析
os.chdir(os.path.dirname(urdf))

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
robot = p.loadURDF(urdf, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

sliders = []
for j in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, j)
    if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        lo, hi = info[8], info[9]
        if lo >= hi:
            lo, hi = -3.14, 3.14
        sliders.append((j, p.addUserDebugParameter(info[1].decode(), lo, hi, 0.0)))

print(f"loaded {urdf}: {p.getNumJoints(robot)} joints, {len(sliders)} movable")
while p.isConnected():
    for j, s in sliders:
        p.resetJointState(robot, j, p.readUserDebugParameter(s))
    time.sleep(1 / 60)
