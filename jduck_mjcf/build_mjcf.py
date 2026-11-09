"""Build the jduck MJCF (robot_walk.xml + scene_walk.xml) from the jduck URDF.

Re-run whenever the URDF changes:

    cd third_party/microduck_rl
    uv run --frozen python ../../jduck_mjcf/build_mjcf.py

robot_walk.xml follows microduck_rl's "walk" collision family. Only the feet collide
with the floor. Legs and torso keep mesh geoms that collide with each other
(self-collision sensor) but not with the floor. Everything else is visual only.

robot_groundcontact.xml (+ scene_groundcontact.xml) is the same robot plus a
`<link>_ground_collision` mesh copy of the torso, hip-pitch, knee and head links that
also collides with the floor, like microduck's "groundcontact" family. It is for tasks
where the body can touch the ground (sit/stand, falls). Walking keeps robot_walk.xml.

Names follow what microduck_rl's training code hard-codes, so its velocity
recipe can be reused unchanged:
  - root body `trunk_base` (URDF `base_link`), with a freejoint
  - joints without the `_joint` suffix: left_hip_yaw, ..., head_roll
  - `mouth_joint` becomes `passive_mouth` (unactuated, matched out by `^(?!passive_)`)
  - foot sites `left_foot` / `right_foot`, foot geoms `left_foot_collision` /
    `right_foot_collision`
  - `imu` site at the URDF IMU_link pose, plus the same sensors as microduck
"""

from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
URDF = HERE.parent / "jduck_description" / "urdf" / "jduck_description.urdf"
MESH_DIR = HERE.parent / "jduck_description" / "meshes"
OUT_ROBOT = HERE / "robot_walk.xml"
OUT_SCENE = HERE / "scene_walk.xml"
OUT_VIEW_ROBOT = HERE / "robot_walk_view.xml"
OUT_VIEW_SCENE = HERE / "scene_view.xml"
OUT_GC_ROBOT = HERE / "robot_groundcontact.xml"
OUT_GC_SCENE = HERE / "scene_groundcontact.xml"

# Links that also get a floor-colliding mesh copy in robot_groundcontact.xml (what can
# touch the ground in a squat, a sit or a fall). contype/conaffinity bit 1 = the floor;
# bit 2 is kept so they still count in the self-collision sensor.
GROUND_CONTACT_LINKS = {
    "base_link", "left_hip_pitch_link", "left_knee_link", "right_hip_pitch_link", "right_knee_link",
    "neck_pitch_link", "head_pitch_link",
}
VIEW_KP = 8.0
VIEW_KD = 0.2

# Joint order = microduck's actuator order (ctrl idx == joint idx).
SERVO_JOINTS = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]

# Placeholder servo model until UBT-12HC bench identification. These are only
# the XML defaults; the training cfg sets the actuator (kp/kd/effort) itself.
JOINT_ARMATURE = 0.005     # reflected rotor inertia [kg m^2] — guess, to be identified
JOINT_FRICTIONLOSS = 0.02  # [N m] — guess
JOINT_DAMPING = 0.05       # [N m s/rad] — guess (DC-motor back-EMF scale)

# Foot collision box, in the ankle link frame. Sized from left_ankle_link.STL bounds
# (x [-0.024, 0.066], y [-0.003, 0.039], z [-0.026, 0.013]); the sole is the -z face.
FOOT_BOX_HALF = (0.045, 0.021, 0.004)
FOOT_BOX_POS_LEFT = (0.0206, 0.0179, -0.0222)
FOOT_SITE_POS_LEFT = (0.0206, 0.0179, -0.0262)

# Links whose mesh geoms take part in self-collision (legs + torso), as in
# microduck's walk model (leg, leg_2, power_support).
SELF_COLLISION_LINKS = {
    "base_link", "left_hip_pitch_link", "left_knee_link",
    "right_hip_pitch_link", "right_knee_link",
}


def _urdf_for_mujoco() -> str:
    text = URDF.read_text()
    text = text.replace("package://jduck_description/meshes/", f"{MESH_DIR}/")
    # Keep both visual and collision geoms, and keep fixed links (IMU) as bodies.
    return re.sub(
        r'(<robot\s+name="[^"]*"\s*>)',
        r'\1<mujoco><compiler discardvisual="false" fusestatic="false" strippath="false"/></mujoco>',
        text,
        count=1,
    )


def _rename(name: str) -> str:
    if name == "base_link":
        return "trunk_base"
    if name == "mouth_joint":
        return "passive_mouth"
    if name.endswith("_joint"):
        return name[: -len("_joint")]
    return name


def build() -> None:
    tmp = HERE / ".tmp_jduck.urdf"
    tmp.write_text(_urdf_for_mujoco())
    try:
        spec = mujoco.MjSpec.from_file(str(tmp))
    finally:
        tmp.unlink()

    spec.modelname = "jduck"
    spec.compiler.meshdir = os.path.relpath(MESH_DIR, HERE)
    for mesh in spec.meshes:
        mesh.file = os.path.basename(mesh.file)

    # --- defaults / classes -------------------------------------------------
    main = spec.default
    vis = spec.add_default("visual", main)
    vis.geom.contype = 0
    vis.geom.conaffinity = 0
    vis.geom.group = 2
    vis.geom.density = 0
    col = spec.add_default("collision", main)
    col.geom.group = 3
    selfc = spec.add_default("self_collision_only", main)
    selfc.geom.group = 3
    selfc.geom.contype = 2      # collides with other contype-2 geoms only
    selfc.geom.conaffinity = 2
    selfc.geom.density = 0
    servo = spec.add_default("servo_joint", main)
    servo.joint.armature = JOINT_ARMATURE
    servo.joint.frictionloss = JOINT_FRICTIONLOSS
    servo.joint.damping = [JOINT_DAMPING, 0, 0]
    passive = spec.add_default("passive_joint", main)
    passive.joint.armature = 1e-4
    passive.joint.damping = [0.01, 0, 0]

    # --- bodies, joints, geoms ---------------------------------------------
    for body in spec.bodies[1:]:
        urdf_name = body.name
        body.name = _rename(urdf_name)
        # URDF importer emits two geoms per link: visual (contype 0) then collision.
        for g in list(body.geoms):
            if g.contype == 0 and g.conaffinity == 0:
                g.classname = vis
                g.contype, g.conaffinity, g.group, g.density = 0, 0, 2, 0
            elif urdf_name in SELF_COLLISION_LINKS or urdf_name in GROUND_CONTACT_LINKS:
                if urdf_name in SELF_COLLISION_LINKS:
                    g.classname = selfc
                    g.contype, g.conaffinity, g.group, g.density = 2, 2, 3, 0
                    g.name = f"{body.name}_self_collision"
                else:                       # head links: kept only for the groundcontact model
                    g.classname = selfc
                    g.contype, g.conaffinity, g.group, g.density = 0, 0, 3, 0
                    g.name = f"{body.name}_ground_collision"
            else:
                spec.delete(g)
        for j in body.joints:
            j.name = _rename(j.name)
            cls = passive if j.name.startswith("passive_") else servo
            j.classname = cls
            # The URDF importer already wrote explicit per-joint values; copy the
            # class values over so the XML is self-consistent.
            j.armature = cls.joint.armature
            j.frictionloss = cls.joint.frictionloss
            j.damping = cls.joint.damping

    trunk = spec.body("trunk_base")
    trunk.pos = (0, 0, 0.18)
    trunk.add_freejoint(name="trunk_base_freejoint")  # same name as microduck (infer_policy.py looks it up)

    # Feet: simple boxes on the ankle links, plus sites at the sole.
    for side, sgn in (("left", 1.0), ("right", -1.0)):
        ankle = spec.body(f"{side}_ankle_link")
        bx, by, bz = FOOT_BOX_POS_LEFT
        sx, sy, sz = FOOT_SITE_POS_LEFT
        ankle.add_geom(
            name=f"{side}_foot_collision",
            default=col,
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=FOOT_BOX_HALF,
            pos=(bx, sgn * by, bz),
            group=3,
            density=0,
            rgba=(0.2, 0.6, 0.2, 0.4),
        )
        ankle.add_site(name=f"{side}_foot", pos=(sx, sgn * sy, sz), group=3)

    # IMU site at the URDF IMU_link pose (child of trunk_base); drop the body itself.
    imu_body = spec.body("IMU_link")
    imu_pos = np.array(imu_body.pos)
    imu_mass = imu_body.mass
    spec.delete(imu_body)
    trunk.add_site(name="imu", pos=imu_pos, group=3)
    trunk.mass += imu_mass

    # Sensors, same set as microduck's sensors.xml.
    spec.add_sensor(name="orientation", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu", noise=0.001)
    spec.add_sensor(name="angular-velocity", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu", noise=0.005)
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_accel", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="root_angmom", type=mujoco.mjtSensor.mjSENS_SUBTREEANGMOM,
                    objtype=mujoco.mjtObj.mjOBJ_BODY, objname="trunk_base")

    # Self-collision exclusions between adjacent links happen automatically
    # (parent/child). Exclude the hip-pitch <-> trunk pair explicitly: they
    # overlap at the hip in the CAD mesh and would always report contact.
    for side in ("left", "right"):
        spec.add_exclude(bodyname1="trunk_base", bodyname2=f"{side}_hip_pitch_link")

    # --- groundcontact variant: written before the viewer actuators are added -------
    gc = spec.copy()
    for b in gc.bodies:
        for g in list(b.geoms):
            urdf_link = None
            if g.name.endswith("_self_collision"):
                urdf_link = g.name[: -len("_self_collision")]
                base = urdf_link
            elif g.name.endswith("_ground_collision"):
                base = g.name[: -len("_ground_collision")]
                g.contype, g.conaffinity = 1, 1          # head shells: floor only
                continue
            else:
                continue
            urdf_name = "base_link" if base == "trunk_base" else base
            if urdf_name in GROUND_CONTACT_LINKS:
                twin = b.add_geom()
                twin.name = f"{base}_ground_collision"
                for attr in ("type", "meshname", "pos", "quat", "size"):
                    setattr(twin, attr, getattr(g, attr))
                twin.contype, twin.conaffinity, twin.group, twin.density = 1, 1, 3, 0
                twin.condim = 3
                twin.rgba = (0.8, 0.4, 0.2, 0.3)
    # bodies that touch each other at the joints would report floor-type contacts with
    # each other too (bit 1 on both sides); exclude the adjacent pairs that overlap.
    for side in ("left", "right"):
        gc.add_exclude(bodyname1=f"{side}_hip_pitch_link", bodyname2=f"{side}_knee_link")
    gc.modelname = "jduck_groundcontact"
    gc_model = gc.compile()
    OUT_GC_ROBOT.write_text(gc.to_xml())
    OUT_GC_SCENE.write_text(SCENE_TEMPLATE.replace("robot_walk.xml", OUT_GC_ROBOT.name))
    print(f"wrote {OUT_GC_ROBOT}  ngeom={gc_model.ngeom} (floor-colliding: "
          f"{sum(1 for i in range(gc_model.ngeom) if gc_model.geom_contype[i] & 1)})")
    # the head shells were only kept for the groundcontact copy: drop them from the walk model
    for b in spec.bodies:
        for g in list(b.geoms):
            if g.name.endswith("_ground_collision"):
                spec.delete(g)

    # robot_walk.xml has NO actuators: mjlab's actuator cfg (jduck_constants)
    # creates them, and a duplicate name would fail to compile.
    model = spec.compile()
    OUT_ROBOT.write_text(spec.to_xml())
    print(f"wrote {OUT_ROBOT}  nbody={model.nbody} njnt={model.njnt} nu={model.nu} "
          f"mass={model.body_subtreemass[1]:.4f}")

    OUT_SCENE.write_text(SCENE_TEMPLATE)
    print(f"wrote {OUT_SCENE}")

    # scene_view.xml = scene + placeholder position actuators (kp=8, kd=0.2,
    # 0.8 N·m, same as jduck_constants) for standalone MuJoCo viewing and
    # CPU rollouts (e.g. infer_policy-style checks). Not used by training.
    for jn in SERVO_JOINTS:
        spec.add_actuator(
            name=jn, target=jn, trntype=mujoco.mjtTrn.mjTRN_JOINT,
            gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[VIEW_KP] + [0] * 9,
            biastype=mujoco.mjtBias.mjBIAS_AFFINE, biasprm=[0, -VIEW_KP, -VIEW_KD] + [0] * 7,
            ctrllimited=1, ctrlrange=spec.joint(jn).range,
            forcelimited=1, forcerange=(-0.8, 0.8),
        )
    OUT_VIEW_ROBOT.write_text(spec.to_xml())
    OUT_VIEW_SCENE.write_text(SCENE_TEMPLATE.replace("robot_walk.xml", OUT_VIEW_ROBOT.name))
    print(f"wrote {OUT_VIEW_SCENE} (with {len(SERVO_JOINTS)} viewer actuators)")


SCENE_TEMPLATE = """<mujoco model="jduck scene">
  <include file="robot_walk.xml"/>
  <option timestep="0.005"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="160" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
             rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""


if __name__ == "__main__":
    build()
