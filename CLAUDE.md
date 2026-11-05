# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**始终用中文回复用户。**（代码、命令、标识符保持原样。）

## What this workspace is

A robot-description + RL workspace (git repo; `third_party/*` are git submodules pointing at the HUANG-Haolun forks, with `upstream` remotes for pollen-robotics — clone with `--recurse-submodules`). It holds SolidWorks-exported robot models (via the SW2URDF exporter) converted into ROS 2 Humble packages, plus vendored upstream reference projects for RL control of a small biped.

- `jduck_description/` — **the current robot model and the target of the RL port.** It is the newer export of the user's Microduck-style biped, which was earlier called MetalDuck. ROS 2 `ament_cmake` package.
  - 15 revolute joints: 5 per leg (hip_yaw/hip_roll/hip_pitch/knee/ankle), neck_pitch, head_pitch/yaw/roll, and mouth.
  - A fixed `IMU_link` hangs off `base_link`.
  - Total mass is 1.366 kg (masses were measured).
  - Every joint has `effort=0.8` and `velocity=3.0`. These values are not yet verified against the UBT-12HC servos.
  - Joint limits are still the suspicious CAD defaults (e.g. hip_pitch [-3.4, 0.43]). They have to be replaced with calibrated ranges.
- `jduck_mjcf/` — MuJoCo model of jduck. It is **generated** by `build_mjcf.py` from the jduck URDF, so don't hand-edit the XMLs; edit the script and rerun it (`cd third_party/microduck_rl && uv run --frozen python ../../jduck_mjcf/build_mjcf.py`).
  - Uses microduck's naming so its velocity recipe can be reused: root body `trunk_base` with freejoint `trunk_base_freejoint` (same as microduck), joints without the `_joint` suffix, `passive_mouth`, `left_foot`/`right_foot` sites, `*_foot_collision` boxes, an `imu` site, and microduck's sensor set.
  - `robot_walk.xml`/`scene_walk.xml` have **no actuators**, because mjlab adds them and duplicate names fail. `scene_view.xml` adds placeholder position actuators for standalone viewing.
- Training task `Mjlab-Velocity-Flat-JDuck` lives on the `jduck` branch of `third_party/microduck_rl`, in `robot/jduck_constants.py`, `tasks/jduck_velocity_env_cfg.py` and `tests/test_jduck_cfg.py`. It reuses `make_microduck_velocity_env_cfg` and swaps in the robot, the head-CoM body list, the reset height, and smaller velocity ranges. **Its actuator is a placeholder PD** (kp=8, kd=0.2, 0.8 N·m, 1–3 step delay) until the UBT-12HC servos are identified. HOME (hip_pitch ±0.25, knee ∓0.30, ankle ±0.05) was found by a sim search inside the current, uncalibrated URDF limits.
- `jduck_tools/` — UBT-12HC servo protocol (`servo_bus.py`) and the joint calibration CLI (`calibrate.py`: scan / monitor / measure-k / joint / report / apply / verify). It runs on the system `python3`, which has pyserial and pyyaml. Its output is `calibration.yaml` (q = sign·(adc−adc_zero)·k), and `apply` writes the calibrated ranges into the jduck URDF, keeping CRLF and backing it up to `*.urdf.precal.bak`. `servo_map.yaml` (joint → servo ID) is a guess until the user confirms the wiring with `monitor`. Offline tests: `env -u PYTHONPATH python3 -m pytest jduck_tools/test_tools.py -q`. **Not yet run on real hardware.** See `jduck_tools/README.md`.
- `MetalDuck_description/` — the previous export of the same robot. Kinematics are identical, but masses are CAD estimates, there is no IMU, and every effort is 0. It is superseded by jduck and kept for reference only.
- `W01-00-000-0000-0831/W01-00-000-0000/` — a second, unrelated SW2URDF-exported robot (dual arms with grippers, head, legs; 40 joints). It is packaged the same way.
- `ros2_ws/` — colcon workspace. `ros2_ws/src/*` are **symlinks** to the package directories above, so you edit the top-level directories, not anything under `ros2_ws/src`.
- `view_urdf.py` — standalone PyBullet viewer with one slider per movable joint. It does not need ROS.
- `third_party/microduck` (Rust robot runtime) and `third_party/microduck_rl` (mjlab/MuJoCo PPO training, ONNX export) — upstream Pollen Robotics repos, vendored as shallow clones for reference. Each has its own `AGENTS.md`/`CLAUDE.md` with commands and invariants; read those before working inside them. `microduck_rl/AGENTS.md` defines the sim2real conventions (61D obs layout, joint ordering, BAM actuators, normalizer baked into ONNX) that any MetalDuck RL port should follow.

## Commands

**ROS environment: the system ROS 2 Jazzy at `/opt/ros/jazzy`, with system `/usr/bin/python3`. Do NOT use the conda `ros_humble` env; the user has said so explicitly.** In an interactive shell the user runs `use_ros_jazzy`, defined in `~/.bashrc`, which deactivates conda and sources Jazzy. Non-interactive shells (Claude's Bash tool) don't get that function. Run ROS commands in a clean env instead:

```bash
env -i HOME=$HOME USER=$USER DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash -c 'source /opt/ros/jazzy/setup.bash && <command>'
```

```bash
# Build and view in RViz (robot_state_publisher + joint_state_publisher_gui + rviz2)
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch jduck_description display.launch.py
ros2 launch W01-00-000-0000 display.launch.py

# Quick PyBullet view without ROS (pybullet 3.2.7 built from source into ~/.local for system python3)
python3 view_urdf.py jduck_description/urdf/jduck_description.urdf
```

`ros2_ws` was rebuilt from scratch with Jazzy on 2026-10-30. The earlier build had baked conda-Humble paths into `install/setup.sh`. If build/install ever get mixed between distros, delete `build install log` and rebuild. The colcon warning that `MetalDuck_description`/`W01-00-000-0000` don't follow lowercase naming is harmless.

The robot uses **UBT-12HC bus servos**, not XL330.
- The protocol spec is `~/Downloads/UBT12HC_总线舵机通讯协议说明.pdf`.
- Official spec (`~/Downloads/UBTECH Company Introduction.pdf` p.16): 0.238/0.198 s per 60° and 8/12 kg·cm at 6.5/8.5 V, which is ≈ 4.8 rad/s and 0.96 N·m at the rated 7.4 V.
- Stock firmware (`send_rev_230400_ult.ino`): `pwm = clamp(|err|, 48, 255)`, deadband 2; `SERVO_ID` is a compile-time `#define`. In sim it never learns to step.
- **Modified firmware `firmware/ubt12hc_pd/`**: `pwm = KP·|err| + FF`, default KP=16 FF=0 DB=0 (runtime-tunable via `jduck_tools/servo_params.py`, stored in EEPROM), 200 ms comm timeout. The protocol is unchanged. It is not yet compiled for AVR or tested on hardware; see its README for the flashing order (one servo first).
- In training, `JDUCK_FW=K,FF,DB JDUCK_NOLOAD=4.8 JDUCK_TORQUE=0.96` selects this firmware law with the official servo. Stepping only appears around iteration 330–400 and depends on the seed, so judge runs at ≥ 500 iterations.
- **Current best policy (2026-11-05):** `JDUCK_HOME=old JDUCK_FW=16,0,0 JDUCK_STAND_QUIET=4 JDUCK_STAND_QUIET_S2=1 JDUCK_ANG_EMA=0.5` (r7 = seed 2, r8 = seeds 1/3). Stands still at zero command (target change ≈0.02°/step) and turns in place. The same policy tolerates DB≤2 and FF≤4 at eval time; FF=8 makes it jitter again. FF>0/DB>0 on the old 16,15,x law made every policy jitter at zero speed.
- `infer_policy.py`/`jduck_eval.py` apply the actuator delay per PHYSICS step (1–3 × 5 ms, like training). Before 2026-11-04 they delayed 1–3 POLICY steps (20–60 ms), so evals from before then are not comparable.

The training repo uses `uv run ...` (see `third_party/microduck_rl/AGENTS.md`). It needs a CUDA GPU. Its `.venv` is already installed (torch 2.9.1+cu128 and warp 1.12 both work on the RTX 5090, sm_120). `uv` lives in `~/.local/bin`. Run it with `uv run --frozen ...` so it never re-resolves or re-downloads. PyPI is unreliable from this network, so install extra packages with `uv pip install --index-url https://mirrors.aliyun.com/pypi/simple/ <pkg>`. Do not run `uv sync` casually: it would remove the dev-only `pytest`/`plotly` installed into `.venv` and may try to re-download torch. If the shell has Jazzy sourced, its `PYTHONPATH` leaks ROS's `launch` package into pytest and breaks collection. Run tests as `env -u PYTHONPATH -u AMENT_PREFIX_PATH uv run --frozen pytest tests/ -q -p no:launch_testing`.

## Editor (Cursor) config

- `rl_control/.vscode/settings.json` sets the default interpreter to `/usr/bin/python3`, with `/opt/ros/jazzy` on the search path. It also disables Cursor Pyright (Pyrefly is the only language server) and points CMake at `/usr/bin/cmake` with `CMAKE_PREFIX_PATH=/opt/ros/jazzy`.
- `rl_control/pyrefly.toml` checks ROS-side code with system python and Jazzy site-packages, and excludes `third_party/`.
- `third_party/microduck_rl/pyrefly.toml` and `.vscode/settings.json` are local-only files (listed in that repo's `.git/info/exclude`). They use `.venv` and disable the noisy type-check categories that mjlab/torch dynamic attributes trigger.

## Things to know about the exported packages

- **Meshes use `package://<pkg>/meshes/...` URIs.** `view_urdf.py` resolves them by `chdir`ing to the URDF's directory. Do not rename a package directory or its `<name>` in `package.xml` without also updating the URDF mesh URIs, `CMakeLists.txt` `project()`, and `get_package_share_directory(...)` in `launch/display.launch.py`.
- `urdf/*.urdf.bak` holds the URDF before hand edits. The `.urdf` next to it is a hand-fixed copy; diff against the `.bak` to see the edits. Re-exporting from SolidWorks overwrites the `.urdf`, so those fixes have to be applied again.
  - MetalDuck: `mouth_link` had zero mass and zero inertia, so both were filled in.
  - jduck: the user entered measured masses in CAD, but the exporter left the old inertia tensors unchanged. Each link's inertia was therefore scaled by m_new / m_old, using MetalDuck's masses as m_old. This assumes uniform density, so it is approximate. The proper fix is to set mass properties per part in SolidWorks and re-export.
- **jduck's URDF uses CRLF line endings.** Edit it with `newline=''` in Python (or the Edit tool), or every line shows up in diffs.
- In MetalDuck's URDF, every joint `<limit>` has `effort="0" velocity="0"`, because the exporter leaves these as placeholders. jduck sets them to 0.8 / 3.0.
- `ros1_backup/` contains the original ROS 1 (catkin) exporter files, kept for reference only. The ROS 2 packaging (`CMakeLists.txt`, `package.xml`, `launch/display.launch.py`) was rewritten by hand. The first entry in `config/joint_names_*.yaml` is an empty string, which is an exporter artifact.
- `export.log` is the SW2URDF exporter log.
