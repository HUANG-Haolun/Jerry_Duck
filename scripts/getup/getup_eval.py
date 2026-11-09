"""Get-up battery in the task's own env (robot-style obs, firmware replica): spawn N envs in one
fallen/starting state, run the policy at zero command for T s, report how many end upright.
    getup_eval.py <task_id> <onnx> [T]
States: face_down, face_up, side, squat, standing. Upright = tilt < 25° and trunk z > 0.122."""
import sys, math, numpy as np, torch, onnxruntime as ort
import mjlab_microduck.tasks
from mjlab.tasks.registry import load_env_cfg
from mjlab.envs import ManagerBasedRlEnv
from mjlab_microduck.tasks import mdp as M
from mjlab_microduck.tasks.jduck_sitstand_env_cfg import JDUCK_SIT_OVERRIDES
task, onnx = sys.argv[1], sys.argv[2]; T = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
cfg = load_env_cfg(task); cfg.scene.num_envs = 128; cfg.episode_length_s = T + 2
for k in [k for k in cfg.events if any(x in k for x in ("push", "tip", "jerk", "topple"))]: cfg.events[k].interval_range_s = (1e6, 1e6)
for k in ("set_ground_state", "random_prone_init"): cfg.events.pop(k, None)
cfg.curriculum = {k: v for k, v in cfg.curriculum.items() if "prone" not in k and "ground_state" not in k and "push" not in k}
for k in ("fell_over", "fallen_too_long"): cfg.terminations.pop(k, None)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); r = env.scene["robot"]; s = ort.InferenceSession(onnx)
ct = env.command_manager.get_term("twist")
def spawn(state):
    env.reset(); ids = torch.arange(env.num_envs, device=env.device)
    if state == "standing": return
    if state.startswith("replay"):
        from mjlab_microduck.tasks.jduck_standup_env_cfg import spawn_from_replay
        import os
        spawn_from_replay(env, ids, 1.0, os.path.expanduser(f"~/jduck_ops/getup_states_{state[7:]}.npz"))
        for _ in range(10): env.step(torch.zeros(env.num_envs, 14, device=env.device))
        z0 = (r.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]).cpu().numpy()
        print(f"   [{state} 起始] 已站着 {np.mean(z0 > 0.122)*100:.0f}%"); return
    if state == "squat":
        M.set_random_ground_state(env, ids, face_down_prob=0, face_up_prob=0, sitting_prob=1, standing_prob=0,
            sitting_joint_overrides=JDUCK_SIT_OVERRIDES, sitting_joint_noise_std=0.03, sitting_tilt_max=math.radians(3),
            sitting_z_min=0.108, sitting_z_max=0.112, standing_z_min=0.136, standing_z_max=0.14)
    else:
        M.maybe_set_random_prone_orientation(env, ids, prone_prob=1.0, face_down_prob=1.0 if state == "face_down" else 0.0,
            side_prob=1.0 if state == "side" else 0.0, prone_z_min=0.035, prone_z_max=0.114)
    # report what the spawn actually is (after 0.2 s of settling with zero action)
    for _ in range(10): env.step(torch.zeros(env.num_envs, 14, device=env.device))
    z0 = (r.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]).cpu().numpy()
    t0 = np.degrees(np.arccos(np.clip(-r.data.projected_gravity_b[:, 2].cpu().numpy(), -1, 1)))
    print(f"   [{state} 起始] z {np.median(z0):.3f} 倾斜 {np.median(t0):.0f}°")
for state in (sys.argv[4].split(",") if len(sys.argv) > 4 else ("standing", "squat", "face_down", "face_up", "side")):
    spawn(state)
    obs = env.observation_manager.compute()
    t_up = np.full(env.num_envs, np.nan)
    for t in range(int(T / 0.02)):
        ct.vel_command_b[:] = 0
        o = obs["actor"].cpu().numpy().astype(np.float32)
        a = np.concatenate([s.run(None, {s.get_inputs()[0].name: o[i:i+1]})[0] for i in range(len(o))])
        obs, *_ = env.step(torch.from_numpy(a).to(env.device))
        z = (r.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]).cpu().numpy()
        tilt = np.degrees(np.arccos(np.clip(-r.data.projected_gravity_b[:, 2].cpu().numpy(), -1, 1)))
        up = (tilt < 25) & (z > 0.122)
        t_up = np.where(np.isnan(t_up) & up, t * 0.02, t_up)
    z = (r.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]).cpu().numpy()
    tilt = np.degrees(np.arccos(np.clip(-r.data.projected_gravity_b[:, 2].cpu().numpy(), -1, 1)))
    end_up = (tilt < 25) & (z > 0.122)
    print(f"{state:9s} 最后站着 {end_up.mean()*100:3.0f}%  中位起身用时 {np.nanmedian(t_up) if np.any(~np.isnan(t_up)) else float('nan'):.1f} s  终点 z {np.median(z):.3f} 倾斜 {np.median(tilt):.0f}°")
