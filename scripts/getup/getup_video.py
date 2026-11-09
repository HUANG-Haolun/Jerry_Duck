"""Render a get-up from face down / face up / side with the training env (one env), to mp4."""
import sys, math, numpy as np, torch, onnxruntime as ort, mujoco, imageio
import mjlab_microduck.tasks
from mjlab.tasks.registry import load_env_cfg
from mjlab.envs import ManagerBasedRlEnv
from mjlab_microduck.tasks import mdp as M
task, onnx, out = sys.argv[1], sys.argv[2], sys.argv[3]
cfg = load_env_cfg(task); cfg.scene.num_envs = 1; cfg.episode_length_s = 60
for k in [k for k in cfg.events if any(x in k for x in ("push", "topple"))]: cfg.events[k].interval_range_s = (1e6, 1e6)
for k in ("set_ground_state", "random_prone_init", "spawn_from_replay"): cfg.events.pop(k, None)
cfg.curriculum = {}
for k in list(cfg.terminations):
    if k != "nan_state": cfg.terminations.pop(k)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); r = env.scene["robot"]; s = ort.InferenceSession(onnx)
ct = env.command_manager.get_term("twist")
m = env.sim.mj_model; d = mujoco.MjData(m); ren = mujoco.Renderer(m, 360, 480)
cam = mujoco.MjvCamera(); cam.distance = 0.6; cam.elevation = -15; cam.azimuth = 120
frames = []
for state in ("face_down", "face_up", "side"):
    env.reset(); ids = torch.arange(1, device=env.device)
    M.maybe_set_random_prone_orientation(env, ids, prone_prob=1.0, face_down_prob=1.0 if state == "face_down" else 0.0,
        side_prob=1.0 if state == "side" else 0.0, prone_z_min=0.035, prone_z_max=0.114)
    obs = env.observation_manager.compute()
    for t in range(200):
        ct.vel_command_b[:] = 0
        a = s.run(None, {s.get_inputs()[0].name: obs["actor"].cpu().numpy().astype(np.float32)})[0]
        obs, *_ = env.step(torch.from_numpy(a).to(env.device))
        if t % 2 == 0:
            d.qpos[:] = env.sim.data.qpos[0].cpu().numpy(); d.qvel[:] = 0; mujoco.mj_forward(m, d)
            cam.lookat[:] = d.qpos[:3]; cam.lookat[2] = 0.07
            ren.update_scene(d, cam); frames.append(ren.render())
imageio.mimsave(out, frames, fps=25)
print("saved", out, len(frames))
