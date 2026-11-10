"""Displacement over 15 s for fixed commands, in the policy's own TRAINING env (pushes/curricula off,
final command curriculum stage), to tell 'never learned to track' from 'sim gap'."""
import sys, math, numpy as np, torch, onnxruntime as ort
import mjlab_microduck.tasks  # noqa
from mjlab.tasks.registry import load_env_cfg
from mjlab.envs import ManagerBasedRlEnv
onnx = sys.argv[1]; N = 64; T = 15.0
cfg = load_env_cfg("Mjlab-Velocity-Flat-JDuck"); cfg.scene.num_envs = N; cfg.episode_length_s = T + 2
for k in [k for k in cfg.events if "push" in k]: cfg.events[k].interval_range_s = (1e6, 1e6)
cfg.curriculum = {}
for k in list(cfg.terminations):
    if k not in ("nan_state", "time_out"): cfg.terminations.pop(k)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); r = env.scene["robot"]; dev = env.device
ct = env.command_manager.get_term("twist")
s = ort.InferenceSession(onnx); nm = s.get_inputs()[0].name
pol = lambda o: np.concatenate([s.run(None, {nm: o[i:i+1]})[0] for i in range(len(o))])
for name, c in (("前进 0.08", (0.08, 0, 0)), ("后退 0.08", (-0.08, 0, 0)), ("左移 0.05", (0, 0.05, 0)), ("前进 0.12", (0.12, 0, 0))):
    env.reset(); obs = env.observation_manager.compute()
    p0 = r.data.root_link_pos_w[:, :2].clone()
    q = r.data.root_link_quat_w; yaw0 = torch.atan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]), 1-2*(q[:,2]**2+q[:,3]**2))
    for t in range(int(T / env.step_dt)):
        ct.vel_command_b[:] = torch.tensor(c, device=dev)
        obs, *_ = env.step(torch.from_numpy(pol(obs["actor"].cpu().numpy().astype(np.float32))).to(dev))
    d = r.data.root_link_pos_w[:, :2] - p0
    lx = torch.cos(yaw0) * d[:, 0] + torch.sin(yaw0) * d[:, 1]; ly = -torch.sin(yaw0) * d[:, 0] + torch.cos(yaw0) * d[:, 1]
    print(f"训练环境 {name}: 起始朝向下 前后 {lx.median().item():+.2f} m 左右 {ly.median().item():+.2f} m（理想 {c[0]*T:+.1f},{c[1]*T:+.1f}）")
