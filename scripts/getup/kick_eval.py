"""Ball-kick battery in the training env (pushes off): 256 episodes of 5 s, ball placed with the
training DR (±1.5 cm). Reports peak ball speed, distance and direction error, how many fell,
how many lifted the support foot, and when the kick happened.
    kick_eval.py <task> <onnx>      (export the run's JDUCK_* vars first)
"""
import sys, math, numpy as np, torch, onnxruntime as ort
import mjlab_microduck.tasks  # noqa: F401
from mjlab.tasks.registry import load_env_cfg
from mjlab.envs import ManagerBasedRlEnv

task, onnx = sys.argv[1], sys.argv[2]
N = 256
cfg = load_env_cfg(task); cfg.scene.num_envs = N
for k in [k for k in cfg.events if "push" in k]: cfg.events[k].interval_range_s = (1e6, 1e6)
cfg.curriculum = {}
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); dev = env.device
r, b = env.scene["robot"], env.scene["ball"]
s = ort.InferenceSession(onnx); name = s.get_inputs()[0].name
batched = isinstance(s.get_inputs()[0].shape[0], str)
pol = (lambda o: s.run(None, {name: o})[0]) if batched else \
      (lambda o: np.concatenate([s.run(None, {name: o[i:i + 1]})[0] for i in range(len(o))]))
env.reset(); obs = env.observation_manager.compute()
q = r.data.root_link_quat_w
yaw0 = torch.atan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
fwd = torch.stack([torch.cos(yaw0), torch.sin(yaw0)], 1)
b0 = b.data.root_link_pos_w[:, :2].clone()
vmax = torch.zeros(N, device=dev); t_kick = torch.full((N,), float("nan"), device=dev)
fell = torch.zeros(N, dtype=torch.bool, device=dev); lifted = torch.zeros(N, device=dev)
done_any = torch.zeros(N, dtype=torch.bool, device=dev)
sup = env.scene.sensors["support_foot_ground_contact"]
T = int(cfg.episode_length_s / env.step_dt) - 1
for t in range(T):
    o = obs["actor"].cpu().numpy().astype(np.float32)
    obs, _, term, trunc, _ = env.step(torch.from_numpy(pol(o)).to(dev))
    alive = ~done_any
    v = b.data.root_link_lin_vel_w[:, :2].norm(dim=1)
    vmax = torch.where(alive, torch.maximum(vmax, v), vmax)
    t_kick = torch.where(alive & torch.isnan(t_kick) & (v > 0.2), torch.full_like(t_kick, t * env.step_dt), t_kick)
    g = -r.data.projected_gravity_b[:, 2]
    fell |= alive & (g < math.cos(math.radians(50)))
    f = sup.data.found
    f = f.sum(-1) if f.dim() > 1 else f
    lifted += (alive & (f < 0.5)).float()
    done_any |= (term | trunc).bool()
d = b.data.root_link_pos_w[:, :2] - b0
dist = d.norm(dim=1)
ang = torch.rad2deg(torch.atan2(fwd[:, 0] * d[:, 1] - fwd[:, 1] * d[:, 0], (fwd * d).sum(1)))
kicked = vmax > 0.2
pc = lambda x: np.percentile(x.cpu().numpy(), [25, 50, 75]).round(2)
print(f"=== {onnx.split('/')[-1]}  ({N} 次, 5 s, 推力关闭)")
print(f"  踢到球（球速 > 0.2 m/s）: {kicked.float().mean() * 100:.0f}%   踢球时刻中位 {np.nanmedian(t_kick.cpu().numpy()):.2f} s")
print(f"  最大球速 m/s  25/50/75 分位: {pc(vmax[kicked]) if kicked.any() else '-'}   （训练目标 1.0）")
print(f"  球滚的距离 m  25/50/75 分位: {pc(dist[kicked]) if kicked.any() else '-'}")
print(f"  方向偏差（相对机器人正前方）° 25/50/75: {pc(ang[kicked]) if kicked.any() else '-'}")
print(f"  摔倒: {fell.float().mean() * 100:.0f}%   支撑脚离地的平均时间: {lifted.mean().item() * env.step_dt:.2f} s")
