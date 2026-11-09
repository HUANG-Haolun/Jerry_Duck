"""Robust get-up search IN the training env: CEM over keyframe sequences, each candidate run on
E envs at once (training DR + smoothed action noise), fitness = fraction upright at the end.
    cem_env.py down|up [noise] [gens] [init.json]"""
import os, sys, json, math, numpy as np, torch
import mjlab_microduck.tasks
from mjlab.tasks.registry import load_env_cfg
from mjlab.envs import ManagerBasedRlEnv
from mjlab_microduck.tasks import mdp as M
START = sys.argv[1]; ROLL = START == "roll"; START = "up" if ROLL else START; NOISE = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03
GENS = int(sys.argv[3]) if len(sys.argv) > 3 else 80
P, E = 32, 32; N = P * E; NK = 4; NP = 10          # 4 keyframes, asymmetric legs + neck + head pitch
HOME = np.array([0,0,0.25,-0.30,0.05, 0,0,0,0, 0,0,-0.25,0.30,-0.05])
cfg = load_env_cfg("Mjlab-StandUp-Flat-JDuck"); cfg.scene.num_envs = N; cfg.episode_length_s = 30
for k in [k for k in cfg.events if "push" in k]: cfg.events[k].interval_range_s = (1e6, 1e6)
for k in ("set_ground_state", "spawn_from_replay"): cfg.events.pop(k, None)
cfg.curriculum = {k: v for k, v in cfg.curriculum.items() if "ground_state" not in k and "push" not in k}
for k in list(cfg.terminations):
    if k != "nan_state": cfg.terminations.pop(k)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); r = env.scene["robot"]; dev = env.device
lo = np.array([-2.48,-2.05,-3.73,-1.52,-1.47,-1.50,-1.98,-0.81,-0.30,-0.46,-0.41,-0.44,-2.15,-1.83]) + 0.02
hi = np.array([0.45,0.48,0.47,2.18,1.85,1.97,2.00,0.64,0.25,2.46,2.15,3.77,1.53,1.52]) - 0.02
def poses(x):                        # x: (NK*NP + NK) → list of (q, dur)
    out = []
    for k in range(NK):
        p = x[k*NP:(k+1)*NP]; q = HOME.copy(); q[1:5] = p[0:4]; q[10:14] = p[4:8]; q[5], q[6] = p[8], p[9]
        out.append((np.clip(q, lo, hi), 0.3 + abs(x[NK*NP + k])))
    return out + [(HOME, 1.0)]
def plan(x):
    tr = []; qc = HOME.copy()
    for g, du in poses(x):
        n = int(du / 0.02); q0 = qc.copy()
        for t in range(n): qc = q0 + min(1, (t + 1) / (0.6 * n)) * (g - q0); tr.append(qc.copy())
    return np.array(tr)
def evaluate(X):
    trs = [plan(x) for x in X]; T = max(len(t) for t in trs) + 75
    A = np.zeros((P, T, 14)) + HOME
    for i, t in enumerate(trs): A[i, :len(t)] = t
    A = torch.tensor(A - HOME, dtype=torch.float32, device=dev).repeat_interleave(E, 0)   # (N, T, 14)
    env.reset(); ids = torch.arange(N, device=dev)
    M.maybe_set_random_prone_orientation(env, ids, prone_prob=1.0, face_down_prob=1.0 if START == "down" else 0.0,
                                         side_prob=0.0, prone_z_min=0.035, prone_z_max=0.05 if START == "down" else 0.114)
    for _ in range(25): env.step(torch.zeros(N, 14, device=dev))
    drift = torch.zeros(N, 14, device=dev); best = torch.zeros(N, device=dev)
    for t in range(T):
        drift = 0.9 * drift + NOISE * 0.436 * torch.randn(N, 14, device=dev)   # stationary std = NOISE
        env.step(A[:, t] + drift)
    z = r.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]; g = -r.data.projected_gravity_b[:, 2]
    if ROLL:
        gb = r.data.projected_gravity_b            # face down (pitched +90°): gravity along body −x? measured below
        fd = (gb[:, 0] * FD_SIGN > 0.8).float()
        up = fd.view(P, E).mean(1); shaped = (gb[:, 0] * FD_SIGN).view(P, E).mean(1) * 0.1
        return (up + shaped).cpu().numpy(), up.cpu().numpy()
    up = ((z > 0.122) & (g > 0.9)).float().view(P, E).mean(1)
    shaped = (-(z - 0.133).abs() * 20 + g).view(P, E).mean(1) * 0.1
    return (up + shaped).cpu().numpy(), up.cpu().numpy()
FD_SIGN = 1.0
if ROLL:   # find which sign of body-x gravity means face down
    env.reset(); ids = torch.arange(N, device=dev)
    M.maybe_set_random_prone_orientation(env, ids, prone_prob=1.0, face_down_prob=1.0, side_prob=0.0, prone_z_min=0.035, prone_z_max=0.05)
    for _ in range(25): env.step(torch.zeros(N, 14, device=dev))
    FD_SIGN = float(torch.sign(r.data.projected_gravity_b[:, 0].mean())); print("face-down gravity_x sign", FD_SIGN, r.data.projected_gravity_b[0].tolist())
rng = np.random.default_rng(0)
if len(sys.argv) > 4:
    mu = np.array(json.load(open(sys.argv[4])))
    sd = np.full(len(mu), 0.25)
else:
    if START == "down":
        k0 = [0, -1.25, -2.23, 2.37, 0, 1.25, 2.23, -2.37, 1.43, -0.04]
    else:
        k0 = [0, 0.45, -1.4, 0.9, 0, -0.45, 1.4, -0.9, -1.0, 0]
    mu = np.array(k0 * NK + [0.8] * NK); sd = np.array([0.5, 1.0, 1.0, 1.0] * 2 + [1.0, 1.0]) ; sd = np.r_[np.tile(sd, NK), [0.4] * NK]
best_f, best_x = -9, mu
for gen in range(GENS):
    Xs = mu + sd * rng.standard_normal((P, len(mu))); Xs[0] = best_x
    f, up = evaluate(Xs); i = np.argsort(f)[::-1]
    if f[i[0]] > best_f: best_f, best_x = f[i[0]], Xs[i[0]].copy()
    el = Xs[i[:6]]; mu = el.mean(0); sd = 0.6 * sd + 0.4 * el.std(0) + 0.03
    print(f"gen {gen:3d} 最佳站起率 {up[i[0]]:.2f}（候选 0 = 历史最佳: {up[0]:.2f}） 前 6 平均 {up[i[:6]].mean():.2f}", flush=True)
    json.dump(best_x.tolist(), open(os.path.expanduser("~/jduck_ops/") + f"cem_env_{'roll' if ROLL else START}.json", "w"))
print("done", best_f)
