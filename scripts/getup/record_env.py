"""Replay a cem_env plan (or roll plan, then down plan) in the training env with action noise and
record (qpos, qvel, demo action) per step for envs that end upright. Output npz with s (states),
a (actions), L (frames per replay).
    record_env.py out.npz down.json [roll.json] [noise]"""
import os, sys, json, math, numpy as np, torch
sys.argv_ = list(sys.argv)
out, down_js = sys.argv[1], sys.argv[2]
roll_js = sys.argv[3] if len(sys.argv) > 3 and sys.argv[3].endswith(".json") else None
NOISE = float(sys.argv[-1]) if not sys.argv[-1].endswith((".json", ".npz")) else 0.03
sys.argv = ["x", "up" if roll_js else "down", "0", "0"]
src = open(os.path.expanduser("~/jduck_ops/cem_env.py")).read().split("rng = np.random.default_rng(0)")[0]
src = src.replace("P, E = 32, 32", "P, E = 1, 512").replace('START = sys.argv[1]; ROLL = START == "roll"; START = "up" if ROLL else START;', 'START = sys.argv[1]; ROLL = False;')
exec(src)
tr = plan(np.array(json.load(open(down_js))))
if roll_js: tr = np.concatenate([plan(np.array(json.load(open(roll_js))))[:-0 or None], tr])
T = len(tr) + 200; A = np.zeros((T, 14)) + HOME; A[:len(tr)] = tr
A = torch.tensor(A - HOME, dtype=torch.float32, device=dev)
S, AC, OK = [], [], []
for rep in range(4):
    env.reset(); ids = torch.arange(N, device=dev)
    M.maybe_set_random_prone_orientation(env, ids, prone_prob=1.0, face_down_prob=0.0 if roll_js else 1.0,
                                         side_prob=0.0, prone_z_min=0.035, prone_z_max=0.114 if roll_js else 0.05)
    for _ in range(25): env.step(torch.zeros(N, 14, device=dev))
    drift = torch.zeros(N, 14, device=dev); st, ac = [], []
    for t in range(T):
        q = torch.as_tensor(env.sim.data.qpos[:]).clone(); q[:, :3] = q[:, :3] - env.scene.env_origins
        st.append(torch.cat([q, torch.as_tensor(env.sim.data.qvel[:]).clone()], 1)); ac.append(A[t].expand(N, 14).clone())
        drift = 0.9 * drift + NOISE * 0.436 * torch.randn(N, 14, device=dev)
        env.step(A[t] + drift)
    z = r.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]; g = -r.data.projected_gravity_b[:, 2]
    ok = (z > 0.122) & (g > 0.9); print(f"rep {rep}: upright {ok.float().mean():.2f}", flush=True)
    S.append(torch.stack(st, 1)[ok][:64]); AC.append(torch.stack(ac, 1)[ok][:64])
S = torch.cat(S).cpu().numpy(); AC = torch.cat(AC).cpu().numpy()
print("replays", S.shape)
np.savez(out, s=S.reshape(-1, S.shape[-1]), a=AC.reshape(-1, 14), L=S.shape[1], nq=env.sim.data.qpos.shape[1])
