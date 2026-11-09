"""DAgger for the get-up. Expert = nearest-neighbour lookup into the robust demo rollouts
(env_states_{down,up}.npz recorded in the training env): for a state, find the closest recorded
frame (joint angles + gravity in body frame), label = that frame's demo action k steps ahead.
Rounds: execute β·expert + (1−β)·policy, record (obs, expert label), retrain on the aggregate.
    dagger_getup.py <base.pt> <out.pt>"""
import sys, os, json, math, copy, numpy as np, torch
import mjlab_microduck.tasks
from mjlab.tasks.registry import load_env_cfg
from mjlab.envs import ManagerBasedRlEnv
from mjlab_microduck.tasks import mdp as M
base, out = sys.argv[1], sys.argv[2]; N = 1024; LOOK = 2
cfg = load_env_cfg("Mjlab-StandUp-Flat-JDuck"); cfg.scene.num_envs = N; cfg.episode_length_s = 40
for k in [k for k in cfg.events if "push" in k]: cfg.events[k].interval_range_s = (1e6, 1e6)
for k in ("set_ground_state", "spawn_from_replay"): cfg.events.pop(k, None)
cfg.curriculum = {k: v for k, v in cfg.curriculum.items() if "ground_state" not in k and "push" not in k}
for k in list(cfg.terminations):
    if k != "nan_state": cfg.terminations.pop(k)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0"); r = env.scene["robot"]; dev = env.device
nq = env.sim.data.qpos.shape[1]
servo = torch.tensor([i for i, n in enumerate(r.joint_names) if not n.startswith("passive_")], device=dev)
def feat_from_qpos(qpos):            # joint angles + body-frame gravity from quaternion
    w, x, y, z = qpos[:, 3], qpos[:, 4], qpos[:, 5], qpos[:, 6]
    gx = -2 * (x * z - w * y); gy = -2 * (y * z + w * x); gz = -(1 - 2 * (x * x + y * y))
    return torch.cat([qpos[:, 7:][:, servo], 3 * torch.stack([gx, gy, gz], 1)], 1)
DB_F, DB_A = [], []
for name in ("down", "up"):
    f = np.load(os.path.expanduser(f"~/jduck_ops/env_states_{name}.npz")); L = int(f["L"])
    S = torch.tensor(f["s"], device=dev).view(-1, L, f["s"].shape[1]); A = torch.tensor(f["a"], device=dev).view(-1, L, 14)
    Aahead = torch.cat([A[:, LOOK:], A[:, -1:].expand(-1, LOOK, -1)], 1)
    DB_F.append(feat_from_qpos(S[:, :, :nq].reshape(-1, nq))); DB_A.append(Aahead.reshape(-1, 14))
DB_F, DB_A = torch.cat(DB_F)[::2], torch.cat(DB_A)[::2]
print("expert db", DB_F.shape)
def expert():
    q = torch.as_tensor(env.sim.data.qpos[:]).clone()
    fq = feat_from_qpos(q); out = torch.empty(N, 14, device=dev)
    for i in range(0, N, 256):
        d = torch.cdist(fq[i:i+256], DB_F); out[i:i+256] = DB_A[d.argmin(1)]
    return out
ck = torch.load(base, map_location=dev, weights_only=False); sd = ck["actor_state_dict"]
mlp = torch.nn.Sequential(torch.nn.Linear(61, 512), torch.nn.ELU(), torch.nn.Linear(512, 256), torch.nn.ELU(), torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 14)).to(dev)
mlp.load_state_dict({k[4:]: v for k, v in sd.items() if k.startswith("mlp.")})
mu, std = sd["obs_normalizer._mean"], sd["obs_normalizer._std"]; norm = lambda o: (o - mu) / (std + 1e-2)
ref = copy.deepcopy(mlp).eval(); ct = env.command_manager.get_term("twist")
def spawn(kind):
    env.reset(); ids = torch.arange(N, device=dev)
    if kind != "standing":
        M.maybe_set_random_prone_orientation(env, ids, prone_prob=1.0, face_down_prob=1.0 if kind == "face_down" else 0.0,
            side_prob=1.0 if kind == "side" else 0.0, prone_z_min=0.035, prone_z_max=0.114)
    return env.observation_manager.compute()
def upright():
    z = r.data.root_link_pos_w[:, 2] - env.scene.terrain.env_origins[:, 2]; g = -r.data.projected_gravity_b[:, 2]
    return ((z > 0.122) & (g > 0.9)).float()
O, A = [], []
def collect(beta, kind, T=900, noise=0.03):
    obs = spawn(kind); drift = torch.zeros(N, 14, device=dev)
    for t in range(T):
        ct.vel_command_b[:] = 0
        o = obs["actor"]; ea = expert()
        with torch.no_grad(): pa = mlp(norm(o))
        O.append(o.clone()); A.append(ea)
        drift = 0.9 * drift + noise * 0.436 * torch.randn(N, 14, device=dev)
        obs, *_ = env.step(beta * ea + (1 - beta) * pa + drift)
    return upright().mean().item()
def fit(steps):
    Oc, Ac = torch.cat(O), torch.cat(A); opt = torch.optim.Adam(mlp.parameters(), lr=3e-4)
    for ep in range(steps):
        i = torch.randint(0, len(Oc), (8192,), device=dev)
        loss = ((mlp(norm(Oc[i])) - Ac[i]) ** 2).mean(); opt.zero_grad(); loss.backward(); opt.step()
    return loss.item(), len(Oc)
for rnd, beta in enumerate((1.0, 0.7, 0.4, 0.2, 0.0, 0.0, 0.0)):
    res = {k: collect(beta, k) for k in ("face_down", "face_up", "side", "standing")}
    loss, n = fit(3000)
    print(f"round {rnd} β={beta}: upright while collecting " + " ".join(f"{k} {v:.2f}" for k, v in res.items()) + f"  | loss {loss:.4f} data {n}", flush=True)
    for k_, v_ in mlp.state_dict().items(): ck["actor_state_dict"]["mlp." + k_] = v_
    ck["iter"] = 0; torch.save(ck, out)
# final pure-policy check (no noise)
for k in ("face_down", "face_up", "side", "standing"):
    obs = spawn(k)
    for t in range(900):
        ct.vel_command_b[:] = 0
        with torch.no_grad(): obs, *_ = env.step(mlp(norm(obs["actor"])))
    print(f"final policy from {k}: upright {upright().mean().item():.2f}", flush=True)
