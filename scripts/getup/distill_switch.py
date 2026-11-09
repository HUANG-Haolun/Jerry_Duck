"""Distil "walk policy + get-up policy + switch" into ONE 61-D policy, by DAgger.

Teacher = jduck_run.PolicySwitch's rule, per env: walk (N1) while upright, the stand-up policy
after a fall (tilt > 45°), back to walking once tilt < 12° and the legs are within 0.35 rad of
HOME for 0.6 s. While the teacher is in get-up mode the stand-up policy gets the command slots
zeroed (as on the robot). The student sees the normal obs (commands included).

Why: PPO + fallen-gated BC (Mjlab-VelStand-Flat-JDuck, jvs1–jvs7) never copied the get-up
precisely enough — jduck's get-up fails under ~0.03 rad of action noise, the BC residual was
~0.08 rad. Supervised DAgger on the student's own states reached ~0.02 rad on the get-up alone.

Env: the VelStand training env (twist/head commands, topple pushes, prone/side/crouch spawns,
UBT-12HC DR), terminations except nan/time-out removed, prone spawns at a fixed 40 %.

    cd third_party/microduck_rl && <JDUCK_* of the velstand run> \
    uv run --frozen python ../../scripts/getup/distill_switch.py WALK.pt GETUP.pt OUT.pt [--rounds 8]

WALK.pt / GETUP.pt are rsl_rl checkpoints (61→512→256→128→14 ELU MLP with a baked
normalizer); OUT.pt keeps WALK.pt's normalizer, so it can be exported with jduck_export.sh.
"""

import argparse
import copy
import math

import numpy as np
import torch

import mjlab_microduck.tasks  # noqa: F401
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

LEGS = [*range(0, 5), *range(9, 14)]


def load_mlp(path, dev):
    ck = torch.load(path, map_location=dev, weights_only=False)
    sd = ck["actor_state_dict"]
    mlp = torch.nn.Sequential(torch.nn.Linear(61, 512), torch.nn.ELU(), torch.nn.Linear(512, 256), torch.nn.ELU(),
                              torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 14)).to(dev)
    mlp.load_state_dict({k[4:]: v for k, v in sd.items() if k.startswith("mlp.")})
    mu, std = sd["obs_normalizer._mean"].clone(), sd["obs_normalizer._std"].clone()
    return ck, mlp, (lambda o, mu=mu, std=std: (o - mu) / (std + 1e-2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("walk"); p.add_argument("getup"); p.add_argument("out")
    p.add_argument("--rounds", type=int, default=8)
    p.add_argument("--envs", type=int, default=2048)
    p.add_argument("--steps", type=int, default=500, help="env steps collected per round")
    p.add_argument("--prone", type=float, default=0.4)
    p.add_argument("--keep", type=int, default=6, help="rounds of data kept (oldest dropped)")
    a = p.parse_args()

    cfg = load_env_cfg("Mjlab-VelStand-Flat-JDuck")
    cfg.scene.num_envs = a.envs
    cfg.curriculum = {}
    for k in list(cfg.terminations):
        if k not in ("nan_state", "time_out"):
            cfg.terminations.pop(k)
    pr = cfg.events["random_prone_init"].params
    pr.update(prone_prob=a.prone, face_down_prob=0.5, side_prob=0.4, crouch_prob=0.1)
    cfg.events["topple_push"].params["velocity_range"] = {"x": (-0.9, 0.9), "y": (-0.9, 0.9)}
    env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
    dev, N = env.device, a.envs

    ck, student, s_norm = load_mlp(a.walk, dev)
    _, walk, w_norm = load_mlp(a.walk, dev)
    _, getup, g_norm = load_mlp(a.getup, dev)
    walk.eval(); getup.eval()
    opt = torch.optim.Adam(student.parameters(), lr=2e-4)

    mode = torch.zeros(N, dtype=torch.bool, device=dev)          # True = get-up
    ok_t = torch.zeros(N, device=dev)
    dt = env.step_dt

    def teacher(o):
        nonlocal mode, ok_t
        g = o[:, 3:6]
        tilt = torch.rad2deg(torch.arccos(torch.clamp(-g[:, 2], -1, 1)))
        dq = o[:, 6:20][:, LEGS].abs().amax(1)
        mode = mode | (tilt > 45)
        ok = mode & (tilt < 12) & (dq < 0.35)
        ok_t = torch.where(ok, ok_t + dt, torch.zeros_like(ok_t))
        mode = mode & ~(ok_t >= 0.6)
        og = o.clone(); og[:, 48:] = 0.0
        with torch.no_grad():
            return torch.where(mode[:, None], getup(g_norm(og)), walk(w_norm(o)))

    data = []
    env.reset()
    obs = env.observation_manager.compute()
    for rnd in range(a.rounds):
        beta = max(0.0, 1.0 - rnd / max(1, a.rounds - 3))
        O, A = [], []
        up_frac = []
        for t in range(a.steps):
            o = obs["actor"]
            ta = teacher(o)
            with torch.no_grad():
                sa = student(s_norm(o))
            O.append(o.clone()); A.append(ta)
            use_t = torch.rand(N, 1, device=dev) < beta
            obs, _, term, trunc, _ = env.step(torch.where(use_t, ta, sa))
            done = (term | trunc).bool()
            mode = mode & ~done; ok_t = torch.where(done, torch.zeros_like(ok_t), ok_t)
            up_frac.append((~mode).float().mean().item())
        data.append((torch.cat(O), torch.cat(A)))
        data = data[-a.keep:]
        Oc = torch.cat([d[0] for d in data]); Ac = torch.cat([d[1] for d in data])
        for ep in range(4000):
            i = torch.randint(0, len(Oc), (16384,), device=dev)
            loss = ((student(s_norm(Oc[i])) - Ac[i]) ** 2).mean()
            opt.zero_grad(); loss.backward(); opt.step()
        rms = math.sqrt(loss.item())
        print(f"round {rnd} β={beta:.2f}: 收集时非起身模式比例 {np.mean(up_frac):.2f}  | 拟合 RMS {rms:.3f} rad  数据 {len(Oc)}", flush=True)
        out = copy.deepcopy(ck)
        for k, v in student.state_dict().items():
            out["actor_state_dict"]["mlp." + k] = v
        out["iter"] = 0
        torch.save(out, a.out)
    print("saved", a.out)


if __name__ == "__main__":
    main()
