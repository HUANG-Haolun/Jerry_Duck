"""Closed-loop joint-space traditional gait trial on the SimRobot MuJoCo plant."""
from __future__ import annotations
import argparse, math, sys
from pathlib import Path
import numpy as np
import mujoco
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import sim_robot as S
from calibration import JOINTS,Calibration,load_servo_map
from servo_bus import CMD_MOVE

def main():
 p=argparse.ArgumentParser(description="连续关节空间传统步态仿真")
 p.add_argument("--seconds",type=float,default=8);p.add_argument("--period",type=float,default=1.6)
 p.add_argument("--stall",type=float,default=0.96,help="堵转扭矩 N·m")
 p.add_argument("--no-load",type=float,default=4.8,help="空载转速 rad/s")
 p.add_argument("--rated-torque",type=float,default=None,help="可选额定扭矩 N·m，需同时设置 rated-speed")
 p.add_argument("--rated-speed",type=float,default=None,help="可选额定转速 rad/s，需同时设置 rated-torque")
 p.add_argument("--servo-latency-ms",type=float,default=0.0,help="每个舵机命令的串行传输时延，按关节顺序依次模拟")
 p.add_argument("--joint-friction",type=float,default=0.02,help="关节库仑摩擦 N·m")
 p.add_argument("--floor-friction",type=float,default=1.0,help="地面滑动摩擦系数")
 p.add_argument("--roll",type=float,default=8);p.add_argument("--hip",type=float,default=8)
 p.add_argument("--knee",type=float,default=12);p.add_argument("--ankle",type=float,default=-6)
 p.add_argument("--balance-kp",type=float,default=0.0,help="姿态PD比例增益，deg/deg")
 p.add_argument("--balance-kd",type=float,default=0.0,help="姿态PD微分增益，deg/(deg/s)")
 p.add_argument("--roll-kp",type=float,default=None,help="独立横滚比例增益，缺省沿用 balance-kp")
 p.add_argument("--pitch-kp",type=float,default=None,help="独立俯仰比例增益，缺省沿用 balance-kp")
 p.add_argument("--adaptive-kp",type=float,default=0.0,help="姿态误差每度增加的比例增益，按误差绝对值调度")
 p.add_argument("--adaptive-kp-negative",type=float,default=None,help="负向姿态误差调度增益，缺省与 adaptive-kp 相同")
 p.add_argument("--adaptive-roll-negative",type=float,default=None,help="独立负向横滚调度增益")
 p.add_argument("--adaptive-pitch-negative",type=float,default=None,help="独立负向俯仰调度增益")
 p.add_argument("--direction",type=float,default=1.0,help="髋Pitch步态方向系数(+1/-1)")
 p.add_argument("--swing-ratio",type=float,default=0.30,help="单腿摆动时间占周期比例")
 p.add_argument("--phase-offset",type=float,default=0.0,help="步态初始相位偏移（周期比例，0..1）")
 p.add_argument("--foot-ik",action="store_true",help="以世界坐标足端轨迹和数值IK控制摆动/支撑脚")
 p.add_argument("--ik-blend",type=float,default=1.0,help="足端IK解与关节轨迹的混合比例（0..1）")
 p.add_argument("--step-length",type=float,default=0.02,help="足端前移距离 m（足端IK模式）")
 p.add_argument("--step-height",type=float,default=0.012,help="足端抬高距离 m（足端IK模式）")
 p.add_argument("--startup",type=float,default=1.0,help="从站姿渐入步态的时间（秒）")
 p.add_argument("--roll-bias",type=float,default=0.0,help="摆动前向支撑侧预移重心（deg）")
 p.add_argument("--contact-roll-gain",type=float,default=0.0,help="单脚支撑时质心侧向误差反馈增益（deg/m）")
 p.add_argument("--initial-roll",type=float,default=0.0,help="初始躯干横滚扰动（deg）")
 p.add_argument("--initial-pitch",type=float,default=0.0,help="初始躯干俯仰扰动（deg）")
 p.add_argument("--initial-roll-rate",type=float,default=0.0,help="初始横滚角速度扰动（deg/s）")
 p.add_argument("--initial-pitch-rate",type=float,default=0.0,help="初始俯仰角速度扰动（deg/s）")
 p.add_argument("--fall-deg",type=float,default=45);p.add_argument("--fw-k",type=float,default=16)
 a=p.parse_args()
 if not .6<=a.period<=4 or not 0<a.seconds<=600: p.error("period/seconds out of range")
 if a.servo_latency_ms<0 or a.servo_latency_ms*len(JOINTS)/1000>.02: p.error("servo latency must fit within the 20 ms control period")
 cal=Calibration.load(HERE/"calibration.yaml"); smap=load_servo_map(HERE/"servo_map.yaml")
 fw=dict(S.SimRobot.FW_PD,K=a.fw_k); robot=S.SimRobot(cal,smap,np.zeros(14),fw=fw,stall=a.stall,no_load=a.no_load,
  rated_torque=a.rated_torque,rated_speed=a.rated_speed,friction=a.joint_friction)
 robot.m.geom("floor").friction[0]=a.floor_friction
 if a.initial_roll or a.initial_pitch:
  qr=np.zeros(4); qp=np.zeros(4); q=np.zeros(4)
  mujoco.mju_axisAngle2Quat(qr,np.array([1.,0.,0.]),a.initial_roll*math.pi/180)
  mujoco.mju_axisAngle2Quat(qp,np.array([0.,1.,0.]),a.initial_pitch*math.pi/180)
  mujoco.mju_mulQuat(q,qr,qp); robot.d.qpos[3:7]=q; mujoco.mj_forward(robot.m,robot.d)
 robot.d.qvel[3]=a.initial_roll_rate*math.pi/180
 robot.d.qvel[4]=a.initial_pitch_rate*math.pi/180
 dt=.02; n=int(a.seconds/dt); deg=math.pi/180; latency_carry=0.0
 x0=robot.d.qpos[0];max_tilt=0.; fallen=False; max_lift=[0.,0.]
 floor_id=robot.m.geom("floor").id
 foot_geom={side:robot.m.geom(f"{side}_foot_collision").id for side in ("left","right")}
 contact_ticks={"left":0,"right":0};contact_load={"left":0.0,"right":0.0}
 last_contacts={"left":True,"right":True}
 lift_ticks={"left":0,"right":0}; switches=0; last_support=None
 inds={name:JOINTS.index(name) for name in JOINTS}
 foot_targets={s:robot.d.site(f"{s}_foot").xpos.copy() for s in ("left","right")}
 swing_start={s:None for s in ("left","right")}; swing_end={s:None for s in ("left","right")}
 was_swing={s:False for s in ("left","right")}
 def foot_ik(side, goal):
  suffix=("hip_pitch","knee","ankle"); js=[inds[f"{side}_{n}"] for n in suffix]
  qa=robot.qa[js]; dofs=robot.va[js]; saved=robot.d.qpos[qa].copy()
  jacp=np.zeros((3,robot.m.nv)); jacr=np.zeros((3,robot.m.nv)); sid=robot.m.site(f"{side}_foot").id
  for _ in range(24):
   mujoco.mj_forward(robot.m,robot.d); pos=robot.d.site(sid).xpos
   err=goal[[0,2]]-pos[[0,2]]
   if np.linalg.norm(err)<3e-4: break
   mujoco.mj_jacSite(robot.m,robot.d,jacp,jacr,sid); J=jacp[[0,2]][:,dofs]
   dq=J.T@np.linalg.solve(J@J.T+3e-4*np.eye(2),err)
   robot.d.qpos[qa]+=np.clip(dq,-0.08,0.08)
  result=robot.d.qpos[qa].copy();robot.d.qpos[qa]=saved;mujoco.mj_forward(robot.m,robot.d)
  return result
 for tick in range(n):
  t=tick*dt; phase=(t/a.period+a.phase_offset)%1
  q=np.zeros(14)
  # Each leg has an explicit swing phase followed by stance; opposite legs are half a cycle apart.
  ratio=float(np.clip(a.swing_ratio,.15,.45)); pr=phase; pl=(phase-.5)%1
  swing_right=pr<ratio; swing_left=pl<ratio
  ramp=float(np.clip(t/max(a.startup,1e-6),0.0,1.0)); ramp=ramp*ramp*(3-2*ramp)
  roll=(a.roll if swing_right else (-a.roll if swing_left else 0.0))*ramp
  # Scheduled weight shift toward the stance side during the swing interval.
  if swing_right:
   roll += a.roll_bias*(1.0-pr/ratio)
  elif swing_left:
   roll -= a.roll_bias*(1.0-pl/ratio)
  # Contact feedback: while one foot bears the load, shift the pelvis toward it.
  if a.contact_roll_gain and last_contacts["left"] != last_contacts["right"]:
   support_side="left" if last_contacts["left"] else "right"
   com_y=float(robot.d.subtree_com[1][1]); foot_y=float(robot.d.site(f"{support_side}_foot").xpos[1])
   roll += float(np.clip(a.contact_roll_gain*(foot_y-com_y),-8.0,8.0))
  for leg in ("left","right"): q[inds[f"{leg}_hip_roll"]]=roll*deg
  for leg,local,is_swing in (("right",pr,swing_right),("left",pl,swing_left)):
   if a.foot_ik:
    if is_swing:
     u=local/ratio; blend=u*u*(3-2*u); hip=-a.hip+2*a.hip*blend
    else:
     u=(local-ratio)/(1-ratio); blend=u*u*(3-2*u); hip=a.hip-2*a.hip*blend
    q[inds[f"{leg}_hip_pitch"]]=a.direction*hip*deg*ramp
    q[inds[f"{leg}_knee"]]=(a.knee*math.sin(math.pi*u)*deg*ramp if is_swing else 0.0)
    q[inds[f"{leg}_ankle"]]=(a.ankle*math.sin(math.pi*u)*deg*ramp if is_swing else 0.0)
    if is_swing and not was_swing[leg]:
     swing_start[leg]=robot.d.site(f"{leg}_foot").xpos.copy()
     swing_end[leg]=swing_start[leg]+np.array([a.step_length,0.,0.])
    if was_swing[leg] and not is_swing:
     foot_targets[leg]=swing_start[leg]+ramp*(swing_end[leg]-swing_start[leg])
    if is_swing:
     u=float(np.clip(local/ratio,0.,1.)); blend=u*u*(3-2*u)
     goal=swing_start[leg]+(blend*ramp)*(swing_end[leg]-swing_start[leg])
     goal[2]+=ramp*a.step_height*math.sin(math.pi*u)
    else: goal=foot_targets[leg]
    if is_swing:
     qik=foot_ik(leg,goal)
     for suffix,val in zip(("hip_pitch","knee","ankle"),qik):
      ji=inds[f"{leg}_{suffix}"]
      q[ji]=(1-a.ik_blend)*q[ji]+a.ik_blend*val
    was_swing[leg]=is_swing
    continue
   if is_swing:
    u=local/ratio; blend=u*u*(3-2*u)
    hip=-a.hip+2*a.hip*blend
    knee=a.knee*math.sin(math.pi*u); ankle=a.ankle*math.sin(math.pi*u)
   else:
    u=(local-ratio)/(1-ratio); blend=u*u*(3-2*u)
    hip=a.hip-2*a.hip*blend
    knee=0.0; ankle=0.0
   q[inds[f"{leg}_hip_pitch"]]=a.direction*hip*deg*ramp
   q[inds[f"{leg}_knee"]]=knee*deg*ramp
   q[inds[f"{leg}_ankle"]]=ankle*deg*ramp
  # Optional trunk attitude PD, evaluated from the simulated IMU/body orientation.
  R=robot.d.xmat[1].reshape(3,3)
  roll_err=math.degrees(math.atan2(R[2,1],R[2,2]))
  pitch_err=math.degrees(math.atan2(-R[2,0],max(1e-8,math.hypot(R[2,1],R[2,2]))))
  omega=robot.d.qvel[3:6]
  roll_rate=math.degrees(omega[0]);pitch_rate=math.degrees(omega[1])
  rkp=a.balance_kp if a.roll_kp is None else a.roll_kp
  pkp=a.balance_kp if a.pitch_kp is None else a.pitch_kp
  neg_gain=a.adaptive_kp if a.adaptive_kp_negative is None else a.adaptive_kp_negative
  rneg=neg_gain if a.adaptive_roll_negative is None else a.adaptive_roll_negative
  pneg=neg_gain if a.adaptive_pitch_negative is None else a.adaptive_pitch_negative
  rkp+=(a.adaptive_kp*max(0.0,roll_err-8.0)+rneg*max(0.0,-roll_err-8.0))
  pkp+=(a.adaptive_kp*max(0.0,pitch_err-8.0)+pneg*max(0.0,-pitch_err-8.0))
  rc=float(np.clip(-rkp*roll_err-a.balance_kd*roll_rate,-10,10))*deg
  pc=float(np.clip(-pkp*pitch_err-a.balance_kd*pitch_rate,-10,10))*deg
  for leg in ("left","right"):
   q[inds[f"{leg}_hip_roll"]]+=rc
   q[inds[f"{leg}_hip_pitch"]]+=pc
   q[inds[f"{leg}_ankle"]]-=pc
  tick_start=robot.d.time
  for i,name in enumerate(JOINTS):
   adc=int(round(cal.joints[name].to_adc(float(q[i]),cal.k_rad_per_count)))
   robot.command(smap[name],CMD_MOVE,adc)
   if a.servo_latency_ms:
    latency_carry+=a.servo_latency_ms
    delay_ticks=math.floor(latency_carry+1e-9)
    if delay_ticks:
     robot.step(delay_ticks*.001);latency_carry-=delay_ticks
  remaining=dt-(robot.d.time-tick_start)
  if remaining>0: robot.step(remaining)
  # MuJoCo collision-pair force is the simulated equivalent of a foot contact sensor.
  forces=np.zeros(6)
  step_contacts={"left":False,"right":False};step_force={"left":0.0,"right":0.0}
  for ci in range(robot.d.ncon):
   con=robot.d.contact[ci]; side=next((s for s,gid in foot_geom.items()
       if {con.geom1,con.geom2}=={gid,floor_id}),None)
   if side is not None:
    mujoco.mj_contactForce(robot.m,robot.d,ci,forces)
    normal=max(0.0,float(forces[0]))
    if normal>0.5:
     step_contacts[side]=True;step_force[side]+=normal
  for side in ("left","right"):
   if step_contacts[side]:contact_ticks[side]+=1;contact_load[side]+=step_force[side]
   else: lift_ticks[side]+=1
  support=tuple(s for s in ("left","right") if step_contacts[s])
  last_contacts=step_contacts.copy()
  if support and support!=last_support: switches+=1;last_support=support
  tilt=robot.tilt_deg();max_tilt=max(max_tilt,tilt)
  max_lift[0]=max(max_lift[0],float(robot.d.site("left_foot").xpos[2]))
  max_lift[1]=max(max_lift[1],float(robot.d.site("right_foot").xpos[2]))
  if tilt>=a.fall_deg:
   support_label=",".join(support) if support else "none"
   print(f"FALL t={robot.d.time:.2f}s tilt={tilt:.1f}deg phase={phase:.2f} "
         f"roll={roll_err:+.1f}deg pitch={pitch_err:+.1f}deg "
         f"rate=({roll_rate:+.1f},{pitch_rate:+.1f})deg/s support={support_label}")
   fallen=True;break
 print(f"RESULT alive={not fallen} time={robot.d.time:.2f}s dx={robot.d.qpos[0]-x0:+.4f}m "
       f"servo_latency={a.servo_latency_ms:.2f}ms/cmd "
       f"max_tilt={max_tilt:.2f}deg foot_lift=({max_lift[0]:.4f},{max_lift[1]:.4f})m "
       f"contact_ratio=({contact_ticks['left']/max(1,tick+1):.2f},{contact_ticks['right']/max(1,tick+1):.2f}) "
       f"air_ratio=({lift_ticks['left']/max(1,tick+1):.2f},{lift_ticks['right']/max(1,tick+1):.2f}) support_switches={switches} "
       f"mean_normal_force=({contact_load['left']/max(1,contact_ticks['left']):.2f},"
       f"{contact_load['right']/max(1,contact_ticks['right']):.2f})N")

if __name__=="__main__":main()
