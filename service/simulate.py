"""
リンク脚四足ロボット (12自由度) PyBulletシミュレーション

構成: 腰ロール(XL330, X軸) x4 + 5節リンク脚(XL330 x2) x4
腰ロールで足を横方向に出せるため、全方位歩容(前後・横歩き・その場旋回)に対応。

使い方:
  python simulate.py --gui                       # 前進 (Ctrl+Cで終了)
  python simulate.py --gui --mode turn_left      # その場左旋回
  python simulate.py --gui --mode strafe_right   # 右横歩き
  python simulate.py --gui --keyboard            # ↑↓:前後 ←→:旋回 A/D:横歩き
  python simulate.py --duration 10               # ヘッドレス10秒(テレメトリ出力)
  ※ --duration 省略時は Ctrl+C まで無限に実行

仕組み:
  - quadruped.urdf 読み込み (URDF_USE_INERTIA_FROM_FILE 必須!)
  - loop_constraints.json の点対点拘束で5節リンクのループを閉じる
  - 足先3D目標 -> 腰ロール角(閉形式) + 平面5節リンクIK(fivebar.py)
  - トロット歩容, ストライドはツイスト指令 (vx, vy, wz) から脚ごとに計算
"""
import argparse, json, signal, sys, time
import numpy as np
import pybullet as p
import pybullet_data
import config
from fivebar import FiveBar, GaitPlanner

LEGS = ['fl', 'fr', 'rl', 'rr']
PHASE = dict(fl=0.0, rr=0.0, fr=0.5, rl=0.5)   # トロット
# 調整パラメータはすべて .env (service/config.py 参照)
FREQ = config.GAIT_FREQ            # 歩容周波数 [Hz] (XL330の速度余裕を確保)
MAX_STRIDE = config.GAIT_MAX_STRIDE  # 前後ストライド上限 [mm]
MAX_SIDE = config.GAIT_MAX_SIDE      # 横ストライド上限 [mm]
# --- Dynamixel XL330-M288-T (5V): ストール0.52N·m / 無負荷103rpm / 定格≈0.10N·m ---
XL330_TORQUE = config.SERVO_TORQUE   # [N·m] 位置制御の実効上限
XL330_VEL = config.SERVO_VEL         # [rad/s] (=103rpm)
SETTLE_T = config.SIM_SETTLE_T


class Leg3D:
    """腰ロール + 平面5節リンクの3D足先IK

    腰ロール軸: X方向, 位置(y0, z0)。リンク平面は軸からy方向にd離れた平面。
    足先目標(x, y, z)[ボディ座標] -> (roll, th1, th2)
    """
    def __init__(self, fb: FiveBar, prm):
        self.fb = fb
        self.y0 = prm['waist']['y']
        self.z0 = prm['waist']['z']
        self.d = prm['y_leg'] - self.y0     # リンク平面の軸からの横オフセット
        self.y_leg = prm['y_leg']

    def default_y(self):
        return self.y_leg

    def ik(self, target, seed=None):
        x, y, z = target
        dy, dz = y - self.y0, z - self.z0
        r2 = dy*dy + dz*dz
        zeta2 = r2 - self.d*self.d
        if zeta2 <= 0:
            return None
        zeta = -np.sqrt(zeta2)              # 足は軸より下の分岐
        roll = np.arctan2(dz, dy) - np.arctan2(zeta, self.d)
        roll = (roll + np.pi) % (2*np.pi) - np.pi
        th = self.fb.ik([x, zeta + self.z0], seed=seed)
        if th is None:
            return None
        return roll, th


class Quadruped:
    def __init__(self, gui=False):
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, config.SIM_GRAVITY)
        self.dt = 1.0 / config.SIM_RATE_HZ
        p.setTimeStep(self.dt)
        p.setPhysicsEngineParameter(numSolverIterations=config.SIM_SOLVER_ITERATIONS)
        self.plane = p.loadURDF('plane.urdf')
        p.changeDynamics(self.plane, -1, lateralFriction=config.GROUND_FRICTION)

        # 注: PyBulletはこのフラグ無しだとURDFの慣性値を無視して衝突形状から
        #     再計算する。軽量リンク+閉ループ拘束では慣性が小さすぎると
        #     ソルバが収束せず機構が崩れるため必須。
        self.rid = p.loadURDF(config.ROBOT_URDF, [0, 0, config.SIM_SPAWN_Z],
                              flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.jidx = {}
        for i in range(p.getNumJoints(self.rid)):
            info = p.getJointInfo(self.rid, i)
            self.jidx[info[1].decode()] = i
            p.setJointMotorControl2(self.rid, i, p.VELOCITY_CONTROL, force=0)
            p.changeDynamics(self.rid, i, lateralFriction=config.ROBOT_FRICTION,
                             linearDamping=config.SIM_DAMPING,
                             angularDamping=config.SIM_DAMPING,
                             localInertiaDiagonal=[config.SIM_MIN_INERTIA]*3)

        # ---- 5節リンクのループ閉鎖拘束 ----
        loops = json.load(open(config.LOOP_CONSTRAINTS))
        name2link = {p.getJointInfo(self.rid, i)[12].decode(): i
                     for i in range(p.getNumJoints(self.rid))}
        for leg, c in loops.items():
            cid = p.createConstraint(self.rid, name2link[c['parent']],
                                     self.rid, name2link[c['child']],
                                     p.JOINT_POINT2POINT, [0, 1, 0],
                                     c['pivot_in_rod'], c['pivot_in_leg'])
            p.changeConstraint(cid, maxForce=config.SIM_LOOP_MAX_FORCE)

        # ---- 運動学 ----
        params = json.load(open(config.LEG_PARAMS))
        self.fb = {k: FiveBar(v) for k, v in params.items()}
        self.gp = {k: GaitPlanner(self.fb[k]) for k in LEGS}
        self.leg3d = {k: Leg3D(self.fb[k], params[k]) for k in LEGS}
        self.foot0 = {}     # 標準接地位置(ボディ座標)
        for k in LEGS:
            o = self.gp[k].opt
            self.foot0[k] = np.array([o['xc'], self.leg3d[k].default_y(), o['z']])
        self.seed = {k: None for k in LEGS}
        self.t = 0.0
        self._settle()

    def _settle(self):
        n = int(SETTLE_T / self.dt)
        starts = {k: np.array([*self.fb[k].fk(self.fb[k].th1_0, self.fb[k].th2_0)['F']])
                  for k in LEGS}
        for i in range(n):
            a = min(1.0, i / (0.6 * n)); a = a*a*(3-2*a)
            tg = {}
            for k in LEGS:
                s0 = np.array([starts[k][0], self.leg3d[k].default_y(), starts[k][1]])
                tg[k] = (1-a)*s0 + a*self.foot0[k]
            self.command_feet(tg)
            p.stepSimulation()

    def command_feet(self, targets):
        for leg, tgt in targets.items():
            sol = self.leg3d[leg].ik(tgt, seed=self.seed[leg])
            if sol is None:
                continue
            roll, th = sol
            self.seed[leg] = th
            fb = self.fb[leg]
            # 腰: URDF軸+X, ゼロ姿勢=モデル姿勢 -> q = roll
            # クランク: URDF軸+Y, 絶対角θ = θ0 - q -> q = θ0 - θ
            cmds = [(f'{leg}_waist', roll), (f'{leg}_hip1', fb.th1_0 - th[0]),
                    (f'{leg}_hip2', fb.th2_0 - th[1])]
            for jn, q in cmds:
                p.setJointMotorControl2(self.rid, self.jidx[jn],
                                        p.POSITION_CONTROL, targetPosition=q,
                                        force=XL330_TORQUE,
                                        positionGain=config.SERVO_POSITION_GAIN,
                                        maxVelocity=XL330_VEL)

    def step_gait(self, vx, vy, wz):
        """ツイスト指令 vx,vy,wz ∈ [-1,1] (前+ / 左+ / 左旋回+)"""
        targets = {}
        for leg in LEGS:
            f0 = self.foot0[leg]
            # 旋回の接線方向 (ボディ中心x=4.55まわり)
            rx, ry = f0[0] - config.GAIT_TURN_CENTER_X, f0[1]
            rn = np.hypot(rx, ry)
            tx, ty = -ry/rn, rx/rn
            sx = np.clip(vx + wz*tx, -1, 1) * MAX_STRIDE
            sy = np.clip(vy + wz*ty, -1, 1) * MAX_SIDE
            ph = (self.t * FREQ + PHASE[leg]) % 1.0
            targets[leg] = self._foot_target(leg, ph, sx, sy)
        self.command_feet(targets)
        p.stepSimulation()
        self.t += self.dt

    def _foot_target(self, leg, phase, sx, sy):
        f0 = self.foot0[leg]
        h = self.gp[leg].opt['h']
        if phase < 0.5:                       # スタンス: +s/2 -> -s/2
            t = phase / 0.5
            k = 0.5 - t
            return f0 + np.array([sx*k, sy*k, 0])
        else:                                 # スイング: -s/2 -> +s/2 (アーチ)
            t = (phase - 0.5) / 0.5
            b = 10*t**3 - 15*t**4 + 6*t**5
            k = -0.5 + b
            lift = np.sin(np.pi*t)**2 * h
            return f0 + np.array([sx*k, sy*k, lift])

    def base_state(self):
        pos, orn = p.getBasePositionAndOrientation(self.rid)
        return np.array(pos), np.degrees(p.getEulerFromQuaternion(orn))


MODES = dict(forward=(1,0,0), backward=(-1,0,0),
             turn_left=(0,0,1), turn_right=(0,0,-1),
             strafe_left=(0,1,0), strafe_right=(0,-1,0),
             arc_left=(0.8,0,0.5), stop=(0,0,0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--mode', default='forward', choices=list(MODES))
    ap.add_argument('--duration', type=float, default=0,
                    help='実行秒数 (0=Ctrl+Cまで無限)')
    ap.add_argument('--keyboard', action='store_true')
    args = ap.parse_args()

    q = Quadruped(gui=args.gui)
    if args.gui:
        p.resetDebugVisualizerCamera(config.GAIT_CAM_DIST, config.GAIT_CAM_YAW,
                                     config.GAIT_CAM_PITCH, [0, 0, 0.08])

    vx, vy, wz = MODES[args.mode]
    t0 = time.time()
    p0, r0 = q.base_state()
    yaw_acc, yaw_prev = 0.0, r0[2]
    last_report = 0.0
    print(f'running mode={args.mode} (Ctrl+C to stop)', flush=True)
    try:
        while args.duration <= 0 or q.t < args.duration:
            if args.keyboard and args.gui:
                keys = p.getKeyboardEvents()
                vx = (p.B3G_UP_ARROW in keys) - (p.B3G_DOWN_ARROW in keys)
                wz = (p.B3G_LEFT_ARROW in keys) - (p.B3G_RIGHT_ARROW in keys)
                vy = (ord('a') in keys) - (ord('d') in keys)
            q.step_gait(vx, vy, wz)
            _, rpy = q.base_state()
            dyaw = (rpy[2] - yaw_prev + 180) % 360 - 180   # アンラップ
            yaw_acc += dyaw; yaw_prev = rpy[2]
            if args.gui:
                lag = q.t - (time.time() - t0)
                if lag > 0:
                    time.sleep(lag)
            elif q.t - last_report >= 2.0:
                last_report = q.t
                pos, rpy = q.base_state()
                print(f'  t={q.t:6.1f}s  pos=({pos[0]*1000:7.0f},{pos[1]*1000:7.0f})mm '
                      f'yaw={yaw_acc:7.1f}deg  h={pos[2]*1000:4.0f}mm', flush=True)
    except KeyboardInterrupt:
        print('\n[Ctrl+C] stopping...', flush=True)
    finally:
        p1, r1 = q.base_state()
        d = p1 - p0
        print(f'mode={args.mode}  t={q.t:.1f}s  Δx={d[0]*1000:.0f}mm Δy={d[1]*1000:.0f}mm '
              f'Δyaw={yaw_acc:.1f}deg  height={p1[2]*1000:.0f}mm', flush=True)
        p.disconnect()


if __name__ == '__main__':
    # docker stop 等のSIGTERMでもKeyboardInterrupt同様に統計を出して終了
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))
    main()
