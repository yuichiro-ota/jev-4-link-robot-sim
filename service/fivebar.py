"""
5節リンク脚の運動学 (XZ平面, 単位mm)
機構: 地面(フレーム) A1--crank1(r1)--U ==脚(カプラ)== L --rod-- P2--crank2(r2)--A2
脚剛体には U, L と足先 F が固定されている。
アクチュエータ: crank1角 th1 (A1まわり), crank2角 th2 (A2まわり) ※XZ平面の絶対角(atan2(z,x))
"""
import numpy as np
import json, os

class FiveBar:
    def __init__(self, p):
        self.A1 = np.array(p['A1'], float)
        self.A2 = np.array(p['A2'], float)
        self.r1 = p['r1']; self.r2 = p['r2']
        self.coupler = p['coupler']   # |U-L|
        self.rod = p['rod']           # |P2-L|
        U = np.array(p['U'], float); L = np.array(p['L'], float); F = np.array(p['F'], float)
        # 脚ローカル座標: e1 = (L-U)/|..|, e2 = e1を+90度回転
        e1 = (L-U)/np.linalg.norm(L-U)
        e2 = np.array([-e1[1], e1[0]])
        self.fL = np.array([(F-U)@e1, (F-U)@e2])   # 足先の脚ローカル座標
        # モデル姿勢の角度(ゼロ位置)
        self.th1_0 = float(np.arctan2(*(np.array(p['pin1'])-self.A1)[::-1]))
        self.th2_0 = float(np.arctan2(*(np.array(p['pin2'])-self.A2)[::-1]))
        # 分岐選択の基準: モデル姿勢でのLの位置
        self._branch_ref = L.copy()

    def fk(self, th1, th2, branch_ref=None):
        """足先位置Fと中間点(U,P2,L)を返す。閉じない場合None"""
        U = self.A1 + self.r1*np.array([np.cos(th1), np.sin(th1)])
        P2 = self.A2 + self.r2*np.array([np.cos(th2), np.sin(th2)])
        d = np.linalg.norm(P2-U)
        a, b = self.coupler, self.rod   # |U-L|, |P2-L|
        if d > a+b or d < abs(a-b) or d < 1e-9:
            return None
        # 円交点
        x = (d*d + a*a - b*b)/(2*d)
        h2 = a*a - x*x
        h = np.sqrt(max(h2, 0.0))
        e = (P2-U)/d; n = np.array([-e[1], e[0]])
        L1 = U + x*e + h*n
        L2 = U + x*e - h*n
        ref = self._branch_ref if branch_ref is None else branch_ref
        L = L1 if np.linalg.norm(L1-ref) <= np.linalg.norm(L2-ref) else L2
        e1 = (L-U)/a; e2 = np.array([-e1[1], e1[0]])
        F = U + self.fL[0]*e1 + self.fL[1]*e2
        return dict(F=F, U=U, P2=P2, L=L)

    def transmission_margin(self, th1, th2):
        """特異点からの余裕: カプラとロッドのなす角のsin (0=特異)"""
        s = self.fk(th1, th2)
        if s is None: return -1.0
        v1 = s['L']-s['U']; v2 = s['L']-s['P2']
        c = abs(v1[0]*v2[1]-v1[1]*v2[0])/(np.linalg.norm(v1)*np.linalg.norm(v2))
        return float(c)

    def ik(self, target, seed=None, iters=60, tol=1e-6):
        """足先targetへのIK (ニュートン法, 数値ヤコビアン)"""
        th = np.array(seed if seed is not None else [self.th1_0, self.th2_0], float)
        target = np.asarray(target, float)
        for _ in range(iters):
            s = self.fk(*th)
            if s is None:
                th += np.random.uniform(-0.05, 0.05, 2); continue
            err = s['F'] - target
            if np.linalg.norm(err) < tol:
                return th
            J = np.zeros((2,2)); h = 1e-5
            for j in range(2):
                tp = th.copy(); tp[j]+=h
                sp = self.fk(*tp, branch_ref=s['L'])
                if sp is None: return None
                J[:,j] = (sp['F']-s['F'])/h
            try:
                dth = np.linalg.solve(J, -err)
            except np.linalg.LinAlgError:
                return None
            step = np.clip(np.linalg.norm(dth), 0, 0.3)/max(np.linalg.norm(dth),1e-12)
            th += dth*step
        s = self.fk(*th)
        if s is not None and np.linalg.norm(s['F']-target) < 0.1:
            return th
        return None

    # ---------- 可動域 ----------
    def workspace(self, n=90, margin=0.25):
        """(th1,th2)全域をスキャンし、特異余裕がmargin以上の足先点群を返す"""
        pts=[]; TH=[]
        t1s = self.th1_0 + np.linspace(-np.pi, np.pi, n, endpoint=False)
        t2s = self.th2_0 + np.linspace(-np.pi, np.pi, n, endpoint=False)
        for t1 in t1s:
            for t2 in t2s:
                s = self.fk(t1, t2)
                if s is None: continue
                if self.transmission_margin(t1, t2) < margin: continue
                pts.append(s['F']); TH.append((t1,t2))
        return np.array(pts), np.array(TH)


# ---------- 歩容軌道 ----------
class GaitPlanner:
    """
    最適化された足先軌道を生成する。
    スタンス: 高さz_s一定の直線 (x: +s/2 -> -s/2 相対、体を前へ押す)
    スイング: 5次多項式で持ち上げ高さhのアーチ
    """
    def __init__(self, fb: FiveBar, margin=0.30, z_clear=6.0):
        self.fb = fb
        self.opt = self._optimize(margin, z_clear)

    STROKE_CAP = 60.0   # 前後脚の干渉回避と実用性のための上限 [mm]
    LIFT_MIN   = 8.0    # スイング持ち上げ最小 [mm]
    LIFT_PREF  = 12.0

    def _path_ok(self, path, margin):
        """経路上の全点でIKが解け、特異余裕が十分か"""
        seed = None
        for p in path:
            th = self.fb.ik(p, seed=seed)
            if th is None or self.fb.transmission_margin(*th) < margin:
                return False
            seed = th
        return True

    def _optimize(self, margin, z_clear):
        # 設計姿勢の足先を基準に探索
        s0 = self.fb.fk(self.fb.th1_0, self.fb.th2_0)
        z_design = s0['F'][1]
        best = None
        for z in np.arange(z_design - 6, z_design + 42, 2.0):
            # この高さで届く連続x範囲を粗く探す
            xs = [x for x in np.arange(s0['F'][0]-90, s0['F'][0]+90, 2.0)
                  if self.fb.ik([x, z]) is not None]
            if len(xs) < 10: continue
            xs = np.array(xs)
            gaps = np.where(np.diff(xs) > 4.0)[0]
            seg = list(zip(np.r_[0, gaps+1], np.r_[gaps, len(xs)-1]))
            for i0, i1 in seg:
                x0, x1 = xs[i0]+3, xs[i1]-3
                if x1 - x0 < 30: continue
                stroke = min(x1-x0, self.STROKE_CAP)
                xc = (x0+x1)/2
                xa, xb = xc-stroke/2, xc+stroke/2
                # スイング持ち上げ高さを大きい方から試す
                h_ok = 0.0
                for h in np.arange(min(self.LIFT_PREF, 16), self.LIFT_MIN-1, -1.0):
                    t = np.linspace(0, 1, 15)
                    swing = np.c_[xa + (xb-xa)*t, z + np.sin(np.pi*t)**2 * h]
                    stance = np.c_[np.linspace(xb, xa, 12), np.full(12, z)]
                    if self._path_ok(np.vstack([stance, swing]), margin):
                        h_ok = h; break
                if h_ok < self.LIFT_MIN: continue
                # スコア: ストローク重視 + 持上げ + 設計高さに近い(低いz)を優遇
                score = stroke + 2.0*h_ok - 0.6*abs(z - z_design)
                if best is None or score > best['score']:
                    best = dict(score=float(score), z=float(z), stroke=float(stroke),
                                h=float(h_ok), xc=float(xc), x0=float(x0), x1=float(x1))
        assert best is not None, 'no feasible stance found'
        return best

    def foot_target(self, phase, stride, height_offset=0.0):
        """
        phase in [0,1). duty=0.5トロット用の1脚周期軌道。
        stride: 実効ストローク(mm, 符号付き: +で前進方向=+X)
        戻り値: 足先目標(x,z) 脚ローカル(=ロボットXZ)座標
        """
        o = self.opt
        s = np.clip(stride, -o['stroke'], o['stroke'])
        xc, z = o['xc'], o['z'] + height_offset
        if phase < 0.5:   # スタンス: 前->後 (体を+Xへ押す)
            t = phase/0.5
            x = xc + s/2 - s*t
            return np.array([x, z])
        else:             # スイング: 後->前, 5次補間で滑らかに持ち上げ
            t = (phase-0.5)/0.5
            b = 10*t**3 - 15*t**4 + 6*t**5        # 位置ブレンド 0->1
            x = xc - s/2 + s*b
            zlift = np.sin(np.pi*t)**2 * o['h']    # 滑らかなアーチ
            return np.array([x, z + zlift])


def load_legs(params_path):
    P = json.load(open(params_path))
    return {k: FiveBar(v) for k, v in P.items()}, P


if __name__ == '__main__':
    import config
    legs, P = load_legs(config.LEG_PARAMS)
    for name, fb in legs.items():
        s0 = fb.fk(fb.th1_0, fb.th2_0)
        gp = GaitPlanner(fb)
        print(f'[{name}] model-pose foot={np.round(s0["F"],2)}  '
              f'opt: stance z={gp.opt["z"]:.1f} stroke={gp.opt["stroke"]:.1f}mm '
              f'lift h={gp.opt["h"]:.1f}mm xc={gp.opt["xc"]:.1f}')
