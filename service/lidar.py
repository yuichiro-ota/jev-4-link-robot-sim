"""前方2D LiDAR (レイキャスト) と、Jevへ渡すセクタ要約

実機想定: 車体前端に載せた測距スキャナ (FOV 180deg / 最大2m)。
PyBulletの rayTestBatch で1スキャン分を1コールで撃つ。

ビームは「ヨーのみ」で回した水平面に撃つ (ロール/ピッチは無視)。
歩行中の車体は±6deg程度うねるため、車体姿勢そのままだと2m先で±0.2mも
上下にぶれて障害物を撃ち抜けてしまう。実機でもスキャナはジンバルやIMUで
水平を保つのが普通なので、水平固定のほうが実態に近い。

Jevに渡すのは3セクタ×2値(OPEN/BLOCKED)だけ。理由は jev.py 参照。
"""
import numpy as np
import pybullet as p

import config

# 通れないと見なすしきい値 [m]。ロボット幅0.23m、前進146mm/s、判断周期0.8s+
# 推論0.4sで約0.18m進むので、その倍くらいの余裕を取る。 -> .env: LIDAR_BLOCKED_M
BLOCKED_M = config.LIDAR_BLOCKED_M

# Jevに見せるセクタ: (名前, 下限角deg, 上限角deg)  角度は +が左
# FRONTの±20degは0.35m先で±0.12m = 車体半幅0.115mにほぼ一致させている。
_FH = config.LIDAR_FRONT_HALF_DEG
_S0, _S1 = config.LIDAR_SIDE_MIN_DEG, config.LIDAR_SIDE_MAX_DEG
FRONT_SECTOR = ('FRONT', -_FH, _FH)
SIDE_SECTORS = [('LEFT', _S0, _S1), ('RIGHT', -_S1, -_S0)]
CONE_DEG = config.LIDAR_CONE_DEG   # 「車体が通る幅の開口」と見なす円錐の半角


def occupancy(d):
    """距離 -> Jevが読むラベル"""
    return 'OPEN' if d >= BLOCKED_M else 'BLOCKED'


class Lidar2D:
    def __init__(self, rid, n_beams=None, fov=None, max_range=None, min_range=None,
                 mount=None):
        n_beams = config.LIDAR_BEAMS if n_beams is None else n_beams
        fov = config.LIDAR_FOV_DEG if fov is None else fov
        max_range = config.LIDAR_RANGE if max_range is None else max_range
        min_range = config.LIDAR_MIN_RANGE if min_range is None else min_range
        if mount is None:
            mount = (config.LIDAR_MOUNT_X, config.LIDAR_MOUNT_Y, config.LIDAR_MOUNT_Z)
        self.rid = rid
        self.max_range = max_range
        self.min_range = min_range
        self.mount = np.asarray(mount, float)
        self.ang = np.radians(np.linspace(fov/2, -fov/2, n_beams))   # 左->右
        self.deg = np.degrees(self.ang)
        self.ranges = np.full(n_beams, max_range)
        self.hit_pts = np.zeros((n_beams, 3))
        self.origin = np.zeros(3)
        self._lines = []

    def scan(self):
        """1スキャン実行。ranges[m] を返す (未検出は max_range)"""
        pos, orn = p.getBasePositionAndOrientation(self.rid)
        yaw = p.getEulerFromQuaternion(orn)[2]
        c, s = np.cos(yaw), np.sin(yaw)
        mx, my, mz = self.mount
        self.origin = np.array([pos[0] + c*mx - s*my, pos[1] + s*mx + c*my, pos[2] + mz])
        a = self.ang + yaw
        d = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)
        froms = np.repeat(self.origin[None, :], len(a), axis=0)
        hits = p.rayTestBatch(froms.tolist(), (froms + d*self.max_range).tolist())
        r = np.full(len(a), self.max_range)
        for i, h in enumerate(hits):
            # 自分の脚を拾った場合は無視 (センサは脚より前だが保険)
            if h[0] >= 0 and h[0] != self.rid:
                r[i] = max(self.min_range, h[2] * self.max_range)
        self.ranges = r
        self.hit_pts = froms + d * r[:, None]
        return r

    def sectors(self):
        """Jevに渡す3値 {'LEFT','FRONT','RIGHT'}: m

        FRONT は帯の最小距離 = 進路上に何かあるか (安全側)。
        LEFT/RIGHT は「最小距離」ではなく「車体幅の開口がある方向の距離」を返す。
        側方を最小距離で潰すと、幅の広い壁では3セクタ全部がBLOCKEDになり
        「どちら側へ回れば抜けられるか」の情報が消えて前進と後退を往復する。
        そこで各方向について±CONE_DEGの円錐内の最小距離を求め、その最大値
        (=最も開けている通り抜け方向) をそのセクタの値とする。
        """
        name, lo, hi = FRONT_SECTOR
        m = (self.deg >= lo) & (self.deg <= hi)
        out = {name: float(self.ranges[m].min()) if m.any() else self.max_range}
        for name, lo, hi in SIDE_SECTORS:
            cand = np.flatnonzero((self.deg >= lo) & (self.deg <= hi))
            best = 0.0
            for i in cand:
                cone = np.abs(self.deg - self.deg[i]) <= CONE_DEG
                best = max(best, float(self.ranges[cone].min()))
            out[name] = best if len(cand) else self.max_range
        return {'LEFT': out['LEFT'], 'FRONT': out['FRONT'], 'RIGHT': out['RIGHT']}

    def draw(self, step=1):
        """GUIにビームを描画 (当たり=赤 / 空=薄緑)。lineは使い回して増殖を防ぐ"""
        idx = range(0, len(self.ang), step)
        o = self.origin.tolist()
        for k, i in enumerate(idx):
            hit = self.ranges[i] < self.max_range - 1e-6
            if k >= len(self._lines):
                self._lines.append(-1)
            col = [1.0, 0.25, 0.1] if hit else [0.35, 0.85, 0.45]
            kw = dict(lineColorRGB=col, lineWidth=2.5 if hit else 1.0)
            if self._lines[k] >= 0:
                kw['replaceItemUniqueId'] = self._lines[k]
            self._lines[k] = p.addUserDebugLine(o, self.hit_pts[i].tolist(), **kw)
