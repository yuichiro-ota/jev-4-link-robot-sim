"""Jev(外部ollama)で判断して障害物を避ける四足ロボットのシミュレーション

  前方LiDAR(レイキャスト) -> 3セクタをOPEN/BLOCKEDに要約 -> 外部ollamaのJevが
  4択で判断 -> 全方位トロット歩容(simulate.py)で実行 -> 判断結果をGUI上に表示

使い方:
  python service/avoid_sim.py --gui                                   # ローカルollama
  python service/avoid_sim.py --gui --ollama http://192.168.1.50:11434 --model gemma3:12b
  python service/avoid_sim.py --gui --course slalom
  python service/avoid_sim.py --gui --course random --seed 3
  python service/avoid_sim.py --brain mock --duration 60              # ollama無しで動作確認
  python service/avoid_sim.py --selftest --ollama http://192.168.1.50:11434   # 判断のみ試験

GUI表示:
  緑/赤の扇      LiDARビーム (赤=当たり)
  太い弧         この先 HUD_ARC_T 秒の予測軌跡 (実測の速度/ヨー速度から)。色は行動の出どころ
                 (水色=Jev / 赤=安全反射 / 橙=ガード / 淡青=旋回ヒステリシス /
                  黄=ollama不通のフォールバック)
  右上の黒パネル 行動・理由・セクタ判定・ゴール・推論レイテンシ・統計
  青い軌跡線     ロボットの通った経路
"""
import argparse
import collections
import json
import math
import signal
import sys
import time

import numpy as np
import pybullet as p

import config
from jev import ACTIONS, Jev, probe
from lidar import BLOCKED_M, Lidar2D, occupancy
from simulate import Quadruped

# 調整パラメータはすべて .env (service/config.py 参照)
# --- Jevより下位の安全反射 (推論の往復を待たずに効く) ---
REFLEX_M = config.AVOID_REFLEX_M        # 正面がこれより近ければ前進を打ち切って後退
REFLEX_HOLD = config.AVOID_REFLEX_HOLD  # 後退を維持する秒数
GOAL_TOL = config.AVOID_GOAL_TOL        # ゴール到達判定 [m]
SCAN_DT = config.AVOID_SCAN_DT          # LiDAR周期 [s]
HUD_DT = config.AVOID_HUD_DT            # 表示更新周期 [s]
YAW_GAIN = config.AVOID_YAW_GAIN        # FORWARD時のゴール向き合わせ (deg -> wz)
YAW_MAX = config.AVOID_YAW_MAX
STUCK_WIN = config.AVOID_STUCK_WIN      # この秒数でこの距離も進めなければ詰まり
STUCK_M = config.AVOID_STUCK_M

# いま機体を動かしているのはどの層か (表示ラベル, 色)
LAYERS = {
    'REFLEX':       ('REFLEX  emergency back-off', [1.0, 0.25, 0.2]),
    'JEV_AVOID':    ('JEV     avoid turn',         [0.25, 0.9, 1.0]),
    'JEV_BACK':     ('JEV     back off',           [0.35, 0.8, 1.0]),
    'ESCAPE':       ('ESCAPE  straight after dodge', [0.6, 0.8, 1.0]),
    'GOAL_TRACK':   ('GOAL    tracking (P control)', [0.4, 1.0, 0.45]),
    'CRUISE':       ('GOAL    on course (wz 0)',   [0.55, 0.85, 0.55]),
    'GOAL_BLOCKED': ('GOAL    side blocked, hold', [1.0, 0.65, 0.2]),
    'WAIT':         ('WAIT    for first Jev reply', [0.6, 0.6, 0.6]),
}

SRC_COLOR = dict(jev=[0.25, 0.9, 1.0], reflex=[1.0, 0.2, 0.2],
                 guard=[1.0, 0.6, 0.1], hyst=[0.6, 0.8, 1.0],
                 fallback=[1.0, 0.85, 0.2], mock=[0.8, 0.8, 0.8],
                 lowconf=[0.85, 0.7, 1.0], init=[0.6, 0.6, 0.6])


# ---------------------------------------------------------------- world
def add_box(x, y, hx, hy, h, rgba=(0.95, 0.5, 0.15, 1)):
    half = [hx, hy, h/2]
    c = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    v = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    return p.createMultiBody(0, c, v, basePosition=[x, y, h/2])


def add_cyl(x, y, r, h, rgba=(0.9, 0.35, 0.55, 1)):
    c = p.createCollisionShape(p.GEOM_CYLINDER, radius=r, height=h)
    v = p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=rgba)
    return p.createMultiBody(0, c, v, basePosition=[x, y, h/2])


# コース定義は conf/courses.json (.env の COURSES_FILE で差し替え可)
COURSE_DEF = json.load(open(config.COURSES_FILE, encoding='utf-8'))
COURSES = sorted(COURSE_DEF['courses'])


def random_specs(seed, n):
    """random コースの障害物をばらまく (パラメータは courses.json の "random")"""
    r = COURSE_DEF['random']
    rng = np.random.default_rng(seed)
    goal = tuple(r['goal'])
    specs, tries = [], 0
    while len(specs) < n and tries < 2000:
        tries += 1
        x = float(rng.uniform(*r['x_range'])); y = float(rng.uniform(*r['y_range']))
        if math.hypot(x, y) < r['min_start_clearance']:
            continue
        if math.hypot(x-goal[0], y-goal[1]) < r['min_goal_clearance']:
            continue
        if any(math.hypot(x-o['x'], y-o['y']) < r['min_spacing'] for o in specs):
            continue
        if rng.random() < r['box_probability']:
            specs.append(dict(kind='box', x=x, y=y, hx=r['box_half'], hy=r['box_half']))
        else:
            specs.append(dict(kind='cyl', x=x, y=y, r=r['cyl_radius']))
    return specs, goal


def build_world(course, seed, n_random):
    h = COURSE_DEF['height']
    if course == 'random':
        specs, goal = random_specs(seed, n_random)
    else:
        c = COURSE_DEF['courses'][course]
        specs, goal = c['obstacles'], tuple(c['goal'])
    ids = []
    for o in specs:
        if o['kind'] == 'box':
            ids.append(add_box(o['x'], o['y'], o['hx'], o['hy'], h))
        else:
            ids.append(add_cyl(o['x'], o['y'], o['r'], h))
    # ゴールマーカ (当たり判定なしの薄い円盤 + 縦線)
    gv = p.createVisualShape(p.GEOM_CYLINDER, radius=GOAL_TOL, length=0.004,
                             rgbaColor=[0.2, 1.0, 0.35, 0.55])
    p.createMultiBody(0, -1, gv, basePosition=[goal[0], goal[1], 0.002])
    p.addUserDebugLine([goal[0], goal[1], 0], [goal[0], goal[1], 0.5],
                       [0.2, 1.0, 0.35], 3.0)
    p.addUserDebugText('GOAL', [goal[0], goal[1], 0.55], [0.2, 1.0, 0.35], 1.4)
    return ids, np.array(goal, float)


# ---------------------------------------------------------------- HUD
def _mat_to_quat(m):
    """3x3回転行列 (local->world) -> pybulletのクォータニオン [x,y,z,w]"""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        k = math.sqrt(tr + 1.0) * 2
        return [(m[2, 1]-m[1, 2])/k, (m[0, 2]-m[2, 0])/k,
                (m[1, 0]-m[0, 1])/k, 0.25*k]
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        k = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return [0.25*k, (m[0, 1]+m[1, 0])/k, (m[0, 2]+m[2, 0])/k,
                (m[2, 1]-m[1, 2])/k]
    if m[1, 1] > m[2, 2]:
        k = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return [(m[0, 1]+m[1, 0])/k, 0.25*k, (m[1, 2]+m[2, 1])/k,
                (m[0, 2]-m[2, 0])/k]
    k = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return [(m[0, 2]+m[2, 0])/k, (m[1, 2]+m[2, 1])/k, 0.25*k,
            (m[1, 0]-m[0, 1])/k]


class Hud:
    """画面右上の黒パネル上の文字 + 行動矢印 + 走行軌跡

    PyBulletのデバッグ描画はワールド座標しか受け付けず、2Dオーバレイが無い。
    そこで getDebugVisualizerCamera() のビュー/射影行列を逆に解いて、
    「画面のこの位置・カメラからこの距離」のワールド点を毎フレーム計算し、
    そこに文字を置く。カメラからの距離を一定に保つので見た目の大きさは
    ズームしても変わらない。背景の黒板は当たり判定なしの薄い箱で、
    文字より少しだけ奥(6%)に置いて必ず文字が前に出るようにしている。
    """

    def __init__(self, n_lines=11):
        self.ids = [-1] * n_lines
        self.arcs = []
        self.prev = None
        self.panel = -1
        self._panel_half = None

    # getDebugVisualizerCamera() が返す horizontal / vertical は「遠方面での
    # 視錐台の幅・高さ」のベクトル。pybullet自身のマウスピッキングがこの値と
    # 遠方面10000で計算しているので、実際の描画と必ず一致する。
    # (同時に返る projectionMatrix は P[0,0]=P[1,1]=1 のプレースホルダで、
    #  FOVもアスペクトも入っていないため使えない。実測で最大0.37mずれた)
    FAR = 10000.0

    @staticmethod
    def _cam():
        """カメラ基底と、距離1あたりの可視半幅・半高"""
        c = p.getDebugVisualizerCamera()
        fwd = np.array(c[5], float)
        horiz = np.array(c[6], float)
        vert = np.array(c[7], float)
        campos = np.array(c[11], float) - c[10] * fwd
        lh, lv = np.linalg.norm(horiz), np.linalg.norm(vert)
        return (campos, fwd, horiz/lh, vert/lv,
                lh/(2*Hud.FAR), lv/(2*Hud.FAR))

    @staticmethod
    def _to_world(cam, ndcx, ndcy, depth):
        """画面座標(左下 -1,-1 / 右上 +1,+1) + カメラからの奥行 -> ワールド座標"""
        campos, fwd, right, up, hw, hh = cam
        return (campos + depth*fwd
                + (ndcx * hw * depth) * right + (ndcy * hh * depth) * up)

    def text(self, lines):
        """画面右上に固定した黒パネルの上へ lines を描く"""
        cam = self._cam()
        campos, fwd, right, up, hw, hh = cam
        d = config.HUD_PANEL_DIST
        n = min(len(lines), len(self.ids))
        ph = (n - 1 + config.HUD_TEXT_DROP) * config.HUD_LINE_H \
            + 2 * config.HUD_PAD + 0.25 * config.HUD_LINE_H
        x1 = 1.0 - config.HUD_MARGIN
        x0 = x1 - config.HUD_PANEL_W
        y1 = 1.0 - config.HUD_MARGIN

        # --- 背景パネル (文字より6%奥に置いて必ず文字を前に出す) ---
        dp = d * 1.06
        quat = _mat_to_quat(np.stack([right, up, -fwd], axis=1))
        hx = config.HUD_PANEL_W / 2 * hw * dp
        hy = ph / 2 * hh * dp
        pos = self._to_world(cam, (x0 + x1)/2, y1 - ph/2, dp)
        if self._panel_half is None or \
                abs(hx - self._panel_half[0]) > 1e-4 or \
                abs(hy - self._panel_half[1]) > 1e-4:
            if self.panel >= 0:
                p.removeBody(self.panel)        # 箱は後からリサイズできない
            vs = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.0004],
                                     rgbaColor=[0.02, 0.02, 0.03, 0.9],
                                     specularColor=[0, 0, 0])
            self.panel = p.createMultiBody(0, -1, vs, pos.tolist(), quat)
            self._panel_half = (hx, hy)
        else:
            p.resetBasePositionAndOrientation(self.panel, pos.tolist(), quat)

        # --- 文字 (向きを渡さないとpybulletが常にカメラ正対で描く) ---
        for i in range(n):
            txt, col = lines[i]
            ndcy = y1 - config.HUD_PAD - config.HUD_LINE_H*(i + config.HUD_TEXT_DROP)
            wp = self._to_world(cam, x0 + config.HUD_PAD, ndcy, d)
            kw = dict(textColorRGB=col, textSize=config.HUD_TEXT_SIZE)
            if self.ids[i] >= 0:
                kw['replaceItemUniqueId'] = self.ids[i]
            self.ids[i] = p.addUserDebugText(txt, wp.tolist(), **kw)

    def show_action(self, base, yaw, v, w, color):
        """実測の前進速度v[m/s]とヨー速度w[rad/s]から、この先の軌跡を弧で描く

        FORWARD でも wz が乗っていれば弧になるので、「なぜ曲がれるのか」が
        そのまま見える。その場旋回(v≈0)は回る側への短い線になる。
        """
        z = base[2] + 0.20
        pts = []
        if abs(v) < 0.005 and abs(w) > 1e-3:          # その場旋回
            a = yaw + math.copysign(math.pi/2, w)
            pts = [(base[0], base[1]), (base[0] + 0.28*math.cos(a),
                                        base[1] + 0.28*math.sin(a))]
        else:
            for k in range(config.HUD_ARC_SEG + 1):
                t = config.HUD_ARC_T * k / config.HUD_ARC_SEG
                if abs(w) > 1e-3:
                    r = v / w
                    x = base[0] + r*(math.sin(yaw + w*t) - math.sin(yaw))
                    y = base[1] - r*(math.cos(yaw + w*t) - math.cos(yaw))
                else:
                    x = base[0] + v*t*math.cos(yaw)
                    y = base[1] + v*t*math.sin(yaw)
                pts.append((x, y))
        while len(self.arcs) < len(pts) - 1:
            self.arcs.append(-1)
        for i in range(len(pts) - 1):
            kw = dict(lineColorRGB=color, lineWidth=6.0)
            if self.arcs[i] >= 0:
                kw['replaceItemUniqueId'] = self.arcs[i]
            self.arcs[i] = p.addUserDebugLine([pts[i][0], pts[i][1], z],
                                              [pts[i+1][0], pts[i+1][1], z], **kw)
        # 余った線分は原点に畳んで見えなくする
        for i in range(len(pts) - 1, len(self.arcs)):
            if self.arcs[i] >= 0:
                self.arcs[i] = p.addUserDebugLine(
                    [pts[-1][0], pts[-1][1], z], [pts[-1][0], pts[-1][1], z],
                    lineColorRGB=color, lineWidth=1.0,
                    replaceItemUniqueId=self.arcs[i])

    def trail(self, pos):
        cur = [pos[0], pos[1], 0.004]
        if self.prev is None:
            self.prev = cur
        elif math.dist(cur, self.prev) > config.HUD_TRAIL_MIN:
            p.addUserDebugLine(self.prev, cur, [0.2, 0.45, 1.0], 3.0)
            self.prev = cur


def sector_line(sec):
    """右上パネルに収まる長さに詰めた1行 (40文字弱)"""
    return ' | '.join(f'{k[0]} {v:.2f} {"OK" if occupancy(v) == "OPEN" else "BLK"}'
                      for k, v in sec.items())


# ---------------------------------------------------------------- selftest
SCENES = [('open',        dict(LEFT=2.00, FRONT=2.00, RIGHT=2.00), 1.5, 0,  'FORWARD'),
          ('narrow gap',  dict(LEFT=0.30, FRONT=1.20, RIGHT=0.28), 1.3, 0,  'FORWARD'),
          ('front block', dict(LEFT=1.40, FRONT=0.22, RIGHT=0.30), 2.4, 18, 'LEFT'),
          ('left block',  dict(LEFT=0.28, FRONT=0.26, RIGHT=1.10), 2.0, -5, 'RIGHT'),
          ('both open',   dict(LEFT=1.20, FRONT=0.30, RIGHT=1.10), 1.5, 25, 'LEFT'),
          ('dead end',    dict(LEFT=0.30, FRONT=0.24, RIGHT=0.26), 1.0, 5,  'BACK')]


BACKENDS = ('ollama', 'localjev', 'mock')


def parse_target(spec, args):
    """"localjev:jev-latest" -> ('localjev', 'jev-latest')

    バックエンド名を省略した場合は ollama のモデル名として扱う (後方互換)。
    """
    head, _, rest = spec.partition(':')
    if head in BACKENDS:
        return head, (rest or None)
    return args.brain, spec


def run_scenes(args, brain, model, verbose=True):
    """既知6シーンを1バックエンド/モデルで流して (正解数, レイテンシ中央値) を返す"""
    host = config.LOCALJEV_URL if brain == 'localjev' else args.ollama
    kw = dict(brain=brain, prompt_style=args.prompt_style, fmt=args.format_mode)
    # 初回はモデルのロードで数十秒かかる。先に長いタイムアウトで暖機する。
    warm = Jev(host, model, timeout=max(args.timeout, 180.0), **kw)
    t0 = time.time()
    try:
        warm.ask(SCENES[0][1], SCENES[0][2], SCENES[0][3])
        if verbose:
            print(f'  (warm-up {time.time()-t0:.1f}s)')
    except Exception as e:
        print(f'  ! warm-up 失敗: {type(e).__name__}: {e}')
    j = Jev(host, model, timeout=args.timeout, **kw)
    n_ok, lats = 0, []
    for name, sec, gd, gb, want in SCENES:
        t0 = time.time()
        try:
            act, reason, src = j.ask(sec, gd, gb)
        except Exception as e:
            act, reason, src = 'ERR', f'{type(e).__name__}: {e}'[:60], 'error'
        dt = max(0.0, time.time() - t0)
        lats.append(dt)
        n_ok += act == want
        if verbose:
            print(f'  {"OK " if act == want else "ng "}{dt:5.2f}s {name:12s} '
                  f'want={want:8s} got={act:8s} [{src}] {reason}')
    med = sorted(lats)[len(lats)//2]
    if verbose:
        print(f'  -> {n_ok}/{len(SCENES)}  median {med:.2f}s')
    return n_ok, med


def selftest(args):
    """歩容を回さずJevの判断だけを既知シーンで確認する"""
    probe_set = {args.brain}
    probe_set |= {x.split(':')[0] for x in args.compare.split(',')
                  if x.split(':')[0] in ('ollama', 'localjev', 'mock')}
    ok, names = True, []
    for b in sorted(probe_set):
        ep = config.LOCALJEV_URL if b == 'localjev' else args.ollama
        o, msg, nm = probe(ep, brain=b)
        print(f'{b} {ep}: {"OK" if o else "NG"} ({msg})')
        if b == args.brain:
            ok, names = o, nm
    specs = [x.strip() for x in args.compare.split(',') if x.strip()]
    targets = [parse_target(x, args) for x in specs] or [(args.brain, args.model)]
    print(f'prompt={args.prompt_style}  format={args.format_mode}  '
          f'{len(SCENES)} scenes')
    results = []
    for brain, model in targets:
        shown = model
        if brain == 'localjev' and model in (None, args.model):
            shown = config.LOCALJEV_MODEL      # localjev は別のキーでモデルを持つ
        elif brain == 'mock':
            shown = None
        label = f'{brain}:{shown or "(default)"}'
        print(f'== {label}')
        if brain == 'ollama' and ok and names and model and model not in names:
            print(f'  ! model {model!r} が見つかりません。入っているのは: {names}')
        results.append((label,) + run_scenes(args, brain, model, verbose=True))
    if len(results) > 1:
        print('\n== 比較')
        print(f'  {"backend:model":32s} {"score":>7s} {"median":>8s}')
        for label, n, med in results:
            print(f'  {label:32s} {n:3d}/{len(SCENES)} {med:7.2f}s')
    return 0 if all(n == len(SCENES) for _, n, _ in results) else 1


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='Jev(ollama)判断による障害物回避シミュレーション')
    ap.add_argument('--gui', action='store_true')
    ap.add_argument('--ollama', default=config.OLLAMA_HOST,
                    help='外部ollamaのURL 例 http://192.168.1.50:11434')
    ap.add_argument('--model', default=config.JEV_MODEL)
    ap.add_argument('--brain', default=config.JEV_BRAIN,
                    choices=['ollama', 'localjev', 'mock'],
                    help='ollama=/api/chat / localjev=local-jevの/v1/systemone / '
                         'mock=推論なしの決定論ポリシー')
    ap.add_argument('--prompt-style', default=config.JEV_PROMPT,
                    choices=['rules', 'bare'],
                    help='rules=判断ルールを書いて渡す(汎用モデル) / '
                         'bare=状態だけ渡す(タスク専用モデル)')
    ap.add_argument('--format-mode', default=config.JEV_FORMAT,
                    choices=['schema', 'json', 'off'],
                    help='schema=JSONスキーマでenum固定 / json=JSON要求のみ / '
                         'off=自由文を正規表現で読む')
    ap.add_argument('--compare', default='',
                    help='--selftest で複数バックエンド/モデルを比較。カンマ区切りで '
                         '"ollama:gemma3:12b,localjev:jev-latest,mock" のように指定 '
                         '(バックエンド名を省略すると ollama のモデル名として扱う)')
    ap.add_argument('--period', type=float, default=config.JEV_PERIOD, help='判断周期 [s]')
    ap.add_argument('--commit', type=float, default=config.AVOID_COMMIT,
                    help='旋回/後退を最低これだけ続ける [s] (振動防止)')
    ap.add_argument('--escape', type=float, default=config.AVOID_ESCAPE,
                    help='迂回旋回のあとゴール追従を止めて直進する秒数 [s]')
    ap.add_argument('--timeout', type=float, default=config.JEV_TIMEOUT,
                    help='ollamaタイムアウト [s]')
    ap.add_argument('--course', default=config.COURSE, choices=COURSES + ['random'])
    ap.add_argument('--seed', type=int, default=config.COURSE_SEED)
    ap.add_argument('--n-obstacles', type=int, default=config.COURSE_N_OBSTACLES,
                    help='--course random の個数')
    ap.add_argument('--duration', type=float, default=config.AVOID_DURATION,
                    help='制限時間 [s] (0=無制限)')
    ap.add_argument('--beams', type=int, default=config.LIDAR_BEAMS)
    ap.add_argument('--range', type=float, default=config.LIDAR_RANGE,
                    help='LiDAR最大距離 [m]')
    ap.add_argument('--no-rays', action='store_true', help='ビーム描画を切る(軽量化)')
    ap.add_argument('--ray-step', type=int, default=config.AVOID_RAY_STEP,
                    help='ビーム描画の間引き (1=全部描く)')
    ap.add_argument('--log', default=config.JEV_LOG, help='判断ログ(JSONL)の出力先')
    ap.add_argument('--selftest', action='store_true', help='判断だけを試験して終了')
    args = ap.parse_args()

    if args.selftest:
        return selftest(args)

    if args.brain != 'mock':
        endpoint = config.LOCALJEV_URL if args.brain == 'localjev' else args.ollama
        ok, msg, names = probe(endpoint, brain=args.brain)
        print(f'[jev] {args.brain} {endpoint} -> {"OK" if ok else "NG"} ({msg})',
              flush=True)
        if ok and names:
            want = config.LOCALJEV_MODEL if args.brain == 'localjev' else args.model
            alias = want in ('jev-latest', 'jev-preview', 'local-jev-latest')
            if not alias and want not in names:
                print(f'[jev] ! model {want!r} が見つかりません: {names}', flush=True)
        if not ok:
            print('[jev] ! 不通なのでフォールバックポリシーで走ります '
                  '(--brain mock で明示切替)', flush=True)

    q = Quadruped(gui=args.gui)
    obstacles, goal = build_world(args.course, args.seed, args.n_obstacles)
    lidar = Lidar2D(q.rid, n_beams=args.beams, max_range=args.range)
    j = Jev(args.ollama, args.model, period=args.period, timeout=args.timeout,
            brain=args.brain, log_path=args.log or None,
            prompt_style=args.prompt_style, fmt=args.format_mode)
    hud = Hud() if args.gui else None
    host_short = j.host.replace('http://', '').replace('https://', '')
    model_short = j.model
    if args.gui:
        p.resetDebugVisualizerCamera(
            config.CAM_DIST, config.CAM_YAW, config.CAM_PITCH,
            [config.CAM_TARGET_X, config.CAM_TARGET_Y, config.CAM_TARGET_Z])

    lidar.scan()
    sec = lidar.sectors()
    reflex_until = escape_until = -1.0
    t_scan = t_hud = -1e9
    t0 = time.time()
    p_start, r_start = q.base_state()
    prev_xy = p_start[:2].copy()
    # 表示用の実測値 (HUD周期ごとに更新してEMAで平滑化)
    t_meas, xy_meas = 0.0, p_start[:2].copy()
    yaw_meas = math.radians(r_start[2])
    v_meas = w_meas = 0.0
    path_len = 0.0
    hist = [(0.0, p_start[:2].copy())]
    collisions, was_touching = 0, False
    layer_time = collections.Counter()      # 層ごとの駆動時間 [s]
    result = 'TIMEOUT'
    print(f'[jev] {config.describe()}', flush=True)
    print(f'[jev] course={args.course} goal=({goal[0]:.2f},{goal[1]:.2f}) '
          f'backend={args.brain} model={j.model} period={args.period}s '
          f'blocked<{BLOCKED_M}m', flush=True)
    try:
        while args.duration <= 0 or q.t < args.duration:
            pos, rpy = q.base_state()
            yaw = math.radians(rpy[2])
            dxy = goal - pos[:2]
            gdist = float(np.hypot(*dxy))
            gbear = math.degrees((math.atan2(dxy[1], dxy[0]) - yaw + math.pi)
                                 % (2*math.pi) - math.pi)
            if gdist < GOAL_TOL:
                result = 'GOAL'
                break

            if q.t - t_scan >= SCAN_DT:
                t_scan = q.t
                lidar.scan()
                sec = lidar.sectors()

            d = j.latest
            # 旋回/後退は最低 --commit 秒続ける。毎周期に判断を上書きすると
            # LiDARの見え方が旋回で激しく変わり、左右にがたつくだけで進まなくなる。
            committed = d.action in ('LEFT', 'RIGHT', 'BACK') and q.t - d.t < args.commit
            if committed and d.action in ('LEFT', 'RIGHT'):
                # 迂回を始めたら、旋回のあとしばらくはゴールに引っ張らせない。
                # 直後にゴール方向へ向き直すと同じ障害物に突っ込み、
                # 「曲がる→引き戻される」のリミットサイクルに入る。
                escape_until = q.t + args.escape
            if not committed:
                j.submit(q.t, sec, gdist, gbear)
                d = j.latest

            act, reason, src = d.action, d.reason, d.source
            vx, vy, wz = d.twist()
            # --- 安全反射: Jevの応答を待たずに正面衝突を止める ---
            if sec['FRONT'] < REFLEX_M and vx > 0:
                if q.t >= reflex_until:              # 立ち上がりのみ1回カウント
                    j.note_reflex()
                reflex_until = q.t + REFLEX_HOLD
            wz_src = 'Jev action'          # wzがどこから来たかの表示用
            layer = 'JEV_AVOID'            # いま機体を動かしている層
            if q.t < reflex_until:
                act, reason, src = 'BACK', f'reflex: front {sec["FRONT"]:.2f}m', 'reflex'
                vx, vy, wz = ACTIONS['BACK']
                wz_src = 'reflex (no steering)'
                layer = 'REFLEX'
            elif act == 'FORWARD':
                # ここが「FORWARDなのに曲がれる」理由。Jevの (1,0,0) の wz を
                # ゴール方位のP制御で上書きしている。
                wz = float(np.clip(gbear * YAW_GAIN, -YAW_MAX, YAW_MAX))   # ゴール追従
                wz_src = f'goal assist (bear {gbear:+.0f})'
                layer = 'GOAL_TRACK'
                if q.t < escape_until:
                    wz = 0.0                                   # 迂回中は直進で抜ける
                    wz_src = 'escaping -> straight'
                    layer = 'ESCAPE'
                elif ((wz > 0 and occupancy(sec['LEFT']) == 'BLOCKED') or
                      (wz < 0 and occupancy(sec['RIGHT']) == 'BLOCKED')):
                    wz = 0.0                                   # 塞がった側へは向き直さない
                    wz_src = 'goal side blocked -> 0'
                    layer = 'GOAL_BLOCKED'
                elif abs(wz) >= YAW_MAX - 1e-9:
                    wz_src = f'goal assist (bear {gbear:+.0f}, max)'
                elif abs(wz) < 1e-3:
                    wz_src = 'on course (bear ~0)'
                    layer = 'CRUISE'
            elif act in ('LEFT', 'RIGHT'):
                wz_src = 'turn in place'
            elif act == 'BACK':
                layer = 'JEV_BACK'
            else:                                              # STOP = 初回応答待ち
                layer = 'WAIT'
            layer_time[layer] += q.dt

            q.step_gait(vx, vy, wz)

            # --- 統計 ---
            step = float(np.hypot(*(pos[:2] - prev_xy)))
            if step > 1e-4:
                path_len += step
                prev_xy = pos[:2].copy()
            touching = any(p.getContactPoints(q.rid, o) for o in obstacles)
            collisions += touching and not was_touching
            was_touching = touching
            hist.append((q.t, pos[:2].copy()))
            while hist and q.t - hist[0][0] > STUCK_WIN:
                hist.pop(0)
            if q.t > STUCK_WIN and np.hypot(*(pos[:2] - hist[0][1])) < STUCK_M:
                result = 'STUCK'
                break

            # --- 表示 ---
            if q.t - t_hud >= HUD_DT:
                t_hud = q.t
                col = SRC_COLOR.get(src, [1, 1, 1])
                # 実測: 機体前方向の速度とヨー速度 (歩容の上下動でばらつくのでEMA)
                dt_m = q.t - t_meas
                if dt_m > 1e-3:
                    dxy_m = pos[:2] - xy_meas
                    fv = (dxy_m[0]*math.cos(yaw) + dxy_m[1]*math.sin(yaw)) / dt_m
                    dyaw_m = (yaw - yaw_meas + math.pi) % (2*math.pi) - math.pi
                    v_meas += 0.35 * (fv - v_meas)
                    w_meas += 0.35 * (dyaw_m/dt_m - w_meas)
                    t_meas, xy_meas, yaw_meas = q.t, pos[:2].copy(), yaw
                if args.gui:
                    if not args.no_rays:
                        lidar.draw(step=max(1, args.ray_step))
                    lab, lcol = LAYERS[layer]
                    if src == 'fallback':
                        lab += ' [fallback]'
                    hud.show_action(pos, yaw, v_meas, w_meas, lcol)
                    hud.trail(pos)
                    hud.text([
                        (lab, lcol),
                        (f'JEV: {act}' + ('  (committed)' if committed else ''), col),
                        (f'why: {reason}', [1.0, 1.0, 1.0]),
                        (sector_line(sec), [0.65, 0.88, 1.0]),
                        (f'twist  vx{vx:+.2f}  vy{vy:+.2f}  wz{wz:+.2f}',
                         [1.0, 0.9, 0.45]),
                        (f'wz: {wz_src}', [1.0, 0.9, 0.45]),
                        (f'meas  {v_meas*1000:+4.0f} mm/s  '
                         f'{math.degrees(w_meas):+5.1f} deg/s', [1.0, 0.8, 0.55]),
                        (f'goal {gdist:.2f} m  bear {gbear:+.0f} deg'
                         + ('  [escaping]' if q.t < escape_until else ''),
                         [0.6, 1.0, 0.6]),
                        ((f'conf {d.conf:.2f}  ' + '  '.join(
                            f'{k[0]}{d.probs[k]:.2f}' for k in
                            sorted(d.probs, key=d.probs.get, reverse=True)[:4])
                          ) if d.conf is not None and d.probs else '',
                         [0.9, 0.75, 1.0]),
                        (f'{model_short} @ {host_short}   {d.latency:.2f} s',
                         [0.85, 0.85, 0.85]),
                        (f'calls {j.calls}  gd {j.guards}  hy {j.hysts}  '
                         f'rx {j.reflexes}  fb {j.errors}', [0.8, 0.8, 0.8]),
                        ((j.last_error[:38] if j.last_error else ''),
                         [1.0, 0.6, 0.4]),
                    ])
                elif int(q.t) != int(q.t - HUD_DT):
                    print(f'  t={q.t:6.1f}s [{layer:12s}] {act:8s}({src}) '
                          f'twist({vx:+.2f},{vy:+.2f},{wz:+.2f}) {wz_src:26s} '
                          f'goal={gdist:5.2f}m bear={gbear:+5.0f} | {sector_line(sec)}',
                          flush=True)

            if args.gui:
                lag = q.t - (time.time() - t0)
                if lag > 0:
                    time.sleep(lag)
    except KeyboardInterrupt:
        result = 'ABORT'
        print('\n[Ctrl+C] stopping...', flush=True)
    finally:
        pos, _ = q.base_state()
        print(f'\n=== {result} ===', flush=True)
        print(f'  sim time      {q.t:.1f} s', flush=True)
        print(f'  final pos     ({pos[0]:.2f}, {pos[1]:.2f}) m   goal '
              f'({goal[0]:.2f}, {goal[1]:.2f})   dist '
              f'{np.hypot(*(goal - pos[:2])):.2f} m', flush=True)
        print(f'  path length   {path_len:.2f} m', flush=True)
        print(f'  jev decisions {j.calls}  guard {j.guards}  hyst {j.hysts}  '
              f'reflex {j.reflexes}  fallback {j.errors}  '
              f'avg latency {j.avg_latency:.2f} s', flush=True)
        print(f'  obstacle hits {collisions}', flush=True)
        if layer_time:
            tot = sum(layer_time.values()) or 1.0
            print('  driven by:', flush=True)
            for k, v in layer_time.most_common():
                print(f'    {k:13s} {v:6.1f} s  {100*v/tot:4.1f}%  '
                      f'{LAYERS[k][0]}', flush=True)
        if j.last_error:
            print(f'  last error    {j.last_error}', flush=True)
        j.close()
        p.disconnect()
    return 0 if result == 'GOAL' else 1


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))
    sys.exit(main())
