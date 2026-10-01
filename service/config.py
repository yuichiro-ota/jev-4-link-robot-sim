"""環境変数による設定 (.env)

優先順位は  コマンドライン引数 > シェルの環境変数 > .env > ここの既定値。
.env はリポジトリ直下に置く (`cp .env.example .env`)。書式は KEY=VALUE、
`#` 以降はコメント。すでにシェルで定義済みの変数は上書きしない
(docker compose から環境変数を渡した場合はそちらが勝つ)。

依存を増やしたくないので python-dotenv は使わず自前で読む。
"""
import os

SERVICE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SERVICE_DIR)


def load_dotenv(path):
    """.env を読んで未定義の環境変数だけ埋める。無ければ何もしない"""
    if not os.path.isfile(path):
        return False
    with open(path, encoding='utf-8') as f:
        for raw in f:
            line = raw.split('#', 1)[0].strip()
            if not line or '=' not in line:
                continue
            k, v = line.split('=', 1)
            k, v = k.strip(), v.strip().strip('"').strip("'")
            if k and k not in os.environ:
                os.environ[k] = v
    return True


DOTENV = os.environ.get('DOTENV_PATH') or os.path.join(ROOT, '.env')
DOTENV_LOADED = load_dotenv(DOTENV)


# ---- 型付きアクセサ (空文字は未設定として扱う: compose の素通しに対応) ----
def s(key, default):
    v = os.environ.get(key, '')
    return v if v != '' else default


def f(key, default):
    v = os.environ.get(key, '')
    try:
        return float(v) if v != '' else float(default)
    except ValueError:
        return float(default)


def i(key, default):
    v = os.environ.get(key, '')
    try:
        return int(float(v)) if v != '' else int(default)
    except ValueError:
        return int(default)


def b(key, default):
    v = os.environ.get(key, '').strip().lower()
    if v in ('1', 'true', 'yes', 'on'):
        return True
    if v in ('0', 'false', 'no', 'off'):
        return False
    return bool(default)


def path(key, *default_parts):
    v = os.environ.get(key, '')
    p = v if v != '' else os.path.join(ROOT, *default_parts)
    return p if os.path.isabs(p) else os.path.join(ROOT, p)


# ---- パス ------------------------------------------------------------------
CONF_DIR = path('CONF_DIR', 'conf')
MESH_DIR = path('MESH_DIR', 'meshes')
ROBOT_URDF = path('ROBOT_URDF', 'conf', 'quadruped.urdf')
LEG_PARAMS = path('LEG_PARAMS', 'conf', 'leg_params.json')
LOOP_CONSTRAINTS = path('LOOP_CONSTRAINTS', 'conf', 'loop_constraints.json')
COURSES_FILE = path('COURSES_FILE', 'conf', 'courses.json')

# ---- 物理シミュレーション --------------------------------------------------
SIM_RATE_HZ = f('SIM_RATE_HZ', 480.0)          # 物理ステップ [Hz]
SIM_SOLVER_ITERATIONS = i('SIM_SOLVER_ITERATIONS', 150)
SIM_SETTLE_T = f('SIM_SETTLE_T', 0.8)          # 初期姿勢へ落とし込む秒数
SIM_LOOP_MAX_FORCE = f('SIM_LOOP_MAX_FORCE', 1200.0)   # ループ閉鎖拘束の上限
SIM_GRAVITY = f('SIM_GRAVITY', -9.81)          # [m/s^2]
SIM_SPAWN_Z = f('SIM_SPAWN_Z', 0.003)          # 初期配置の浮かし量 [m]
SIM_DAMPING = f('SIM_DAMPING', 0.01)           # 各リンクの線形/角速度ダンピング
SIM_MIN_INERTIA = f('SIM_MIN_INERTIA', 1e-4)   # 慣性の下限 (軽量リンクの発散対策)
GROUND_FRICTION = f('GROUND_FRICTION', 1.0)
ROBOT_FRICTION = f('ROBOT_FRICTION', 0.9)

# ---- 歩容 ------------------------------------------------------------------
GAIT_FREQ = f('GAIT_FREQ', 1.4)                # 歩容周波数 [Hz]
GAIT_MAX_STRIDE = f('GAIT_MAX_STRIDE', 55.0)   # 前後ストライド上限 [mm]
GAIT_MAX_SIDE = f('GAIT_MAX_SIDE', 26.0)       # 横ストライド上限 [mm]
# その場旋回の中心 (ボディ座標X) [mm]。脚配置から決まる値で通常は触らない
GAIT_TURN_CENTER_X = f('GAIT_TURN_CENTER_X', 4.55)

# ---- サーボ (Dynamixel XL330-M288-T @5V) -----------------------------------
SERVO_TORQUE = f('SERVO_TORQUE', 0.45)         # [N·m] 位置制御の実効上限
SERVO_VEL = f('SERVO_VEL', 10.8)               # [rad/s] (=103rpm)
SERVO_POSITION_GAIN = f('SERVO_POSITION_GAIN', 0.35)

# ---- LiDAR -----------------------------------------------------------------
LIDAR_BEAMS = i('LIDAR_BEAMS', 61)
LIDAR_FOV_DEG = f('LIDAR_FOV_DEG', 180.0)
LIDAR_RANGE = f('LIDAR_RANGE', 2.0)            # 最大距離 [m]
LIDAR_MIN_RANGE = f('LIDAR_MIN_RANGE', 0.04)
LIDAR_MOUNT_X = f('LIDAR_MOUNT_X', 0.105)      # 車体座標の取付位置 [m]
LIDAR_MOUNT_Y = f('LIDAR_MOUNT_Y', 0.0)
LIDAR_MOUNT_Z = f('LIDAR_MOUNT_Z', 0.115)
LIDAR_BLOCKED_M = f('LIDAR_BLOCKED_M', 0.35)   # これ未満をBLOCKEDと判定 [m]
LIDAR_CONE_DEG = f('LIDAR_CONE_DEG', 12.0)     # 側方で「車体幅が通る」円錐の半角
LIDAR_FRONT_HALF_DEG = f('LIDAR_FRONT_HALF_DEG', 20.0)
LIDAR_SIDE_MIN_DEG = f('LIDAR_SIDE_MIN_DEG', 20.0)
LIDAR_SIDE_MAX_DEG = f('LIDAR_SIDE_MAX_DEG', 85.0)

# ---- Jev (外部ollama) ------------------------------------------------------
OLLAMA_HOST = s('OLLAMA_HOST', 'http://127.0.0.1:11434')
JEV_MODEL = s('JEV_MODEL', 'gemma3:12b')
JEV_BRAIN = s('JEV_BRAIN', 'ollama')           # ollama / localjev / mock
JEV_PERIOD = f('JEV_PERIOD', 0.8)              # 判断周期 [s]
JEV_TIMEOUT = f('JEV_TIMEOUT', 20.0)           # HTTPタイムアウト [s]
JEV_TEMPERATURE = f('JEV_TEMPERATURE', 0.0)
JEV_NUM_PREDICT = i('JEV_NUM_PREDICT', 60)
# プロンプトの作り方: rules = 汎用モデル向けに判断ルールを書いて渡す /
#                     bare  = 状態だけ渡す (タスク専用に作られたモデル向け)
JEV_PROMPT = s('JEV_PROMPT', 'rules')
# 出力の縛り方: schema = JSONスキーマでenum固定 (推奨) / json = JSONを要求するだけ /
#               off    = 自由文を正規表現で読む (制約デコード非対応モデル向け)
JEV_FORMAT = s('JEV_FORMAT', 'schema')

# ---- local-jev バックエンド (https://github.com/amithgc/local-jev) ----------
# ollamaとは別のサーバ。POST /v1/systemone に state と型付きの質問を投げると、
# 選択肢ごとの確率つきで choice が返る (TypeSafe Jev のワイヤ互換)。
LOCALJEV_URL = s('LOCALJEV_URL', 'http://127.0.0.1:8765')
LOCALJEV_MODEL = s('LOCALJEV_MODEL', 'jev-latest')   # jev-latest = サーバの既定モデル
LOCALJEV_API_KEY = s('LOCALJEV_API_KEY', '')         # サーバ側で設定していれば必要
# confidence がこれ未満なら決定論ポリシーに落とす (0 = 常にlocal-jevの答えを使う)
JEV_MIN_CONFIDENCE = f('JEV_MIN_CONFIDENCE', 0.0)
# 旋回方向ヒステリシス: 反対側がこれ以上広いときだけ乗り換える [m]
JEV_HYST_MARGIN = f('JEV_HYST_MARGIN', 0.10)
# brain=mock のときの疑似レイテンシ [s]
JEV_MOCK_LATENCY = f('JEV_MOCK_LATENCY', 0.05)
# 疎通確認 (/api/tags, /v1/models) のタイムアウト [s]
JEV_PROBE_TIMEOUT = f('JEV_PROBE_TIMEOUT', 5.0)

# ---- 回避コントローラ ------------------------------------------------------
AVOID_REFLEX_M = f('AVOID_REFLEX_M', 0.22)     # 正面がこれ未満なら即後退 [m]
AVOID_REFLEX_HOLD = f('AVOID_REFLEX_HOLD', 0.6)
AVOID_GOAL_TOL = f('AVOID_GOAL_TOL', 0.25)     # ゴール到達判定 [m]
AVOID_SCAN_DT = f('AVOID_SCAN_DT', 0.1)        # LiDAR周期 [s]
AVOID_HUD_DT = f('AVOID_HUD_DT', 0.2)          # 表示更新周期 [s] (描画はGLが重い)
AVOID_YAW_GAIN = f('AVOID_YAW_GAIN', 1.0/30.0)  # ゴール追従 (deg -> wz)
AVOID_YAW_MAX = f('AVOID_YAW_MAX', 0.4)
AVOID_COMMIT = f('AVOID_COMMIT', 1.0)          # 旋回/後退の最低継続 [s]
AVOID_ESCAPE = f('AVOID_ESCAPE', 2.0)          # 迂回後にゴール追従を切る [s]
AVOID_STUCK_WIN = f('AVOID_STUCK_WIN', 35.0)   # 詰まり判定の窓 [s]
AVOID_STUCK_M = f('AVOID_STUCK_M', 0.15)       # 窓内でこれ未満なら詰まり [m]
AVOID_DURATION = f('AVOID_DURATION', 180.0)    # 制限時間 [s] (0=無制限)
AVOID_RAY_STEP = i('AVOID_RAY_STEP', 2)        # ビーム描画の間引き
# ---- GUIオーバレイ (画面右上の黒パネル) ------------------------------------
# HUD_PANEL_W / HUD_LINE_H / HUD_MARGIN / HUD_PAD の単位は画面の正規化座標
# (画面の半幅・半高を1とした比率。幅0.95なら画面幅の約48%)。
# 文字が小さい/大きいときは HUD_SCALE だけ動かせば全体が相似に変わる。
HUD_SCALE = f('HUD_SCALE', 1.0)             # 文字・行間・パネル幅の一括倍率
HUD_PANEL_DIST = f('HUD_PANEL_DIST', 0.6)   # カメラからパネルまでの距離 [m]
HUD_TEXT_SIZE = f('HUD_TEXT_SIZE', 1.0) * HUD_SCALE
HUD_PANEL_W = f('HUD_PANEL_W', 0.95) * HUD_SCALE
HUD_LINE_H = f('HUD_LINE_H', 0.065) * HUD_SCALE
HUD_MARGIN = f('HUD_MARGIN', 0.02)
HUD_PAD = f('HUD_PAD', 0.013) * HUD_SCALE
# 1行目をパネル上端から何行ぶん下げるか。pybulletの文字はアンカーから上へ
# 伸びるので、1.0未満にすると上端が画面外に見切れる。
HUD_TEXT_DROP = f('HUD_TEXT_DROP', 1.05)
HUD_ARC_T = f('HUD_ARC_T', 1.6)             # 予測軌跡を何秒先まで描くか [s]
HUD_ARC_SEG = i('HUD_ARC_SEG', 10)          # 予測軌跡の分割数
HUD_TRAIL_MIN = f('HUD_TRAIL_MIN', 0.02)    # 軌跡線を伸ばす最小移動量 [m]

# ---- GUIの初期カメラ (距離[m], ヨー[deg], ピッチ[deg], 注視点[m]) --------------
CAM_DIST = f('CAM_DIST', 2.0)
CAM_YAW = f('CAM_YAW', 50.0)
CAM_PITCH = f('CAM_PITCH', -38.0)
CAM_TARGET_X = f('CAM_TARGET_X', 1.2)
CAM_TARGET_Y = f('CAM_TARGET_Y', 0.0)
CAM_TARGET_Z = f('CAM_TARGET_Z', 0.1)
# 歩容のみ(simulate.py)の初期カメラ
GAIT_CAM_DIST = f('GAIT_CAM_DIST', 0.6)
GAIT_CAM_YAW = f('GAIT_CAM_YAW', 40.0)
GAIT_CAM_PITCH = f('GAIT_CAM_PITCH', -25.0)

COURSE = s('COURSE', 'default')
COURSE_SEED = i('COURSE_SEED', 0)
COURSE_N_OBSTACLES = i('COURSE_N_OBSTACLES', 5)
JEV_LOG = s('JEV_LOG', '')                     # 判断ログ(JSONL)の出力先

# ---- メッシュ再生成ツール (service/waist_build2.py, make_urdf.py) ----------
STL_SRC_V1 = s('STL_SRC_V1', os.path.join(ROOT, 'stl', '四足ロボット.stl'))
STL_SRC_V2 = s('STL_SRC_V2', os.path.join(ROOT, 'stl', '四足ロボット2.stl'))
BUILD_OUT_DIR = s('BUILD_OUT_DIR', os.path.join(ROOT, 'build'))
URDF_MESH_PREFIX = s('URDF_MESH_PREFIX', '../meshes')   # URDF内のメッシュ相対パス


def describe():
    """起動時に効いている設定源を1行で返す"""
    return (f'.env {"loaded" if DOTENV_LOADED else "not found"} ({DOTENV})')
