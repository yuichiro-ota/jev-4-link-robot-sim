"""Jev — LiDARを見て「最小判断」で回避方向を決める判断レイヤ (外部ollama)

外部ollamaサーバ (IP指定) をHTTPで叩く。依存は標準ライブラリのみ。

なぜこの形なのか (gemma3で実測しながら決めた):
  * 出力は4択 (FORWARD / LEFT / RIGHT / BACK) のみ。ゴールへの向き合わせは
    下位のコントローラがやるので、Jevは「進むか、どちらへ逃げるか」だけ決める。
  * LiDARの生の距離を渡すと小型モデルは数値比較ができず、gemma3:4bは常に
    FORWARDと答えた。距離 -> OPEN/BLOCKED に前処理し、セクタも5->3に削ったところ
    12bで11/12問正解・中央値0.43秒になった (4bは8/12で実用外。既知の限界)。
  * ollamaの structured outputs (format=JSONスキーマ) でactionをenum固定するので
    パース失敗が原理的に起きない。
  * 推論に0.4〜1秒かかるので別スレッドで投げ、歩容は止めずに前回の判断で歩き続ける。
"""
import json
import threading
import time
import urllib.request

import config
from lidar import occupancy

# 行動 -> ツイスト指令 (vx, vy, wz)。旋回はその場旋回 (前進を混ぜると避けきれない)
ACTIONS = {
    'FORWARD': (1.0, 0.0, 0.0),
    'LEFT':    (0.0, 0.0, 1.0),
    'RIGHT':   (0.0, 0.0, -1.0),
    'BACK':    (-1.0, 0.0, 0.0),
    'STOP':    (0.0, 0.0, 0.0),
}
CHOICES = ['FORWARD', 'LEFT', 'RIGHT', 'BACK']

SCHEMA = {'type': 'object',
          'properties': {'action': {'type': 'string', 'enum': CHOICES},
                         'reason': {'type': 'string'}},
          'required': ['action', 'reason']}

# --- JEV_PROMPT=rules: 汎用モデル向け。判断ルールを明示して渡す ---
SYSTEM = (
    "You steer a small walking robot with a front LiDAR. The scan is reduced to three "
    "sectors, each already judged OPEN or BLOCKED. Pick exactly one action.\n"
    "FORWARD = keep walking (steering toward the goal is automatic)\n"
    "LEFT / RIGHT = turn in place toward that side\n"
    "BACK = back away\n\n"
    "Rules, applied strictly in this order:\n"
    "1. FRONT OPEN -> FORWARD, even if both sides are BLOCKED (a narrow gap is fine).\n"
    "2. FRONT BLOCKED and exactly one side OPEN -> turn to that side.\n"
    "3. FRONT BLOCKED and both sides OPEN -> turn toward the goal.\n"
    "4. FRONT, LEFT and RIGHT all BLOCKED -> BACK.\n"
    "Never turn toward a BLOCKED side, not even to reach the goal.\n"
    "Reason: under 8 words."
)


# --- JEV_PROMPT=bare: タスク専用モデル向け。状態だけを最小限で渡す ---
SYSTEM_BARE = ("Obstacle-avoidance decision layer for a walking robot. "
               "Reply with one action: FORWARD, LEFT, RIGHT or BACK.")


def build_prompt(sec, goal_dist, goal_bearing_deg, style=None):
    """LiDARセクタとゴール方位を、モデルに渡す文面にする

    style='rules' (既定) は汎用モデル向けの読みやすい形。
    style='bare' は専用モデル向けに語数を削った形。
    """
    style = config.JEV_PROMPT if style is None else style
    lab = {k: occupancy(v) for k, v in sec.items()}
    if style == 'bare':
        g = ('LEFT' if goal_bearing_deg > 2 else
             'RIGHT' if goal_bearing_deg < -2 else 'AHEAD')
        return (f"LEFT={lab['LEFT']} FRONT={lab['FRONT']} RIGHT={lab['RIGHT']} "
                f"GOAL={g} DIST={goal_dist:.1f}")
    side = ('goal on the left' if goal_bearing_deg > 2 else
            'goal on the right' if goal_bearing_deg < -2 else 'goal straight ahead')
    return (f"LIDAR   LEFT={lab['LEFT']}   FRONT={lab['FRONT']}   "
            f"RIGHT={lab['RIGHT']}\n"
            f"GOAL    {goal_dist:.1f} m away, {side}")


# local-jev は「state(判断対象) + 型付きの質問」という形。行動は choice の
# criteria として渡す。
#
# 重要: 既定モデル nli-deberta-large は含意(NLI)モデルで、各選択肢を仮説として
# stateに対して判定する。そのため criteria は「方針の説明」ではなく
# 「stateから含意できる言明」でなければならない。実測 (6シーン):
#   方針の説明 ("Correct whenever FRONT is OPEN...")          3/6
#   短い行動名だけ ("Turn left.")                              5/6
#   stateから含意できる言明 (下記)                             6/6
# さらに state もJSONオブジェクトではなく散文にする必要がある (JSONだと3/6)。
# ゴール方向でのタイブレークも、言明に「だから〜すべき」まで書くと両方向で通る。
LOCALJEV_QUESTION = {
    'type': 'choice',
    'instructions': 'Which motion should the walking robot take next',
    'criteria': {
        'FORWARD': ('The space ahead is clear, so the robot can keep walking '
                    'forward.'),
        'LEFT': ('The space ahead is blocked and the left is clear, so the '
                 'robot should turn left.'),
        'RIGHT': ('The space ahead is blocked and the right is clear, so the '
                  'robot should turn right.'),
        'BACK': ('The space ahead, the left and the right are all blocked, so '
                 'the robot must back away.'),
    },
}

_WORD = {'OPEN': 'clear', 'BLOCKED': 'blocked'}


def build_state(sec, goal_dist, goal_bearing_deg):
    """local-jev に渡す state (含意判定が効くように散文で書く)"""
    g = ('left' if goal_bearing_deg > 2 else
         'right' if goal_bearing_deg < -2 else 'straight ahead')
    return (f"The space directly ahead of the robot is "
            f"{_WORD[occupancy(sec['FRONT'])]}. "
            f"The space to its left is {_WORD[occupancy(sec['LEFT'])]}. "
            f"The space to its right is {_WORD[occupancy(sec['RIGHT'])]}. "
            f"The goal is {goal_dist:.1f} m away, to the {g}.")


def system_prompt(style=None):
    style = config.JEV_PROMPT if style is None else style
    return SYSTEM_BARE if style == 'bare' else SYSTEM


def parse_reply(text):
    """モデルの返答から (action, reason) を取り出す

    JSONで返ってくればそれを読み、自由文なら最初に現れた行動語を拾う。
    JEV_FORMAT=off の専用モデル用。
    """
    t = text.strip()
    if t.startswith('{'):
        try:
            d = json.loads(t)
            act = str(d.get('action', '')).upper()
            if act in ACTIONS:
                return act, str(d.get('reason', ''))[:64]
        except (ValueError, AttributeError):
            pass
    up = t.upper()
    hits = sorted((up.find(a), a) for a in CHOICES if a in up)
    if not hits:
        raise ValueError(f'no action word in {t[:60]!r}')
    return hits[0][1], t.replace('\n', ' ')[:64]


class Decision:
    __slots__ = ('action', 'reason', 'latency', 'source', 't', 'conf', 'probs')

    def __init__(self, action='STOP', reason='waiting for Jev', latency=0.0,
                 source='init', t=0.0, conf=None, probs=None):
        self.action, self.reason = action, reason
        self.latency, self.source, self.t = latency, source, t
        self.conf, self.probs = conf, probs        # local-jev のときだけ入る

    def twist(self):
        return ACTIONS[self.action]


def fallback_policy(sec):
    """ollama不通・タイムアウト時の保険 (Jevと同じ4択を決定論的に再現)"""
    if occupancy(sec['FRONT']) == 'OPEN':
        return 'FORWARD', 'fallback: front open'
    l, r = sec['LEFT'], sec['RIGHT']
    if occupancy(l) == 'BLOCKED' and occupancy(r) == 'BLOCKED':
        return 'BACK', 'fallback: all blocked'
    return ('LEFT', 'fallback: left is wider') if l >= r else ('RIGHT', 'fallback: right is wider')


def guard(action, sec):
    """塞がっている側への旋回だけ弾く最終チェック

    12bはたまに「ゴールがそちらだから」と塞がった側へ曲がろうとする (実測1/12)。
    反対が開いていれば向きを反転し、両方塞がっていればBACKにする。
    """
    if action not in ('LEFT', 'RIGHT'):
        return action, None
    side, other = ('LEFT', 'RIGHT') if action == 'LEFT' else ('RIGHT', 'LEFT')
    if occupancy(sec[side]) == 'OPEN':
        return action, None
    if occupancy(sec[other]) == 'OPEN':
        return other, f'guard: {side} blocked, use {other}'
    return 'BACK', f'guard: {side} blocked, both sides blocked'


class Jev:
    """非同期に1件ずつ問い合わせる判断エンジン

    submit() は投げるだけで即返る。結果は latest に入る。
    """

    def __init__(self, host=None, model=None, period=None, timeout=None,
                 brain=None, log_path=None, prompt_style=None, fmt=None):
        host = config.OLLAMA_HOST if host is None else host
        model = config.JEV_MODEL if model is None else model
        period = config.JEV_PERIOD if period is None else period
        timeout = config.JEV_TIMEOUT if timeout is None else timeout
        brain = config.JEV_BRAIN if brain is None else brain
        self.prompt_style = config.JEV_PROMPT if prompt_style is None else prompt_style
        self.fmt = config.JEV_FORMAT if fmt is None else fmt
        if brain == 'localjev':
            host = config.LOCALJEV_URL if host in (None, config.OLLAMA_HOST) else host
            model = config.LOCALJEV_MODEL if model in (None, config.JEV_MODEL) else model
            self.url = host.rstrip('/') + '/v1/systemone'
        else:
            self.url = host.rstrip('/') + '/api/chat'
        self.host, self.model, self.brain = host, model, brain
        self.period, self.timeout = period, timeout
        self.latest = Decision()
        self.busy = False
        self.last_turn = ''      # 旋回方向のヒステリシス用
        self.calls = self.errors = self.reflexes = self.guards = self.hysts = 0
        self.lat_sum = 0.0
        self.last_error = ''
        self._t_last = -1e9
        self._lock = threading.Lock()
        self._log = open(log_path, 'a') if log_path else None

    # ---- 同期API (単発テスト用) ----
    def ask(self, sec, goal_dist, goal_bearing):
        """(action, reason, source) を返す。local-jev のときは conf/probs も"""
        if self.brain == 'mock':
            time.sleep(config.JEV_MOCK_LATENCY)
            a, r = fallback_policy(sec)
            return a, 'mock: ' + r.split(': ')[-1], 'mock'
        if self.brain == 'localjev':
            act, reason, src, _c, _p = self.ask_localjev(sec, goal_dist, goal_bearing)
            return act, reason, src
        prompt = build_prompt(sec, goal_dist, goal_bearing, self.prompt_style)
        payload = {'model': self.model, 'stream': False,
                   'options': {'temperature': config.JEV_TEMPERATURE,
                               'num_predict': config.JEV_NUM_PREDICT},
                   'messages': [{'role': 'system',
                                 'content': system_prompt(self.prompt_style)},
                                {'role': 'user', 'content': prompt}]}
        if self.fmt == 'schema':
            payload['format'] = SCHEMA          # enum固定の制約デコード
        elif self.fmt == 'json':
            payload['format'] = 'json'          # JSONであることだけ要求
        req = urllib.request.Request(self.url, json.dumps(payload).encode(),
                                     {'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=self.timeout) as f:
            msg = json.load(f)['message']['content']
        act, reason = parse_reply(msg)
        return act, reason, 'jev'

    def ask_localjev(self, sec, goal_dist, goal_bearing):
        """local-jev の /v1/systemone に choice 質問を1件投げる

        戻り: (action, reason, source, confidence, probabilities)
        """
        body = json.dumps({'state': build_state(sec, goal_dist, goal_bearing),
                           'model': self.model,
                           'questions': {'action': LOCALJEV_QUESTION}}).encode()
        hdr = {'Content-Type': 'application/json'}
        if config.LOCALJEV_API_KEY:
            hdr['Authorization'] = 'Bearer ' + config.LOCALJEV_API_KEY
        req = urllib.request.Request(self.url, body, hdr)
        with urllib.request.urlopen(req, timeout=self.timeout) as fh:
            res = json.load(fh)
        a = res['answers']['action']
        act = str(a['choice']).upper()
        if act not in ACTIONS:
            raise ValueError(f'unknown action {act!r}')
        conf = float(a.get('confidence', 1.0))
        probs = {k.upper(): float(v) for k, v in (a.get('probabilities') or {}).items()}
        top = ' '.join(f'{k[0]}{probs[k]:.2f}'
                       for k in sorted(probs, key=probs.get, reverse=True)[:3])
        # confidence が低いときは決定論ポリシーに落とす (local-jevの設計思想どおり)
        if conf < config.JEV_MIN_CONFIDENCE:
            fa, fr = fallback_policy(sec)
            return fa, f'low conf {conf:.2f} -> {fr.split(": ")[-1]}', 'lowconf', conf, probs
        return act, f'conf {conf:.2f}  {top}', 'jev', conf, probs

    # ---- 非同期API (シミュレーションループ用) ----
    def submit(self, sim_t, sec, goal_dist, goal_bearing):
        """周期が来ていて前回が終わっていれば新しい問い合わせを投げる"""
        if self.busy or sim_t - self._t_last < self.period:
            return False
        self.busy = True
        self._t_last = sim_t
        threading.Thread(target=self._work, args=(sim_t, dict(sec), goal_dist,
                                                 goal_bearing), daemon=True).start()
        return True

    def _work(self, sim_t, sec, gd, gb):
        t0 = time.time()
        conf = probs = None
        try:
            if self.brain == 'localjev':
                act, reason, src, conf, probs = self.ask_localjev(sec, gd, gb)
            else:
                act, reason, src = self.ask(sec, gd, gb)
            self.calls += 1
        except Exception as e:                       # 不通/タイムアウト/不正応答
            act, reason = fallback_policy(sec)
            src = 'fallback'
            self.errors += 1
            self.last_error = f'{type(e).__name__}: {e}'[:80]
        fixed, note = guard(act, sec)
        if note:
            self.guards += 1
            act, reason, src = fixed, note, 'guard'
        # 旋回方向のヒステリシス: 左右が同程度に開いていてゴールも正面だと
        # Jevは毎回どちらを答えてもよく、左右にがたつくだけで前に出ない。
        # 反対側が明確に広い(10cm以上)ときだけ乗り換える。
        if act in ('LEFT', 'RIGHT'):
            keep = self.last_turn
            if keep and keep != act and occupancy(sec[keep]) == 'OPEN' \
                    and sec[keep] >= sec[act] - config.JEV_HYST_MARGIN:
                self.hysts += 1
                act, reason, src = keep, f'hysteresis: keep turning {keep}', 'hyst'
            self.last_turn = act
        elif act == 'FORWARD':
            self.last_turn = ''
        dt = max(0.0, time.time() - t0)
        self.lat_sum += dt
        with self._lock:
            self.latest = Decision(act, reason, dt, src, sim_t, conf, probs)
        if self._log:
            self._log.write(json.dumps(
                {'t': round(sim_t, 2),
                 'sectors': {k: round(v, 3) for k, v in sec.items()},
                 'labels': {k: occupancy(v) for k, v in sec.items()},
                 'goal_dist': round(gd, 3), 'goal_bearing': round(gb, 1),
                 'action': act, 'reason': reason, 'source': src,
                 'latency': round(dt, 3),
                 'confidence': None if conf is None else round(conf, 3),
                 'probabilities': probs}) + '\n')
            self._log.flush()
        self.busy = False

    def note_reflex(self):
        self.reflexes += 1

    @property
    def avg_latency(self):
        n = self.calls + self.errors
        return self.lat_sum / n if n else 0.0

    def close(self):
        if self._log:
            self._log.close()


def probe(host, timeout=None, brain='ollama'):
    """起動時の疎通確認。(ok, メッセージ, モデル一覧)"""
    timeout = config.JEV_PROBE_TIMEOUT if timeout is None else timeout
    if brain == 'mock':
        return True, 'mock backend', []
    try:
        if brain == 'localjev':
            url = host.rstrip('/') + '/v1/models'
            with urllib.request.urlopen(url, timeout=timeout) as f:
                names = [m['name'] for m in json.load(f).get('models', [])]
        else:
            with urllib.request.urlopen(host.rstrip('/') + '/api/tags',
                                       timeout=timeout) as f:
                names = [m['name'] for m in json.load(f).get('models', [])]
        return True, f'{len(names)} models', names
    except Exception as e:
        return False, f'{type(e).__name__}: {e}', []
