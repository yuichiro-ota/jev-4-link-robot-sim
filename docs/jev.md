# Jev判断レイヤの実装

判断レイヤ (`service/jev.py`) がどうやって4択を選んでいるか、
バックエンドごとに入力の作り方がどう違うかをまとめてあります。

## 判断バックエンドは2つあり、切り替えて比較できる

`JEV_BRAIN` (または `--brain`) で選びます。入力(3セクタのOPEN/BLOCKED + ゴール方向)と
出力(4択)は共通なので、下位の歩容・反射・ゴール追従には一切手を入れずに差し替わります。

| `JEV_BRAIN` | 相手 | 入口 | 返るもの |
|---|---|---|---|
| `ollama` (既定) | ollama | `POST /api/chat` | JSONスキーマでenum固定した `action` + 一言の理由 |
| `localjev` | [local-jev](https://github.com/amithgc/local-jev) | `POST /v1/systemone` | `choice` + **選択肢ごとの確率** + **confidence** |
| `mock` | なし | — | 同じ4択を決定論的に計算 (推論なしの基準値) |

**local-jev は ollama のモデルではなく別のサーバです。** TypeSafe の Jev API の
ワイヤ形式を実装したローカルサーバで、「state(判断対象) + 型付きの質問」を投げると
型付きの答えが返ります。こちらでは4択を `choice` の `criteria` として説明文つきで渡し、
返ってきた `choice` を行動に、`confidence` と `probabilities` をHUDに出します。

```
POST /v1/systemone
{
  "state": {"lidar": {"left":"OPEN","front":"BLOCKED","right":"BLOCKED"},
            "goal":  {"distance_m": 2.4, "direction": "right"}},
  "model": "jev-latest",
  "questions": {"action": {"type":"choice",
      "instructions":"Which motion should the walking robot take next",
      "criteria": {"FORWARD":"Keep walking forward. Correct whenever FRONT is OPEN...",
                   "LEFT":"...", "RIGHT":"...", "BACK":"..."}}}}
```

`JEV_MIN_CONFIDENCE` を0より大きくすると、confidence がそれ未満のときだけ決定論
ポリシーに落ちます (local-jev の README が勧めている使い方)。HUDでは `lowconf` として
別色で表示されます。`ollama` バックエンドには確率が無いのでこの機能は効きません。

## local-jev に渡す形は ollama とまったく違う

既定モデル `nli-deberta-large` は**含意(NLI)モデル**で、選択肢それぞれを仮説として
state に対して判定します。そのため `criteria` は「方針の説明」ではなく
**state から含意できる言明**にしないと当たりません。実測で大きく変わりました。

| criteria の書き方 | 正答 |
|---|---|
| 方針の説明 (`Correct whenever FRONT is OPEN...`) | 3/6 |
| 短い行動名だけ (`Turn left.`) | 5/6 |
| **state から含意できる言明** (採用) | **6/6** |

state も同様で、JSONオブジェクトのまま渡すと 3/6、散文にして 6/6 でした。
さらに言明に「だから〜すべき」まで書くと、左右が同程度に開いているときの
ゴール方向によるタイブレークも両方向で通ります (言明だけだと片方向で外す)。

```python
state = ("The space directly ahead of the robot is blocked. "
         "The space to its left is clear. The space to its right is blocked. "
         "The goal is 2.4 m away, to the right.")
criteria = {
  'FORWARD': 'The space ahead is clear, so the robot can keep walking forward.',
  'LEFT':    'The space ahead is blocked and the left is clear, so the robot should turn left.',
  'RIGHT':   'The space ahead is blocked and the right is clear, so the robot should turn right.',
  'BACK':    'The space ahead, the left and the right are all blocked, so the robot must back away.',
}
```

同じ入力・同じ4択でも、**モデルの種類に合わせて渡し方を作り直す必要がある**という
ことで、ollama側 (ルールを書いて制約デコード) とは逆方向の設計になっています。

`JEV_MIN_CONFIDENCE` の目安: このモデルは正答でも confidence が 0.34〜0.90 と幅広く、
0.5 にすると正しいタイブレーク (0.34) まで弾いてしまいます。既定は 0 (無効) のままに
してあるので、使うなら自分のコースで実測して決めてください。

## ollamaバックエンドのプロンプト

```
LIDAR   LEFT=OPEN   FRONT=BLOCKED   RIGHT=OPEN
GOAL    2.4 m away, goal on the left
```

これだけです。距離の生値ではなく `OPEN` / `BLOCKED` の2値に前処理してあるのが要点で、
gemma3クラスに実数の比較をさせると成立しません (実測は下表)。応答はollamaの
structured outputs (JSONスキーマ + enum) で固定するので、パース失敗が起きません。

- `FRONT` = 正面±20degの**最小**距離 (進路上に何かあるか)
- `LEFT` / `RIGHT` = 側方20〜85degのうち**車体幅(±12deg)が通る開口がある方向**の距離
  → 側方も最小距離にすると、幅の広い壁で3セクタ全部がBLOCKEDになり
    「どちら回りで抜けられるか」が消えて前進と後退を往復します
- しきい値 `BLOCKED_M = 0.35 m` (`lidar.py`)。車体幅0.23m、前進146mm/s、
  判断周期0.8s + 推論0.4sで約0.18m進むので、その倍の余裕

## 判断モデルを差し替える

`--model` (または `JEV_MODEL`) でollama上の任意のモデルを指定できます。専用に
作られたモデルを使う場合は、プロンプトと出力の縛り方も切り替えられます。

| 切替 | 値 | 用途 |
|---|---|---|
| `--prompt-style` / `JEV_PROMPT` | `rules` (既定) | 判断ルールを明示して渡す。汎用モデル向け |
| | `bare` | 状態だけ渡す (`LEFT=OPEN FRONT=BLOCKED ... GOAL=RIGHT DIST=2.4`)。タスク専用モデル向け |
| `--format-mode` / `JEV_FORMAT` | `schema` (既定) | JSONスキーマ+enumで制約デコード。パース失敗が起きない |
| | `json` | JSONであることだけ要求 |
| | `off` | 自由文から行動語を拾う。制約デコード非対応モデル向け |

`--selftest --compare a,b` で既知6シーンの正答率とレイテンシを並べて比較できます。

```bash
python service/avoid_sim.py --selftest --compare gemma3:12b,local-jev
python service/avoid_sim.py --gui --model local-jev --prompt-style bare --format-mode off
```

参考: gemma3:12b を `bare` + `off` で回すと入力が短くなり **0.46s -> 0.10s**、
正答は 6/6 -> 5/6 でした (ルールを外しても大きくは崩れない)。

## プロンプト設計の実測 (12シーンの既知解と照合)

| 入力の与え方 | gemma3:12b | gemma3:4b |
|---|---|---|
| 生の距離5セクタ | 4/7 | 2/7 |
| 3段ラベル(BLOCKED/TIGHT/CLEAR) 5セクタ | 7/9 | 6/9 |
| **3段ラベル 3セクタ** | 10/12 | 7/12 |
| **2値(OPEN/BLOCKED) 3セクタ** ← 採用 | **11/12** | 8/12 |

レイテンシは 12b / 4b とも中央値0.4秒前後 (初回のモデルロードのみ数十秒)。
**4bは判断が安定しないので実用外**です (左右の非対称な誤答が残る)。既定は12b。

