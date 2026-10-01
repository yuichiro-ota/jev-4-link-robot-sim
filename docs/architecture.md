# 回避シミュレーションの構成

## 全体像

`service/avoid_sim.py`。前方LiDARの見え方をJevに渡し、返ってきた行動で歩容を駆動して
ゴールまで進みます。判断内容はGUI上にそのまま表示されます。

```bash
python service/avoid_sim.py --gui
python service/avoid_sim.py --gui --ollama http://192.168.1.50:11434 --model gemma3:12b
python service/avoid_sim.py --gui --course slalom     # default/slalom/wall/empty/random
python service/avoid_sim.py --gui --course random --seed 3
python service/avoid_sim.py --selftest                # 判断だけを6シーンで試験
python service/avoid_sim.py --brain mock --duration 60  # ollama無しで機構だけ確認
```

すべてのパラメータは `.env` に書けます (`--ollama` は `OLLAMA_HOST`、`--model` は
`JEV_MODEL` など)。コマンドライン引数を渡せばそちらが勝ちます。

## 構成 (4層)

| 層 | 周期 (既定) | 中身 |
|---|---|---|
| LiDAR | 0.1s (`AVOID_SCAN_DT`) | `rayTestBatch` で前方 `LIDAR_FOV_DEG`=180deg・`LIDAR_BEAMS`=61本・`LIDAR_RANGE`=2m。3セクタに要約 |
| **Jev** | 0.8s (`JEV_PERIOD`) | ollama または local-jev へHTTP。`FORWARD / LEFT / RIGHT / BACK` の4択を返す |
| 安全反射 | 毎ステップ | 正面が `AVOID_REFLEX_M`=0.22m 未満なら推論を待たず後退 |
| 歩容 | 480Hz (`SIM_RATE_HZ`) | `service/simulate.py` の全方位トロットにツイスト指令を渡す |

表の数値はすべて `.env` の既定値です (括弧内がキー名)。

推論は0.4秒前後かかるので別スレッドで投げ、その間も前回の判断で歩き続けます。
ゴールへの向き合わせ (FORWARD時の微小な旋回) は下位が自動で行うので、
Jevは「進むか、どちらへ逃げるか」だけを決めます = 最小判断。

## 振動対策 (リアクティブ制御の定番)

反応だけで避けると「曲がる→ゴールに引き戻される→また同じ障害物」の
リミットサイクルに入ります。Jevの判断は変えず、下位で3つ入れてあります。

- `--commit 1.0` 旋回/後退は最低1秒続ける (毎周期の上書きを止める)
- `--escape 2.0` 迂回旋回のあと2秒はゴール追従を切って直進で抜ける
- 旋回方向のヒステリシス: 左右が同程度に開いているときは前回と同じ側へ
  (反対が10cm以上広ければ乗り換える)

加えて、塞がっている側へ曲がろうとした判断は `guard` が弾きます
(12bでも実測1/12で「ゴールがそちらだから」と塞がった側へ曲がろうとする)。

## GUI表示

| 表示 | 意味 |
|---|---|
| 緑/赤の扇 | LiDARビーム (赤=当たり) |
| 太い弧 | この先 `HUD_ARC_T`=1.6秒 の予測軌跡。実測の前進速度とヨー速度から描くので、旋回中は弧になる。色は下の駆動層と連動 |
| 右上の黒パネル | **駆動層**・行動・理由・セクタ判定・**ツイスト指令 (vx, vy, wz)** と **wzの出どころ**・実測速度/ヨー速度・ゴール距離と方位・モデルとレイテンシ・統計 |
| 青い線 | 走行軌跡 |

```
GOAL    tracking (P control)          <- いま機体を動かしている層 (色も連動)
JEV: FORWARD
why: front open, proceed
L 2.00 OK | F 0.60 OK | R 2.00 OK
twist  vx+1.00  vy+0.00  wz-0.40      <- 実際に歩容へ渡している指令
wz: goal assist (bear -17, max)       <- wzを誰が決めたか
meas   +94 mm/s  -13.8 deg/s          <- 実測の前進速度とヨー速度
goal 1.83 m  bear -17 deg
gemma3:12b @ 127.0.0.1:11434   0.42 s
calls 22  gd 0  hy 1  rx 3  fb 0
```

### 1行目: いまどの層が動かしているか

制御が多層なので、先頭行に現在の駆動層を出します。予測軌跡の弧もこの色になります。

| 表示 | 色 | 意味 |
|---|---|---|
| `REFLEX  emergency back-off` | 赤 | 正面 `AVOID_REFLEX_M` 未満。推論を待たず後退中 |
| `JEV     avoid turn` | 水色 | Jevが選んだ `LEFT`/`RIGHT` のその場旋回を実行中 |
| `JEV     back off` | 水色 | Jevが `BACK` を選んで後退中 (反射ではない) |
| `ESCAPE  straight after dodge` | 淡青 | 迂回直後。ゴール追従を切って直進で抜けている |
| `GOAL    tracking (P control)` | 緑 | 前進しながらゴール方位のP制御で舵を当てている |
| `GOAL    on course (wz 0)` | 淡緑 | 前進中、ゴール方位ほぼ0で舵ゼロ |
| `GOAL    side blocked, hold` | 橙 | 曲がりたい側が塞がっているので向き直さず直進 |
| `WAIT    for first Jev reply` | 灰 | 起動直後、初回の推論待ち |

ollamaが不通でフォールバックしている間は `[fallback]` が付きます。

終了時には層ごとの駆動時間も出ます (slalom コースの実測)。

```
driven by:
  GOAL_TRACK      35.6 s  60.8%  GOAL    tracking (P control)
  JEV_AVOID       10.3 s  17.6%  JEV     avoid turn
  ESCAPE           6.1 s  10.3%  ESCAPE  straight after dodge
  REFLEX           4.7 s   8.1%  REFLEX  emergency back-off
  WAIT             1.5 s   2.6%  WAIT    for first Jev reply
  GOAL_BLOCKED     0.4 s   0.7%  GOAL    side blocked, hold
```

`wz:` の行は次のどれかになります。**Jevが `FORWARD` を選んでいても `wz` は0ではない**
のがポイントで、これが「FORWARDなのに曲がれる」理由です。

| `wz:` の表示 | 意味 |
|---|---|
| `goal assist (bear +12)` | ゴール方位のP制御。`wz = bear × AVOID_YAW_GAIN` |
| `goal assist (bear +42, max)` | 同じだが `AVOID_YAW_MAX` で飽和 (既定0.4 = 実測13.8deg/s) |
| `escaping -> straight` | 迂回直後なのでゴール追従を切って直進中 |
| `goal side blocked -> 0` | 曲がりたい側が塞がっているので向き直さない |
| `turn in place` | Jevが `LEFT`/`RIGHT` を選んだ (vx=0 のその場旋回) |
| `reflex (no steering)` | 安全反射で後退中 |

ヘッドレス実行でも同じ情報が1秒ごとに出ます。

```
t=   5.0s [REFLEX      ] BACK    (reflex) twist(-1.00,+0.00,+0.00) reflex (no steering)       goal= 2.28m bear=   -1 | L 2.00 OK | F 0.28 BLK | R 2.00 OK
t=   7.1s [JEV_AVOID   ] RIGHT   (jev) twist(+0.00,+0.00,-1.00) turn in place              goal= 2.31m bear=   +4 | L 2.00 OK | F 0.32 BLK | R 2.00 OK
t=  12.0s [GOAL_TRACK  ] FORWARD (jev) twist(+1.00,+0.00,-0.40) goal assist (bear -42, max) goal= 2.04m bear=  -42 | L 2.00 OK | F 0.88 OK | R 1.46 OK
```

黒パネルは画面右上に固定されます。PyBulletのデバッグ描画はワールド座標しか
受け付けず2Dオーバレイが無いため、`getDebugVisualizerCamera()` の視錐台ベクトル
(pybullet自身のマウスピッキングが使うもの) から「画面のこの位置・カメラから
0.6m」のワールド点を毎フレーム逆算して文字を置き、その6%奥に当たり判定なしの
黒い薄板を1枚置いています。カメラからの距離を一定に保つのでズームしても文字の
大きさは変わりません。同時に返る `projectionMatrix` はFOVもアスペクトも入って
いないプレースホルダ(`P[0,0]=P[1,1]=1`)なので使えません(実測で最大0.37mずれる)。

大きさが合わないときは `.env` の `HUD_SCALE` を動かすと文字・行間・パネル幅が
相似に変わります (個別に `HUD_TEXT_SIZE` `HUD_LINE_H` `HUD_PANEL_W` も可)。
文字はアンカー位置から上へ伸びるので、1行目の上端が見切れる場合は
`HUD_TEXT_DROP` (既定1.05行) を大きくすると文字ブロック全体が下がります。

`--log out.jsonl` で1判断1行のJSONLも残せます (セクタ値・ラベル・行動・理由・レイテンシ)。

## 判断バックエンドが不通のとき

起動時に `/api/tags` で疎通確認して結果を表示し、不通でもシミュレーションは
止めません。毎判断ごとに同じ4択を決定論的に再現する `fallback_policy` で走り続け、
GUIは黄色、統計に `fallback N` と最後のエラーが出ます。
明示的にollama無しで回すなら `--brain mock`。

## 障害物コースを足す

`conf/courses.json` の `courses` にエントリを追加するだけです
(`box` は `hx`/`hy` 半寸法、`cyl` は `r` 半径、高さは共通の `height`)。
`random` コースのばらまき範囲も同じファイルの `random` で調整します。

