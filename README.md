# リンク脚四足ロボット — LiDAR + LLM判断による障害物回避シミュレーション

12自由度のリンク脚四足ロボット(PyBullet)が、前方LiDARの見え方をLLMに渡して
`FORWARD / LEFT / RIGHT / BACK` の4択で判断し、障害物を避けてゴールへ向かいます。
判断バックエンドは **ollama** と **local-jev** を切り替えて比較できます。

このREADMEは**シミュレーションが動くまでの手順**だけを扱います。
仕組み・設計の根拠・実測値は [docs/](#ドキュメント) にあります。

---

## 必要なもの

- Docker (Docker Desktop + WSL2 でも可)
- GUI表示: Linux + X11 (WSL2ならWSLgでそのまま動く)
- 判断バックエンドのどちらか、または両方
  - **ollama**: 別途インストールして `gemma3:12b` を引いておく
  - **local-jev**: このリポジトリの compose で立ち上がる (GPU推奨)
- GPUを使う場合: NVIDIA ドライバ + Docker の GPU パススルー
  (RTX 50系は CUDA wheel が `cu128` 以降必要 → [docs/gpu.md](docs/gpu.md))

Dockerを使わずホストで直接動かす場合は `pip install pybullet numpy` だけで足ります。

---

## 1. 初回セットアップ

### 1-1. 設定ファイルを作る

```bash
cp .env.example .env
```

全92項目に既定値が入っているので、**そのままでも動きます**。

### 1-2. シミュレータのイメージをビルド

```bash
docker compose build
```

pybulletをソースからビルドするので数分かかります (以降はキャッシュされます)。

### 1-3. 判断バックエンドを用意する

**ollama を使う場合** — ホスト側にインストールしてモデルを引きます。

```bash
ollama pull gemma3:12b
```

**local-jev を使う場合** — イメージをビルドして起動します。初回起動時に
Hugging Face からモデルの重みを落とします (`nli-deberta-large` 約0.9GB)。

```bash
# GPU版 (推奨。イメージ約7.5GB)
docker compose --profile localjev-gpu build local-jev-gpu
docker compose --profile localjev-gpu up -d local-jev-gpu

# CPU版 (イメージ約1.4GB。判断が約20倍遅い)
docker compose build local-jev
docker compose up -d local-jev
```

重みのダウンロードが終わって `healthy` になるまで待ちます (GPU版で1〜2分)。

```bash
docker compose --profile localjev-gpu ps
# local-jev-gpu   Up 2 minutes (healthy)   ← これになればOK
```

> 8765番ポートを共有しているので、CPU版とGPU版は**どちらか一方だけ**起動します。
> GPU版にはコンテナ名 `local-jev` の別名を付けてあるので、どちらを起動しても
> `LOCALJEV_URL` の既定 (`http://local-jev:8765`) でそのまま届きます。

### 1-4. 疎通確認

```bash
# 判断だけを既知6シーンで試す (歩容は動かさない)
docker compose up avoid                                  # ollama
ARGS="--selftest" docker compose up avoid                 # ollama / 判断のみ
JEV_BRAIN=localjev ARGS="--selftest" docker compose up avoid   # local-jev / 判断のみ
```

`-> 6/6` と出れば判断レイヤは正常です。

---

## 2. 2回目以降の起動

イメージとモデルの重み (`localjev-models` ボリューム) は残るので、起動だけで済みます。

```bash
# local-jev サーバを起動 (GPU版なら約10秒でhealthy)
docker compose --profile localjev-gpu up -d local-jev-gpu

# あとはシミュレーションを走らせるだけ (次の章)
```

ollama だけで使うならサーバの起動は不要です (ホストで常駐しているため)。

止めるとき:

```bash
docker compose --profile localjev-gpu stop local-jev-gpu    # 止める
docker compose --profile localjev-gpu down                   # コンテナを削除
docker compose --profile localjev-gpu down -v                # 重みも消す (再DLになる)
```

---

## 3. シミュレーションを動かす

### 同じコースを ollama と local-jev で走らせる

GUI表示で見比べるのが分かりやすいです。判断周期 `JEV_PERIOD` は
**バックエンドの推論時間に合わせる**のが重要です (詳細は
[docs/comparison.md](docs/comparison.md))。

ホストで直接 (WSLgならこれが一番手軽):

```bash
# local-jev (GPU)  推論0.06s なので周期0.5s
JEV_PERIOD=0.5 python3 service/avoid_sim.py --gui --brain localjev --course default

# ollama (gemma3:12b)  推論0.40s なので周期0.8s
JEV_PERIOD=0.8 python3 service/avoid_sim.py --gui --brain ollama --course default
```

Docker から (事前に `xhost +local:docker`):

```bash
JEV_BRAIN=localjev JEV_PERIOD=0.5 COURSE=default docker compose --profile gui up avoid-gui
JEV_BRAIN=ollama   JEV_PERIOD=0.8 COURSE=default docker compose --profile gui up avoid-gui
```

乱数コースで条件を揃えて比べる:

```bash
JEV_PERIOD=0.5 python3 service/avoid_sim.py --gui --brain localjev --course random --seed 7
JEV_PERIOD=0.8 python3 service/avoid_sim.py --gui --brain ollama   --course random --seed 7
```

判断ログを残して後で突き合わせる:

```bash
JEV_PERIOD=0.5 python3 service/avoid_sim.py --brain localjev --course default --log lj.jsonl
JEV_PERIOD=0.8 python3 service/avoid_sim.py --brain ollama   --course default --log ol.jsonl
```

### ヘッドレスで走らせる

```bash
JEV_BRAIN=localjev docker compose up avoid         # 1秒ごとに判断をログ出力
docker compose up avoid                             # ollama

# 両方を6シーンで比較 (local-jev サーバを先に healthy にしておく)
docker compose --profile localjev-gpu up compare     # GPU版を使っている場合
docker compose --profile localjev up compare          # CPU版を使っている場合
```

### コース

`default` / `slalom` / `wall` / `empty` / `random` 
定義は `conf/courses.json`。追加は [docs/architecture.md](docs/architecture.md)。

### バックエンド

| `JEV_BRAIN` | 相手 |
|---|---|
| `ollama` (既定) | ollama の `/api/chat` |
| `localjev` | local-jev の `/v1/systemone` |
| `mock` | 推論なし。同じ4択を決定論的に計算 (基準値・ollama不要) |

### 歩容だけ動かす (判断なし)

```bash
docker compose up sim                                      # 前進
MODE=turn_left docker compose up sim                        # 旋回
docker compose --profile gui up sim-gui                     # GUI
ARGS="--keyboard" docker compose --profile gui up sim-gui    # ↑↓←→ と A/D で操縦
```

`MODE`: `forward` / `backward` / `turn_left` / `turn_right` / `strafe_left` /
`strafe_right` / `arc_left` / `stop`

歩容のみは **Ctrl+C まで無限に走ります** (終了時に走行統計を表示)。
秒数を区切るなら `docker compose run --rm sim --mode turn_left --duration 6`。

---

## 4. 環境変数の設定

パラメータはコードに埋め込まず、すべて `.env` から読みます。
雛形は `.env.example` (全92項目、各項目にコメントつき)。

```bash
cp .env.example .env
```

優先順位は **コマンドライン引数 > シェルの環境変数 > `.env` > コード内の既定値**。
`.env` は変えたい行だけ残して他を消しても動きます。読み込みは
`service/config.py` が自前で行うので追加の依存はありません。

```bash
# .env に書く (恒久的)
JEV_BRAIN=localjev
JEV_PERIOD=0.5

# シェルで1回だけ上書き
JEV_PERIOD=0.3 python3 service/avoid_sim.py --gui

# コマンドライン引数が最優先
python3 service/avoid_sim.py --gui --period 0.3
```

起動時に `.env loaded (...)` / `.env not found (...)` を表示するので、
どの設定が効いているか確認できます。

### よく触るキー

| キー | 既定 | 意味 |
|---|---|---|
| `JEV_BRAIN` | `ollama` | 判断バックエンド (`ollama` / `localjev` / `mock`) |
| `JEV_PERIOD` | `0.8` | 判断周期 [s]。**推論時間に合わせる** (localjev+GPU=0.5 / localjev+CPU=1.2 / ollama=0.8) |
| `JEV_MODEL` | `gemma3:12b` | ollama側のモデル |
| `OLLAMA_HOST` | (未設定) | 別マシンのollamaを使うとき。例 `http://192.168.1.50:11434` |
| `LOCALJEV_MODEL` | `jev-latest` | local-jev側のモデル (`jev-latest` はサーバ既定の別名) |
| `LOCAL_JEV_MODEL` | `nli-deberta-large` | local-jev **サーバ**が起動時に載せるモデル (compose用) |
| `COURSE` | `default` | コース |
| `AVOID_DURATION` | `180` | 回避シミュの制限時間 [s]。`0` で Ctrl+C まで無限 (歩容のみの `sim` には効きません) |
| `HUD_SCALE` | `1.0` | 画面右上パネルの文字・行間・幅の一括倍率 |

> `OLLAMA_HOST` に `127.0.0.1` と書くとコンテナ内でコンテナ自身を指してしまいます。
> 未設定なら ネイティブ実行=`127.0.0.1:11434` / Docker=`host.docker.internal:11434`
> が自動で選ばれるので、**別マシンを指すときだけ**設定してください。

全項目は [docs/config.md](docs/config.md) と `.env.example` を参照。

---

## 5. 困ったとき

| 症状 | 対処 |
|---|---|
| `ollama ... -> NG` | ホストでollamaが動いているか、`OLLAMA_HOST` が正しいか確認。不通でも決定論ポリシーで走り続けます (HUDが黄色、統計に `fallback N`) |
| `localjev ... -> NG` | `docker compose --profile localjev-gpu ps` で `healthy` か確認。初回は重みのダウンロード待ち |
| Dockerから `ollama ... -> NG (Connection refused)` | `avoid` / `compare` は `network_mode: host` で動かしています。これを外すとホストのollamaに届きません (ollamaはループバックしか待ち受けないため `host.docker.internal` でも不可。実測で確認) |
| 8765番が使えない | CPU版とGPU版の両方が起動していないか確認。片方だけにする |
| GUIが開かない | `xhost +local:docker` を実行。WSL2なら `echo $DISPLAY` が空でないか確認 |
| 右上の文字が小さい/大きい | `.env` の `HUD_SCALE` を調整。1行目が切れるなら `HUD_TEXT_DROP` を上げる |
| GPUで動いていない気がする | [docs/gpu.md](docs/gpu.md) の確認コマンドを実行 |
| 判断が遅れて障害物に当たる | `JEV_PERIOD` が推論時間に合っているか確認 ([docs/comparison.md](docs/comparison.md)) |
| ollamaもlocal-jevも用意できない | `JEV_BRAIN=mock` で推論なしで動きます |

---

## ドキュメント

| | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 回避シミュレーションの4層構成、GUI表示の読み方、コースの追加 |
| [docs/jev.md](docs/jev.md) | 判断レイヤの実装。プロンプト・JSONスキーマ・バックエンドごとの入力設計 |
| [docs/comparison.md](docs/comparison.md) | ollama と local-jev の実測比較 (レイテンシ・実走・`JEV_PERIOD`の最適値) |
| [docs/gpu.md](docs/gpu.md) | local-jev をGPUで動かす手順と確認方法 |
| [docs/mechanism.md](docs/mechanism.md) | 機構(STL解析結果)・歩容・URDFの技術ノート |
| [docs/config.md](docs/config.md) | 環境変数の全リスト |
| [docs/files.md](docs/files.md) | ファイル構成 |
