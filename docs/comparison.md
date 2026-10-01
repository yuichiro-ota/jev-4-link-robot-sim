# バックエンド比較の実測

ollama (自前のJev風実装) と local-jev を同一条件で測った記録です。
測定環境: WSL2 / RTX 5070 Ti 16GB / ドライバ591.86 / WSLメモリ11GB。

## 比較のやり方

`--compare` に `backend:model` をカンマ区切りで渡すと、既知6シーンの正答率と
レイテンシ中央値を並べます。

```bash
python service/avoid_sim.py --selftest --compare ollama:gemma3:12b,localjev:jev-latest,mock
docker compose up compare      # 同じことをコンテナで (COMPARE= で指定を変えられる)
```

## 判断レイテンシ (6シーン・中央値)

| backend:model | 正答 | CPU | GPU | VRAM |
|---|---|---|---|---|
| `localjev:nli-deberta-large` (435M) | 6/6 | 0.97 s | **0.05 s** | 2.4 GB |
| `localjev:llm-qwen3.5-2b` | 6/6 | 2.15 s | **0.07 s** | — |
| `ollama:gemma3:12b` | 6/6 | — | 0.41 s | 約9 GB |
| `mock` (推論なし) | 6/6 | 0.05 s | — | — |

**正答率は3者とも同じで、差はレイテンシだけ**です。local-jev がGPUで ollama の
6〜8倍速いのはモデルサイズの差だけではなく、出力トークン数が構造的に違うためです。

- local-jev: `usage.output_tokens: 1` — 選択肢を仮説として判定し、**答えのトークン
  1個のロジットを読むだけ**。生成しない
- ollama: 21トークン生成 (`{"action":"LEFT","reason":"FRONT BLOCKED, LEFT OPEN."}`)

2BのLLMでもGPUで0.07秒なのがその証拠です。

## 実走比較 (default コース、各2回、ヘッドレス)

| 構成 | 所要(平均) | 経路長(平均) | 判断回数 | 緊急後退 | 接触 |
|---|---|---|---|---|---|
| localjev GPU `JEV_PERIOD=0.3` | 26.2 s | 3.17 m | 88 / 73 | **0 / 0** | 0 / 0 |
| **localjev GPU `JEV_PERIOD=0.5`** | **26.7 s** | **3.14 m** | 51 / 50 | **0 / 0** | 0 / 0 |
| localjev GPU `JEV_PERIOD=1.2` | 32.1 s | 3.84 m | 26 / 28 | 0 / 1 | 0 / 0 |
| ollama `JEV_PERIOD=0.8` | 37.4 s | 4.31 m | 51 / 36 | 4 / 2 | 0 / 0 |
| localjev **CPU** `JEV_PERIOD=0.8` | 72.9 s | 8.15 m | 40 | 6 | 2 |
| localjev **CPU** `JEV_PERIOD=1.2` | 56.5 s | 6.34 m | 28 / 32 | 2 / 4 | 0 / 0 |

GUI (実時間) でも同じ傾向でした。

| 構成 | 所要 | 経路長 | レイテンシ | 緊急後退 |
|---|---|---|---|---|
| localjev GPU 0.5s | 23.2 s | 2.90 m | 0.10 s | 0回 (0%) |
| ollama 0.8s | 28.8 s | 3.68 m | 0.46 s | 2回 (6.6%) |

**最も効いた差は緊急後退の回数**です。GPU版 local-jev は8回の走行すべてで0回。
レイテンシ0.06秒なら、機体が反射の発動距離 (`AVOID_REFLEX_M` = 0.22m) に近づく前に
判断が返るので、安全反射が一度も出番を必要としません。ollama (0.4秒) では毎回
2〜4回発動しています。

## `JEV_PERIOD` は推論時間に合わせる

| backend | 推論時間 | 最適な `JEV_PERIOD` |
|---|---|---|
| localjev (GPU) | 0.05〜0.07 s | **0.5**（0.3でも可。ばらつきは増える） |
| localjev (CPU) | 1.0〜1.1 s | **1.2** |
| ollama (GPU, 12B) | 0.40 s | **0.8** |

推論時間より短くすると古い状況に基づく判断が混ざり、長すぎると反応が遅れます。
CPU版 local-jev で 0.8 → 1.2 にしただけで経路長が 8.15m → 6.34m、接触 2 → 0 に
なりました。

走行そのものを比べるなら、同じコースを両方で走らせて最後の統計 (到達/所要/経路長/
接触/層ごとの駆動時間) を見比べます。

```bash
python service/avoid_sim.py --brain ollama   --course slalom --log ollama.jsonl
python service/avoid_sim.py --brain localjev --course slalom --log localjev.jsonl
```

## コース別の動作実績 (ollama gemma3:12b, ヘッドレス)

| コース | 結果 | 所要 | 経路長 | 判断数 | 接触 |
|---|---|---|---|---|---|
| default | GOAL | 22.1s | 2.74m | 25 | 0 |
| slalom | GOAL | 44.1s | 5.23m | 52 | 0 |
| wall | GOAL | 34.6s | 4.26m | 41 | 3 |
| random seed1/3 | GOAL | 33s/40s | 3.7m/4.8m | 38/47 | 0/0 |
| random seed2 | STUCK | — | — | 43 | 2 |

**既知の限界**: 地図も履歴も持たない反応型なので、袋小路やUの字に囲まれた配置
(random seed2 など)では詰まります。回避そのものの成否はJevの4択の質で決まるため、
ollamaのモデルを差し替えると挙動が変わります。

