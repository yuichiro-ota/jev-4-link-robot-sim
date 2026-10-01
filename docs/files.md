# ファイル構成

| ファイル | 内容 |
|---|---|
| `conf/quadruped.urdf` | 12自由度ロボット記述 (5節リンクはknee/rodjで開ループ化) |
| `conf/loop_constraints.json` | ループ閉鎖用の点対点拘束 (rod↔leg、点L) |
| `conf/leg_params.json` | 各脚のリンク寸法・関節軸座標 |
| `conf/courses.json` | 障害物コース定義 (default / slalom / wall / empty / random) |
| `meshes/` | 部品分割済みSTL (コア + 脚ユニット4 + リンク部品16) |
| `service/config.py` | `.env` の読み込みと全パラメータの既定値 |
| `service/fivebar.py` | 5節リンク FK/IK・可動域解析・歩容最適化 |
| `service/simulate.py` | シミュレーション + 全方位歩容コントローラ |
| `service/avoid_sim.py` | Jev判断による障害物回避シミュレーション (LiDAR + ollama + 表示) |
| `service/lidar.py` | 前方2D LiDAR (レイキャスト) とセクタ要約 |
| `service/jev.py` | 外部ollamaを叩く判断レイヤ (プロンプト・非同期・保険) |
| `service/make_urdf.py` | `conf/leg_params.json` → URDF + ループ拘束の生成 |
| `service/waist_build2.py` | 新旧STL→整列済みメッシュ/パラメータの再生成スクリプト |
| `.env.example` | 設定サンプル (`cp .env.example .env`) |
| `docker/local-jev.Dockerfile` | local-jev サーバのイメージ (本家にDockerfileが無いので自作) |
| `Dockerfile` / `docker-compose.yml` | コンテナ実行環境 |

