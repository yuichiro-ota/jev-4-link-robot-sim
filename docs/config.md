# 環境変数の全リスト

雛形と各項目のコメントは `.env.example` にあります (92項目)。
ここではグループごとの一覧とパス周りの扱いを説明します。

コード内にパラメータをハードコードせず、すべて `.env` から読みます。
雛形は `.env.example`（全項目に既定値とコメント入り）。

```bash
cp .env.example .env
```

優先順位は **コマンドライン引数 > シェルの環境変数 > `.env` > コード内の既定値**。
`.env` は変えたい行だけ残して他を消しても動きます。読み込みは
`service/config.py` が自前で行うので、追加の依存パッケージはありません
(空文字は未設定として扱うため、docker composeで素通しした変数も安全)。

| グループ | 主なキー |
|---|---|
| Jev | `OLLAMA_HOST` `JEV_MODEL` `JEV_BRAIN` `JEV_PERIOD` `JEV_TIMEOUT` `JEV_TEMPERATURE` `JEV_PROMPT` `JEV_FORMAT` `JEV_LOG` |
| LiDAR | `LIDAR_BEAMS` `LIDAR_RANGE` `LIDAR_BLOCKED_M` `LIDAR_CONE_DEG` `LIDAR_MOUNT_X/Y/Z` `LIDAR_FRONT_HALF_DEG` `LIDAR_SIDE_MIN/MAX_DEG` |
| 表示 | `HUD_SCALE` `HUD_TEXT_SIZE` `HUD_PANEL_W` `HUD_LINE_H` `HUD_PANEL_DIST` `HUD_MARGIN` `HUD_PAD` `HUD_TEXT_DROP` |
| 回避 | `AVOID_REFLEX_M` `AVOID_GOAL_TOL` `AVOID_YAW_GAIN` `AVOID_YAW_MAX` `AVOID_COMMIT` `AVOID_ESCAPE` `AVOID_STUCK_WIN/M` `AVOID_DURATION` `AVOID_RAY_STEP` |
| コース | `COURSE` `COURSE_SEED` `COURSE_N_OBSTACLES` |
| 歩容 | `GAIT_FREQ` `GAIT_MAX_STRIDE` `GAIT_MAX_SIDE` |
| サーボ | `SERVO_TORQUE` `SERVO_VEL` `SERVO_POSITION_GAIN` |
| 物理 | `SIM_RATE_HZ` `SIM_SOLVER_ITERATIONS` `SIM_SETTLE_T` `SIM_LOOP_MAX_FORCE` `GROUND_FRICTION` `ROBOT_FRICTION` |
| パス | `ROBOT_URDF` `LEG_PARAMS` `LOOP_CONSTRAINTS` `COURSES_FILE` `MESH_DIR` |
| 再生成ツール | `STL_SRC_V1` `STL_SRC_V2` `BUILD_OUT_DIR` `URDF_MESH_PREFIX` |

起動時に `.env loaded (...)` / `.env not found (...)` を表示するので、
どの設定が効いているか確認できます。

Docker では compose が `.env` を読み、`env_file`（無くても起動する設定）で
コンテナへ渡します。1回だけ上書きしたいときはシェルで渡してください。

```bash
COURSE=slalom docker compose up avoid
OLLAMA_HOST=http://192.168.1.50:11434 docker compose up avoid
ARGS="--course random --seed 3" docker compose --profile gui up avoid-gui
```

