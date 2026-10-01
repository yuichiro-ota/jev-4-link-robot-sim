# GPUで動かす (local-jev)

`docker/local-jev.Dockerfile` は既定でCPU版torchを入れます (イメージ1.39GB)。
GPUを使う場合は CUDA版torch + デバイス渡しの `local-jev-gpu` を起動します
(イメージ7.5GB)。重みのボリュームはCPU版と共用です。

```bash
docker compose up local-jev                              # CPU版
docker compose --profile localjev-gpu up local-jev-gpu    # GPU版 (8765で衝突するので片方だけ)
```

WSL2 + Docker Desktop でもGPUは通ります (ollamaのようにWindowsホスト側へ出す必要は
ありません)。実測した環境は RTX 5070 Ti / ドライバ591.86 で、`/dev/dxg` 経由・
`--gpus all` で動作しました。

**CUDA版wheelの選び方に注意**: RTX 50系は Blackwell (sm_120) なので `cu128` 以降が
必要です。古い `cu121` 等だと `torch.cuda.is_available()` が True でも実際の計算で
落ちます。`TORCH_INDEX` ビルド引数で切り替えられます。

```bash
# コンテナ内で確認できる
docker run --rm --gpus all local-jev-server-gpu python -c \
  "import torch; print(torch.__version__, torch.cuda.get_arch_list())"
# -> 2.11.0+cu128 ['sm_75','sm_80','sm_86','sm_90','sm_100','sm_120']
```

起動は CPU版54秒 → GPU版11秒。VRAMは `nli-deberta-large` で2.4GBでした。
ollamaの12Bと同時に載せると16GBでも埋まるので、常用するなら片方だけにするか
`LOCAL_JEV_MEMORY_GB` で上限を切ってください。

