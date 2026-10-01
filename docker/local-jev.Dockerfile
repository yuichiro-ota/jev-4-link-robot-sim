# local-jev (https://github.com/amithgc/local-jev) をサーバとして動かすイメージ
#
# 本家リポジトリにDockerfileは無いので、ここで用意しています。
# torch の取得先は TORCH_INDEX で切り替えます。
#   CPU版 (既定) : https://download.pytorch.org/whl/cpu   -> イメージ約1.4GB
#   CUDA版       : https://download.pytorch.org/whl/cu128 -> イメージ約7GB
# CUDA版を使うときは compose 側でデバイスも渡す必要があります (local-jev-gpu)。
FROM python:3.12-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl \
    && rm -rf /var/lib/apt/lists/*

# どのコミットを使うか固定できるように (既定: main)
ARG LOCALJEV_REF=main
ARG TORCH_INDEX=https://download.pytorch.org/whl/cpu

RUN pip install --no-cache-dir --index-url ${TORCH_INDEX} torch \
 && pip install --no-cache-dir \
        "local-jev @ git+https://github.com/amithgc/local-jev@${LOCALJEV_REF}"

# モデルの重みはHugging Faceから初回起動時に落ちる。ボリュームに残して使い回す。
#   nli-deberta-large  約0.9GB  (既定。軽くて速い)
#   llm-qwen3.5-4b     約9.3GB  (本家の既定。精度は高いがCPUでは遅い)
ENV HF_HOME=/models \
    LOCAL_JEV_MODEL=nli-deberta-large

EXPOSE 8765
HEALTHCHECK --interval=15s --timeout=5s --start-period=600s --retries=40 \
    CMD curl -fsS http://127.0.0.1:8765/healthz || exit 1

# --host 0.0.0.0 でコンテナ外から届くようにする (モデル名は LOCAL_JEV_MODEL から)
CMD ["local-jev", "serve", "--host", "0.0.0.0", "--port", "8765"]
