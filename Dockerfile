FROM python:3.11-slim

# pybulletのソースビルド用 + GUI(X11/OpenGL)用ライブラリ
# libgl1-mesa-dri はGPUが無い環境でのソフトウェアレンダリング(llvmpipe)用
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        libgl1 libglu1-mesa libgl1-mesa-dri \
        libx11-6 libxext6 libxrender1 libxi6 libxrandr2 libxinerama1 libxcursor1 \
    && rm -rf /var/lib/apt/lists/*

# pybulletはソースからのビルドで数分かかる(このレイヤはキャッシュされる)
RUN pip install --no-cache-dir numpy && \
    pip install --no-cache-dir pybullet

WORKDIR /app
COPY service/ ./service/
COPY conf/ ./conf/
COPY meshes/ ./meshes/

# パラメータは .env ではなく環境変数で渡す (compose の env_file / environment)
# SIGINT/SIGTERM を各スクリプトが受けて統計を出力して終了する
ENTRYPOINT ["python", "-u", "service/simulate.py"]
CMD ["--mode", "forward"]
