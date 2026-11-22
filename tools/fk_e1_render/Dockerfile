# FK-E1: 軽い描画（R-GZ / R-PY）を GPU なしで回すためのイメージ。
# Gazebo Classic 11 のカメラ（OGRE）と pyrender（OSMesa）を、Mesa のソフトウェア OpenGL で動かす。
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYOPENGL_PLATFORM=osmesa \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        gazebo \
        xvfb xauth \
        libgl1 libglx-mesa0 libgl1-mesa-dri mesa-utils \
        libosmesa6 libosmesa6-dev \
        python3 python3-pip python3-dev \
    && rm -rf /var/lib/apt/lists/*

# pyrender 0.1.45 は PyOpenGL==3.1.0 を厳密に固定しており、その版には
# OSMesaCreateContextAttribs が無い。依存を外して入れ、PyOpenGL は 3.1.7 を使う。
RUN pip3 install --no-cache-dir \
        "numpy<2" \
        "networkx>=2.8" \
        "pyglet<2" \
        scipy \
        pillow \
        imageio \
        freetype-py \
        six \
        trimesh \
        "PyOpenGL==3.1.7" \
    && pip3 install --no-cache-dir --no-deps "pyrender==0.1.45"

WORKDIR /work
