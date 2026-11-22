# 描画の像（`fk-sim`）。FK-E1（`tools/fk_e1_render/Dockerfile`）を元に、
# 描画の芯（`fk_sim.render`。pyrender + OSMesa）だけに絞る。Gazebo・Blender は入れない。
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYOPENGL_PLATFORM=osmesa \
    PYGLET_HEADLESS=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe \
    PYTHONPATH=/work/fk_ws/src/fk_sim

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        libgl1 libglx-mesa0 libgl1-mesa-dri libglu1-mesa mesa-utils \
        libosmesa6 libosmesa6-dev \
        python3 python3-pip python3-yaml python3-dev \
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
        opencv-python-headless \
        pytest \
        "PyOpenGL==3.1.7" \
    && pip3 install --no-cache-dir --no-deps "pyrender==0.1.45"

WORKDIR /work
