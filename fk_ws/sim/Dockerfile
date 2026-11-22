# 描画の像（`fk-sim`）。FK-E1（`tools/fk_e1_render/Dockerfile`）を元に、
# 描画の芯（`fk_sim.render`。pyrender + OSMesa）と描画サービス（`sim_render`。
# WP-SIM-03。rclpy と `gazebo_msgs` を使うので ROS の像に寄せる）に絞る。
# Gazebo・Blender は入れない。hve の偽物（昇降部・カメラ・映像の偽物）は
# 像に入れず、読み取り専用でマウントして `/opt/hve-venv` の Python で動かす
# （hve と ss の numpy・aiohttp の版が ROS の Python とぶつかるため）。
FROM ros:humble-ros-base

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
        python3 python3-pip python3-yaml python3-dev python3-venv \
        ros-humble-gazebo-msgs \
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

# hve の偽物の依存（版は hve の `tools/requirements-dev.txt` のまま。ROS の
# system Python とは別 venv に入れる。hve・ss のリポジトリ自体は入れない）。
RUN python3 -m venv /opt/hve-venv \
    && /opt/hve-venv/bin/pip install --no-cache-dir \
        "pytest==9.1.1" \
        "pytest-aiohttp==1.1.1" \
        "aiohttp==3.14.3" \
        "opencv-python-headless==5.0.0.93" \
        "numpy==2.2.6" \
        "smbus2==0.6.1" \
        "tomli==2.4.1"

WORKDIR /work
