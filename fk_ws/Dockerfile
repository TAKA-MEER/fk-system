FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=10
# ホストの作業ツリー（マウントした src）に root 所有の __pycache__ を残さない
ENV PYTHONDONTWRITEBYTECODE=1

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/fk_ws

# src・scripts・th_import・th_ref は docker-compose.yml でマウントする。
# build・install・log はコンテナの中に置く（ホストに root 所有のものを作らない）。
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/fk_ws/install/setup.bash ] && source /root/fk_ws/install/setup.bash" >> /root/.bashrc

SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]
