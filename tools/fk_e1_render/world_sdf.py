"""FK-E1: 書き出したシーンから Gazebo Classic のワールド（.world）を作る。

- `.dae` を静的モデルとして置く（形は Blender のワールド座標のまま）
- Blender のカメラを Gazebo のカメラセンサにする（同じ位置・同じ画角・1280x720）
- 照明は JSON から近いものを置く（面積光源は Gazebo に無いので、明るい順に点光源で近似し、
  全体を方向光源と ambient で底上げする）

Gazebo のカメラはリンクの +X が前、+Y が左、+Z が上。Blender のカメラは -Z が前、+Y が上、
+X が右なので、列を [前, 左, 上] = [-back, -right, up] に組み替えて SDF の RPY にする。

使い方（コンテナの中）:
  python3 world_sdf.py --export-dir <dir> --out-world <file.world> --save-path <dir>
"""

import argparse
import json
import math
import os

import numpy as np


def rpy_from_matrix(R):
    """SDF の RPY（R = Rz(yaw) Ry(pitch) Rx(roll)）に直す。"""
    sp = -R[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(math.cos(pitch)) < 1e-6:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def camera_pose(cam):
    M = np.array(cam["matrix_world"], float)
    R = M[:3, :3]
    right, up, back = R[:, 0], R[:, 1], R[:, 2]
    Rgz = np.column_stack([-back, -right, up])
    roll, pitch, yaw = rpy_from_matrix(Rgz)
    return list(M[:3, 3]) + [roll, pitch, yaw]


def fmt(v):
    return " ".join(f"{x:.6f}" for x in v)


def build_world(info, mesh_uri, save_path, update_rate, ambient, bg):
    cam = info["camera"]
    W, H = cam["resolution"]
    pose = camera_pose(cam)
    lights = sorted(info.get("lights", []), key=lambda q: q.get("energy", 0.0), reverse=True)
    point_lights = [q for q in lights if q["type"] in {"POINT", "SPOT", "AREA"}][:4]
    light_xml = []
    for i, q in enumerate(point_lights):
        c = q["color"]
        # 面積光源の energy は Gazebo の点光源に直接対応しないので、色だけ使い明るさは固定にする
        diff = " ".join(f"{min(1.0, 0.5 + 0.5 * x):.3f}" for x in c)
        loc = q["location"]
        light_xml.append(
            f"""    <light name="fk_light_{i}" type="point">
      <pose>{fmt(loc)} 0 0 0</pose>
      <diffuse>{diff} 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>30</range><constant>0.7</constant><linear>0.01</linear><quadratic>0.0</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>"""
        )
    world = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="fk_e1">
    <scene>
      <ambient>{ambient} {ambient} {ambient} 1</ambient>
      <background>{bg} {bg} {bg} 1</background>
      <shadows>true</shadows>
    </scene>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>0</real_time_factor>
    </physics>
    <light name="fk_sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0.6 0.4</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.3 0.3 -1</direction>
    </light>
{os.linesep.join(light_xml)}
    <model name="fk_scene">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{mesh_uri}</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
    <model name="fk_camera">
      <static>true</static>
      <pose>{fmt(pose)}</pose>
      <link name="link">
        <sensor name="fk_camera" type="camera">
          <camera>
            <save enabled="true">
              <path>{save_path}</path>
            </save>
            <horizontal_fov>{math.radians(cam['hfov_deg']):.8f}</horizontal_fov>
            <image>
              <width>{W}</width>
              <height>{H}</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>{cam['near']:.4f}</near>
              <far>{cam['far']:.4f}</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>{update_rate}</update_rate>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""
    return world


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--export-dir", required=True)
    ap.add_argument("--out-world", required=True)
    ap.add_argument("--save-path", required=True)
    ap.add_argument("--update-rate", type=float, default=5.0)
    ap.add_argument("--ambient", type=float, default=0.55)
    ap.add_argument("--bg", type=float, default=0.8)
    ap.add_argument("--mesh-uri", default=None)
    args = ap.parse_args()

    with open(os.path.join(args.export_dir, "camera_lights.json"), encoding="utf-8") as f:
        info = json.load(f)
    mesh = args.mesh_uri or ("file://" + os.path.abspath(os.path.join(args.export_dir, "scene.dae")).replace("\\", "/"))
    os.makedirs(args.save_path, exist_ok=True)
    world = build_world(info, mesh, args.save_path, args.update_rate, args.ambient, args.bg)
    os.makedirs(os.path.dirname(os.path.abspath(args.out_world)), exist_ok=True)
    with open(args.out_world, "w", encoding="utf-8") as f:
        f.write(world)
    print(f"[world] {args.out_world} mesh={mesh} lights={min(4, len(info.get('lights', [])))}")


if __name__ == "__main__":
    main()
