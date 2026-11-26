"""記述から Gazebo のワールド（SDF 1.6 の箱だけ）を作る。

壁・床・盤・障害物は箱（LiDAR が当たる形）。天井は見た目だけ（衝突なし）。
色は記述の room から。機体の真の位置を描画サービスが読めるよう
gazebo_ros_state のプラグインを入れる（FK-E1 のワールドには無かった）。
"""

from . import model


def _fmt(v):
    return " ".join(f"{x:.6f}" for x in v)


def _rgba(color, alpha=1.0):
    return f"{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} {alpha:.1f}"


def _box_link(name, size, pose, color, collision=True):
    coll = ""
    if collision:
        coll = f"""
        <collision name="{name}_collision">
          <geometry><box><size>{_fmt(size)}</size></box></geometry>
        </collision>"""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{_fmt(pose)}</pose>
      <link name="link">{coll}
        <visual name="{name}_visual">
          <geometry><box><size>{_fmt(size)}</size></box></geometry>
          <material><ambient>{_rgba(color)}</ambient><diffuse>{_rgba(color)}</diffuse></material>
        </visual>
      </link>
    </model>"""


def build_world(scene):
    """検査済みの scene から world.world の文字列を作る。"""
    room = scene["room"]
    lx, ly = room["size_x"], room["size_y"]
    wall_h, wall_t = room["wall_h"], room["wall_t"]
    ceiling = room["ceiling_h"]
    parts = [_box_link("floor", [lx, ly, 0.1], [lx / 2, ly / 2, -0.05, 0, 0, 0],
                       room["floor_color"])]
    parts.append(_box_link("wall_south", [lx, wall_t, wall_h], [lx / 2, 0.0, wall_h / 2, 0, 0, 0],
                           room["wall_color"]))
    parts.append(_box_link("wall_north", [lx, wall_t, wall_h], [lx / 2, ly, wall_h / 2, 0, 0, 0],
                           room["wall_color"]))
    parts.append(_box_link("wall_west", [wall_t, ly, wall_h], [0.0, ly / 2, wall_h / 2, 0, 0, 0],
                           room["wall_color"]))
    parts.append(_box_link("wall_east", [wall_t, ly, wall_h], [lx, ly / 2, wall_h / 2, 0, 0, 0],
                           room["wall_color"]))
    parts.append(_box_link("ceiling", [lx, ly, 0.1], [lx / 2, ly / 2, ceiling + 0.05, 0, 0, 0],
                           room["wall_color"], collision=False))
    for panel in scene["panels"]:
        cx, cy = panel["center"]
        parts.append(_box_link(f"panel_{panel['id']}",
                               [panel["width"], panel["depth"], panel["height"]],
                               [cx, cy, panel["height"] / 2, 0, 0, model.panel_yaw(panel)],
                               [0.88, 0.86, 0.78]))  # blender/build_scene.py の PANEL_RGB と同じ
    for i, ob in enumerate(scene["obstacles"]):
        cx, cy = ob["center"]
        parts.append(_box_link(f"obstacle_{i}", list(ob["size"]),
                               [cx, cy, ob["size"][2] / 2, 0, 0, ob["yaw"]],
                               [0.4, 0.5, 0.6]))
    light_xml = []
    for i, li in enumerate(scene["lights"]):
        if li["type"] == "point":
            energy = min(1.0, li["energy"])
            light_xml.append(f"""
    <light name="scene_light_{i}" type="point">
      <pose>{_fmt(list(li["position"]) + [0, 0, 0])}</pose>
      <diffuse>{energy:.3f} {energy:.3f} {energy:.3f} 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>30</range><constant>0.7</constant><linear>0.01</linear><quadratic>0.0</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")
    world = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="{scene['name']}">
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>false</shadows>
    </scene>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>0</real_time_factor>
    </physics>
    <light name="fk_sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0.6 0.4</pose>
      <diffuse>0.35 0.35 0.35 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0.3 0.3 -1</direction>
    </light>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
    {''.join(light_xml)}
    {''.join(parts)}
  </world>
</sdf>
"""
    return world
