#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第五关大地图：每次启动时随机摆放障碍柱、门、方环、二维码和图片。

约束：
  - 8 根柱子半径 0.1m、高 1.5m，从 pillar_layouts.json 中随机取一组
    （离线预生成的 APF 可通行布局）
  - 两个门：一个固定在 x = 2，另一个在 x ∈ [-1, 0] 随机，
    左/中/右三种门型里随机取两种
  - 方环内径 1m × 0.75m，中心高 1.2m，x ∈ [-1, 1.5]，y ∈ [-5.3, -3.7]
  - 二维码三种（doubao / qianwen / pangu）随机一个，
    三张图片随机摆在 (5.5,-2)、(5.5,-4)、(5.5,-6)
"""
import json
import os
import random

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String

GAZEBO_MATERIAL = "file://media/materials/scripts/gazebo.material"
WHITE = ("<material><script><uri>%s</uri><name>Gazebo/White</name></script></material>"
         % GAZEBO_MATERIAL)

PILLAR_R = 0.1
PILLAR_H = 1.5
PILLAR_LAYOUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "pillar_layouts.json")
RING_IN_W = 1.0           # y 方向内径
RING_IN_H = 0.75          # z 方向内径
RING_TUBE = 0.105
RING_Z = 1.2


def box_sdf(name, size, z_offset=0.0):
    return """<?xml version='1.0'?>
<sdf version='1.6'><model name='%s'><static>1</static><link name='link'>
<collision name='c'><geometry><box><size>%s</size></box></geometry>
<pose>0 0 %s 0 0 0</pose></collision>
<visual name='v'><geometry><box><size>%s</size></box></geometry>
<pose>0 0 %s 0 0 0</pose>%s</visual>
</link></model></sdf>""" % (name, size, z_offset, size, z_offset, WHITE)


def pillar_sdf(name):
    return """<?xml version='1.0'?>
<sdf version='1.6'><model name='%s'><static>1</static><link name='link'>
<collision name='c'><geometry><cylinder><radius>%s</radius><length>%s</length></cylinder></geometry></collision>
<visual name='v'><geometry><cylinder><radius>%s</radius><length>%s</length></cylinder></geometry>%s</visual>
</link></model></sdf>""" % (name, PILLAR_R, PILLAR_H, PILLAR_R, PILLAR_H, WHITE)


def ring_sdf(name):
    hy = RING_IN_W / 2.0
    hz = RING_IN_H / 2.0
    t = RING_TUBE
    parts = []
    # 左右两根竖梁
    for sign, tag in ((-1, "l"), (1, "r")):
        parts.append(
            "<collision name='c_%s'><geometry><box><size>%s %s %s</size></box></geometry>"
            "<pose>0 %s %s 0 0 0</pose></collision>"
            "<visual name='v_%s'><geometry><box><size>%s %s %s</size></box></geometry>"
            "<pose>0 %s %s 0 0 0</pose>%s</visual>"
            % (tag, t, t, RING_IN_H, sign * (hy + t / 2.0), 0,
               tag, t, t, RING_IN_H, sign * (hy + t / 2.0), 0, WHITE))
    # 上下两根横梁
    for sign, tag in ((-1, "b"), (1, "t")):
        parts.append(
            "<collision name='c_%s'><geometry><box><size>%s %s %s</size></box></geometry>"
            "<pose>0 0 %s 0 0 0</pose></collision>"
            "<visual name='v_%s'><geometry><box><size>%s %s %s</size></box></geometry>"
            "<pose>0 0 %s 0 0 0</pose>%s</visual>"
            % (tag, t, RING_IN_W + 2 * t, t, sign * (hz + t / 2.0),
               tag, t, RING_IN_W + 2 * t, t, sign * (hz + t / 2.0), WHITE))
    # 两条腿，从方环下沿接到地面（模型原点在 RING_Z，地面就是 z = -RING_Z）
    leg_top = -(hz + t / 2.0) - t / 2.0
    leg_h = RING_Z + leg_top
    for sign, tag in ((-1, "legl"), (1, "legr")):
        parts.append(
            "<collision name='c_%s'><geometry><box><size>%s %s %s</size></box></geometry>"
            "<pose>0 %s %s 0 0 0</pose></collision>"
            "<visual name='v_%s'><geometry><box><size>%s %s %s</size></box></geometry>"
            "<pose>0 %s %s 0 0 0</pose>%s</visual>"
            % (tag, t, t, leg_h, sign * (hy + t / 2.0), (leg_top - RING_Z) / 2.0,
               tag, t, t, leg_h, sign * (hy + t / 2.0), (leg_top - RING_Z) / 2.0, WHITE))
    return ("<?xml version='1.0'?><sdf version='1.6'><model name='%s'><static>1</static>"
            "<link name='link'>%s</link></model></sdf>" % (name, "".join(parts)))


def pad_sdf(name, model, size, material):
    return """<?xml version='1.0'?>
<sdf version='1.6'><model name='%s'><static>1</static><link name='link'>
<collision name='c'><geometry><box><size>%s %s 0.002</size></box></geometry></collision>
<visual name='v'><geometry><plane><normal>0 0 1</normal><size>%s %s</size></plane></geometry>
<material><script><uri>model://%s/materials/scripts</uri>
<uri>model://%s/materials/textures</uri><name>%s/Texture</name></script></material>
</visual></link></model></sdf>""" % (name, size, size, size, size, model, model, material)


class Spawner(object):
    def __init__(self):
        self.srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.srv.wait_for_service(timeout=60.0)

    def spawn(self, name, sdf, x, y, z=0.0):
        pose = Pose(position=Point(x=x, y=y, z=z))
        self.srv(model_name=name, model_xml=sdf, robot_namespace="",
                 initial_pose=pose, reference_frame="world")
        rospy.loginfo("spawn %-12s (%.2f, %.2f, %.2f)", name, x, y, z)

    # ---- 各类物体 ----
    def pillars(self):
        with open(PILLAR_LAYOUT_FILE) as f:
            layouts = json.load(f)["layouts"]
        layout = random.choice(layouts)
        pts = [tuple(p) for p in layout["pillars"]]
        rospy.loginfo("柱子布局 #%d（离线生成，尝试 %d 次）", layout["id"], layout["attempts"])
        for i, (x, y) in enumerate(pts):
            self.spawn("pillar_%d" % (i + 1), pillar_sdf("pillar_%d" % (i + 1)),
                       x, y, PILLAR_H / 2.0)
        return pts

    def doors(self):
        xs = [2.0]
        while len(xs) < 2:
            x = random.uniform(-1.0, 0.0)
            if all(abs(x - o) > 1.5 for o in xs):
                xs.append(x)
        xs.sort()
        slots = random.sample([0, 1, 2], 2)      # 0 左门 / 1 中门 / 2 右门
        info = []
        for i, (x, slot) in enumerate(zip(xs, slots)):
            if slot == 0:                          # 通道在 y ∈ [-1, 0]
                segs = [(-2.0, 2.0)]
            elif slot == 2:                        # 通道在 y ∈ [-3, -2]
                segs = [(-1.0, 2.0)]
            else:                                  # 通道在 y ∈ [-2, -1]
                segs = [(-0.5, 1.0), (-2.5, 1.0)]
            for j, (cy, length) in enumerate(segs):
                name = "door%d_%d" % (i + 1, j + 1)
                self.spawn(name, box_sdf(name, "0.2 %s 2" % length), x, cy, 1.0)
            info.append((x, slot))
        return info

    def ring(self):
        x = random.uniform(-1.0, 1.5)
        y = random.uniform(-5.3, -3.7)
        self.spawn("ring", ring_sdf("ring"), x, y, RING_Z)
        return x, y

    def vision(self):
        answer = random.choice(["doubao", "qianwen", "pangu"])
        self.spawn("qr_code",
                   pad_sdf("qr_code", "qr_" + answer, 0.5, "Qr" + answer.capitalize()),
                   3.5, -4.0, 0.003)
        names = ["doubao", "qianwen", "pangu"]
        random.shuffle(names)
        for name, y in zip(names, (-2.0, -4.0, -6.0)):
            self.spawn("pad_" + name,
                       pad_sdf("pad_" + name, "pad_" + name, 0.6, "Pad" + name.capitalize()),
                       5.5, y, 0.003)
        return answer, dict(zip(names, (-2.0, -4.0, -6.0)))


def main():
    rospy.init_node("mission_spawner")
    sp = Spawner()

    pillars = sp.pillars()
    doors = sp.doors()
    ring = sp.ring()
    answer, layout = sp.vision()

    pub = rospy.Publisher("/mission/qr_text", String, queue_size=1, latch=True)
    rospy.sleep(0.5)
    pub.publish(String(data=answer))

    rospy.loginfo("=" * 46)
    rospy.loginfo("柱子: %s" % ["(%.2f, %.2f)" % p for p in pillars])
    rospy.loginfo("门:   %s" % [("x=%.2f/%s" % (x, ["左", "中", "右"][s])) for x, s in doors])
    rospy.loginfo("方环: (%.2f, %.2f)" % ring)
    rospy.loginfo("二维码内容: %s" % answer)
    rospy.loginfo("图片位置: %s" % layout)
    rospy.loginfo("=" * 46)


if __name__ == "__main__":
    main()
