"""生成器の試験（ROS 2 なし）。fk_sim/test/conftest.py が rclpy を読むので分ける。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
