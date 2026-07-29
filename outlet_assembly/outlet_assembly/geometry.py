"""플러그 삽입에 쓰는 순수 계산 함수.

ROS도 로봇도 모르는 함수만 둔다 — 그래야 하드웨어 없이 테스트할 수 있다.
"""

import math
import time

import numpy as np


def calculate_angle_diff(pt1, pt2):
    dx = pt2[0] - pt1[0]
    dy = -(pt2[1] - pt1[1]) # OpenCV Y축 반전 보정
    angle = math.degrees(math.atan2(dy, dx))
    
    # -90 ~ 90도로 강제 정규화 (대칭성)
    if angle > 90.0:
        angle -= 180.0
    elif angle <= -90.0:
        angle += 180.0
        
    return angle

def posx_to_matrix(posx):
    x, y, z, a, b, c = posx
    a, b, c = math.radians(a), math.radians(b), math.radians(c)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    R = np.array([
        [ca*cb*cc - sa*sc, -ca*cb*sc - sa*cc, ca*sb],
        [sa*cb*cc + ca*sc, -sa*cb*sc + ca*cc, sa*sb],
        [-sb*cc,           sb*sc,             cb]
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T

def print_log(msg, level="INFO"):
    symbols = {"INFO": "ℹ️", "SUCCESS": "✨", "WARNING": "⚠️", "ERROR": "❌", "ACTION": "🤖"}
    print(f"[{time.strftime('%H:%M:%S')}] {symbols.get(level, '🔹')} {msg}")
