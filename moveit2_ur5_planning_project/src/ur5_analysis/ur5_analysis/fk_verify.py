import math
from typing import Iterable

import numpy as np


UR5_DH = [
    # a, alpha, d, theta_offset
    (0.0, math.pi / 2.0, 0.089159, 0.0),
    (-0.425, 0.0, 0.0, 0.0),
    (-0.39225, 0.0, 0.0, 0.0),
    (0.0, math.pi / 2.0, 0.10915, 0.0),
    (0.0, -math.pi / 2.0, 0.09465, 0.0),
    (0.0, 0.0, 0.0823, 0.0),
]


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def forward_kinematics(joints: Iterable[float]) -> np.ndarray:
    transform = np.eye(4)
    for joint, (a, alpha, d, offset) in zip(joints, UR5_DH):
        transform = transform @ dh_transform(a, alpha, d, joint + offset)
    return transform


def main():
    samples = {
        "home_like": [0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0],
        "target_a_seed": [0.2, -1.3, 1.2, -1.4, -1.57, 0.0],
        "target_b_seed": [-0.4, -1.1, 1.4, -1.7, -1.2, 0.4],
    }

    print("UR5 hand-written FK verification module")
    print("DH parameters use common UR5 nominal dimensions in meters.")
    for name, joints in samples.items():
        transform = forward_kinematics(joints)
        position = transform[:3, 3]
        print(f"\n{name}")
        print(f"  joints(rad): {[round(j, 4) for j in joints]}")
        print(f"  tool position xyz(m): {[round(v, 4) for v in position]}")
        print("  transform:")
        for row in transform:
            print("   ", " ".join(f"{value: .4f}" for value in row))


if __name__ == "__main__":
    main()
