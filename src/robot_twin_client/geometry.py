"""좌표·기하 헬퍼 — **기반 층**이다.

모든 작업 모듈(`pick_n_place` · `stack` · `peg_in_hole`)과 `scene_reset` · `mcp_server` 가 쓰는
순수 계산이다. **표준 라이브러리만 import 한다** — 트윈도, 작업 절차도 모른다. 예전에는 첫 작업
모듈인 `pick_n_place` 안에 있어서 수학만 쓰려는 모듈이 작업 모듈에 의존했고, 그 의존을 피하려던
`peg_in_hole` 이 `down_with_yaw` 를 복사해 두 벌이 됐다 (2026-10-23 이 모듈로 옮겼다).

쿼터니언은 ROS 순서(xyzw)의 dict 이고, pose 는 ``{'position': {x, y, z}, 'orientation': {x, y, z, w}}``
다. 여기서 만드는 pose 는 **손끝(TCP) 기준**이다. 팔에 명령할 프레임으로 바꾸는 것은 그리퍼에 묶인
규약이라 여기 두지 않는다 — Panda Hand(mock · Isaac)는 :mod:`robot_twin_client.panda_hand` 가
맡는다. 프레임을 스스로 변환하는 트윈(펑션베이)은 `grasp_center` 목표를 받는다.
그 기준점은 실제 손끝과 다르므로 호출자가 목표 지점의 의미를 맞춰야 한다.
"""

from __future__ import annotations

from typing import Optional

import math


# x축 180° — 그리퍼가 아래를 본다. **손끝(TCP) 기준 자세**다.
DOWN = {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}


def quat_mul(a: dict, b: dict) -> dict:
    """쿼터니언 Hamilton 곱 ``a ⊗ b`` (성분 순서는 ROS 의 xyzw)."""
    return {'x': a['w'] * b['x'] + a['x'] * b['w'] + a['y'] * b['z'] - a['z'] * b['y'],
            'y': a['w'] * b['y'] - a['x'] * b['z'] + a['y'] * b['w'] + a['z'] * b['x'],
            'z': a['w'] * b['z'] + a['x'] * b['y'] - a['y'] * b['x'] + a['z'] * b['w'],
            'w': a['w'] * b['w'] - a['x'] * b['x'] - a['y'] * b['y'] - a['z'] * b['z']}


def quat_rotate(q: dict, v: tuple[float, float, float]) -> tuple[float, float, float]:
    """벡터 ``v`` 를 쿼터니언 ``q`` 로 회전한다."""
    qx, qy, qz, qw = q['x'], q['y'], q['z'], q['w']
    vx, vy, vz = v
    # t = 2 * (q_vec × v);  v' = v + q_w * t + q_vec × t
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + qy * tz - qz * ty,
            vy + qw * ty + qz * tx - qx * tz,
            vz + qw * tz + qx * ty - qy * tx)


def yaw_of(orientation: dict) -> float:
    """쿼터니언(ROS xyzw)의 z축 회전(rad)."""
    x, y, z, w = orientation['x'], orientation['y'], orientation['z'], orientation['w']
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def down_with_yaw(yaw_rad: float) -> dict:
    """`DOWN`(아래를 보는 자세)을 베이스 z축으로 `yaw_rad` 만큼 돌린 자세 — 접근축이 정확히 연직 하향이다.

    ⚠️ **자세는 절대값으로 만든다 — 현재 자세를 읽어 다음 목표로 주면 안 된다.**
    처짐은 위치만이 아니라 자세도 틀어놓으므로 그 오차가 지워지지 않고 쌓인다.
    실측(2026-09-09, 펑션베이): 그렇게 16회 이동했더니 접근축이 연직에서 **22.3° 기울었다.**
    """
    half = yaw_rad / 2.0
    return {'x': math.cos(half), 'y': math.sin(half), 'z': 0.0, 'w': 0.0}


def above_of(grasp: dict, *, clearance_m: float = 0.20) -> dict:
    """파지 지점 바로 위의 접근 지점. 자세는 같고 z 만 띄운다."""
    return {'position': {**grasp['position'],
                         'z': grasp['position']['z'] + clearance_m},
            'orientation': grasp['orientation']}


def rotated_about_base_z(pose: dict, angle_deg: float) -> dict:
    """베이스 z축(`panda_link0`) 둘레로 돌린 자세. 양수면 위에서 볼 때 반시계다.

    위치는 베이스 원점 둘레의 원호를 따라 돌고, **자세도 같은 각만큼 함께 돈다** —
    그래야 쥔 물체와 그리퍼의 상대 자세가 유지된다. 높이는 건드리지 않는다.
    """
    rad = math.radians(angle_deg)
    sin, cos = math.sin(rad), math.cos(rad)
    p, q = pose['position'], pose['orientation']

    # 자세는 회전 quaternion 을 왼쪽에서 곱한다 (world frame 기준 회전).
    # (0, 0, sin(θ/2), cos(θ/2)) ⊗ q — Hamilton 곱.
    half_sin, half_cos = math.sin(rad / 2.0), math.cos(rad / 2.0)

    return {'position': {'x': p['x'] * cos - p['y'] * sin,
                         'y': p['x'] * sin + p['y'] * cos,
                         'z': p['z']},
            'orientation': {'x': half_cos * q['x'] - half_sin * q['y'],
                            'y': half_cos * q['y'] + half_sin * q['x'],
                            'z': half_cos * q['z'] + half_sin * q['w'],
                            'w': half_cos * q['w'] - half_sin * q['z']}}


def center_of(obj: dict) -> dict:
    """물체의 중심 좌표. **트윈이 주는 두 가지 모양을 모두 받는다.**

    같은 물체가 출처에 따라 다르게 온다:

        reset_scene / begin_task 의 outputs.objects 원소
            {'name': ..., 'type': ..., 'dimensions': [...], 'position': {x, y, z}}

        read_state('scene_objects') 의 objects[<이름>]
            {'type': ..., 'dimensions': [...], 'pose': {'position': {...}, 'orientation': {...}}}

    앞의 것만 받으면 **씬을 읽어서 집으려는 쪽이 `KeyError: 'position'` 으로 죽는다.**
    MCP 도구 설명이 두 출처를 모두 쓰라고 안내하므로 실제로 그렇게 불린다. 이름이 다른
    두 함수를 두는 대신 여기서 흡수한다 — 호출자가 출처를 기억해야 하는 것 자체가 함정이다.

    맞는 키가 하나도 없으면 **어느 모양도 아니라고 말해 준다.** 그냥 KeyError 를 내면
    'position' 한 단어만 남아 무엇을 잘못 넘겼는지 알 수 없다 (실제로 그렇게 겪었다).
    """
    if 'position' in obj:
        return dict(obj['position'])
    pose = obj.get('pose')
    if isinstance(pose, dict) and 'position' in pose:
        return dict(pose['position'])
    raise KeyError(
        "object has no 'position' nor 'pose.position' — reset_scene/begin_task 의 "
        f"objects 원소나 scene_objects 의 값이어야 한다 (받은 키: {sorted(obj)})")


def orientation_of(obj: dict) -> Optional[dict]:
    """물체의 자세. `center_of` 와 같은 두 모양을 모두 받는다. 없으면 None."""
    if 'orientation' in obj:
        return obj['orientation']
    pose = obj.get('pose')
    if isinstance(pose, dict) and 'orientation' in pose:
        return pose['orientation']
    return None


def grasp_yaw_of(obj: dict) -> float:
    """물체를 **면으로** 잡기 위한 손목 yaw(rad).

    물체가 돌아가 있으면 손목도 그만큼 돌려야 손가락이 면에 닿는다. 자세를 0 으로
    고정한 채 돌아간 정육면체를 잡으면 **모서리를 물게 되고**, 손가락 힘이 물체를 죄는
    대신 돌리는 데 쓰인다 — 받침이 좁으면(실린더 위) 반력이 안 생겨 파지 판정이 서지
    않는다 (2026-09-07 Isaac 실측: 면 파지 폭 0.045 / 모서리 0.063~0.071, 후자는
    `at_goal` 이 5 초 안에 서지 않았다).

    **대칭 주기만큼 접는다.** 정육면체는 90° 마다 같은 모양이므로 손목을 최대 45° 만
    돌리면 충분하다 — 접지 않으면 90° 가까이 돌리려다 손목 가동범위를 낭비한다.

    - 실린더 · 구 · 원뿔: 회전 대칭이라 0 을 준다 (돌릴 이유가 없다).
    - 단면이 정사각인 상자: 90° 주기로 접는다.
    - 그 밖의 상자: 180° 주기로 접는다. ⚠️ **어느 변을 가로질러 무는지는 검증하지
      않았다** — 직사각 물체를 쓸 일이 생기면 재고 나서 이 분기를 고친다.
    """
    kind = obj.get('type', 'box')
    if kind in ('cylinder', 'sphere', 'cone'):
        return 0.0
    orientation = orientation_of(obj)
    if orientation is None:
        return 0.0
    dims = obj.get('dimensions') or []
    square = len(dims) >= 2 and abs(float(dims[0]) - float(dims[1])) < 1e-3
    period = math.pi / 2.0 if square else math.pi
    return (yaw_of(orientation) + period / 2.0) % period - period / 2.0


def grasp_pose_of(obj: dict) -> dict:
    """배치된 물체의 파지 지점. 물체 중심을 위에서 잡는다고 본다.

    **손끝(TCP) 기준 pose 를 돌려준다** — Panda Hand 스택(mock · Isaac)에서는 팔에 명령하기
    전에 `panda_hand.to_arm_command` 를 거쳐야 한다.

    `reset_scene` 의 ``outputs.objects`` 원소도, `scene_objects` 로 읽은 값도 받는다
    (`center_of`). 물체를 랜덤하게 놓았으면 **그 자리를 집어야** 하므로 좌표를 따로
    들고 있지 않는다.

    **자세도 물체를 따라간다** (`grasp_yaw_of`) — 물체가 돌아가 있으면 손목을 그만큼
    돌려 면으로 잡는다. 물체에 자세가 없으면 `DOWN` 그대로다.
    """
    return {'position': center_of(obj), 'orientation': down_with_yaw(grasp_yaw_of(obj))}


__all__ = ['DOWN', 'above_of', 'center_of', 'down_with_yaw', 'grasp_pose_of', 'grasp_yaw_of',
           'orientation_of', 'quat_mul', 'quat_rotate', 'rotated_about_base_z', 'yaw_of']
