"""씬 리셋 + pick-and-place (사용 설명서 5.9 절).

물체를 레시피대로 배치하고(`reset_scene`), 그 물체를 집어서 베이스 z축 둘레로 회전한
자리에 놓는다.

    reset_scene ─ pick → place ─ return_home

**세션/에피소드 경계는 다루지 않는다.** 이 스크립트는 **로봇 조작만** 한다. 같은
실행을 학습 데이터로 남기려면 경계를 찍는 쪽이 이 흐름을 감싸야 한다 — 조작과 수집을
한 함수에 묶으면 조작만 하고 싶을 때 쓸 수가 없다. 경계까지 포함한 수집 루프는
사용 설명서 5.10 절을 참고한다.

**파지 좌표는 배치된 물체에서 가져온다.** 물체를 랜덤하게 놓고 고정 좌표를 집으면 그
자리에 아무것도 없다. `-x/-y/-z` 로 축별 덮어쓰기는 가능하다.

**좌표는 손끝(TCP) 기준으로 다루고, 팔에 보낼 때만 `to_arm_command` 로 바꾼다.**
트윈의 `move_linear` 는 `panda_link8` 을 움직이는데 손끝은 거기서 약 10.3 cm 더
뻗어 있어, 물체 중심을 그대로 명령하면 **그리퍼가 바닥을 파고든다.**

**이 프로그램은 실제로 로봇을 움직인다.** 파지 판정(`stalled`)은 그 자리에 실제로
물체가 있어야 성립하고, 놓을 자리도 회전 결과가 작업 공간 안이어야 한다.

필요한 것: 트윈과 백엔드 씬 노드(mock 은 `mock_scene_state_node`). 씬 노드는 mock 계열
launch 가 기본으로 띄우므로 제어 스택(`robot_control panda_mock`)만으로 충분하다 —
세션 연산을 쓰지 않으므로 수집 계층(`rdfp`)이 필요 없다.
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import math
import urllib.error

from .client import OperationFailed, RobotTwinClient, TwinError
from .ops import move_linear_verified, move_to_named_target_verified, set_gripper


# x축 180° — 그리퍼가 아래를 본다. **손끝(TCP) 기준 자세**다.
DOWN = {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}

# panda_hand 원점에서 손끝까지의 거리(m). URDF 의 finger 링크 원점(0.0584)에 손가락
# 길이를 더한 값이며, Franka 공식 description 의 `panda_hand_tcp` 프레임과 같다.
TCP_OFFSET_M = 0.1034

# panda_link8 → panda_hand 는 **이동 0, z축 -45° 회전**이다 (URDF `panda_hand_joint`).
# 명령은 link8 기준으로 나가므로 반대 부호로 되돌려 줘야 손가락이 의도한 방향에 선다.
_HAND_YAW_RAD = math.pi / 4


def _quat_mul(a: dict, b: dict) -> dict:
    """쿼터니언 Hamilton 곱 ``a ⊗ b`` (성분 순서는 ROS 의 xyzw)."""
    return {'x': a['w'] * b['x'] + a['x'] * b['w'] + a['y'] * b['z'] - a['z'] * b['y'],
            'y': a['w'] * b['y'] - a['x'] * b['z'] + a['y'] * b['w'] + a['z'] * b['x'],
            'z': a['w'] * b['z'] + a['x'] * b['y'] - a['y'] * b['x'] + a['z'] * b['w'],
            'w': a['w'] * b['w'] - a['x'] * b['x'] - a['y'] * b['y'] - a['z'] * b['z']}


def _quat_rotate(q: dict, v: tuple[float, float, float]) -> tuple[float, float, float]:
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


def to_arm_command(tcp_pose: dict) -> dict:
    """**손끝(TCP) 기준** 목표를 팔에 명령할 **`panda_link8` 기준** pose 로 바꾼다.

    이 보정이 없으면 그리퍼가 바닥을 파고든다. 트윈의 `move_linear` 는 planning
    group `panda_arm` 의 tip link, 즉 `panda_link8` 을 움직이는데 손끝은 거기서
    약 10.3 cm 더 뻗어 있다. 물체 중심을 그대로 명령하면 손끝이 그만큼 아래로
    내려가므로, 5 cm 큐브(중심 z=0.025)의 경우 손끝이 z=-0.078 이 된다.

    두 가지를 되돌린다.

    1. **위치** — 목표 자세로 회전시킨 오프셋만큼 빼서 link8 을 뒤로 물린다.
       DOWN 자세면 결과적으로 z 가 10.3 cm 올라간다.
    2. **자세** — link8 기준 명령에 z축 +45° 를 곱한다. `panda_link8` 과
       `panda_hand` 가 45° 어긋나 있어, 보정하지 않으면 손가락이 큐브 모서리
       방향으로 물린다.

    두 프레임의 **위치는 같다**(이동 0). 그래서 `move_linear_verified` 가
    `outputs.final_pose`(= `panda_hand`)와 비교하는 것은 이 변환 뒤에도 유효하다.
    """
    q = tcp_pose['orientation']
    ox, oy, oz = _quat_rotate(q, (0.0, 0.0, TCP_OFFSET_M))
    p = tcp_pose['position']
    half = _HAND_YAW_RAD / 2.0
    yaw = {'x': 0.0, 'y': 0.0, 'z': math.sin(half), 'w': math.cos(half)}
    return {'position': {'x': p['x'] - ox, 'y': p['y'] - oy, 'z': p['z'] - oz},
            'orientation': _quat_mul(q, yaw)}


def tcp_from_arm_pose(arm_pose: dict) -> dict:
    """팔이 보고한 pose(`ee_pose` / `outputs.final_pose`)를 **손끝(TCP) 기준**으로 바꾼다.

    `to_arm_command` 의 역이며 **위치만** 되돌린다. 팔이 보고하는 자세는 이미
    `panda_hand` 의 것이므로 그대로 쓰면 되고, 위치만 손끝까지 10.3 cm 더 밀어 준다.

    쓰임새는 **쥔 물체를 어디에 물었는지 재는 것**이다. 손끝을 물체 중심에 맞춰
    닫아도 물체는 손가락 사이에서 미끄러진다 — 실측에서 실린더가 12 mm 내려갔다.
    그 상태로 상수 `TCP_OFFSET_M` 만 믿고 내려놓으면 그만큼 띄워 떨어뜨린다.
    """
    q = arm_pose['orientation']
    ox, oy, oz = _quat_rotate(q, (0.0, 0.0, TCP_OFFSET_M))
    p = arm_pose['position']
    return {'position': {'x': p['x'] + ox, 'y': p['y'] + oy, 'z': p['z'] + oz},
            'orientation': dict(q)}


# **도달 판정 전에 정착을 기다린다.** `move_linear` 의 반환은 도달을 뜻하지 않는다 —
# 끝나자마자 읽으면 아직 움직이는 중의 값을 본다. 2026-09-19 Isaac 실측(쌓기 20회)에서
# **실패 7건 중 4건이 "접근 실패"였는데 MoveIt 은 `fraction 1.0` · error code 1(SUCCESS)로
# 전부 성공**이었다 — 정착 없이 5 mm 허용오차로 읽은 값이 떨어진 것이다.
#
# ⚠️ **고정 대기가 아니라 상한이다** — 실제로는 `ee_pose` 가 멈출 때까지 기다린다
# (`wait_until_still`). 펑션베이에서 고정 3 초를 줬다가 하강 뒤 오차를 7.55 mm 로 잘못
# 읽었고(실제 0.04 mm), 그 값으로 다음 목표를 만들어 삽입이 실패했다 (2026-09-11).
SETTLE_SEC = 10.0


def pick(twin: RobotTwinClient, above: dict, grasp: dict, *,
         require_grasp: bool = True) -> bool:
    """접근 → 하강 → 파지 → 상승. 각 단계의 결과를 확인하고 진행한다.

    `require_grasp=False` 는 **mock 백엔드 전용 우회**다. mock 은 물리가 없어 물체를
    물어도 `stalled` 가 서지 않는다. 판정을 그대로 적용하면 mock 에서는 파지에서
    항상 멈춰 **그 이후 단계(상승·이송·놓기)를 전혀 검증할 수 없다.** 끄면 빈손인
    채로 나머지 시퀀스를 진행하므로, 실제 로봇에서는 켜 둔다.

    **임시방편이다 — 걷어낼 조건이 정해져 있다.** 지금 붙일 수 있는 스택이 mock
    뿐이라 두는 것이고, 물리가 있는 스택(gazebo / isaac-sim)을 쓸 수 있게 되면 이
    인자와 `--allow-empty-grasp` 를 함께 없앤다. 파지 실패는 원칙대로 오류다.
    """
    if not set_gripper(twin, 'open')['reached_goal']:
        print('그리퍼를 열지 못했다'); return False

    # 접근 지점까지는 자유 계획이 맞지만 move_to_pose 가 미구현이라 직선으로 간다.
    if not move_linear_verified(twin, to_arm_command(above), settle_sec=SETTLE_SEC)['reached']:
        print('접근 실패'); return False
    if not move_linear_verified(twin, to_arm_command(grasp), settle_sec=SETTLE_SEC)['reached']:
        print('하강 실패'); return False

    # 힘을 주며 닫는다. 물체를 물면 stalled 로 끝나고, 끝까지 닫혔다면 빈손이다.
    if not set_gripper(twin, 'grasp')['stalled']:
        if require_grasp:
            print('아무것도 잡지 못했다'); return False
        print('경고: 파지되지 않았지만 계속한다 (--allow-empty-grasp)')

    return move_linear_verified(twin, to_arm_command(above), settle_sec=SETTLE_SEC)['reached']


def place(twin: RobotTwinClient, above: dict, target: dict) -> bool:
    """접근 → 하강 → 놓기 → 상승. 물체를 쥔 상태에서 시작한다.

    **대기 자세 복귀는 하지 않는다.** 복귀는 놓기가 아니라 그 다음 일이고, 호출자가
    이어서 무엇을 할지(다음 물체를 집는다 / 끝낸다)에 따라 필요 여부가 갈린다.
    복귀까지 원하면 `return_home` 을 이어서 부른다.
    """
    # pick 이 끝난 지점과 여기 above 는 높이가 같다 — 직선으로 이어도 바닥을 스치지 않는다.
    if not move_linear_verified(twin, to_arm_command(above), settle_sec=SETTLE_SEC)['reached']:
        print('놓을 자리 접근 실패'); return False
    if not move_linear_verified(twin, to_arm_command(target), settle_sec=SETTLE_SEC)['reached']:
        print('하강 실패'); return False

    # 놓을 때는 힘이 필요 없다. 끝까지 열려야(reached_goal) 물체가 손을 떠난 것이다.
    if not set_gripper(twin, 'open')['reached_goal']:
        print('그리퍼를 열지 못했다 — 아직 쥐고 있다'); return False

    if not move_linear_verified(twin, to_arm_command(above), settle_sec=SETTLE_SEC)['reached']:
        print('상승 실패 — 물체는 놓았다'); return False

    return True


def return_home(twin: RobotTwinClient, home: str = 'ready') -> bool:
    """SRDF named target 으로 복귀한다.

    관절 공간 이동이라 `to_arm_command` 보정이 필요 없다 — 목표가 좌표가 아니라
    관절값이므로 어느 링크를 기준으로 하느냐는 문제가 되지 않는다.
    """
    retreat = move_to_named_target_verified(twin, home)
    if not retreat['reached']:
        print(f"'{home}' 복귀 실패"
              f"{': ' + retreat['why'] if 'why' in retreat else ''}")
    return retreat['reached']


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


def yaw_of(orientation: dict) -> float:
    """쿼터니언(ROS xyzw)의 z축 회전(rad)."""
    x, y, z, w = orientation['x'], orientation['y'], orientation['z'], orientation['w']
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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

    - 실린더 · 구: 회전 대칭이라 0 을 준다 (돌릴 이유가 없다).
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


def down_with_yaw(yaw_rad: float) -> dict:
    """`DOWN`(아래를 보는 자세)을 베이스 z축으로 `yaw_rad` 만큼 돌린 자세."""
    half = yaw_rad / 2.0
    return {'x': math.cos(half), 'y': math.sin(half), 'z': 0.0, 'w': 0.0}


def grasp_pose_of(obj: dict) -> dict:
    """배치된 물체의 파지 지점. 물체 중심을 위에서 잡는다고 본다.

    **손끝(TCP) 기준 pose 를 돌려준다** — 팔에 명령하기 전에 `to_arm_command` 를
    거쳐야 한다.

    `reset_scene` 의 ``outputs.objects`` 원소도, `scene_objects` 로 읽은 값도 받는다
    (`center_of`). 물체를 랜덤하게 놓았으면 **그 자리를 집어야** 하므로 좌표를 따로
    들고 있지 않는다.

    **자세도 물체를 따라간다** (`grasp_yaw_of`) — 물체가 돌아가 있으면 손목을 그만큼
    돌려 면으로 잡는다. 물체에 자세가 없으면 `DOWN` 그대로다.
    """
    return {'position': center_of(obj), 'orientation': down_with_yaw(grasp_yaw_of(obj))}


def reset_and_pick_and_place(twin: RobotTwinClient, args: argparse.Namespace) -> bool:
    """씬을 새로 만들고 pick-and-place 를 수행한다.

    **세션/에피소드 경계는 다루지 않는다.** 이 스크립트는 로봇 조작만 한다 — 같은
    실행을 학습 데이터로 남기려면 경계를 찍는 쪽이 이 함수를 감싸야 한다. 조작과
    수집을 한 함수에 묶으면 조작만 하고 싶을 때 쓸 수가 없다.

    씬 리셋은 여전히 여기서 한다. 파지 좌표의 출처가 그 결과이기 때문이다 — 물체를
    랜덤하게 놓고 고정 좌표를 집으면 그 자리에 아무것도 없다.
    """
    # 씬 리셋은 `scene` 과 `arm` 을 함께 잡는다 — 팔이 움직이는 중이면 409 다.
    placement = twin.outputs('reset_scene', {'scene': args.scene, 'seed': args.seed})
    objects = placement['objects']
    if not objects:
        print(f"씬 '{args.scene}' 에 물체가 없다 — 집을 대상이 없다")
        return False

    grasp = _grasp_from(objects[0], args)
    target = rotated_about_base_z(grasp, args.angle)
    print(f"물체 {objects[0]['name']} 배치: "
          f"({grasp['position']['x']:.3f}, {grasp['position']['y']:.3f}, "
          f"{grasp['position']['z']:.3f})")

    ok = pick(twin, above_of(grasp, clearance_m=args.clearance), grasp,
              require_grasp=not getattr(args, 'allow_empty_grasp', False))
    if ok:
        ok = place(twin, above_of(target, clearance_m=args.clearance), target)

    # 복귀는 놓기와 별개다. 실패해도 **물체는 이미 놓인 상태**이므로 작업 성패를
    # 뒤집지 않고 경고만 남긴다 — 팔이 어중간한 자세로 남았다는 뜻일 뿐이다.
    if ok:
        return_home(twin, args.home)

    if ok:
        p = target['position']
        print(f"{args.angle}° 회전한 자리에 놓았다 "
              f"(x={p['x']:.3f}, y={p['y']:.3f}, z={p['z']:.3f})")
    return ok


def _grasp_from(obj: dict, args: argparse.Namespace) -> dict:
    """배치된 물체에서 파지 지점을 만들되, 명시된 축은 인자 값으로 덮어쓴다."""
    grasp = grasp_pose_of(obj)
    for axis, value in (('x', args.x), ('y', args.y), ('z', args.z)):
        if value is not None:
            grasp['position'][axis] = value
    return grasp


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    """좌표는 셋업마다 다르므로 인자로 받는다. 기본값은 설명서의 예시 값이다."""
    parser = argparse.ArgumentParser(
        description='Reset the scene, then pick an object and place it rotated about base z')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8801)
    parser.add_argument('--twin-id', default='panda01')
    parser.add_argument('--scene', default='one_cube',
                        help='scene recipe name declared in the twin config')
    parser.add_argument('--seed', type=int, default=0,
                        help='randomisation seed; the same seed gives the same placement')
    # 기본값이 None 이다 — 주지 않으면 **배치된 물체 좌표**를 쓴다. 물체를 랜덤하게
    # 놓고도 고정 좌표를 집으면 그 자리에 아무것도 없다.
    parser.add_argument('-x', type=float, help='override grasp x (m); default: placed object')
    parser.add_argument('-y', type=float, help='override grasp y (m); default: placed object')
    parser.add_argument('-z', type=float, help='override grasp z (m); default: placed object')
    parser.add_argument('--clearance', type=float, default=0.10,
                        help='approach height above the grasp point (m)')
    parser.add_argument('--angle', type=float, default=30.0,
                        help='place angle CCW about the base z axis (deg)')
    parser.add_argument('--allow-empty-grasp', action='store_true',
                        help='파지 판정(stalled)을 건너뛴다 — 물리가 없는 mock 백엔드 전용')
    parser.add_argument('--home', default='ready',
                        help="SRDF named target to return to when done ('panda_arm' group)")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    twin = RobotTwinClient(host=args.host, port=args.port, twin_id=args.twin_id)

    # 팔 연산은 MoveGroup 이 준비되어야 시작된다. 그리퍼만 쓸 거라면 불필요하다.
    try:
        health = twin.health()
    except urllib.error.URLError as exc:
        # 트윈이 아직 안 떴거나 주소가 틀린 경우 — 첫 요청에서 여기로 온다.
        print(f'트윈에 연결하지 못했다 ({twin.base}): {exc.reason}')
        return 1

    if health['move_group'] != 'READY':
        print(f"move_group 미준비: {health.get('move_group_error')}")
        return 1

    try:
        ok = reset_and_pick_and_place(twin, args)
    except OperationFailed as exc:
        # 접수 뒤 실행 실패 — 트윈이 준 사유가 실려 있다 (대개 백엔드 미기동).
        print(f'실행 실패: {exc}')
        return 1
    except TwinError as exc:
        # 400(입력) / 409(자원 점유·E-stop) / 503(백엔드 미준비) — 세션이 만들어지지 않았다.
        # reset_scene 의 409 는 팔이 움직이는 중이라는 뜻이다 (scene + arm 을 함께 잡는다).
        print(f'거절됨: {exc}')
        return 1
    except TimeoutError as exc:
        # 폴링 상한 초과. 로봇은 아직 움직이는 중일 수 있다 — 상태를 확인하고 판단한다.
        print(f'시간 초과: {exc}  (현재 자원: {twin.resources()})')
        return 1

    if not ok:
        print('실패 — 중간 자세로 남아 있을 수 있다')
    return 0 if ok else 1


__all__ = ['DOWN', 'TCP_OFFSET_M', 'above_of', 'grasp_pose_of', 'main', 'pick',
           'place', 'reset_and_pick_and_place', 'return_home',
           'rotated_about_base_z', 'tcp_from_arm_pose', 'to_arm_command']


if __name__ == '__main__':
    raise SystemExit(main())
