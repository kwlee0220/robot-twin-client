"""씬 리셋 + pick-and-place (사용자 가이드 5.9 절).

물체를 레시피대로 배치하고(`reset_scene`), 그 물체를 집어서 베이스 z축 둘레로 회전한
자리에 놓는다.

    reset_scene ─ pick → place ─ return_home

**세션/에피소드 경계는 다루지 않는다.** 이 스크립트는 **로봇 조작만** 한다. 같은
실행을 학습 데이터로 남기려면 경계를 찍는 쪽이 이 흐름을 감싸야 한다 — 조작과 수집을
한 함수에 묶으면 조작만 하고 싶을 때 쓸 수가 없다. 경계까지 포함한 수집 루프는
사용자 가이드 5.10 절을 참고한다.

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
import urllib.error

from .client import (DEFAULT_BASE_URL, OperationFailed, RobotTwinClient, TwinError, WrongBackend,
                     require_backend)
# 좌표·기하 헬퍼는 기반 층(`geometry` · `panda_hand`)에 있다. 예전 import 경로
# (`from robot_twin_client.pick_n_place import grasp_pose_of`)가 계속 되도록 여기서 다시 내보낸다.
from .geometry import (DOWN, above_of, center_of, down_with_yaw, grasp_pose_of, grasp_yaw_of,
                       orientation_of, rotated_about_base_z, yaw_of)
from .ops import move_linear_verified, move_to_named_target_verified, set_gripper
from .panda_hand import TCP_OFFSET_M, tcp_from_arm_pose, to_arm_command


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
    parser.add_argument('--url', default=DEFAULT_BASE_URL,
                        help='트윈 서버 루트 URL (http(s)://호스트[:포트])')
    parser.add_argument('--twin-id', default='panda01', help='Isaac 트윈의 id')
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
    twin = RobotTwinClient(base_url=args.url, twin_id=args.twin_id, agent='robot-twin-pick')

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
    # 좌표를 Panda Hand 규약(`to_arm_command`)으로 보정하므로 그 그리퍼의 스택에서만 맞다 —
    # 프레임을 스스로 바꾸는 펑션베이 트윈에서는 보정이 두 번 걸려 약 10 cm 어긋난다.
    try:
        require_backend(twin, 'panda_mock', 'panda_isaac', purpose='집기·놓기(Panda Hand 보정)')
    except WrongBackend as exc:
        print(exc)
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


__all__ = ['main', 'pick', 'place', 'reset_and_pick_and_place', 'return_home',
           # 기반 층에서 다시 내보내는 이름 (호환)
           'DOWN', 'TCP_OFFSET_M', 'above_of', 'center_of', 'down_with_yaw', 'grasp_pose_of',
           'grasp_yaw_of', 'orientation_of', 'rotated_about_base_z', 'tcp_from_arm_pose',
           'to_arm_command', 'yaw_of']


if __name__ == '__main__':
    raise SystemExit(main())
