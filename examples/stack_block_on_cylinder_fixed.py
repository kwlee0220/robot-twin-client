#!/usr/bin/env python3
"""빨간 블록을 실린더 위에 쌓는다 — **씬을 읽지 않고, 미리 계산한 좌표 값을 그대로 쓴다.**

`stack_block_on_cylinder.py` 와 같은 동작을 하되, 물체 위치를 `scene_objects` 에서 읽지도,
실행 중에 좌표를 계산하지도 않는다. 각 단계가 트윈에 보내는 pose 는 아래 숫자 그대로다.

    uv run python examples/stack_block_on_cylinder_fixed.py

**robot-twin-client 함수를 단계마다 직접 부른다** — `set_gripper` · `move_linear_verified` ·
`move_to_named_target_verified` 를 차례로 호출하고 결과를 그 자리에서 판정한다.

좌표가 어디서 왔나 (2026-10-19 Isaac 씬, `panda_link0` 기준, m)
--------------------------------------------------------------------

    블록 중심         (0.420, -0.100, 0.325)   50 mm 정육면체, 회전 없음  ← scene_objects 실측
    실린더 중심       (0.420,  0.100, 0.330)   높이 60 · 반지름 30 mm     ← scene_objects 실측
    물린 위치 보정    (-0.11, -1.29, -0.84) mm  '손끝 - 블록 중심', MCP 작업에서 잰 값
    틈                4 mm                      실린더 상면과 블록 사이
    이송 높이 여유    0.15

    쌓은 뒤 블록 중심 z = 0.330 + 0.030 + 0.025 + 0.004            = 0.389
    이송 높이 z        = max(0.325, 0.389) + 0.15                   = 0.539
    놓을 손끝          = (0.420 - 0.00011, 0.100 - 0.00129, 0.389 - 0.00084)
                       = (0.41989, 0.09871, 0.38816)

손끝(TCP) 좌표 → 트윈에 보내는 팔 명령(`panda_link8`) 좌표
---------------------------------------------------------

트윈의 `move_linear` 은 `panda_link8` 을 움직이는데 손끝은 거기서 0.1034 m 더 뻗어 있다.
그리퍼가 아래를 볼 때 그 방향이 -z 이므로 **명령 z = 손끝 z + 0.1034** 이고 x · y 는 같다.
자세는 '아래를 봄'(x 축 180°)에 z 축 +45° 를 곱한 값이다 — `panda_link8` 과 `panda_hand` 가
45° 어긋나 있어, 곱하지 않으면 손가락이 블록 모서리 방향으로 물린다.
(`robot_twin_client.panda_hand.to_arm_command` 가 하는 계산이며, 아래 값은 그것으로 구했다.)

    단계            손끝 (x, y, z)                팔 명령 (x, y, z)
    블록 위         (0.42000, -0.10000, 0.53900)  (0.42000, -0.10000, 0.64240)
    파지            (0.42000, -0.10000, 0.32500)  (0.42000, -0.10000, 0.42840)
    놓을 자리 위    (0.41989,  0.09871, 0.53900)  (0.41989,  0.09871, 0.64240)
    놓을 자리       (0.41989,  0.09871, 0.38816)  (0.41989,  0.09871, 0.49156)
    자세 (공통)                                    (0.9238795, -0.3826834, 0, 0)

⚠️ **씬이 처음 배치여야 한다.** 물체가 실제로 거기 있는지 확인할 수단이 없다 — 옮겨져 있으면
빈 곳을 집으러 간다. 그래도 쥐기에서 `stalled` 가 서지 않으면(빈손) 거기서 멈추므로, 물체를
엉뚱한 곳에 놓는 일은 없다. 위치를 실측해서 쓰려면 `stack_block_on_cylinder.py` 를 쓴다.
처음 배치로 되돌리려면 먼저 `uv run robot-twin-reset` 을 부른다 (트윈의 지정 위치 레시피
`initial` — 코드에서는 `twin.outputs('reset_scene', {'scene': 'initial'})`).

종료 코드: 모든 단계가 성공하면 0, 하나라도 실패하면 1 — **실패한 단계에서 멈추고 이어 가지
않는다.** 쌓였는지는 확인하지 않는다(씬을 읽지 않으므로) — 필요하면 따로 `scene_objects` 를
읽어 본다.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence

import argparse
import sys

from robot_twin_client import DEFAULT_BASE_URL, RobotTwinClient
from robot_twin_client.client import OperationFailed, TwinError
from robot_twin_client.ops import move_linear_verified, move_to_named_target_verified, set_gripper

# 팔 명령(`panda_link8`) 자세 — 그리퍼가 아래를 보고 손가락이 블록 면과 나란하다 (모듈 docstring).
ARM_DOWN = {'x': 0.9238795, 'y': -0.3826834, 'z': 0.0, 'w': 0.0}


class StepFailed(RuntimeError):
    """한 단계가 목표를 이루지 못했다 — 이어 가지 않는다."""


class RecordingTwin(RobotTwinClient):
    """트윈에 보낸 연산을 보낸 순서대로 기록하는 클라이언트.

    `set_gripper` · `move_linear_verified` · `move_to_named_target_verified` 가 모두 내부에서
    `run` 을 부르므로 여기서 가로채면 빠짐없이 남는다. 동작은 바꾸지 않는다.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.log: list[dict[str, Any]] = []

    def run(self, operation: str, inputs: Optional[dict] = None, **kwargs: Any) -> dict:
        state = super().run(operation, inputs, **kwargs)
        self.log.append({'name': operation, 'inputs': dict(inputs or {}), 'state': state})
        return state


def run_fixed_sequence(twin: RobotTwinClient, *, tol_m: float, home: str) -> None:
    """고정 좌표로 집어서 올린다. 실패하면 `StepFailed` 로 그 자리에서 멈춘다."""
    # 1. 그리퍼 열기
    gripper = set_gripper(twin, 'open')
    if not gripper['at_goal']:
        raise StepFailed(f'1. 그리퍼 열기 — at_goal 이 서지 않았다: {gripper}')
    print(f"  1. 그리퍼 열기          폭 {gripper['width'] * 1000:.1f} mm")

    # 2. 블록 위 이송 높이로 — 손끝 (0.42000, -0.10000, 0.53900)
    move = move_linear_verified(
        twin, {'position': {'x': 0.42000, 'y': -0.10000, 'z': 0.64240}, 'orientation': ARM_DOWN},
        tol_m=tol_m)
    if not move['reached']:
        raise StepFailed('2. 블록 위 이송 높이로 — '
                         f'도달하지 못했다 (error_m={move.get("error_m")}, why={move.get("why")})')
    print(f"  2. 블록 위 이송 높이로   오차 {move['error_m'] * 1000:.2f} mm")

    # 3. 파지 높이로 하강 — 손끝 (0.42000, -0.10000, 0.32500) = 블록 중심
    move = move_linear_verified(
        twin, {'position': {'x': 0.42000, 'y': -0.10000, 'z': 0.42840}, 'orientation': ARM_DOWN},
        tol_m=tol_m)
    if not move['reached']:
        raise StepFailed('3. 파지 높이로 하강 — '
                         f'도달하지 못했다 (error_m={move.get("error_m")}, why={move.get("why")})')
    print(f"  3. 파지 높이로 하강      오차 {move['error_m'] * 1000:.2f} mm")

    # 4. 쥐기 — 힘을 주어 닫는다. 씬을 읽지 않으므로 **`stalled` 가 블록이 그 자리에 있었다는
    #    유일한 증거다.** 막히지 않고 닫혔으면 빈손이다.
    gripper = set_gripper(twin, 'grasp')
    if not (gripper['at_goal'] and gripper['stalled']):
        raise StepFailed(f'4. 쥐기 — 빈손이다 (씬이 처음 배치가 아닐 수 있다): {gripper}')
    print(f"  4. 쥐기                폭 {gripper['width'] * 1000:.1f} mm"
          f"  stalled={gripper['stalled']}")

    # 5. 이송 높이로 상승 — 손끝 (0.42000, -0.10000, 0.53900)
    move = move_linear_verified(
        twin, {'position': {'x': 0.42000, 'y': -0.10000, 'z': 0.64240}, 'orientation': ARM_DOWN},
        tol_m=tol_m)
    if not move['reached']:
        raise StepFailed('5. 이송 높이로 상승 — '
                         f'도달하지 못했다 (error_m={move.get("error_m")}, why={move.get("why")})')
    print(f"  5. 이송 높이로 상승      오차 {move['error_m'] * 1000:.2f} mm")

    # 6. 놓을 자리 위로 — 손끝 (0.41989, 0.09871, 0.53900)
    move = move_linear_verified(
        twin, {'position': {'x': 0.41989, 'y': 0.09871, 'z': 0.64240}, 'orientation': ARM_DOWN},
        tol_m=tol_m)
    if not move['reached']:
        raise StepFailed('6. 놓을 자리 위로 — '
                         f'도달하지 못했다 (error_m={move.get("error_m")}, why={move.get("why")})')
    print(f"  6. 놓을 자리 위로        오차 {move['error_m'] * 1000:.2f} mm")

    # 7. 놓을 높이로 하강 — 손끝 (0.41989, 0.09871, 0.38816)
    move = move_linear_verified(
        twin, {'position': {'x': 0.41989, 'y': 0.09871, 'z': 0.49156}, 'orientation': ARM_DOWN},
        tol_m=tol_m)
    if not move['reached']:
        raise StepFailed('7. 놓을 높이로 하강 — '
                         f'도달하지 못했다 (error_m={move.get("error_m")}, why={move.get("why")})')
    print(f"  7. 놓을 높이로 하강      오차 {move['error_m'] * 1000:.2f} mm")

    # 8. 놓기 — 끝까지 열려야(at_goal) 블록이 손을 떠난 것이다.
    gripper = set_gripper(twin, 'open')
    if not gripper['at_goal']:
        raise StepFailed(f'8. 놓기 — 다 열리지 않았다 (아직 쥐고 있을 수 있다): {gripper}')
    print(f"  8. 놓기                폭 {gripper['width'] * 1000:.1f} mm")

    # 9. 이송 높이로 상승 — 손끝 (0.41989, 0.09871, 0.53900)
    move = move_linear_verified(
        twin, {'position': {'x': 0.41989, 'y': 0.09871, 'z': 0.64240}, 'orientation': ARM_DOWN},
        tol_m=tol_m)
    if not move['reached']:
        raise StepFailed('9. 이송 높이로 상승 — '
                         f'도달하지 못했다 (error_m={move.get("error_m")}, why={move.get("why")})')
    print(f"  9. 이송 높이로 상승      오차 {move['error_m'] * 1000:.2f} mm")

    # 10. 복귀 — 블록은 이미 놓였다. 복귀 실패는 알리되 쌓기 결과를 뒤집지 않는다.
    home_result = move_to_named_target_verified(twin, home, velocity_scaling=0.3)
    if home_result['reached']:
        print(f" 10. {home} 복귀")
    else:
        print(f" 10. {home} 복귀 미확인: {home_result.get('why')}")


def _xyz(p: dict) -> str:
    return f"({p['x']:.4f}, {p['y']:.4f}, {p['z']:.4f})"


def print_log(twin: RecordingTwin) -> None:
    """트윈에 보낸 연산과 인자를 순서대로 출력한다."""
    print('\n── 트윈에 보낸 연산 (보낸 순서) ' + '─' * 50)
    for n, entry in enumerate(twin.log, 1):
        inputs, state = entry['inputs'], entry['state']
        if entry['name'] == 'move_linear':
            p, q = inputs['pose']['position'], inputs['pose']['orientation']
            args = (f"pose={_xyz(p)} q=({q['x']:.4f}, {q['y']:.4f}, {q['z']:.4f}, {q['w']:.4f}) "
                    f"velocity_scaling={inputs.get('velocity_scaling')}")
        else:
            args = ' '.join(f'{k}={v}' for k, v in inputs.items())
        out = state.get('outputs') or {}
        if state.get('status') != 'COMPLETED':
            result = f"{state.get('status')} {(state.get('error') or {}).get('code', '')}"
        elif 'final_pose' in out:
            result = f"COMPLETED 팔 {_xyz(out['final_pose']['pose']['position'])}"
        elif 'width' in out:
            result = f"COMPLETED 폭 {out['width'] * 1000:.1f} mm stalled={out.get('stalled')}"
        else:
            result = f"COMPLETED closed_loop={out.get('closed_loop')}"
        print(f'{n:>3}. {entry["name"]:<22} {args}\n     → {result}')


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--url', default=DEFAULT_BASE_URL,
                   help='트윈 서버 루트 URL (http(s)://호스트[:포트])')
    p.add_argument('--twin-id', default='panda01')
    p.add_argument('--tolerance', type=float, default=0.005, help='이동 도달 허용 오차(m)')
    p.add_argument('--home', default='ready', help='끝나고 돌아갈 named target')
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    twin = RecordingTwin(base_url=args.url, twin_id=args.twin_id)
    print(f'트윈 {twin.base_url}/{args.twin_id} — 고정 좌표로 블록을 실린더 위에 올린다\n')
    try:
        run_fixed_sequence(twin, tol_m=args.tolerance, home=args.home)
    except (StepFailed, OperationFailed, TwinError) as exc:
        print(f'\n중단: {exc}')
        print_log(twin)
        return 1
    print_log(twin)
    return 0


if __name__ == '__main__':
    sys.exit(main())
