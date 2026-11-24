#!/usr/bin/env python3
"""빨간 블록(`block_a`)을 실린더(`cylinder_a`) 위에 쌓는다 — MCP 도구로 한 단계씩 한 작업을 옮긴 것.

robot-twin-client 의 API 만 쓴다(HTTP). ROS 도 MCP 서버도 필요 없고, 로봇 스택과 트윈만
떠 있으면 된다.

    uv run python examples/stack_block_on_cylinder.py      # Isaac 트윈(panda01 · 8801)

단계는 MCP 도구 호출과 1:1 이다 (괄호 안이 같은 일을 하는 MCP 도구).

     1. 물체 읽기                (read_state scene_objects)
     2. 파지 자세 계산            (grasp_pose_of_object)        — 로봇은 안 움직인다
     3. 그리퍼 열기              (move_gripper_to_target open)
     4. 블록 위 이송 높이로       (move_linear)
     5. 파지 높이로 하강          (move_linear)
     6. 쥐기                    (move_gripper_to_target grasp)
     7. 이송 높이로 상승          (move_linear)
     8. 물린 위치 실측            (read_states ee_pose · scene_objects)
     9. 놓을 자리 위로            (move_linear)
    10. 놓을 높이로 하강          (move_linear)
    11. 놓기                    (move_gripper_to_target open)
    12. 이송 높이로 상승          (move_linear)
    13. ready 복귀               (move_to_named_target)
    14. 쌓였는지 확인            (read_state scene_objects)

**MCP 도구와 같은 경로를 탄다** — 이동은 `to_arm_command` 로 손끝 목표를 팔 명령으로 바꿔
`move_linear_verified`(허용 오차 5 mm, 정착 대기 없음)로, 복귀는 `move_to_named_target_verified`
(속도 배율 0.3)로 한다. 같은 일을 한 번에 하는 `robot_twin_client.stack.stack()` 은 이동마다
정착을 기다린다는 점만 다르다.

어느 단계든 실패하면 **그 자리에서 멈춘다** — 도달하지 못한 채 다음 동작을 이어 가면 엉뚱한
위치에서 계속되기 때문이다. 끝에 트윈에 보낸 연산과 인자를 순서대로 출력한다.

종료 코드: 쌓기가 확인되면 0, 아니면 1.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence

import argparse
import sys

from robot_twin_client import DEFAULT_BASE_URL, RobotTwinClient
from robot_twin_client.client import OperationFailed, TwinError
from robot_twin_client.ops import move_linear_verified, move_to_named_target_verified, set_gripper
from robot_twin_client.geometry import center_of, grasp_pose_of
from robot_twin_client.panda_hand import to_arm_command
from robot_twin_client.stack import (DEFAULT_CLEARANCE_M, DEFAULT_GAP_M, half_height_of, held_offset,
                                     scene_object, stack_pose_of, verify_stacked)


class StepFailed(RuntimeError):
    """한 단계가 목표를 이루지 못했다 — 이어 가지 않는다."""


class RecordingTwin(RobotTwinClient):
    """트윈에 보낸 연산과 변수 읽기를 **보낸 순서대로** 기록하는 클라이언트.

    헬퍼(`set_gripper` · `move_linear_verified` …)는 모두 `run` 과 `read` 를 거치므로 여기서
    가로채면 빠짐없이 남는다. `outputs` 도 내부에서 `run` 을 부른다.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.log: list[dict[str, Any]] = []

    def run(self, operation: str, inputs: Optional[dict] = None, **kwargs: Any) -> dict:
        state = super().run(operation, inputs, **kwargs)
        self.log.append({'kind': 'op', 'name': operation, 'inputs': dict(inputs or {}),
                         'state': state})
        return state

    def read(self, name: str, *, require_ok: bool = True) -> Any:
        value = super().read(name, require_ok=require_ok)
        self.log.append({'kind': 'read', 'name': name})
        return value


def _at_height(pose: dict, z: float) -> dict:
    """같은 x·y·자세로 높이만 바꾼 pose."""
    return {'position': {**pose['position'], 'z': z}, 'orientation': dict(pose['orientation'])}


def _xyz(p: dict) -> str:
    return f"({p['x']:.4f}, {p['y']:.4f}, {p['z']:.4f})"


def _quat(q: dict) -> str:
    return f"({q['x']:.4f}, {q['y']:.4f}, {q['z']:.4f}, {q['w']:.4f})"


def _move(twin: RobotTwinClient, step: int, label: str, tcp_pose: dict, tol_m: float) -> None:
    """손끝(TCP) 목표로 직선 이동하고 도달을 확인한다 (MCP `move_linear` 와 같다)."""
    result = move_linear_verified(twin, to_arm_command(tcp_pose), tol_m=tol_m)
    if not result['reached']:
        why = result.get('why') or result.get('result', {}).get('error') or '경로가 잘렸거나 접촉했다'
        error = result.get('error_m')
        detail = f'오차 {error * 1000:.2f} mm' if error is not None else why
        raise StepFailed(f'{step}. {label} — 도달하지 못했다 ({detail})')
    print(f"{step:>3}. {label:<22} 손끝 {_xyz(tcp_pose['position'])}  도달 · 오차 "
          f"{result['error_m'] * 1000:.2f} mm")


def _gripper(twin: RobotTwinClient, step: int, label: str, target: str) -> dict:
    """그리퍼를 구동하고 `at_goal` 로 판정한다 (MCP `move_gripper_to_target` 와 같다)."""
    outputs = set_gripper(twin, target)
    if not outputs.get('at_goal'):
        raise StepFailed(f'{step}. {label} — at_goal 이 서지 않았다: {outputs}')
    print(f"{step:>3}. {label:<22} 폭 {outputs['width'] * 1000:.1f} mm · stalled={outputs['stalled']}")
    return outputs


def stack_with_steps(twin: RobotTwinClient, source_name: str, base_name: str, *,
                     clearance_m: float, gap_m: float, tol_m: float, home: str) -> dict:
    """앞의 MCP 작업을 같은 순서로 수행한다. 쌓기 확인 결과(`verify_stacked`)를 돌려준다."""
    # 1. 물체 읽기 — 집을 좌표와 놓을 자리를 실제 배치에서 가져온다.
    source = scene_object(twin, source_name)
    base = scene_object(twin, base_name)
    print(f"  1. 물체 읽기               {source_name} {_xyz(center_of(source))} · "
          f"{base_name} {_xyz(center_of(base))}")

    # 2. 파지 자세 — 물체 중심에 '아래를 보는' 자세. 물체가 돌아가 있으면 손목도 돈다.
    grasp = grasp_pose_of(source)
    stacked_z = center_of(base)['z'] + half_height_of(base) + half_height_of(source) + gap_m
    transit_z = max(grasp['position']['z'], stacked_z) + clearance_m
    print(f"  2. 파지 자세 계산          손끝 {_xyz(grasp['position'])} 자세 "
          f"{_quat(grasp['orientation'])} · 이송 높이 {transit_z:.4f}")

    # 3~7. 집기
    _gripper(twin, 3, '그리퍼 열기', 'open')
    _move(twin, 4, f'{source_name} 위 이송 높이로', _at_height(grasp, transit_z), tol_m)
    _move(twin, 5, '파지 높이로 하강', grasp, tol_m)
    held = _gripper(twin, 6, '쥐기 (grasp)', 'grasp')
    if not held.get('stalled'):
        raise StepFailed('6. 쥐기 — 물체에 막히지 않고 닫혔다 (빈손)')
    _move(twin, 7, '이송 높이로 상승', _at_height(grasp, transit_z), tol_m)

    # 8. 물린 위치 — **들어올린 뒤에** 잰다. 손끝과 물체 중심의 실제 차이만큼 놓을 좌표를 옮긴다.
    offset = held_offset(twin, source_name)
    print(f"  8. 물린 위치 실측          dx={offset['x'] * 1000:+.2f} dy={offset['y'] * 1000:+.2f} "
          f"dz={offset['z'] * 1000:+.2f} mm")

    # 받침은 다시 읽는다 — 집는 동안 스쳐 밀렸을 수 있다. 자세는 집을 때 것을 그대로 쓴다.
    target = stack_pose_of(scene_object(twin, base_name), half_height_of(source), offset,
                           gap_m=gap_m, orientation=grasp['orientation'])

    # 9~12. 놓기
    _move(twin, 9, '놓을 자리 위로', _at_height(target, transit_z), tol_m)
    _move(twin, 10, '놓을 높이로 하강', target, tol_m)
    _gripper(twin, 11, '놓기 (open)', 'open')
    _move(twin, 12, '이송 높이로 상승', _at_height(target, transit_z), tol_m)

    # 13. 복귀 — 물체는 이미 놓였으므로 실패해도 쌓기 판정을 뒤집지 않는다.
    result = move_to_named_target_verified(twin, home, velocity_scaling=0.3)
    print(f" 13. {home} 복귀{'':<17} {'도달' if result['reached'] else '미확인: ' + str(result.get('why'))}")

    # 14. 확인 — `place` 의 성공은 '팔이 움직였다'까지다. 두 물체의 실제 위치로 본다.
    check = verify_stacked(twin, source_name, base_name)
    print(f" 14. 쌓였는지 확인           {'쌓였다' if check['stacked'] else '실패'} — 높이차 "
          f"{check['height_gap_m'] * 1000:.1f} mm (기대 {check['expected_gap_m'] * 1000:.1f}), "
          f"가로 어긋남 {check['lateral_m'] * 1000:.1f} mm")
    return check


def _format_inputs(name: str, inputs: dict) -> str:
    if name == 'move_linear':
        pose = inputs['pose']
        rest = {k: v for k, v in inputs.items() if k != 'pose'}
        return (f"pose={_xyz(pose['position'])} q={_quat(pose['orientation'])} "
                + ' '.join(f'{k}={v}' for k, v in rest.items()))
    return ' '.join(f'{k}={v}' for k, v in inputs.items()) or '-'


def _format_result(name: str, state: dict) -> str:
    status = state.get('status', '?')
    out = state.get('outputs') or {}
    if status != 'COMPLETED':
        return f"{status} {(state.get('error') or {}).get('code', '')}".strip()
    if name == 'move_linear' and 'final_pose' in out:
        return f"COMPLETED 팔 {_xyz(out['final_pose']['pose']['position'])}"
    if name == 'move_gripper_to_target':
        return f"COMPLETED 폭 {out.get('width', 0) * 1000:.1f} mm stalled={out.get('stalled')}"
    if name == 'move_to_named_target':
        return f"COMPLETED closed_loop={out.get('closed_loop')}"
    return status


def print_log(twin: RecordingTwin) -> None:
    """트윈에 보낸 연산과 인자를 순서대로 출력한다. 변수 읽기는 줄여서 보인다."""
    print('\n── 트윈에 보낸 연산 (보낸 순서) ' + '─' * 50)
    n = 0
    for entry in twin.log:
        if entry['kind'] == 'read':
            print(f"     · 변수 읽기 {entry['name']}")
            continue
        n += 1
        print(f"{n:>3}. {entry['name']:<22} {_format_inputs(entry['name'], entry['inputs'])}")
        print(f"     → {_format_result(entry['name'], entry['state'])}")


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    p.add_argument('--url', default=DEFAULT_BASE_URL,
                   help='트윈 서버 루트 URL (http(s)://호스트[:포트])')
    p.add_argument('--twin-id', default='panda01')
    p.add_argument('--source', default='block_a', help='집어서 올릴 물체')
    p.add_argument('--base', default='cylinder_a', help='받침 물체')
    p.add_argument('--clearance', type=float, default=DEFAULT_CLEARANCE_M,
                   help='이송 높이 여유(m) — 두 지점 중 높은 쪽 위로 이만큼 띄운다')
    p.add_argument('--gap', type=float, default=DEFAULT_GAP_M, help='받침 상면과의 틈(m)')
    p.add_argument('--tolerance', type=float, default=0.005, help='이동 도달 허용 오차(m)')
    p.add_argument('--home', default='ready', help='끝나고 돌아갈 named target')
    return p.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    twin = RecordingTwin(base_url=args.url, twin_id=args.twin_id)
    print(f'트윈 {twin.base_url}/{args.twin_id} — {args.source} 를 {args.base} 위에 올린다\n')
    try:
        check = stack_with_steps(twin, args.source, args.base, clearance_m=args.clearance,
                                 gap_m=args.gap, tol_m=args.tolerance, home=args.home)
    except (StepFailed, OperationFailed, TwinError, KeyError) as exc:
        print(f'\n중단: {exc}')
        print_log(twin)
        return 1
    print_log(twin)
    return 0 if check['stacked'] else 1


if __name__ == '__main__':
    sys.exit(main())
