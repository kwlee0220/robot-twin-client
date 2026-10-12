"""씬 리셋 + pick-and-place 전체 흐름 테스트.

이 스크립트는 **조작만** 한다 — 세션/에피소드 경계는 다루지 않는다.
경계까지 필요한 흐름은 MCP 서버의 `begin_task` / `end_task` 를 쓴다.
"""

from __future__ import annotations

import argparse

import pytest

from robot_twin_client.pick_n_place import (
    TCP_OFFSET_M, grasp_pose_of, reset_and_pick_and_place
)

from conftest import SCENE_OBJECTS, FakeTwin


def _args(**overrides) -> argparse.Namespace:
    base = dict(scene='one_cube', seed=7,
                x=None, y=None, z=None, clearance=0.1, angle=30.0, home='ready')
    base.update(overrides)
    return argparse.Namespace(**base)


# 끝까지 닫힌 그리퍼 = 빈손. mock 은 파지에 성공해도 이 응답을 준다.
EMPTY_GRIP = {'open': {'reached_goal': True, 'stalled': False, 'position': 0.04},
              'grasp': {'reached_goal': True, 'stalled': False, 'position': 0.0}}


def _ops(twin: FakeTwin) -> list:
    return [op for op, _ in twin.calls]


def _input_of(twin: FakeTwin, operation: str) -> dict:
    return next(inputs for op, inputs in twin.calls if op == operation)


# ----- 호출 순서 -----


# ----- 파지 좌표 -----

def test_grasp_comes_from_the_placed_object() -> None:
    """물체를 랜덤하게 놓고 고정 좌표를 집으면 그 자리에 아무것도 없다."""
    twin = FakeTwin()

    reset_and_pick_and_place(twin, _args())

    # 하강 지점(두 번째 move_linear)에서 **손끝**이 물체 위치에 와야 한다.
    # 명령은 panda_link8 기준이라 TCP 오프셋만큼 위에 있다 (to_arm_command).
    poses = [i['pose']['position'] for op, i in twin.calls if op == 'move_linear']
    want = SCENE_OBJECTS[0]['position']
    assert poses[1]['x'] == pytest.approx(want['x'])
    assert poses[1]['y'] == pytest.approx(want['y'])
    assert poses[1]['z'] - TCP_OFFSET_M == pytest.approx(want['z'])


def test_explicit_axis_overrides_the_placed_position() -> None:
    twin = FakeTwin()

    reset_and_pick_and_place(twin, _args(x=0.55))

    poses = [i['pose']['position'] for op, i in twin.calls if op == 'move_linear']
    assert poses[1]['x'] == 0.55
    assert poses[1]['y'] == SCENE_OBJECTS[0]['position']['y']   # 나머지 축은 그대로


def test_grasp_pose_of_does_not_mutate_the_input() -> None:
    obj = {'name': 'cube_0', 'position': {'x': 0.4, 'y': 0.0, 'z': 0.02}}

    grasp_pose_of(obj)['position']['x'] = 99.0

    assert obj['position']['x'] == 0.4


# ----- 에피소드 종료 기록 -----


# ----- 방어 -----


# ----- 조작만 한다 -----

SESSION_OPS = {'start_session', 'stop_session', 'start_episode', 'stop_episode'}


def test_no_session_or_episode_operation_is_called() -> None:
    """조작과 수집을 한 함수에 묶으면 조작만 하고 싶을 때 쓸 수가 없다."""
    twin = FakeTwin()

    assert reset_and_pick_and_place(twin, _args()) is True

    assert SESSION_OPS.isdisjoint(_ops(twin))


def test_session_state_is_not_even_read() -> None:
    """상태를 보지 않으므로 수집 계층(`rdfp`)이 떠 있지 않아도 동작한다."""
    twin = FakeTwin()
    read = twin.read

    def guarded(name: str, **kwargs):
        # 정착 대기가 `ee_pose` 를 읽는 것은 정상이다 — 막는 것은 세션 상태뿐이다.
        if name == 'session_state':
            pytest.fail('session_state 를 읽으면 안 된다')
        return read(name, **kwargs)

    twin.read = guarded

    assert reset_and_pick_and_place(twin, _args()) is True


def test_scene_is_reset_before_picking() -> None:
    """파지 좌표의 출처가 리셋 결과다 — 순서가 뒤집히면 옛 좌표를 집는다."""
    twin = FakeTwin()

    reset_and_pick_and_place(twin, _args())

    ops = _ops(twin)
    assert ops[0] == 'reset_scene'
    assert ops.index('reset_scene') < ops.index('move_linear')


def test_empty_grasp_aborts_the_flow_by_default() -> None:
    """실제 로봇에서는 빈손으로 이송을 이어가면 안 된다."""
    twin = FakeTwin(gripper_by_target=EMPTY_GRIP)

    assert reset_and_pick_and_place(twin, _args()) is False


def test_allow_empty_grasp_carries_the_flow_past_the_grasp() -> None:
    """플래그가 `pick` 까지 닿지 않으면 mock 검증이라는 목적을 못 이룬다."""
    twin = FakeTwin(gripper_by_target=EMPTY_GRIP)

    assert reset_and_pick_and_place(twin, _args(allow_empty_grasp=True)) is True


def test_empty_scene_aborts_before_moving() -> None:
    """집을 대상이 없으면 팔을 움직이지 않는다."""
    twin = FakeTwin(scene_objects=[])

    assert reset_and_pick_and_place(twin, _args()) is False

    assert _ops(twin) == ['reset_scene']
