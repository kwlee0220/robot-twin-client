"""`move_linear_verified` / `move_to_named_target_verified` / `set_gripper` 의 판정 규칙 테스트."""

from __future__ import annotations

import pytest

from robot_twin_client.client import OperationFailed
from robot_twin_client.ops import (move_linear_verified, move_to_named_target_verified,
                                   set_gripper)

from conftest import FakeTwin, POSE


def test_move_linear_verified_reports_reached_when_within_tolerance() -> None:
    twin = FakeTwin()
    check = move_linear_verified(twin, POSE)

    assert check['reached'] is True
    assert check['error_m'] == 0.0
    assert twin.calls == [('move_linear', {'pose': POSE, 'velocity_scaling': 0.2})]


def test_move_linear_verified_rejects_error_beyond_tolerance() -> None:
    # 요청보다 1 cm 못 미친 지점에서 멈춘 상황.
    twin = FakeTwin(pose_offset_m=0.01)
    check = move_linear_verified(twin, POSE, tol_m=0.005)

    assert check['reached'] is False
    assert check['error_m'] == pytest.approx(0.01)


def test_move_linear_verified_treats_missing_final_pose_as_not_reached() -> None:
    """변수가 비면 키 자체가 없다 — 도달로 낙관하면 안 된다."""
    twin = FakeTwin(pose_outputs={})
    check = move_linear_verified(twin, POSE)

    assert check['reached'] is False
    assert check['why'] == 'no final_pose in outputs'


def test_move_linear_verified_does_not_inspect_outputs_when_not_completed() -> None:
    twin = FakeTwin(pose_status='FAILED')
    check = move_linear_verified(twin, POSE)

    assert check['reached'] is False
    assert 'error_m' not in check


def test_move_to_named_target_verified_accepts_completed_closed_loop() -> None:
    twin = FakeTwin()
    check = move_to_named_target_verified(twin, 'ready')

    assert check['reached'] is True
    assert twin.calls == [('move_to_named_target',
                           {'target': 'ready', 'velocity_scaling': 0.3})]


def test_move_to_named_target_verified_passes_velocity_scaling() -> None:
    twin = FakeTwin()
    move_to_named_target_verified(twin, 'ready', velocity_scaling=0.1)

    assert twin.calls[0][1]['velocity_scaling'] == 0.1


def test_move_to_named_target_verified_rejects_open_loop() -> None:
    """JGPC 는 명령 스트리밍이라 COMPLETED 가 도달을 뜻하지 않는다 (설명서 4.3)."""
    twin = FakeTwin(closed_loop=False)
    check = move_to_named_target_verified(twin, 'ready')

    assert check['reached'] is False
    assert 'open loop' in check['why']


def test_move_to_named_target_verified_rejects_failed_status() -> None:
    """이름 오타는 400 이 아니라 계획 단계의 FAILED 로 온다."""
    twin = FakeTwin(named_status='FAILED')
    check = move_to_named_target_verified(twin, 'redy')

    assert check['reached'] is False
    assert 'why' not in check          # 상태부터 실패라 outputs 를 보지 않는다


def test_set_gripper_passes_target_and_returns_outputs() -> None:
    twin = FakeTwin(gripper={'stalled': True, 'reached_goal': False, 'position': 0.02})
    out = set_gripper(twin, 'grasp')

    assert out['stalled'] is True
    assert twin.calls == [('move_gripper_to_target', {'target': 'grasp'})]


# ----- 그리퍼 결과 규격 호환 -------------------------------------------------

class _GripperTwin:
    """`move_gripper_to_target` 의 outputs 를 그대로 돌려주는 최소 대역."""

    def __init__(self, outputs: dict) -> None:
        self._outputs = outputs

    def run(self, operation: str, inputs=None) -> dict:
        assert operation == 'move_gripper_to_target'
        return {'status': 'COMPLETED', 'outputs': self._outputs}

    def outputs(self, operation: str, inputs=None, **kwargs) -> dict:
        """실물 `RobotTwinClient.outputs` 와 같은 계약 (conftest 의 FakeTwin 과 같다)."""
        state = self.run(operation, inputs)
        if state.get('status') != 'COMPLETED':
            raise OperationFailed(operation, state)
        return state.get('outputs') or {}


def test_set_gripper_fills_reached_goal_from_at_goal() -> None:
    """트윈은 `at_goal`/`width` 를 보낸다 — 옛 이름만 읽으면 `KeyError` 로 죽는다.

    실제로 Isaac 트윈에서 `pick` 이 그렇게 죽었다. 규격이 바뀐 것이지 그리퍼가
    실패한 것이 아니라, 증상('대상이 잘못되었다')이 원인을 안 가리킨다.
    """
    twin = _GripperTwin({'target': 'open', 'goal': 'open', 'width': 0.07,
                         'stalled': False, 'at_goal': True})

    result = set_gripper(twin, 'open')

    assert result['reached_goal'] is True
    assert result['at_goal'] is True
    assert result['stalled'] is False


def test_set_gripper_keeps_a_real_reached_goal_untouched() -> None:
    """옛 규격을 보내는 트윈이면 그 값을 덮어쓰지 않는다."""
    twin = _GripperTwin({'reached_goal': False, 'stalled': True,
                         'position': 0.02, 'at_goal': True})

    assert set_gripper(twin, 'grasp')['reached_goal'] is False


def test_set_gripper_grasp_failure_is_not_reported_as_reached() -> None:
    """`at_goal` 이 서지 않으면 파지 실패다 — 물체를 놓친 것이다."""
    twin = _GripperTwin({'target': 'grasp', 'goal': 'grasp', 'width': 0.0,
                         'stalled': False, 'at_goal': False})

    assert set_gripper(twin, 'grasp')['reached_goal'] is False
