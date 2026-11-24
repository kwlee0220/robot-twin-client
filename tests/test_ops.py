"""`move_linear_verified` / `move_to_named_target_verified` / `set_gripper` 의 판정 규칙 테스트."""

from __future__ import annotations

import pytest
from copy import deepcopy
from datetime import datetime, timezone

from robot_twin_client import ops
from robot_twin_client.client import OperationFailed
from robot_twin_client.ops import (failure_why, move_linear_verified,
                                   move_to_named_target_verified, set_gripper)

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


# ----- 실패 코드 → 다음에 할 일 -----

@pytest.mark.parametrize('code,expected', [
    ('PLANNING_FAILED', '목표나 시작 자세를 바꾼다'),
    ('EXECUTION_ABORTED', 'ee_pose'),
    ('TIMEOUT', 'ee_pose'),
])
def test_failed_move_says_what_to_do_next(code, expected) -> None:
    """계획 실패와 실행 실패는 대응이 정반대다 — 코드만 주면 같은 호출을 다시 부른다."""
    for move in (lambda t: move_linear_verified(t, POSE),
                 lambda t: move_to_named_target_verified(t, 'ready')):
        twin = FakeTwin(failures={'move_linear': code, 'move_to_named_target': code})
        check = move(twin)

        assert check['reached'] is False
        assert expected in check['why']
        assert check['result']['error']['code'] == code


def test_planning_failure_says_the_robot_did_not_move() -> None:
    assert '움직이지 않았다' in failure_why({'error': {'code': 'PLANNING_FAILED'}})


def test_unknown_failure_code_has_no_hint() -> None:
    assert failure_why({'error': {'code': 'SOMETHING_NEW'}}) is None
    assert failure_why({'status': 'FAILED'}) is None
    assert failure_why(None) is None


class _SamplesTwin(FakeTwin):
    """응답 목록을 순서대로 읽고 이후에는 마지막 캐시를 반복한다."""

    def __init__(self, samples):
        super().__init__()
        self.samples = samples
        self.reads = 0

    def read_raw(self, name):
        assert name == 'ee_pose'
        sample = self.samples[min(self.reads, len(self.samples) - 1)]
        self.reads += 1
        return deepcopy(sample)


def _sample(t, *, x=0.4, quality='OK', stamp=None):
    tick = round(t * 10**9)
    pose = deepcopy(POSE)
    pose['position']['x'] = x
    return {'quality': quality, 'value': {'pose': pose},
            'received_at': datetime.fromtimestamp(t, timezone.utc).isoformat(),
            'stamp': stamp if stamp is not None else
            {'sec': tick // 10**9, 'nanosec': tick % 10**9}}


@pytest.mark.parametrize('kind', ['cached', 'stale', 'no_data', 'missing_time',
                                  'bad_time', 'nan_position', 'frozen_source'])
def test_unusable_or_repeated_samples_never_confirm_settling(kind):
    samples = [_sample(i * 0.3) for i in range(12)]
    if kind == 'cached':
        samples = [samples[0]]
    for body in samples:
        if kind == 'stale':
            body['quality'] = 'STALE'
        elif kind == 'no_data':
            body.update(quality='NO_DATA', value=None)
        elif kind == 'missing_time':
            body.pop('received_at')
        elif kind == 'bad_time':
            body['received_at'] = 'not-a-time'
        elif kind == 'nan_position':
            body['value']['pose']['position']['x'] = float('nan')
        elif kind == 'frozen_source':
            body['stamp'] = {'sec': 0, 'nanosec': 0}
    twin = _SamplesTwin(samples)
    assert ops.wait_until_still(twin, timeout_sec=2) is None


@pytest.mark.parametrize('with_stamp', [True, False])
def test_new_stationary_samples_confirm_settling(with_stamp):
    samples = [_sample(i * 0.3) for i in range(6)]
    if not with_stamp:
        for sample in samples:
            sample['stamp'] = None
    twin = _SamplesTwin(samples)
    result = ops.wait_until_still(twin, timeout_sec=2)
    assert result == samples[2]['value']
    assert twin.reads == 3


def test_duplicate_polls_do_not_confirm_settling_but_later_new_samples_can():
    twin = _SamplesTwin([_sample(0)] * 4 + [_sample(0.3), _sample(0.6)])
    assert ops.wait_until_still(twin, timeout_sec=3) is not None
    assert twin.reads == 6


def test_slow_drift_cannot_accumulate_into_a_false_stop():
    # 인접 표본 간 변화는 0.2 mm 미만이어도 기준점에서 계속 이동한다.
    twin = _SamplesTwin([_sample(i * 0.3, x=0.4 + i * 0.00015) for i in range(15)])
    assert ops.wait_until_still(twin, timeout_sec=3) is None


def test_invalid_quality_restarts_the_entire_quiet_interval():
    twin = _SamplesTwin([_sample(0), _sample(.3), _sample(.6, quality='STALE'),
                         _sample(.9), _sample(1.2), _sample(1.5)])
    assert ops.wait_until_still(twin, timeout_sec=3) is not None
    assert twin.reads == 6


@pytest.mark.parametrize('clock', ['receipt', 'source'])
def test_clock_rewind_restarts_settling(clock):
    samples = [_sample(t) for t in (10, 10.3, 0, .3, .6)]
    if clock == 'source':
        for i, sample in enumerate(samples):
            sample['received_at'] = _sample(10 + i * .4)['received_at']
    twin = _SamplesTwin(samples)
    assert ops.wait_until_still(twin, timeout_sec=3) is not None
    assert twin.reads == 5


def test_settle_timeout_does_not_fall_back_to_a_successful_final_pose():
    twin = _SamplesTwin([_sample(0)])
    check = move_linear_verified(twin, POSE, settle_sec=1)
    assert check['result']['outputs']['final_pose']['pose']['position'] == POSE['position']
    assert check['reached'] is False
    assert check['reason'] == 'SETTLE_TIMEOUT'
    assert 'why' in check and 'error_m' not in check
    assert 'settled_pose' not in check              # 정착하지 못한 값을 정착값처럼 싣지 않는다


def test_settled_observation_replaces_end_of_operation_snapshot():
    twin = _SamplesTwin([_sample(i * .3, x=.42) for i in range(6)])
    check = move_linear_verified(twin, POSE, settle_sec=2)
    assert check['reached'] is False
    assert check['error_m'] == pytest.approx(.02)


def test_settled_pose_is_the_observation_the_verdict_used():
    """판정에 쓴 정착 후 관측값을 돌려준다 — 종료 시점 스냅샷(`final_pose`)과 다르다.

    없으면 호출자가 `ee_pose` 를 다시 읽어야 하고, 그 사이 값이 바뀌면 판정과 다른 위치로
    다음 목표를 만든다.
    """
    twin = _SamplesTwin([_sample(i * .3, x=.42) for i in range(6)])
    check = move_linear_verified(twin, POSE, settle_sec=2)
    assert check['settled_pose']['pose']['position']['x'] == pytest.approx(.42)
    snapshot = check['result']['outputs']['final_pose']['pose']['position']
    assert snapshot['x'] == POSE['position']['x']


def test_no_settled_pose_without_settling():
    """`settle_sec=0` 이면 정착을 확인하지 않았으므로 그 이름의 값도 없다."""
    check = move_linear_verified(FakeTwin(), POSE)
    assert check['reached'] is True
    assert 'settled_pose' not in check


@pytest.mark.parametrize('settle_sec', [-1, float('nan'), float('inf')])
def test_invalid_settle_limit_is_rejected_before_command(settle_sec):
    twin = FakeTwin()
    with pytest.raises(ValueError, match='settle_sec'):
        move_linear_verified(twin, POSE, settle_sec=settle_sec)
    assert twin.calls == []
