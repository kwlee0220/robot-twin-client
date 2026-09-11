"""클라이언트 계층 테스트 (HTTP 는 stub 으로 대체)."""

from __future__ import annotations

from typing import Any, Optional

import pytest

from robot_twin_client.client import OperationFailed, RobotTwinClient, TwinError


class StubTwin(RobotTwinClient):
    """`_request` 만 응답표로 대체한 클라이언트."""

    def __init__(self, responses: list[tuple[int, Any]]) -> None:
        super().__init__()
        self._responses = list(responses)
        self.requests: list[tuple[str, str, Optional[dict]]] = []

    def _request(self, path: str, method: str = 'GET',
                 body: Optional[dict] = None) -> tuple[int, Any]:
        self.requests.append((path, method, body))
        return self._responses.pop(0)


def test_read_returns_the_value_when_quality_is_ok() -> None:
    twin = StubTwin([(200, {'quality': 'OK', 'value': {'position': {'panda_joint1': 0.1}}})])

    assert twin.read('joint_states')['position']['panda_joint1'] == 0.1


def test_read_refuses_a_stale_value() -> None:
    """quality 를 확인하지 않고 값을 쓰는 것을 막는다."""
    twin = StubTwin([(200, {'quality': 'STALE', 'reason': 'no message for 3.0s',
                            'value': None})])

    with pytest.raises(RuntimeError, match='STALE'):
        twin.read('joint_states')

    twin = StubTwin([(200, {'quality': 'STALE', 'value': None})])
    assert twin.read('joint_states', require_ok=False) is None


def test_run_returns_immediately_for_a_synchronous_operation() -> None:
    twin = StubTwin([(200, {'status': 'COMPLETED', 'outputs': {'reached_goal': True}})])

    result = twin.run('move_gripper_to_target', {'target': 'open'})

    assert result['status'] == 'COMPLETED'
    assert twin.requests == [('/operations/move_gripper_to_target', 'POST',
                              {'inputs': {'target': 'open'}})]


def test_run_polls_the_session_until_it_leaves_running() -> None:
    twin = StubTwin([
        (202, {'session_endpoint': '/api/v1/sessions/abc'}),
        (200, {'status': 'RUNNING', 'phase': 'EXECUTING'}),
        (200, {'status': 'COMPLETED', 'outputs': {}}),
    ])

    result = twin.run('move_to_named_target', {'target': 'ready'}, poll_sec=0.0)

    assert result['status'] == 'COMPLETED'
    assert twin.requests[-1][0] == twin.root + '/api/v1/sessions/abc'


def test_run_raises_timeout_error_instead_of_polling_forever() -> None:
    twin = StubTwin([(202, {'session_endpoint': '/api/v1/sessions/abc'})]
                    + [(200, {'status': 'RUNNING'})] * 4)

    with pytest.raises(TimeoutError, match='move_linear'):
        twin.run('move_linear', {}, poll_sec=0.0, timeout_sec=0.0)


def test_twin_error_exposes_the_error_code() -> None:
    exc = TwinError(409, {'error': {'code': 'RESOURCE_BUSY', 'message': 'arm is busy'}})

    assert exc.code == 'RESOURCE_BUSY'
    assert exc.status == 409
    assert 'RESOURCE_BUSY' in str(exc)


# ---------- 실패한 연산이 사유를 잃지 않는가 ----------
#
# `run` 은 실패해도 세션 상태를 그대로 돌려준다. 호출자들이 그것을 잊고
# `run(...)['outputs']` 로 바로 들어가는 바람에 **`KeyError: 'outputs'`** 로 터졌고,
# 트윈이 준 진짜 사유가 통째로 버려졌다 — 사용자에게는 `'outputs'` 한 단어만 남았다.
# 실제로 겪은 사유:
#
#     scene reset failed: '/set_entity_state' not ready — is Isaac running with the
#     isaacsim.ros2.sim_control extension enabled?

ISAAC_DOWN = {
    'status': 'FAILED',
    'error': {'code': 'EXECUTION_ABORTED',
              'message': "scene reset failed: '/set_entity_state' not ready — "
                         'is Isaac running with the isaacsim.ros2.sim_control '
                         'extension enabled?'},
}


class _StubTwin(RobotTwinClient):
    """`run` 만 정해진 상태로 바꾼 트윈 — HTTP 는 타지 않는다."""

    def __init__(self, state: dict) -> None:
        super().__init__()
        self._state = state
        self.calls: list = []

    def run(self, operation, inputs=None, **kwargs):      # type: ignore[override]
        self.calls.append((operation, inputs))
        return self._state


def test_outputs_returns_the_outputs_when_completed() -> None:
    twin = _StubTwin({'status': 'COMPLETED', 'outputs': {'objects': [{'name': 'block_a'}]}})

    assert twin.outputs('reset_scene', {'scene': 'one_block'}) == {
        'objects': [{'name': 'block_a'}]}
    assert twin.calls == [('reset_scene', {'scene': 'one_block'})]


def test_outputs_raises_with_the_twins_own_reason() -> None:
    """사유가 메시지에 그대로 있어야 한다 — 이것이 없으면 원인을 찾을 수 없다."""
    twin = _StubTwin(ISAAC_DOWN)

    with pytest.raises(OperationFailed) as exc:
        twin.outputs('reset_scene', {'scene': 'one_block'})

    message = str(exc.value)
    assert 'reset_scene' in message
    assert 'FAILED' in message
    assert 'EXECUTION_ABORTED' in message
    assert '/set_entity_state' in message           # 트윈이 준 사유가 살아 있다
    assert exc.value.code == 'EXECUTION_ABORTED'
    assert exc.value.state is ISAAC_DOWN            # 원본 상태도 붙여 둔다


def test_outputs_does_not_hide_a_missing_reason() -> None:
    """트윈이 error 를 안 줘도 무엇이 어떤 상태로 끝났는지는 남긴다."""
    twin = _StubTwin({'status': 'CANCELED'})

    with pytest.raises(OperationFailed) as exc:
        twin.outputs('move_linear')

    assert 'CANCELED' in str(exc.value) and 'move_linear' in str(exc.value)


def test_outputs_treats_a_missing_outputs_key_as_empty() -> None:
    """성공했는데 outputs 가 없는 연산(있다)도 KeyError 를 내면 안 된다."""
    twin = _StubTwin({'status': 'COMPLETED'})

    assert twin.outputs('stop_session') == {}
