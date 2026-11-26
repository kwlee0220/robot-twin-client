"""클라이언트 계층 테스트 (HTTP 는 stub 으로 대체)."""

from __future__ import annotations

from typing import Any, Optional

import pytest

from robot_twin_client import client as client_module
from robot_twin_client.client import DEFAULT_BASE_URL, OperationFailed, RobotTwinClient, TwinError


class StubTwin(RobotTwinClient):
    """`_send` 만 응답표로 대체한 클라이언트.

    트윈 기준 요청은 상대 경로로, 그 밖(세션 폴링)은 URL 그대로 기록한다.
    """

    def __init__(self, responses: list[tuple[int, Any]], base_url: str = DEFAULT_BASE_URL) -> None:
        super().__init__(base_url)
        self._responses = list(responses)
        self.requests: list[tuple[str, str, Optional[dict]]] = []

    def _send(self, url: str, method: str = 'GET',
              body: Optional[dict] = None) -> tuple[int, Any]:
        path = url[len(self.base):] if url.startswith(self.base) else url
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
    assert twin.requests[-1][0] == 'http://127.0.0.1:8801/api/v1/sessions/abc'


def test_run_raises_timeout_error_instead_of_polling_forever() -> None:
    twin = StubTwin([(202, {'session_endpoint': '/api/v1/sessions/abc'})]
                    + [(200, {'status': 'RUNNING'})] * 4)

    with pytest.raises(TimeoutError, match='move_linear'):
        twin.run('move_linear', {}, poll_sec=0.0, timeout_sec=0.0)


def test_polling_timeout_names_the_endpoint_to_cancel() -> None:
    """기다리기를 그만둘 뿐 연산은 계속된다 — 멈추려면 그 연산 세션을 취소해야 한다."""
    twin = StubTwin([(202, {'session_endpoint': '/api/v1/sessions/abc'})]
                    + [(200, {'status': 'RUNNING'})] * 4)

    with pytest.raises(TimeoutError, match='/api/v1/sessions/abc'):
        twin.run('move_linear', {}, poll_sec=0.0, timeout_sec=0.0)


class _Response:
    status = 200

    def read(self) -> bytes:
        return b'{}'

    def __enter__(self) -> '_Response':
        return self

    def __exit__(self, *exc: Any) -> bool:
        return False


def _capture_timeouts(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    seen: list[float] = []

    def fake_urlopen(req: Any, timeout: float) -> _Response:
        seen.append(timeout)
        return _Response()

    monkeypatch.setattr(client_module.urllib.request, 'urlopen', fake_urlopen)
    return seen


def test_http_timeout_outlasts_the_longest_synchronous_operation(monkeypatch) -> None:
    """동기 연산은 POST 응답이 `sync_timeout_sec` 뒤에 온다 — 펑션베이 그리퍼는 12초다.

    10초였을 때는 트윈이 `COMPLETED` 든 `TIMEOUT` 이든 답하기 전에 클라이언트가 먼저 끊겼다.
    """
    seen = _capture_timeouts(monkeypatch)

    RobotTwinClient().health()

    assert seen[0] > 12.0


def _capture_headers(monkeypatch: pytest.MonkeyPatch) -> list[dict]:
    seen: list[dict] = []

    def fake_urlopen(req: Any, timeout: float) -> _Response:
        seen.append({k.lower(): v for k, v in req.header_items()})
        return _Response()

    monkeypatch.setattr(client_module.urllib.request, 'urlopen', fake_urlopen)
    return seen


def test_requests_carry_a_client_user_agent(monkeypatch) -> None:
    """트윈 감사 로그가 `User-Agent` 로 요청 주체를 가른다 — urllib 기본값이면 안 된다."""
    seen = _capture_headers(monkeypatch)

    RobotTwinClient().health()

    assert seen[0]['user-agent'].startswith('robot-twin-client/')
    assert 'content-type' not in seen[0]           # 본문 없는 GET 에는 붙이지 않는다


def test_agent_name_is_appended_and_kept_on_posts(monkeypatch) -> None:
    seen = _capture_headers(monkeypatch)

    RobotTwinClient(agent='mcp')._request('/operations/x', 'POST', {'inputs': {}})

    assert seen[0]['user-agent'].endswith(' (mcp)')
    assert seen[0]['content-type'] == 'application/json'


def test_http_timeout_can_be_raised(monkeypatch) -> None:
    seen = _capture_timeouts(monkeypatch)

    RobotTwinClient(http_timeout_sec=90.0).health()

    assert seen == [90.0]


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


# ----- 환경 확인 --------------------------------------------------------------
#
# 출하 트윈은 환경마다 id 가 다르다 (2026-10-25 — `rdfp01` · `panda01` · `mock01`). 기본
# 접속 대상은 펑션베이 트윈이고, 환경은 id 가 아니라 트윈 메타의 `backend` 로 안다.

def test_default_target_is_the_functionbay_twin() -> None:
    assert RobotTwinClient().base == 'http://127.0.0.1:8801/api/v1/robot_twins/rdfp01'


# ----- 접속 URL ---------------------------------------------------------------

@pytest.mark.parametrize('url, root', [
    ('http://127.0.0.1:8801', 'http://127.0.0.1:8801'),
    ('http://127.0.0.1:8801/', 'http://127.0.0.1:8801'),
    ('https://twin.example', 'https://twin.example'),
    ('http://[::1]:8801', 'http://[::1]:8801')
])
def test_base_url_is_the_server_root(url: str, root: str) -> None:
    twin = RobotTwinClient(url, 'panda02')

    assert twin.base_url == root
    assert twin.base == f'{root}/api/v1/robot_twins/panda02'


@pytest.mark.parametrize('url', [
    '127.0.0.1:8801',
    'localhost:8801',
    'ftp://127.0.0.1:8801',
    'http://',
    'http://127.0.0.1:abc'
])
def test_base_url_without_a_usable_scheme_and_host_is_refused(url: str) -> None:
    """스킴을 빠뜨린 주소는 생성할 때 거부한다 — 첫 호출의 `unknown url type` 보다 낫다."""
    with pytest.raises(ValueError):
        RobotTwinClient(url)


@pytest.mark.parametrize('url', [
    'https://gw.example/twin',
    'http://127.0.0.1:8801/api/v1/robot_twins/panda01',
    'http://127.0.0.1:8801/?x=1'
])
def test_base_url_with_a_path_is_refused(url: str) -> None:
    """트윈이 세션 endpoint 를 접두사 없는 절대 경로로 주므로 접두사가 붙으면 폴링이 깨진다."""
    with pytest.raises(ValueError, match='without a path'):
        RobotTwinClient(url)


def test_session_polling_and_cancel_go_to_the_server_root_over_https() -> None:
    twin = StubTwin([
        (202, {'session_endpoint': '/api/v1/sessions/abc'}),
        (200, {'status': 'COMPLETED', 'outputs': {}}),
        (200, {'status': 'CANCELED'})
    ], base_url='https://twin.example')

    twin.run('move_linear', {}, poll_sec=0.0)
    twin.cancel('/api/v1/sessions/abc')

    assert [r[:2] for r in twin.requests] == [
        ('/operations/move_linear', 'POST'),
        ('https://twin.example/api/v1/sessions/abc', 'GET'),
        ('https://twin.example/api/v1/sessions/abc', 'DELETE')
    ]


def test_meta_reads_the_twin_resource_itself() -> None:
    twin = StubTwin([(200, {'id': 'panda01', 'backend': 'panda_isaac'})])

    assert twin.meta()['backend'] == 'panda_isaac'
    assert twin.requests == [('', 'GET', None)]


def test_require_backend_returns_the_matching_name() -> None:
    twin = StubTwin([(200, {'id': 'panda01', 'backend': 'panda_isaac'})])

    assert client_module.require_backend(twin, 'panda_isaac', purpose='쌓기') == 'panda_isaac'


def test_require_backend_names_both_sides_when_the_environment_differs() -> None:
    twin = StubTwin([(200, {'id': 'panda01', 'backend': 'panda_mock'})])

    with pytest.raises(client_module.WrongBackend) as exc:
        client_module.require_backend(twin, 'panda_isaac', purpose='쌓기')
    assert exc.value.actual == 'panda_mock'
    assert 'panda_isaac' in str(exc.value) and 'panda_mock' in str(exc.value)
    assert '--backend panda_isaac' in str(exc.value)


def test_require_backend_rejects_a_twin_that_does_not_say() -> None:
    """옛 트윈은 메타에 `backend` 가 없다 — 모르는 것을 맞다고 치지 않는다."""
    twin = StubTwin([(200, {'id': 'panda01'})])

    with pytest.raises(client_module.WrongBackend, match='옛 트윈'):
        client_module.require_backend(twin, 'panda_isaac', purpose='쌓기')
