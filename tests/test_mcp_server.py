"""MCP 서버 — 도구 구성과 작업 경계 보장.

여기서 지키려는 것은 두 가지다.

1. **위험한 raw 연산이 새어 나가지 않는다** — 미구현 연산과 검증 없는 이동은
   에이전트에게 보이면 안 된다.
2. **에피소드 경계의 짝이 맞는다** — 에이전트가 잊어도 서버가 닫는다.
"""

from __future__ import annotations

from typing import Any

import pytest

from conftest import SCENE_OBJECTS, FakeTwin
from robot_twin_client.client import OperationFailed, TwinError
from robot_twin_client.mcp_server import InvalidToolInput, TwinChanged, TwinTools, _error_text
from robot_twin_client.panda_hand import TCP_OFFSET_M


def _tools(**kwargs: Any) -> tuple[TwinTools, FakeTwin]:
    twin = FakeTwin(**kwargs)
    return TwinTools(twin), twin


def _names(tools: TwinTools) -> set[str]:
    return {t.name for t in tools.list_tools()}


# ----------------------------------------------------------------------
# 도구 구성
# ----------------------------------------------------------------------

@pytest.mark.parametrize('hidden', ['move_to_pose', 'move_gripper'])
def test_unimplemented_operations_are_not_exposed(hidden):
    tools, _ = _tools()
    assert hidden not in _names(tools)


@pytest.mark.parametrize('opening', ['start_session', 'start_episode'])
def test_episode_opening_operations_are_not_exposed(opening):
    """여는 연산을 감춰야 begin_task 가 짝을 보장할 수 있다."""
    tools, _ = _tools()
    assert opening not in _names(tools)


@pytest.mark.parametrize('closing', ['stop_session', 'stop_episode'])
def test_closing_operations_stay_exposed_for_recovery(closing):
    """이전 실행이 남긴 세션을 에이전트가 스스로 정리할 수 있어야 한다."""
    tools, _ = _tools()
    assert closing in _names(tools)


def test_catalog_operations_become_tools():
    """트윈에만 있고 서버 코드에는 없는 연산도 도구가 된다."""
    tools, _ = _tools()
    assert 'move_gripper_to_target' in _names(tools)
    assert 'reset_scene' in _names(tools)


def test_extra_tools_are_present():
    tools, _ = _tools()
    names = _names(tools)
    assert {'read_state', 'read_states', 'list_state_variables', 'get_resource_status',
            'pose_above', 'pose_rotated_about_base_z', 'grasp_pose_of_object',
            'begin_task', 'end_task', 'emergency_stop',
            'release_emergency_stop'} <= names


def test_every_tool_has_a_description_and_object_schema():
    """설명이 비면 에이전트는 이름만 보고 추측한다. 스키마 최상위는 object 여야 한다."""
    tools, _ = _tools()
    for tool in tools.list_tools():
        assert tool.description, tool.name
        assert tool.input_schema['type'] == 'object', tool.name


def test_read_state_recommends_variables_that_exist():
    """설명이 권하는 변수가 없으면 에이전트는 그대로 읽고 NOT_FOUND 를 받는다.

    `gripper_last_command_result` 는 `gripper_state`(`at_goal`)로 합쳐져 트윈에서 없어졌다.
    """
    tools, _ = _tools()
    desc = next(t for t in tools.list_tools() if t.name == 'read_state').description
    assert 'gripper_last_command_result' not in desc
    assert 'gripper_state' in desc and 'at_goal' in desc


def test_resource_holding_tools_mention_busy_recovery():
    """자원 경합은 트윈 공통 규약이라 설정 설명에 없다 — 서버가 붙인다."""
    tools, _ = _tools()
    reset = next(t for t in tools.list_tools() if t.name == 'reset_scene')
    assert 'RESOURCE_BUSY' in reset.description
    assert 'scene' in reset.description and 'arm' in reset.description


def test_unknown_tool_is_rejected():
    tools, _ = _tools()
    with pytest.raises(LookupError):
        tools.call('no_such_tool', {})


def test_hidden_tool_cannot_be_called_directly():
    """목록에서 감춘 것을 이름으로 찔러도 실행되면 안 된다."""
    tools, twin = _tools()
    with pytest.raises(LookupError):
        tools.call('start_episode', {})
    assert twin.calls == []


# ----------------------------------------------------------------------
# 입력 검증 — MCP 는 스키마를 전달할 뿐 검증하지 않는다
# ----------------------------------------------------------------------

def test_out_of_range_input_is_refused_before_reaching_the_twin():
    """스키마 상한(1.0)을 넘는 값이 트윈까지 가면 안 된다 — 트윈도 jsonschema 가 있어야 막는다."""
    tools, twin = _tools()
    with pytest.raises(InvalidToolInput, match='velocity_scaling'):
        tools.call('move_to_named_target', {'target': 'ready', 'velocity_scaling': 2.0})
    assert twin.calls == []


def test_missing_required_input_names_the_field():
    """필수 인자가 빠지면 `KeyError` 가 아니라 무엇이 빠졌는지를 말한다."""
    tools, twin = _tools()
    with pytest.raises(InvalidToolInput, match='outcome'):
        tools.call('end_task', {})
    assert twin.calls == []


def test_catalog_tools_are_checked_against_the_catalog_schema():
    """자동 생성 도구도 같은 검증을 탄다 — 스키마는 `tools/list` 와 같은 출처에서 온다."""
    tools, twin = _tools()
    with pytest.raises(InvalidToolInput, match='target'):
        tools.call('move_gripper_to_target', {'target': 5})
    assert twin.calls == []


def test_valid_input_still_runs():
    tools, twin = _tools()
    tools.call('move_to_named_target', {'target': 'ready', 'velocity_scaling': 0.5})
    assert twin.calls[0][0] == 'move_to_named_target'


def test_invalid_input_error_says_nothing_ran():
    text = _error_text(InvalidToolInput("invalid input for 'end_task' at <root>: x"))
    assert '실행하지 않았다' in text and '스키마' in text


def test_end_task_describes_what_actually_happens_if_left_open():
    """닫지 않으면 다음 begin_task 가 failure 로 닫는다 — 경계가 사라지는 것이 아니다."""
    tools, _ = _tools()
    desc = next(t for t in tools.list_tools() if t.name == 'end_task').description
    assert 'failure 로 닫는다' in desc
    assert '경계를 잃' not in desc


# ----------------------------------------------------------------------
# 검증 래퍼가 raw 연산을 대체한다
# ----------------------------------------------------------------------

def test_move_linear_reports_not_reached_when_path_is_truncated():
    """트윈은 COMPLETED 로 끝내지만 도달하지 않았다 — 그대로 노출하면 안 되는 이유다."""
    tools, _ = _tools(pose_offset_m=0.05)
    result = tools.call('move_linear', {'pose': dict(POSE)})
    assert result['reached'] is False
    assert result['error_m'] == pytest.approx(0.05)


def test_move_linear_reports_reached_within_tolerance():
    tools, _ = _tools(pose_offset_m=0.001)
    assert tools.call('move_linear', {'pose': dict(POSE)})['reached'] is True


@pytest.mark.parametrize('limit', [None, 15.0])
def test_move_linear_passes_default_or_requested_settle_limit(monkeypatch, limit):
    from robot_twin_client import mcp_server
    calls = []
    def move(twin, pose, **kwargs):
        calls.append(kwargs)
        return {'reached': False, 'reason': 'SETTLE_TIMEOUT', 'why': 'not settled'}
    monkeypatch.setattr(mcp_server, 'move_linear_verified', move)
    tools, _ = _tools()
    args = {'pose': dict(POSE)}
    if limit is not None:
        args['settle_sec'] = limit
    result = tools.call('move_linear', args)
    assert calls[0]['settle_sec'] == (10.0 if limit is None else limit)
    assert result['reason'] == 'SETTLE_TIMEOUT'


def test_mcp_move_cannot_report_reached_from_a_frozen_pose_cache():
    class FrozenTwin(FakeTwin):
        cached = None
        def read_raw(self, name):
            if self.cached is None:
                self.cached = super().read_raw(name)
            return self.cached
    twin = FrozenTwin()
    result = TwinTools(twin).call('move_linear', {'pose': dict(POSE)})
    assert result['reached'] is False
    assert result['reason'] == 'SETTLE_TIMEOUT'
    assert 'error_m' not in result
    assert len(twin.calls) == 1  # 자동 재시도하지 않는다.


@pytest.mark.parametrize('limit', [0, -1])
def test_mcp_cannot_disable_settling(limit):
    tools, twin = _tools()
    with pytest.raises(InvalidToolInput, match='settle_sec'):
        tools.call('move_linear', {'pose': dict(POSE), 'settle_sec': limit})
    assert twin.calls == []


def test_mcp_move_schema_and_description_explain_settle_failure():
    tools, _ = _tools()
    tool = next(t for t in tools.list_tools() if t.name == 'move_linear')
    assert tool.input_schema['properties']['settle_sec']['default'] == 10.0
    assert 'SETTLE_TIMEOUT' in tool.description


def test_move_to_named_target_flags_open_loop():
    tools, _ = _tools(closed_loop=False)
    result = tools.call('move_to_named_target', {'target': 'ready'})
    assert result['reached'] is False
    assert 'open loop' in result['why']


# ----------------------------------------------------------------------
# 기하 도구 — 로봇을 움직이지 않는다
# ----------------------------------------------------------------------

POSE = {'position': {'x': 0.4, 'y': 0.0, 'z': 0.1},
        'orientation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}


def test_geometry_tools_do_not_touch_the_robot():
    tools, twin = _tools()
    tools.call('pose_above', {'pose': POSE})
    tools.call('pose_rotated_about_base_z', {'pose': POSE, 'angle_deg': 50})
    tools.call('grasp_pose_of_object', {'object': SCENE_OBJECTS[0]})
    assert twin.calls == []


def test_pose_above_only_raises_z():
    tools, _ = _tools()
    out = tools.call('pose_above', {'pose': POSE, 'clearance_m': 0.2})
    assert out['position']['z'] == pytest.approx(0.3)
    assert out['position']['x'] == pytest.approx(0.4)
    assert out['orientation'] == POSE['orientation']


def test_grasp_pose_uses_the_object_position():
    """랜덤 배치된 물체를 집으려면 좌표가 물체에서 와야 한다."""
    tools, _ = _tools()
    out = tools.call('grasp_pose_of_object', {'object': SCENE_OBJECTS[0]})
    assert out['position'] == SCENE_OBJECTS[0]['position']


# ----------------------------------------------------------------------
# 작업 경계
# ----------------------------------------------------------------------

def _ops(twin: FakeTwin) -> list[str]:
    return [name for name, _ in twin.calls]


def test_begin_task_resets_the_scene_before_opening_the_episode():
    """씬 리셋이 에피소드 안에 들어가면 물체 순간이동이 데이터에 섞인다."""
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo', 'scene': 'one_cube', 'seed': 7})
    ops = _ops(twin)
    assert ops == ['reset_scene', 'start_session', 'start_episode']


def test_begin_task_returns_the_actual_placement():
    tools, _ = _tools()
    out = tools.call('begin_task', {'task_label': 'demo', 'scene': 'one_cube'})
    assert out['objects'] == SCENE_OBJECTS


def test_begin_task_without_scene_leaves_the_scene_alone():
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo'})
    assert 'reset_scene' not in _ops(twin)


def test_begin_task_closes_a_session_left_open_by_a_previous_run():
    tools, twin = _tools(session_state='IN_EPISODE')
    out = tools.call('begin_task', {'task_label': 'demo'})
    assert _ops(twin)[0] == 'stop_session'
    assert any('IN_EPISODE' in w for w in out['warnings'])


def test_begin_task_closes_a_still_open_task_as_failure():
    """에이전트가 end_task 를 잊고 새 작업을 시작한 경우다."""
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'first', 'scene': 'one_cube'})
    twin.calls.clear()
    out = tools.call('begin_task', {'task_label': 'second'})

    assert _ops(twin)[:2] == ['stop_episode', 'stop_session']
    closed = next(inputs for name, inputs in twin.calls if name == 'stop_episode')
    assert closed['outcome'] == 'failure'
    assert out['warnings']


def test_begin_task_fails_when_the_session_is_refused():
    """세션 연산은 동기라 `run` 은 `FAILED` 를 예외 없이 돌려준다 — 그대로 쓰면 에피소드가
    안 열렸는데 작업이 시작된 것으로 보고된다."""
    tools, twin = _tools(failures={'start_session': 'PRECONDITION_FAILED'})

    with pytest.raises(OperationFailed, match='PRECONDITION_FAILED'):
        tools.call('begin_task', {'task_label': 'demo'})

    assert 'start_episode' not in _ops(twin)
    assert tools._task is None


def test_begin_task_closes_the_session_when_the_episode_is_refused():
    """반쯤 열린 상태가 가장 나쁘다 — 세션만 열린 채 남으면 다음 작업이 막힌다."""
    tools, twin = _tools(failures={'start_episode': 'PRECONDITION_FAILED'})

    with pytest.raises(OperationFailed):
        tools.call('begin_task', {'task_label': 'demo'})

    assert _ops(twin)[-1] == 'stop_session'
    assert tools._task is None


def test_end_task_reports_a_refused_stop_episode():
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo'})
    twin._failures['stop_episode'] = 'PRECONDITION_FAILED'

    result = tools.call('end_task', {'outcome': 'success'})

    assert result['closed'] is False
    assert any('stop_episode' in e and 'PRECONDITION_FAILED' in e for e in result['errors'])
    assert 'stop_session' in _ops(twin)


def test_end_task_closes_both_boundaries():
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo'})
    twin.calls.clear()
    result = tools.call('end_task', {'outcome': 'success'})

    assert _ops(twin) == ['stop_episode', 'stop_session']
    assert result['closed'] is True


def test_end_task_merges_the_recorded_placement_into_metadata():
    """재현의 근거는 seed 가 아니라 실제 배치다 — 에이전트가 되돌려 주기를 기대하지 않는다."""
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo', 'scene': 'one_cube', 'seed': 3})
    tools.call('end_task', {'outcome': 'success', 'metadata': {'note': 'hi'}})

    inputs = next(i for name, i in twin.calls if name == 'stop_episode')
    assert inputs['metadata']['objects'] == SCENE_OBJECTS
    assert inputs['metadata']['scene'] == 'one_cube'
    assert inputs['metadata']['seed'] == 3
    assert inputs['metadata']['note'] == 'hi'


def test_end_task_keeps_agent_metadata_over_recorded_values():
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo', 'scene': 'one_cube', 'seed': 3})
    tools.call('end_task', {'outcome': 'failure', 'metadata': {'seed': 99}})

    inputs = next(i for name, i in twin.calls if name == 'stop_episode')
    assert inputs['metadata']['seed'] == 99


def test_end_task_still_stops_the_session_when_stop_episode_fails():
    """한쪽이 실패해도 나머지를 시도한다 — 세션이 열린 채 남으면 다음 작업이 막힌다."""
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo'})

    original = twin.run

    def failing(operation, inputs=None):
        if operation == 'stop_episode':
            twin.calls.append((operation, inputs or {}))
            raise TwinError(409, {'error': {'code': 'PRECONDITION_FAILED', 'message': 'nope'}})
        return original(operation, inputs)

    twin.run = failing
    twin.calls.clear()
    result = tools.call('end_task', {'outcome': 'success'})

    assert 'stop_session' in _ops(twin)
    assert result['closed'] is False
    assert result['errors']


def test_shutdown_closes_an_open_task():
    """에이전트가 end_task 없이 연결을 끊은 경우 — 열린 에피소드를 남기지 않는다."""
    tools, twin = _tools()
    tools.call('begin_task', {'task_label': 'demo'})
    twin.calls.clear()

    tools.close_open_task()
    assert _ops(twin) == ['stop_episode', 'stop_session']


def test_shutdown_is_a_noop_without_an_open_task():
    tools, twin = _tools()
    tools.close_open_task()
    assert twin.calls == []


# ----------------------------------------------------------------------
# 오류 문장 — 에이전트의 유일한 피드백 채널
# ----------------------------------------------------------------------

@pytest.mark.parametrize('status,code,expected', [
    (409, 'RESOURCE_BUSY', 'get_resource_status'),
    (404, 'NOT_FOUND', 'list_state_variables'),
    (503, 'PRECONDITION_FAILED', '사람에게 알린다'),
])
def test_error_text_tells_the_agent_how_to_recover(status, code, expected):
    text = _error_text(TwinError(status, {'error': {'code': code, 'message': 'x'}}))
    assert expected in text


def test_backend_absence_is_not_reported_as_a_state_mismatch():
    """세션을 만들기 전의 `PRECONDITION_FAILED`(503)는 백엔드 부재다 — 상태를 맞춰도 안 풀린다.

    예전 문장은 `session_state` 를 보라고 했는데, 수집 계층이 없으면 그 변수부터 읽히지 않는다.
    """
    text = _error_text(TwinError(503, {'error': {
        'code': 'PRECONDITION_FAILED',
        'message': 'session_control_node is not available; start the collection layer'}}))

    assert 'session_state' not in text
    assert 'session_control_node' in text          # 무엇이 없는지는 트윈 메시지가 말한다


@pytest.mark.parametrize('code,expected', [
    ('PLANNING_FAILED', '목표나 시작 자세를 바꾼다'),
    ('EXECUTION_ABORTED', 'ee_pose'),
    ('PRECONDITION_FAILED', 'session_state'),     # 접수 뒤의 거절은 상태 불일치다
])
def test_failed_operation_text_tells_the_agent_how_to_recover(code, expected):
    exc = OperationFailed('move_linear', {'status': 'FAILED',
                                          'error': {'code': code, 'message': 'x'}})
    text = _error_text(exc)

    assert code in text
    assert expected in text


def test_failed_operation_text_names_the_moveit_cause():
    """`PLANNING_FAILED` 만으로는 시작 자세 문제인지 목표 문제인지 모른다 — 원인을 함께 준다."""
    exc = OperationFailed('move_linear', {'status': 'FAILED', 'error': {
        'code': 'PLANNING_FAILED', 'message': 'x',
        'details': {'moveit_error_code': -10, 'moveit_error_name': 'START_STATE_IN_COLLISION'}}})
    text = _error_text(exc)

    assert 'START_STATE_IN_COLLISION' in text
    assert '목표를 바꿔도 결과가 같다' in text


def test_raw_operation_failure_carries_a_hint():
    """카탈로그에서 자동 생성된 도구도 실패하면 다음 행동이 붙는다."""
    tools, _ = _tools(failures={'stop_session': 'PRECONDITION_FAILED'})

    result = tools.call('stop_session', {})

    assert result['status'] == 'FAILED'
    assert 'session_state' in result['why']


def test_raw_operation_success_is_returned_untouched():
    tools, _ = _tools()
    assert 'why' not in tools.call('stop_session', {})


def test_timeout_error_text_points_at_the_robot_state():
    assert 'ee_pose' in _error_text(TimeoutError('too slow'))


# ----------------------------------------------------------------------
# 프레임 보정 — 기하 도구는 손끝 기준, 이동 도구가 link8 로 바꾼다
# ----------------------------------------------------------------------

def test_move_linear_applies_the_tcp_correction():
    """보정하지 않으면 그리퍼가 바닥을 파고든다 (손끝이 link8 보다 10.3 cm 앞이다)."""
    tools, twin = _tools()
    tcp = tools.call('grasp_pose_of_object', {'object': SCENE_OBJECTS[0]})

    tools.call('move_linear', {'pose': tcp})

    sent = next(i['pose']['position'] for op, i in twin.calls if op == 'move_linear')
    assert sent['z'] == pytest.approx(tcp['position']['z'] + TCP_OFFSET_M)
    assert sent['z'] - TCP_OFFSET_M == pytest.approx(SCENE_OBJECTS[0]['position']['z'])


def test_move_linear_does_not_shift_x_and_y_for_a_top_down_grasp():
    tools, twin = _tools()
    tcp = tools.call('grasp_pose_of_object', {'object': SCENE_OBJECTS[0]})

    tools.call('move_linear', {'pose': tcp})

    sent = next(i['pose']['position'] for op, i in twin.calls if op == 'move_linear')
    assert sent['x'] == pytest.approx(tcp['position']['x'])
    assert sent['y'] == pytest.approx(tcp['position']['y'])


def test_geometry_tools_still_return_tcp_frame_poses():
    """에이전트가 다루는 좌표는 '손끝이 어디 가야 하는가' 여야 한다."""
    tools, _ = _tools()
    grasp = tools.call('grasp_pose_of_object', {'object': SCENE_OBJECTS[0]})

    assert grasp['position'] == SCENE_OBJECTS[0]['position']   # 보정 전 그대로


def test_move_to_named_target_is_not_corrected():
    """관절 공간 이동이라 기준 링크가 무관하다 — pose 를 보내지 않는다."""
    tools, twin = _tools()

    tools.call('move_to_named_target', {'target': 'ready'})

    sent = next(i for op, i in twin.calls if op == 'move_to_named_target')
    assert 'pose' not in sent


def test_move_linear_returns_the_settled_observation():
    """정착 후 위치를 `ee_pose` 로 다시 읽지 않아도 되게 판정에 쓴 관측값을 함께 준다."""
    tools, _ = _tools(pose_offset_m=0.001)
    result = tools.call('move_linear', {'pose': dict(POSE)})
    assert result['reached'] is True
    assert 'position' in result['settled_pose']['pose']


# ----------------------------------------------------------------------
# 트윈 교체 감지 (2026-10-23)
# ----------------------------------------------------------------------
#
# 같은 id 의 트윈을 다른 설정으로 다시 띄우면 이 서버는 연결된 채 옛 카탈로그를 쓴다. 그 카탈로그로 `move_linear` 의 프레임 보정 여부를 정하므로, 감지하지
# 않으면 mock → 펑션베이에서 보정이 덧씌워져 그리퍼가 10.3 cm 아래를 겨냥한다.

class _SwitchingTwin(FakeTwin):
    """`switch()` 뒤에는 펑션베이 트윈처럼 보인다 — backend · `frame` 입력 · `fixtures` 가 바뀐다."""

    def __init__(self) -> None:
        super().__init__(backend='panda_mock', has_fixtures=False)
        self._frame = False

    def switch(self) -> None:
        self._backend = 'panda_functionbay'
        self._has_fixtures = True
        self._frame = True

    def operations(self) -> list:
        ops = super().operations()
        if not self._frame:
            return ops
        out = []
        for op in ops:
            if op['name'] == 'move_linear':
                schema = dict(op['inputs_schema'])
                schema['properties'] = {**schema.get('properties', {}), 'frame': {'type': 'string'}}
                op = {**op, 'inputs_schema': schema}
            out.append(op)
        return out


def _switched() -> tuple[TwinTools, _SwitchingTwin]:
    twin = _SwitchingTwin()
    tools = TwinTools(twin)
    tools.list_tools()                       # 옛 트윈의 카탈로그를 잡아 둔다
    assert tools._twin_resolves_frames() is False
    twin.switch()
    return tools, twin


def test_first_move_after_a_twin_switch_is_refused_and_the_catalog_reloaded(monkeypatch):
    from robot_twin_client import mcp_server
    poses = []
    monkeypatch.setattr(mcp_server, 'move_linear_verified',
                        lambda twin, pose, **kw: poses.append(pose) or {'reached': True})
    tools, _ = _switched()

    with pytest.raises(TwinChanged) as exc:
        tools.call('move_linear', {'pose': dict(POSE)})
    assert poses == []                                    # 아무것도 실행하지 않았다
    assert (exc.value.before[0], exc.value.after[0]) == ('panda_mock', 'panda_functionbay')
    assert tools._twin_resolves_frames() is True          # 새 카탈로그로 판별한다

    tools.call('move_linear', {'pose': dict(POSE)})       # 다음 호출은 실행된다
    assert poses == [POSE]                                # 트윈이 변환하므로 보정하지 않는다


def test_reads_after_a_twin_switch_are_not_refused():
    """바뀐 뒤에 할 일이 상태를 다시 읽는 것이다 — 그것까지 막으면 복구할 길이 없다."""
    tools, _ = _switched()
    assert tools.call('list_state_variables', {}) is not None


def test_tool_list_follows_the_new_twin():
    tools, _ = _switched()
    assert {'peg_status', 'move_peg'} <= _names(tools)    # 펑션베이에는 고정물이 있다


def test_geometry_and_estop_do_not_query_the_twin():
    """기하 도구는 트윈이 꺼져 있어도 돌아야 하고, 비상정지 앞에는 조회를 끼우지 않는다."""
    class _NoMeta(FakeTwin):
        stopped = False

        def meta(self):
            raise AssertionError('meta() must not be called')

        def estop(self):
            self.stopped = True
            return {'estop': 'ENGAGED'}

    twin = _NoMeta()
    tools = TwinTools(twin)
    tools.call('pose_above', {'pose': dict(POSE)})
    tools.call('emergency_stop', {})
    assert twin.stopped


def test_twin_change_message_says_nothing_ran():
    text = _error_text(TwinChanged(('panda_mock',), ('panda_functionbay',)))
    assert '아무것도 실행하지 않았다' in text and 'read_state' in text
