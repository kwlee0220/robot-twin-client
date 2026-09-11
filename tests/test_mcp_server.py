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
from robot_twin_client.client import TwinError
from robot_twin_client.mcp_server import TwinTools, _error_text
from robot_twin_client.pick_n_place import TCP_OFFSET_M


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

@pytest.mark.parametrize('code,expected', [
    ('RESOURCE_BUSY', 'get_resource_status'),
    ('PRECONDITION_FAILED', 'session_state'),
    ('NOT_FOUND', 'list_state_variables'),
])
def test_error_text_tells_the_agent_how_to_recover(code, expected):
    text = _error_text(TwinError(409, {'error': {'code': code, 'message': 'x'}}))
    assert expected in text


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
