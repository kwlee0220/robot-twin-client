"""MCP 프로토콜 왕복 — 서버가 실제로 규격대로 말하는지 확인한다.

단위 테스트는 `TwinTools` 를 직접 부르므로 **배선이 틀려도 통과한다.** 여기서는
메모리 트랜스포트로 진짜 클라이언트를 붙여, 초기화 → tools/list → tools/call 이
프로토콜 위에서 도는지 본다.
"""

from __future__ import annotations

from typing import Any

import anyio
import pytest

from conftest import SCENE_OBJECTS, FakeTwin
from mcp.client.session import ClientSession
from mcp.shared.memory import create_client_server_memory_streams
from robot_twin_client.mcp_server import TwinTools, build_server


async def _with_session(fn: Any, **twin_kwargs: Any) -> Any:
    """메모리 트랜스포트로 서버와 클라이언트를 연결하고 `fn(session)` 을 실행한다."""
    twin = FakeTwin(**twin_kwargs)
    server = build_server(TwinTools(twin))
    options = server.create_initialization_options()

    async with create_client_server_memory_streams() as ((c_read, c_write), (s_read, s_write)):
        async with anyio.create_task_group() as tg:
            tg.start_soon(lambda: server.run(s_read, s_write, options))
            async with ClientSession(c_read, c_write) as session:
                await session.initialize()
                result = await fn(session)
            tg.cancel_scope.cancel()
    return result, twin


def test_initialize_advertises_the_tools_capability():
    async def scenario(session: ClientSession):
        return (await session.list_tools()).tools

    tools, _ = anyio.run(_with_session, scenario)
    names = {t.name for t in tools}
    assert 'begin_task' in names
    assert 'move_to_pose' not in names          # 미구현은 프로토콜에도 안 나온다
    assert 'start_episode' not in names


def test_listed_schemas_survive_the_wire():
    """카탈로그의 inputs_schema 가 그대로 클라이언트에 도착해야 한다."""
    async def scenario(session: ClientSession):
        return (await session.list_tools()).tools

    tools, _ = anyio.run(_with_session, scenario)
    rotate = next(t for t in tools if t.name == 'pose_rotated_about_base_z')
    assert rotate.input_schema['required'] == ['pose', 'angle_deg']

    gripper = next(t for t in tools if t.name == 'move_gripper_to_target')
    assert gripper.input_schema['properties']['target']['type'] == 'string'


def test_call_tool_round_trip_returns_json_text():
    async def scenario(session: ClientSession):
        return await session.call_tool('grasp_pose_of_object',
                                       {'object': SCENE_OBJECTS[0]})

    result, _ = anyio.run(_with_session, scenario)
    assert result.is_error is not True
    import json
    payload = json.loads(result.content[0].text)
    assert payload['position'] == SCENE_OBJECTS[0]['position']


def test_call_tool_reports_failure_as_tool_error_not_transport_error():
    """도구 실패는 예외가 아니라 is_error 결과여야 한다 — 세션이 끊기면 안 된다."""
    async def scenario(session: ClientSession):
        first = await session.call_tool('no_such_tool', {})
        # 실패 뒤에도 세션이 살아 있어야 한다.
        second = await session.call_tool('get_resource_status', {})
        return first, second

    (failed, ok), _ = anyio.run(_with_session, scenario)
    assert failed.is_error is True
    assert 'no_such_tool' in failed.content[0].text
    assert ok.is_error is not True


def test_begin_task_over_the_protocol_opens_the_episode():
    async def scenario(session: ClientSession):
        return await session.call_tool('begin_task',
                                       {'task_label': 'demo', 'scene': 'one_cube'})

    result, twin = anyio.run(_with_session, scenario)
    assert result.is_error is not True
    assert [name for name, _ in twin.calls] == ['reset_scene', 'start_session', 'start_episode']


@pytest.mark.parametrize('tool', ['emergency_stop', 'release_emergency_stop'])
def test_safety_tools_are_reachable(tool):
    """비상 정지는 언제나 부를 수 있어야 한다."""
    async def scenario(session: ClientSession):
        return {t.name for t in (await session.list_tools()).tools}

    names, _ = anyio.run(_with_session, scenario)
    assert tool in names


# --- 프롬프트 (2026-09-11) ----------------------------------------------------
#
# **단위 테스트로는 배선이 틀려도 통과한다.** `_PROMPTS` 를 채워도 핸들러를 등록하지
# 않으면 클라이언트에는 아무것도 안 보인다 — 그 경우가 정확히 "설정했는데 안 먹는다" 다.

def test_prompts_reach_the_client_over_the_wire():
    async def scenario(session: ClientSession):
        return (await session.list_prompts()).prompts

    prompts, _ = anyio.run(_with_session, scenario)

    assert 'peg_in_hole' in {p.name for p in prompts}


def test_prompt_body_carries_the_two_rules_that_lose_the_object():
    """물체를 잃는 두 경로를 프롬프트가 막아야 한다.

    ① 앉음 확인 전에 그리퍼를 여는 것 — 물체가 씬을 이탈하고 **재시작 외에 복구가 없다**
    ② 쥔 채로 멈춘 뒤 같은 호출을 재시도하는 것 — 원인이 그대로면 결과도 같다
    """
    async def scenario(session: ClientSession):
        return await session.get_prompt('peg_in_hole')

    result, _ = anyio.run(_with_session, scenario)
    text = '\n'.join(m.content.text for m in result.messages)

    assert 'move_gripper_to_target open' in text      # 직접 열지 말라고 이름을 대야 한다
    assert '재시도' in text
    assert '사람에게 알린다' in text
    assert 'peg_status' in text                        # 목적지를 지어내지 않게


def test_prompt_carries_no_magic_numbers():
    """**수치는 코드가 갖는다.**

    프롬프트에 적으면 에이전트가 그것으로 산수를 하게 되고, 틀려도 조용하다.
    """
    async def scenario(session: ClientSession):
        return await session.get_prompt('peg_in_hole')

    result, _ = anyio.run(_with_session, scenario)
    text = '\n'.join(m.content.text for m in result.messages)

    for number in ('0.028', '0.044', '44 mm', '-54', '0.5 mm', '0.017', '0.120'):
        assert number not in text, f'프롬프트에 수치 {number!r} 가 있다'
