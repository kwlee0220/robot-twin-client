"""peg-in-hole MCP 도구 — **설명과 오류 문장**이 요점이다.

절차 자체는 `test_peg_in_hole.py` 가 본다. 여기서 고정하는 것은 **에이전트가 읽는 것**:
도구가 목록에 나오는가, 설명이 위험을 말하는가, 실패가 다음 행동을 알려 주는가.
오류 메시지가 에이전트의 유일한 피드백 채널이라, 코드만 던지면 같은 호출을 재시도한다.
"""

from __future__ import annotations

import pytest

mcp = pytest.importorskip('mcp')          # noqa: F841 — extra 없이 돌 때 건너뛴다

from robot_twin_client.mcp_server import (  # noqa: E402
    _EXTRA, TwinTools, _error_text)
from robot_twin_client.peg_in_hole import PegInHoleError    # noqa: E402

from conftest import FakeTwin                               # noqa: E402

PEG_TOOLS = ('peg_status', 'pick_peg', 'place_peg', 'move_peg')


def test_all_four_tools_are_exposed() -> None:
    names = {t.name for t in TwinTools(FakeTwin()).list_tools()}

    assert set(PEG_TOOLS) <= names


def test_place_peg_requires_the_gap_from_pick_peg() -> None:
    """`grasp_gap_m` 없이는 어디까지 내려야 하는지 알 수 없다 — 스키마가 강제한다."""
    schema = _EXTRA['place_peg']['schema']

    assert set(schema['required']) == {'destination', 'grasp_gap_m'}


def test_move_peg_destination_is_optional() -> None:
    """*"다른 hole 로"* 는 이름을 안 준다 — 생략하면 반대쪽으로 간다."""
    assert 'required' not in _EXTRA['move_peg']['schema']


def test_descriptions_warn_that_releasing_loses_the_object() -> None:
    """**놓으면 물체를 잃는다**는 것을 쥐게 되는 도구가 말해야 한다.

    놓아도 되는 상황과 놓으면 잃는 상황이 겉보기로 같아서, 안 적으면 에이전트가
    `move_gripper_to_target open` 을 불러 버린다.
    """
    for name in ('pick_peg', 'move_peg', 'place_peg'):
        text = _EXTRA[name]['description']
        assert '이탈' in text or '쥔 채' in text, f'{name}: 위험을 안 말한다'


def test_descriptions_carry_no_magic_numbers() -> None:
    """**수치는 코드가 갖는다.**

    44 mm(손끝) · 0.028(입구) · −54 mm(파지 간격) · 0.5 mm(여유)를 설명에 적으면
    에이전트가 그것으로 산수를 하게 되고, 틀려도 조용하다.
    """
    for name in PEG_TOOLS:
        text = _EXTRA[name]['description']
        for number in ('0.028', '44 mm', '0.044', '-54', '0.5 mm', '0.017'):
            assert number not in text, f'{name}: 설명에 수치 {number!r} 가 있다'


def test_holding_failure_says_not_to_retry_and_not_to_release() -> None:
    """쥔 채로 멈춘 것은 **다음 판단이 다르다** — 재시도도 개방도 하면 안 된다."""
    text = _error_text(PegInHoleError('밀려 올라갔다', holding=True))

    assert '쥔 채' in text
    assert '재시도' in text
    assert '사람에게' in text


def test_pre_motion_failure_says_the_robot_is_safe() -> None:
    """움직이기 전에 멈춘 것은 안전하다 — 그 사실을 알려야 불필요한 복구를 안 한다."""
    text = _error_text(PegInHoleError('목적지를 정할 수 없다'))

    assert '안전' in text
    assert '쥔 채' not in text
