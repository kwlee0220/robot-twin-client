"""수집 — 세션 경계가 실패하면 조용히 넘어가지 않는다.

세션 연산은 동기라 `run` 은 `FAILED` 를 예외 없이 돌려준다. 그대로 쓰면 에피소드가 안
열렸는데 쌓기를 돌려 **데이터 없는 회차가 성공으로** 기록된다.
"""

from __future__ import annotations

import importlib

import pytest

from conftest import SCENE_OBJECTS, FakeTwin
from robot_twin_client.client import OperationFailed

# 패키지가 같은 이름의 함수 `collect` 를 내보내 `from robot_twin_client import collect` 는
# 모듈이 아니라 그 함수를 준다.
collect = importlib.import_module('robot_twin_client.collect')


@pytest.fixture(autouse=True)
def _no_scene_or_home(monkeypatch: pytest.MonkeyPatch) -> None:
    """배치와 복귀는 이 테스트의 대상이 아니다 — 경계 처리만 본다."""
    monkeypatch.setattr(collect, 'reset_scene', lambda twin, scene, seed: SCENE_OBJECTS)
    monkeypatch.setattr(collect, 'return_home', lambda twin, home: True)


def _ops(twin: FakeTwin) -> list[str]:
    return [name for name, _ in twin.calls]


def test_episode_that_could_not_be_opened_is_not_run() -> None:
    twin = FakeTwin(failures={'start_episode': 'PRECONDITION_FAILED'})

    results = collect.collect(twin, 'cube_0', 'cube_0', times=1)

    assert results[0]['ok'] is False
    assert 'PRECONDITION_FAILED' in results[0]['error']
    assert 'move_linear' not in _ops(twin)          # 쌓기를 돌리지 않았다
    assert _ops(twin)[-1] == 'stop_session'         # 세션은 닫는다


def test_session_that_could_not_be_opened_stops_the_run() -> None:
    twin = FakeTwin(failures={'start_session': 'PRECONDITION_FAILED'})

    with pytest.raises(OperationFailed, match='PRECONDITION_FAILED'):
        collect.collect(twin, 'cube_0', 'cube_0', times=3)

    assert _ops(twin) == ['start_session']


def test_main_reports_a_refused_session_instead_of_crashing(monkeypatch, capsys) -> None:
    twin = FakeTwin(failures={'start_session': 'PRECONDITION_FAILED'})
    twin.health = lambda: {'move_group': 'READY', 'variables': {'session_state': 'OK'}}
    monkeypatch.setattr(collect, 'RobotTwinClient', lambda **kwargs: twin)

    assert collect.main(['--repeat', '1']) == 1
    assert 'PRECONDITION_FAILED' in capsys.readouterr().out
