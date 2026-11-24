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


def test_main_refuses_a_twin_of_another_environment(monkeypatch, capsys) -> None:
    """트윈은 환경과 무관하게 같은 id · 포트라 mock 트윈에도 연결은 된다 — 메타로 거른다."""
    twin = FakeTwin(backend='panda_mock')
    twin.health = lambda: {'move_group': 'READY', 'variables': {'session_state': 'OK'}}
    monkeypatch.setattr(collect, 'RobotTwinClient', lambda **kwargs: twin)

    assert collect.main(['--repeat', '1']) == 1
    assert 'panda_mock' in capsys.readouterr().out
    assert twin.calls == []                      # 세션도 안 열었다


# ----- 기록된 에피소드 수와 종료 코드 (2026-10-23) ----------------------------
#
# 회차 수 = 기록된 에피소드 수가 아니다. 예전에는 늘 "N 건 모두 기록됐다"를 찍고 0 으로
# 끝나, 리셋 · 복귀 · 경계 요청이 실패해 에피소드가 빠진 회차가 보이지 않았다.

def _main_with(monkeypatch, results):
    twin = FakeTwin(session_state='IDLE')
    twin.health = lambda: {'move_group': 'READY', 'variables': {'session_state': 'OK'}}
    monkeypatch.setattr(collect, 'RobotTwinClient', lambda **kwargs: twin)
    monkeypatch.setattr(collect, '_close_stale_session', lambda twin: None)
    monkeypatch.setattr(collect, 'collect', lambda *a, **kw: results)
    return collect.main(['--repeat', str(len(results))])


def test_failed_task_episodes_still_exit_zero(monkeypatch, capsys) -> None:
    """작업 실패 에피소드는 정상적인 학습 데이터다."""
    results = [{'ok': True, 'seed': 1, 'episode': True},
               {'ok': False, 'seed': 2, 'episode': True, 'error': 'not stacked'}]
    assert _main_with(monkeypatch, results) == 0
    assert '에피소드 2 건 기록, 작업 성공 1 건' in capsys.readouterr().out


def test_rounds_without_an_episode_exit_one_and_are_named(monkeypatch, capsys) -> None:
    results = [{'ok': True, 'seed': 1, 'episode': True},
               {'ok': False, 'seed': 2, 'episode': False, 'error': "'ready' 복귀 실패"}]
    assert _main_with(monkeypatch, results) == 1
    out = capsys.readouterr().out
    assert '에피소드 1 건 기록' in out
    assert '에피소드를 남기지 못한 회의 seed: 2' in out


def test_home_failure_leaves_no_episode(monkeypatch) -> None:
    """복귀에 실패하면 경계를 열지 않는다 — 그 회차는 에피소드가 없다고 표시한다."""
    monkeypatch.setattr(collect, 'return_home', lambda twin, home: False)
    twin = FakeTwin()
    result = collect.collect_episode(twin, 'cube_0', 'cube_0')
    assert result['episode'] is False
    assert 'start_episode' not in _ops(twin)
