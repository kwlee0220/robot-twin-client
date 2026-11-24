"""`robot-twin-reset` — 레시피 이름과 seed 를 트윈에 그대로 넘기고, 배치를 보인다."""

from __future__ import annotations

import pytest

from robot_twin_client import scene_reset

from conftest import SCENE_OBJECTS, FakeTwin

# 놓인 뒤의 실측 — 명령한 자리(SCENE_OBJECTS 의 cube_0, z 0.025)에서 z 만 1 mm 내려앉았다.
OBSERVED = {'cube_0': {'type': 'box', 'dimensions': [0.05, 0.05, 0.05],
                       'pose': {'position': {'x': 0.42, 'y': -0.08, 'z': 0.024}}}}


class ResetTwin(FakeTwin):
    """`scene_objects` 를 읽을 수 있고, 카탈로그가 레시피 이름(enum)을 밝히는 대역."""

    def __init__(self, *, scenes=('initial', 'two_objects'), **kwargs) -> None:
        super().__init__(**kwargs)
        self._scenes = list(scenes) if scenes is not None else None
        self.reads: list[str] = []

    def read(self, name: str, *, require_ok: bool = True):
        self.reads.append(name)
        if name == 'scene_objects':
            return {'objects': OBSERVED}
        return super().read(name, require_ok=require_ok)

    def operations(self) -> list:
        ops = super().operations()
        for op in ops:
            if op['name'] == 'reset_scene' and self._scenes is not None:
                op['inputs_schema']['properties']['scene']['enum'] = self._scenes
        return ops


@pytest.fixture
def twin(monkeypatch: pytest.MonkeyPatch) -> ResetTwin:
    fake = ResetTwin()
    monkeypatch.setattr(scene_reset, 'RobotTwinClient', lambda **kwargs: fake)
    return fake


def _resets(twin: FakeTwin) -> list:
    return [inputs for name, inputs in twin.calls if name == 'reset_scene']


def test_default_is_the_fixed_initial_layout(twin, capsys) -> None:
    """인자 없이 부르면 처음 배치(지정 위치) 레시피다."""
    assert scene_reset.main([]) == 0
    assert _resets(twin)[0]['scene'] == 'initial'
    assert "scene 'initial'" in capsys.readouterr().out


def test_a_missing_seed_is_drawn_and_shown(twin, capsys) -> None:
    """**seed 를 안 주면 새로 뽑는다** — 기본값 0 이면 무작위 레시피가 매번 같은 배치다.

    뽑은 값을 출력해야 `--seed` 로 같은 배치를 다시 만들 수 있다.
    """
    assert scene_reset.main(['--scene', 'two_objects']) == 0
    seed = _resets(twin)[0]['seed']
    assert isinstance(seed, int)
    assert f'seed {seed}, 새로 뽑음' in capsys.readouterr().out


def test_an_explicit_seed_is_passed_through(twin) -> None:
    assert scene_reset.main(['--scene', 'two_objects', '--seed', '7']) == 0
    assert _resets(twin)[0] == {'scene': 'two_objects', 'seed': 7}


def test_the_robot_does_not_move(twin) -> None:
    """씬만 바꾼다 — 팔·그리퍼 연산을 보내지 않는다."""
    scene_reset.main([])
    assert [name for name, _ in twin.calls] == ['reset_scene']


def test_it_shows_where_objects_came_to_rest(twin, capsys) -> None:
    """명령한 자리와 실제로 놓인 자리를 함께 보인다 — 파지 좌표의 근거는 놓인 자리다."""
    assert scene_reset.main([]) == 0
    out = capsys.readouterr().out
    assert '배치한 자리' in out and '놓인 자리' in out
    assert '차이   1.0 mm' in out
    assert 'scene_objects' in twin.reads


def test_no_check_skips_reading_the_scene(twin, capsys) -> None:
    assert scene_reset.main(['--no-check']) == 0
    assert 'scene_objects' not in twin.reads
    assert '놓인 자리' not in capsys.readouterr().out


def test_list_shows_the_recipes_the_twin_declares(twin, capsys) -> None:
    assert scene_reset.main(['--list']) == 0
    assert 'initial, two_objects' in capsys.readouterr().out
    assert _resets(twin) == []


def test_list_fails_when_the_twin_names_no_recipes(monkeypatch, capsys) -> None:
    fake = ResetTwin(scenes=None)
    monkeypatch.setattr(scene_reset, 'RobotTwinClient', lambda **kwargs: fake)
    assert scene_reset.main(['--list']) == 1


def test_a_refused_reset_exits_1_and_names_the_recipes(monkeypatch, capsys) -> None:
    """모르는 레시피는 트윈이 거부한다 — 고를 수 있는 이름을 함께 알려 준다."""
    fake = ResetTwin(failures={'reset_scene': 'EXECUTION_ABORTED'})
    monkeypatch.setattr(scene_reset, 'RobotTwinClient', lambda **kwargs: fake)
    assert scene_reset.main(['--scene', 'no_such_scene']) == 1
    out = capsys.readouterr().out
    assert '리셋 실패' in out
    assert 'initial, two_objects' in out


def test_applied_layout_lists_every_object(twin, capsys) -> None:
    scene_reset.main(['--no-check'])
    out = capsys.readouterr().out
    for obj in SCENE_OBJECTS:
        assert obj['name'] in out
