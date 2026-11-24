"""모듈 층 — 기하 헬퍼는 기반 층에 있고, 기반 층은 작업 모듈을 모른다.

기하 헬퍼가 첫 작업 모듈(`pick_n_place`) 안에 있던 시절에는 수학만 쓰려는 모듈이 작업 모듈에
의존했고, 그 의존을 피하려던 `peg_in_hole` 이 `down_with_yaw` 를 복사해 두 벌이 됐다. 여기서는
그 구조가 되돌아오지 않는지 import 를 AST 로 읽어 확인한다 — 실행하지 않으므로 `mcp` extra 가
없어도 돈다.
"""

from __future__ import annotations

import ast
import pathlib

import pytest

from robot_twin_client import geometry, panda_hand, peg_in_hole, pick_n_place

_PKG = pathlib.Path(geometry.__file__).parent

# 기반 층이 import 하면 안 되는 모듈 — 작업 모듈과 그 위.
_UPPER = {'pick_n_place', 'stack', 'peg_in_hole', 'scene_reset', 'collect', 'mcp_server'}


def _package_imports(name: str) -> set:
    """`name.py` 가 import 하는 패키지 안 모듈 이름들."""
    tree = ast.parse((_PKG / f'{name}.py').read_text(encoding='utf-8'))
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            module = node.module or ''
            if node.level >= 1:
                found.add(module.split('.')[0])
            elif module.startswith('robot_twin_client.'):
                found.add(module.split('.')[1])
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith('robot_twin_client.'):
                    found.add(alias.name.split('.')[1])
    return found


def test_geometry_imports_nothing_from_the_package() -> None:
    assert _package_imports('geometry') == set()


def test_panda_hand_depends_only_on_geometry() -> None:
    assert _package_imports('panda_hand') == {'geometry'}


@pytest.mark.parametrize('name', ['client', 'ops', 'geometry', 'panda_hand'])
def test_foundation_does_not_import_task_modules(name: str) -> None:
    assert not (_package_imports(name) & _UPPER), name


def test_there_is_one_down_with_yaw() -> None:
    """`peg_in_hole` 에 있던 사본이 다시 생기면 안 된다 — 같은 객체여야 한다."""
    assert peg_in_hole.down_with_yaw is geometry.down_with_yaw


@pytest.mark.parametrize('name', ['DOWN', 'above_of', 'center_of', 'grasp_pose_of', 'grasp_yaw_of',
                                  'orientation_of', 'rotated_about_base_z', 'yaw_of',
                                  'down_with_yaw'])
def test_pick_n_place_still_exports_geometry_names(name: str) -> None:
    """옛 import 경로(`from robot_twin_client.pick_n_place import grasp_pose_of`)가 계속 된다."""
    assert getattr(pick_n_place, name) is getattr(geometry, name)


@pytest.mark.parametrize('name', ['TCP_OFFSET_M', 'to_arm_command', 'tcp_from_arm_pose'])
def test_pick_n_place_still_exports_panda_hand_names(name: str) -> None:
    assert getattr(pick_n_place, name) is getattr(panda_hand, name)
