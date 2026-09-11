"""TCP → `panda_link8` 프레임 보정.

트윈의 `move_linear` 는 planning group 의 tip link(`panda_link8`)를 움직이는데
손끝은 거기서 10.3 cm 더 뻗어 있다. 보정하지 않으면 물체 중심을 명령했을 때
그리퍼가 바닥을 파고든다.
"""

from __future__ import annotations

import math

import pytest

from conftest import SCENE_OBJECTS
from robot_twin_client.pick_n_place import (
    DOWN, TCP_OFFSET_M, _quat_mul, above_of, grasp_pose_of, rotated_about_base_z,
    to_arm_command
)

# URDF `panda_hand_joint` — link8 에서 hand 로 가는 회전. 명령을 되돌릴 때 쓴다.
_LINK8_TO_HAND = {'x': 0.0, 'y': 0.0,
                  'z': math.sin(math.radians(-45) / 2), 'w': math.cos(math.radians(-45) / 2)}


def _hand_orientation_of(command: dict) -> dict:
    """link8 명령에 URDF 변환을 적용해 실제 손 자세를 복원한다."""
    return _quat_mul(command['orientation'], _LINK8_TO_HAND)


def test_down_grasp_raises_the_command_by_the_tcp_offset():
    """물체 중심을 그대로 명령하면 손끝이 바닥 아래로 간다 — 그만큼 올려야 한다."""
    tcp = grasp_pose_of(SCENE_OBJECTS[0])
    cmd = to_arm_command(tcp)
    assert cmd['position']['z'] == pytest.approx(tcp['position']['z'] + TCP_OFFSET_M)


def test_down_grasp_keeps_x_and_y():
    tcp = grasp_pose_of(SCENE_OBJECTS[0])
    cmd = to_arm_command(tcp)
    assert cmd['position']['x'] == pytest.approx(tcp['position']['x'])
    assert cmd['position']['y'] == pytest.approx(tcp['position']['y'])


def test_the_5cm_cube_no_longer_ends_up_under_the_floor():
    """5 cm 큐브(중심 z=0.025) — 보정 전 손끝은 z=-0.078 이었다."""
    cmd = to_arm_command(grasp_pose_of(SCENE_OBJECTS[0]))
    fingertip_z = cmd['position']['z'] - TCP_OFFSET_M
    assert fingertip_z > 0.0
    assert fingertip_z == pytest.approx(0.025)


def test_yaw_correction_restores_the_intended_hand_orientation():
    """link8 에 45° 를 곱해 두어야 손가락이 의도한 방향에 선다."""
    hand = _hand_orientation_of(to_arm_command(grasp_pose_of(SCENE_OBJECTS[0])))
    for axis in 'xyzw':
        assert hand[axis] == pytest.approx(DOWN[axis], abs=1e-9)


def test_command_orientation_differs_from_the_tcp_orientation():
    """보정이 실제로 일어났는지 — 그대로 통과시키면 45° 어긋난다."""
    tcp = grasp_pose_of(SCENE_OBJECTS[0])
    cmd = to_arm_command(tcp)
    assert cmd['orientation'] != tcp['orientation']


def test_offset_follows_the_orientation_not_the_world_axis():
    """자세가 옆을 보면 오프셋도 옆으로 빠져야 한다 — z 만 빼는 구현이면 틀린다."""
    # y축 +90°: 손 z축이 월드 +x 를 향한다 → link8 은 -x 로 물러난다.
    half = math.radians(90) / 2
    sideways = {'position': {'x': 0.4, 'y': 0.0, 'z': 0.3},
                'orientation': {'x': 0.0, 'y': math.sin(half), 'z': 0.0, 'w': math.cos(half)}}
    cmd = to_arm_command(sideways)
    assert cmd['position']['x'] == pytest.approx(0.4 - TCP_OFFSET_M)
    assert cmd['position']['z'] == pytest.approx(0.3)


def test_conversion_commutes_with_the_world_frame_helpers():
    """`above_of` / `rotated_about_base_z` 뒤에 변환해도 결과가 같아야 한다.

    두 헬퍼는 월드 기준 연산이라 순서를 바꿔도 같은 pose 가 나와야 한다. 어긋나면
    파이프라인 어디서 변환하느냐에 따라 로봇이 다른 곳으로 간다.
    """
    tcp = grasp_pose_of(SCENE_OBJECTS[0])
    a = to_arm_command(rotated_about_base_z(above_of(tcp), 50.0))
    b = rotated_about_base_z(above_of(to_arm_command(tcp)), 50.0)
    for axis in 'xyz':
        assert a['position'][axis] == pytest.approx(b['position'][axis])
    for axis in 'xyzw':
        assert a['orientation'][axis] == pytest.approx(b['orientation'][axis])


# --- 이중 보정 방지 (2026-09-11) ----------------------------------------------
#
# ⚠️ **보정을 두 번 하면 10 cm 어긋난다.** 트윈이 `move_linear` 에 `frame` 입력을
# 가지면 자기가 EE 프레임(예: `grasp_center`)을 기준으로 해석한다 — 그때 클라이언트가
# `to_arm_command` 를 또 얹으면 그리퍼가 10.3 cm 아래를 겨냥한다.

def test_client_does_not_correct_when_the_twin_resolves_frames() -> None:
    """`frame` 입력이 있으면 트윈이 변환한다 — 클라이언트는 손대지 않는다."""
    mcp = pytest.importorskip('mcp')          # noqa: F841
    from robot_twin_client.mcp_server import TwinTools

    class _Twin:
        def operations(self):
            return [{'name': 'move_linear',
                     'inputs_schema': {'properties': {'pose': {}, 'frame': {}}}}]

    assert TwinTools(_Twin())._twin_resolves_frames() is True


def test_client_corrects_for_a_twin_without_frame_support() -> None:
    """옛 스택(mock)은 tip link 기준으로만 해석한다 — 여기서 보정해야 한다."""
    mcp = pytest.importorskip('mcp')          # noqa: F841
    from robot_twin_client.mcp_server import TwinTools

    class _Twin:
        def operations(self):
            return [{'name': 'move_linear', 'inputs_schema': {'properties': {'pose': {}}}}]

    assert TwinTools(_Twin())._twin_resolves_frames() is False


def test_capability_is_read_from_the_catalog_not_a_backend_table() -> None:
    """백엔드 이름으로 갈래를 두면 트윈이 늘 때 **조용히 낡는다.**

    `move_linear` 연산 자체가 없으면 보정하는 쪽(보수적)으로 떨어진다.
    """
    mcp = pytest.importorskip('mcp')          # noqa: F841
    from robot_twin_client.mcp_server import TwinTools

    class _Twin:
        def operations(self):
            return [{'name': 'move_to_joints', 'inputs_schema': {}}]

    assert TwinTools(_Twin())._twin_resolves_frames() is False
