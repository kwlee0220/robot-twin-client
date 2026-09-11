"""pick / place 시퀀스의 진행·중단 조건 테스트."""

from __future__ import annotations

import math

import pytest

from robot_twin_client.pick_n_place import (
    DOWN, above_of, center_of, grasp_pose_of, grasp_yaw_of, pick, place,
    return_home, rotated_about_base_z, yaw_of
)

from conftest import FakeTwin, POSE


GRASP = POSE
OPENED = {'reached_goal': True, 'stalled': False, 'position': 0.04}
HELD = {'reached_goal': False, 'stalled': True, 'position': 0.02}
EMPTY = {'reached_goal': True, 'stalled': False, 'position': 0.0}


def test_above_of_lifts_z_and_keeps_orientation() -> None:
    above = above_of(GRASP, clearance_m=0.1)

    assert above['position'] == {'x': 0.4, 'y': 0.0, 'z': 0.2}
    assert above['orientation'] == GRASP['orientation']
    assert GRASP['position']['z'] == 0.1      # 원본을 건드리지 않는다


def test_pick_runs_the_full_sequence_when_the_object_is_held() -> None:
    twin = FakeTwin(gripper_by_target={'open': OPENED, 'grasp': HELD})
    above = above_of(GRASP)

    assert pick(twin, above, GRASP) is True
    assert [op for op, _ in twin.calls] == [
        'move_gripper_to_target',   # open
        'move_linear',              # 접근
        'move_linear',              # 하강
        'move_gripper_to_target',   # grasp
        'move_linear',              # 상승
    ]


def test_pick_stops_before_moving_when_the_gripper_cannot_open() -> None:
    twin = FakeTwin(gripper={'reached_goal': False, 'stalled': False, 'position': 0.0})

    assert pick(twin, above_of(GRASP), GRASP) is False
    assert len(twin.calls) == 1                # 팔은 움직이지 않았다


def test_pick_fails_when_the_gripper_closes_all_the_way() -> None:
    """`grasp` 인데 reached_goal 이면 끝까지 닫혔다는 뜻 = 빈손이다."""
    twin = FakeTwin(gripper_by_target={'open': OPENED, 'grasp': EMPTY})

    assert pick(twin, above_of(GRASP), GRASP) is False
    assert [op for op, _ in twin.calls].count('move_linear') == 2   # 상승하지 않는다


def test_pick_continues_without_a_grasp_when_the_check_is_off() -> None:
    """mock 백엔드는 물리가 없어 물체를 물어도 stalled 가 서지 않는다.

    판정을 그대로 적용하면 파지에서 멈춰 상승·이송·놓기를 검증할 수 없으므로,
    `require_grasp=False` 로 뒤까지 진행시킨다.
    """
    twin = FakeTwin(gripper_by_target={'open': OPENED, 'grasp': EMPTY})

    assert pick(twin, above_of(GRASP), GRASP, require_grasp=False) is True
    assert [op for op, _ in twin.calls].count('move_linear') == 3   # 상승까지 간다


def test_pick_aborts_when_the_approach_does_not_reach() -> None:
    twin = FakeTwin(gripper_by_target={'open': OPENED}, pose_offset_m=0.05)

    assert pick(twin, above_of(GRASP), GRASP) is False
    assert [op for op, _ in twin.calls] == ['move_gripper_to_target', 'move_linear']


def test_down_orientation_is_a_unit_quaternion() -> None:
    assert sum(value * value for value in DOWN.values()) == 1.0


# ----- place -----

def test_place_runs_the_full_sequence() -> None:
    target = rotated_about_base_z(GRASP, 30.0)
    twin = FakeTwin(gripper=OPENED)

    assert place(twin, above_of(target), target) is True
    assert [op for op, _ in twin.calls] == [
        'move_linear',              # 접근
        'move_linear',              # 하강
        'move_gripper_to_target',   # 놓기
        'move_linear',              # 상승
    ]
    assert twin.calls[2][1]['target'] == 'open'


def test_place_does_not_return_home() -> None:
    """복귀는 놓기가 아니라 그 다음 일이다 — 호출자가 정한다."""
    twin = FakeTwin(gripper=OPENED)

    place(twin, above_of(GRASP), GRASP)

    assert 'move_to_named_target' not in [op for op, _ in twin.calls]


# ----- return_home -----

def test_return_home_uses_the_requested_target() -> None:
    twin = FakeTwin()

    assert return_home(twin, 'extended') is True
    assert twin.calls == [('move_to_named_target',
                           {'target': 'extended', 'velocity_scaling': 0.3})]


def test_return_home_defaults_to_ready() -> None:
    twin = FakeTwin()

    assert return_home(twin) is True
    assert twin.calls[0][1]['target'] == 'ready'


def test_return_home_reports_failure_when_it_does_not_reach() -> None:
    twin = FakeTwin(named_status='FAILED')

    assert return_home(twin) is False


def test_return_home_needs_no_frame_correction() -> None:
    """관절 공간 이동이라 to_arm_command 보정 대상이 아니다 — pose 를 보내지 않는다."""
    twin = FakeTwin()

    return_home(twin)

    assert 'pose' not in twin.calls[0][1]


def test_place_stops_when_the_gripper_does_not_open() -> None:
    """열리지 않으면 물체를 아직 쥐고 있다 — 상승하면 물체가 딸려 온다."""
    twin = FakeTwin(gripper={'reached_goal': False, 'stalled': True, 'position': 0.02})

    assert place(twin, above_of(GRASP), GRASP) is False
    assert [op for op, _ in twin.calls].count('move_linear') == 2   # 상승하지 않는다
    assert 'move_to_named_target' not in [op for op, _ in twin.calls]


def test_place_aborts_when_the_approach_does_not_reach() -> None:
    twin = FakeTwin(gripper=OPENED, pose_offset_m=0.05)

    assert place(twin, above_of(GRASP), GRASP) is False
    assert len(twin.calls) == 1                # 그리퍼도 복귀도 없다


# ----- rotated_about_base_z -----

def test_rotation_moves_the_point_along_an_arc_ccw() -> None:
    rotated = rotated_about_base_z(GRASP, 30.0)

    assert math.isclose(rotated['position']['x'], 0.4 * math.cos(math.radians(30)))
    assert math.isclose(rotated['position']['y'], 0.4 * math.sin(math.radians(30)))
    assert rotated['position']['y'] > 0        # 반시계 = +y 쪽
    assert rotated['position']['z'] == GRASP['position']['z']


def test_rotation_keeps_the_distance_from_the_base() -> None:
    """원호를 따라 도는 것이므로 베이스로부터의 반경은 그대로다."""
    def radius(pose: dict) -> float:
        p = pose['position']
        return math.hypot(p['x'], p['y'])

    assert math.isclose(radius(rotated_about_base_z(GRASP, 30.0)), radius(GRASP))


def test_rotation_keeps_the_orientation_a_unit_quaternion() -> None:
    q = rotated_about_base_z(GRASP, 30.0)['orientation']

    assert math.isclose(sum(value * value for value in q.values()), 1.0)


def test_rotating_by_zero_is_the_identity() -> None:
    assert rotated_about_base_z(GRASP, 0.0) == GRASP


def test_rotation_does_not_mutate_the_input() -> None:
    rotated_about_base_z(GRASP, 30.0)

    assert GRASP == POSE


# ---------- 물체의 두 가지 모양 ----------
#
# 같은 물체가 출처에 따라 다르게 온다 — reset_scene/begin_task 는 `position` 을 최상위에,
# scene_objects 는 `pose` 안에 둔다. `grasp_pose_of` 가 앞의 것만 받던 시절 **씬을 읽어서
# 집으려는 쪽이 `KeyError: 'position'` 으로 죽었다** (MCP 도구 설명은 두 출처를 모두
# 쓰라고 안내하므로 실제로 그렇게 불린다).

FLAT_OBJECT = {'name': 'cylinder_a', 'type': 'cylinder', 'dimensions': [0.06, 0.025],
               'position': {'x': 0.42, 'y': 0.10, 'z': 0.43}}
NESTED_OBJECT = {'type': 'cylinder', 'dimensions': [0.06, 0.025],
                 'pose': {'position': {'x': 0.42, 'y': 0.10, 'z': 0.43},
                          'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}}


def test_center_of_reads_the_flat_shape() -> None:
    """reset_scene / begin_task 의 objects 원소."""
    assert center_of(FLAT_OBJECT) == {'x': 0.42, 'y': 0.10, 'z': 0.43}


def test_center_of_reads_the_nested_shape() -> None:
    """scene_objects 의 objects[<이름>] — 이쪽이 깨져 있었다."""
    assert center_of(NESTED_OBJECT) == {'x': 0.42, 'y': 0.10, 'z': 0.43}


def test_center_of_does_not_alias_the_input() -> None:
    """돌려준 dict 를 고쳐도 원본이 바뀌면 안 된다 — 호출자가 z 를 얹어 쓴다."""
    for obj, source in ((FLAT_OBJECT, FLAT_OBJECT['position']),
                        (NESTED_OBJECT, NESTED_OBJECT['pose']['position'])):
        center = center_of(obj)
        center['z'] = 99.0
        assert source['z'] == 0.43


def test_center_of_says_what_it_wanted() -> None:
    """`'position'` 한 단어만 남으면 무엇을 잘못 넘겼는지 알 수 없다."""
    with pytest.raises(KeyError) as exc:
        center_of({'type': 'cylinder', 'dimensions': [0.06, 0.025]})

    message = str(exc.value)
    assert 'pose.position' in message and 'dimensions' in message


@pytest.mark.parametrize('obj', [FLAT_OBJECT, NESTED_OBJECT])
def test_grasp_pose_accepts_both_shapes(obj) -> None:
    pose = grasp_pose_of(obj)

    assert pose['position'] == {'x': 0.42, 'y': 0.10, 'z': 0.43}
    assert pose['orientation'] == DOWN


# ---------- 돌아간 물체를 면으로 잡기 ----------
#
# 씬 레시피가 `yaw` 를 뽑기 시작하면 손목을 고정한 채로는 **모서리를 물게 된다.**
# 2026-09-07 Isaac 실측: 면 파지 폭 0.045 / 모서리 0.063~0.071 이고, 후자는 받침이
# 좁을 때(실린더 위) 손가락 힘이 물체를 돌리는 데 쓰여 `at_goal` 이 서지 않았다.

def _q(deg):
    half = math.radians(deg) / 2.0
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(half), 'w': math.cos(half)}


def _cube(deg):
    return {'type': 'box', 'dimensions': [0.05, 0.05, 0.05],
            'pose': {'position': {'x': 0.4, 'y': 0.0, 'z': 0.425}, 'orientation': _q(deg)}}


@pytest.mark.parametrize('deg', [0.0, 12.5, 44.9, -30.0])
def test_grasp_yaw_follows_the_object_inside_one_symmetry_period(deg) -> None:
    assert math.degrees(grasp_yaw_of(_cube(deg))) == pytest.approx(deg, abs=1e-6)


@pytest.mark.parametrize('deg,want', [(45.0, -45.0), (70.0, -20.0), (89.0, -1.0),
                                      (90.0, 0.0), (135.0, -45.0)])
def test_grasp_yaw_folds_into_plus_minus_45(deg, want) -> None:
    """정육면체는 90° 마다 같은 모양이다 — 손목을 45° 넘게 돌릴 이유가 없다."""
    assert math.degrees(grasp_yaw_of(_cube(deg))) == pytest.approx(want, abs=1e-6)


def test_grasp_yaw_is_zero_for_round_objects() -> None:
    """실린더는 회전 대칭이라 손목을 돌리면 가동범위만 쓴다."""
    obj = {'type': 'cylinder', 'dimensions': [0.06, 0.025],
           'pose': {'position': {'x': 0.4, 'y': 0.0, 'z': 0.43}, 'orientation': _q(37.0)}}

    assert grasp_yaw_of(obj) == 0.0


def test_grasp_yaw_is_zero_when_the_object_has_no_orientation() -> None:
    """`reset_scene` 이 자세를 안 싣던 시절의 값도 그대로 받아야 한다."""
    assert grasp_yaw_of({'type': 'box', 'dimensions': [0.05, 0.05, 0.05],
                         'position': {'x': 0.4, 'y': 0.0, 'z': 0.425}}) == 0.0


def test_grasp_pose_carries_the_aligned_orientation() -> None:
    pose = grasp_pose_of(_cube(30.0))

    assert pose['position'] == {'x': 0.4, 'y': 0.0, 'z': 0.425}
    assert math.degrees(yaw_of(pose['orientation'])) == pytest.approx(30.0, abs=1e-6)


def test_grasp_pose_is_still_down_for_an_unrotated_object() -> None:
    assert grasp_pose_of(_cube(0.0))['orientation'] == pytest.approx(DOWN)


def test_aligned_orientation_stays_a_unit_quaternion() -> None:
    for deg in (0.0, 17.0, -44.0, 88.0):
        q = grasp_pose_of(_cube(deg))['orientation']
        assert sum(v * v for v in q.values()) == pytest.approx(1.0)
