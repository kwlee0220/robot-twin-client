"""쌓기 시퀀스 — 치수 해석 · 물린 위치 보정 · 진행/중단 조건.

**여기서 지키는 성질은 두 가지다.**

1. `dimensions` 순서가 타입마다 다르다 (`SolidPrimitive` 규약). 뒤집어도 값이 나오고
   크래시하지 않으므로 테스트가 아니면 드러나지 않는다.
2. 놓을 높이가 **상수가 아니라 실측한 물린 위치**에서 나온다. 미끄러진 만큼 보정하지
   않으면 띄워 떨어뜨린다.
"""

from __future__ import annotations

from typing import Optional

import math

import pytest

from robot_twin_client.pick_n_place import TCP_OFFSET_M, tcp_from_arm_pose
from robot_twin_client.stack import (
    await_fresh_scene, center_of, half_height_of, held_offset, iteration_seeds,
    repeat_stack, reset_and_stack,
    reset_scene, stack, stack_pose_of, verify_stacked
)

from conftest import FakeTwin


BLOCK = {'type': 'box', 'dimensions': [0.05, 0.05, 0.05],
         'pose': {'position': {'x': 0.42, 'y': 0.22, 'z': 0.425}}}
CYLINDER = {'type': 'cylinder', 'dimensions': [0.06, 0.025],
            'pose': {'position': {'x': 0.43, 'y': 0.03, 'z': 0.43}}}

HELD = {'reached_goal': False, 'stalled': True, 'position': 0.02}
OPENED = {'reached_goal': True, 'stalled': False, 'position': 0.04}

# 손이 아래를 볼 때(x축 180°) 손끝은 손 원점보다 TCP_OFFSET_M 만큼 **아래**다.
DOWN_HAND = {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}


class SceneTwin(FakeTwin):
    """`scene_objects` / `ee_pose` 를 읽을 수 있는 대역.

    `FakeTwin.read` 는 `session_state` 만 안다 — 공용 conftest 를 건드리지 않고
    여기서만 넓힌다.
    """

    def __init__(self, *, objects: Optional[dict] = None,
                 hand_z: float = 0.60, **kwargs) -> None:
        super().__init__(**kwargs)
        self._objects = objects if objects is not None else {
            'block_a': BLOCK, 'cylinder_a': CYLINDER}
        self._hand_z = hand_z

    def read(self, name: str, *, require_ok: bool = True):
        if name == 'scene_objects':
            return {'objects': self._objects}
        if name == 'ee_pose':
            return {'pose': {'position': {'x': 0.43, 'y': 0.03, 'z': self._hand_z},
                             'orientation': dict(DOWN_HAND)}}
        return super().read(name, require_ok=require_ok)


# ----- 치수 해석 -------------------------------------------------------------

def test_box_half_height_uses_the_z_dimension() -> None:
    assert half_height_of(BLOCK) == pytest.approx(0.025)


def test_cylinder_half_height_uses_the_height_not_the_radius() -> None:
    """`SolidPrimitive` 의 cylinder 는 [높이, 반지름] 이다 — 직관과 반대다.

    뒤집어 읽으면 0.0125 가 나와 물체가 17.5 mm 낮게 놓인다. 크래시가 아니라
    '그럴듯하게 틀린 높이'라 실행해 봐서는 안 갈린다.
    """
    assert half_height_of(CYLINDER) == pytest.approx(0.03)


def test_unknown_type_is_rejected_rather_than_guessed() -> None:
    with pytest.raises(ValueError, match='mesh'):
        half_height_of({'type': 'mesh', 'dimensions': []})


# ----- 물린 위치 -------------------------------------------------------------

def test_held_offset_measures_the_slip_between_tcp_and_object() -> None:
    """손끝과 물체 중심의 실제 차이를 잰다.

    손이 z=0.60 에서 아래를 보므로 손끝은 0.60 - 0.1034 = 0.4966 이다. 물체 중심이
    0.43 이면 12 mm 넘게 미끄러져 물린 것이다 — Isaac 에서 실제로 나온 상황이다.
    """
    twin = SceneTwin(hand_z=0.60)

    offset = held_offset(twin, 'cylinder_a')

    assert offset['z'] == pytest.approx(0.60 - TCP_OFFSET_M - 0.43)
    assert offset['x'] == pytest.approx(0.0)
    assert offset['y'] == pytest.approx(0.0)


def test_stack_pose_puts_the_object_on_the_base_surface() -> None:
    """오프셋이 0 이면 손끝이 곧 물체 중심이다 — 상면 + 반높이 + 틈."""
    pose = stack_pose_of(BLOCK, half_height_of(CYLINDER),
                         {'x': 0.0, 'y': 0.0, 'z': 0.0}, gap_m=0.0)

    assert pose['position']['z'] == pytest.approx(0.425 + 0.025 + 0.03)
    assert pose['position']['x'] == pytest.approx(0.42)


def test_stack_pose_adds_the_held_offset() -> None:
    """미끄러진 만큼 손끝을 더 내려야 물체가 상면에 닿는다.

    보정하지 않으면 물체가 offset 만큼 뜬 채 놓여 원통은 굴러떨어진다.
    """
    offset = {'x': 0.001, 'y': -0.002, 'z': 0.012}

    pose = stack_pose_of(BLOCK, 0.03, offset, gap_m=0.004)

    assert pose['position']['z'] == pytest.approx(0.425 + 0.025 + 0.03 + 0.004 + 0.012)
    assert pose['position']['x'] == pytest.approx(0.42 + 0.001)
    assert pose['position']['y'] == pytest.approx(0.22 - 0.002)


# ----- 시퀀스 ---------------------------------------------------------------

def test_stack_runs_pick_then_place_then_home() -> None:
    twin = SceneTwin(gripper_by_target={'open': OPENED, 'grasp': HELD})

    assert stack(twin, 'cylinder_a', 'block_a') is True
    assert [op for op, _ in twin.calls] == [
        'move_gripper_to_target',   # open
        'move_linear',              # 접근
        'move_linear',              # 하강
        'move_gripper_to_target',   # grasp
        'move_linear',              # 상승 (여기서 물린 위치를 잰다)
        'move_linear',              # 놓을 자리 위
        'move_linear',              # 하강
        'move_gripper_to_target',   # open
        'move_linear',              # 상승
        'move_to_named_target',     # 복귀
    ]


def test_stack_stops_when_nothing_was_grasped() -> None:
    """빈손으로 이어가면 '놓았다'는 거짓 성공이 남는다."""
    empty = {'reached_goal': True, 'stalled': False, 'position': 0.0}
    twin = SceneTwin(gripper_by_target={'open': OPENED, 'grasp': empty})

    assert stack(twin, 'cylinder_a', 'block_a') is False
    assert 'move_to_named_target' not in [op for op, _ in twin.calls]


def test_unknown_object_fails_before_the_arm_moves() -> None:
    """이름 오타가 팔이 움직인 뒤에 터지면 안 된다."""
    twin = SceneTwin()

    with pytest.raises(KeyError, match='cube_9'):
        stack(twin, 'cube_9', 'block_a')
    assert twin.calls == []


def test_stacking_an_object_on_itself_is_rejected() -> None:
    twin = SceneTwin()

    with pytest.raises(ValueError, match='itself'):
        stack(twin, 'block_a', 'block_a')
    assert twin.calls == []


def test_transit_height_is_shared_and_clears_the_finished_stack() -> None:
    """이송 높이는 두 지점에 **공통**이고, **쌓은 뒤의 높이** 기준으로 잡아야 한다.

    집는 지점 기준으로만 잡으면 받침이 높을 때 이송 경로가 그 위를 스친다. 여기서는
    실린더(중심 0.43)를 블록(상면 0.45) 위에 얹으므로 **놓는 쪽이 더 높다** — 집는
    지점 기준으로 잡은 높이는 이 조건을 못 넘는다.
    """
    clearance, gap = 0.15, 0.004
    twin = SceneTwin(gripper_by_target={'open': OPENED, 'grasp': HELD})
    stack(twin, 'cylinder_a', 'block_a', clearance_m=clearance, gap_m=gap)

    # **기록된 pose 는 팔(`panda_link8`) 기준이다** — `to_arm_command` 를 거쳤기
    # 때문이다. 손끝 기준 값과 비교하려면 되돌려야 한다. 안 되돌리면 모든 높이가
    # 10.3 cm 씩 부풀어 임계값을 그냥 넘어가고, 검사가 아무것도 안 잡는다.
    zs = [tcp_from_arm_pose(i['pose'])['position']['z']
          for op, i in twin.calls if op == 'move_linear']
    # 접근·상승·놓을자리위·상승 — 네 번의 이송 높이가 모두 같아야 한다.
    assert zs[0] == pytest.approx(zs[2]) == pytest.approx(zs[3]) == pytest.approx(zs[5])
    # 얹힌 뒤 실린더 중심 = 받침 상면 + 실린더 반높이 + 틈
    stacked_center = 0.425 + 0.025 + 0.03 + gap
    assert zs[0] >= stacked_center + clearance - 1e-9


# ----- 결과 확인 -------------------------------------------------------------

def test_verify_stacked_accepts_a_real_stack() -> None:
    stacked = {'block_a': BLOCK,
               'cylinder_a': {**CYLINDER,
                              'pose': {'position': {'x': 0.422, 'y': 0.219, 'z': 0.480}}}}

    check = verify_stacked(SceneTwin(objects=stacked), 'cylinder_a', 'block_a')

    assert check['stacked'] is True
    assert check['expected_gap_m'] == pytest.approx(0.055)


def test_verify_stacked_rejects_an_object_that_rolled_off() -> None:
    """놓기가 성공해도 얹혔다는 뜻은 아니다 — 굴러떨어지면 높이차가 사라진다."""
    rolled = {'block_a': BLOCK,
              'cylinder_a': {**CYLINDER,
                             'pose': {'position': {'x': 0.49, 'y': 0.25, 'z': 0.43}}}}

    assert verify_stacked(SceneTwin(objects=rolled), 'cylinder_a', 'block_a')['stacked'] is False


def test_center_of_reads_the_nested_pose() -> None:
    """`stack` 은 scene_objects 를 읽으므로 중첩 모양이다 (구현은 pick_n_place 에 있다)."""
    """`scene_objects` 는 `pose.position` 이고 `reset_scene` 은 평평한 `position` 이다."""
    assert center_of(BLOCK) == {'x': 0.42, 'y': 0.22, 'z': 0.425}


# ----- 씬 리셋과의 조합 ------------------------------------------------------

class ResetTwin(SceneTwin):
    """`reset_scene` 뒤 `scene_objects` 가 **한 박자 늦게** 갱신되는 트윈.

    `/scene/reset` 이 끝나도 씬 변수는 주기 발행이라 직전 배치를 한동안 더 말한다.
    그 지연을 재현해 `await_fresh_scene` 이 실제로 기다리는지 본다.
    """

    def __init__(self, *, stale_reads: int = 2, **kwargs) -> None:
        super().__init__(**kwargs)
        self._ticks = 0
        self._stale_reads = stale_reads

    def read_raw(self, name: str) -> dict:
        assert name == 'scene_objects'
        self._ticks += 1
        # 발행이 두 번 지나간 뒤부터 새 표본이 나온다.
        return {'name': name, 'quality': 'OK',
                'received_at': f'sample-{self._ticks // 2}',
                'value': self.read(name)}


def test_reset_and_stack_places_objects_before_picking() -> None:
    """리셋이 **먼저**여야 한다 — 뒤에 오면 방금 쌓은 것을 흩어 버린다."""
    twin = ResetTwin(gripper_by_target={'open': OPENED, 'grasp': HELD},
                     scene_objects=[{'name': 'block_a'}, {'name': 'cylinder_a'}])

    assert reset_and_stack(twin, 'cylinder_a', 'block_a') is True
    ops = [op for op, _ in twin.calls]
    assert ops[0] == 'reset_scene'
    assert ops[1] == 'move_gripper_to_target'


def test_reset_and_stack_passes_scene_and_seed() -> None:
    twin = ResetTwin(gripper_by_target={'open': OPENED, 'grasp': HELD},
                     scene_objects=[{'name': 'block_a'}, {'name': 'cylinder_a'}])

    reset_and_stack(twin, 'cylinder_a', 'block_a', scene='two_objects', seed=7)

    assert twin.calls[0] == ('reset_scene', {'scene': 'two_objects', 'seed': 7})


def test_reset_and_stack_rejects_names_the_recipe_does_not_place() -> None:
    """레시피가 안 놓는 이름이면 팔이 움직이기 전에 멈춰야 한다.

    씬 변수에는 남아 있을 수 있으므로 `scene_object` 조회만으로는 안 걸린다 —
    **리셋 결과**와 대조해야 걸린다.
    """
    twin = ResetTwin(scene_objects=[{'name': 'block_a'}])

    with pytest.raises(KeyError, match='cylinder_a'):
        reset_and_stack(twin, 'cylinder_a', 'block_a')
    assert [op for op, _ in twin.calls] == ['reset_scene']


class _FreshEveryRead(SceneTwin):
    """읽을 때마다 새 표본을 주는 트윈. 조회 횟수를 센다."""

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.reads = 0

    def read_raw(self, name: str) -> dict:
        self.reads += 1
        return {'name': name, 'quality': 'OK', 'received_at': f'sample-{self.reads}',
                'value': self.read(name)}


def test_reset_scene_waits_for_the_scene_variable_to_catch_up() -> None:
    """리셋 뒤 바로 좌표를 읽으면 **직전 배치**를 집으러 간다 — 거기엔 아무것도 없다.

    `/scene/reset` 이 끝나도 `scene_objects` 는 주기 발행이라 한 박자 늦는다. 기다림을
    빼면 시퀀스는 끝까지 정상으로 보이고 파지만 조용히 빗나간다.
    """
    twin = _FreshEveryRead(scene_objects=[{'name': 'block_a'}])

    reset_scene(twin, 'two_objects', 3)

    assert twin.calls[0] == ('reset_scene', {'scene': 'two_objects', 'seed': 3})
    assert twin.reads > 0, 'scene_objects 를 한 번도 확인하지 않았다'


def test_await_fresh_scene_waits_for_one_more_sample_than_asked_minus_one() -> None:
    """표본 하나로는 부족하다 — 리셋 직전에 떠난 발행이 바로 도착할 수 있다.

    매 조회가 새 표본인 트윈에서 `samples=2` 는 `samples=1` 보다 **정확히 한 번 더**
    읽어야 한다. 첫 변화에서 돌아오면 두 값이 같아진다.
    """
    one, two = _FreshEveryRead(), _FreshEveryRead()

    assert await_fresh_scene(one, samples=1, timeout_sec=2.0) is True
    assert await_fresh_scene(two, samples=2, timeout_sec=2.0) is True
    assert two.reads == one.reads + 1


def test_await_fresh_scene_returns_immediately_without_timestamps() -> None:
    """타임스탬프를 안 주는 트윈에서는 기다릴 근거가 없다."""
    assert await_fresh_scene(SceneTwin(), timeout_sec=0.5) is True


# ---------- 돌아간 물체 ----------
#
# 씬 레시피가 `yaw` 를 뽑기 시작하면서 필요해졌다. 손목을 물체에 맞춰 **면으로** 잡고,
# 이송 중에는 그 자세를 유지해야 한다 — 도중에 손목이 0 으로 돌아가면 쥔 물체가 함께
# 돌아 물림이 흔들린다.

ROTATED_BLOCK = {'type': 'box', 'dimensions': [0.05, 0.05, 0.05],
                 'pose': {'position': {'x': 0.42, 'y': 0.22, 'z': 0.425},
                          'orientation': {'x': 0.0, 'y': 0.0,
                                          'z': 0.25881904510252074,     # 30°
                                          'w': 0.9659258262890683}}}


def _yaw_deg(quat) -> float:
    return math.degrees(math.atan2(2.0 * (quat['w'] * quat['z'] + quat['x'] * quat['y']),
                                   1.0 - 2.0 * (quat['y'] ** 2 + quat['z'] ** 2)))


def test_stack_grasps_a_rotated_block_on_its_face() -> None:
    twin = SceneTwin(objects={'block_a': ROTATED_BLOCK, 'cylinder_a': CYLINDER},
                     gripper_by_target={'open': OPENED, 'grasp': HELD})

    assert stack(twin, 'block_a', 'cylinder_a', home=None) is True
    poses = [inputs['pose'] for name, inputs in twin.calls if name == 'move_linear']
    # 명령은 link8 기준이라 손 보정(+45°)이 얹혀 있다 — 상대 비교로 본다.
    yaws = [_yaw_deg(p['orientation']) for p in poses]
    assert max(yaws) - min(yaws) < 1e-6, f'이송 중 손목이 돌았다: {yaws}'
    assert abs(yaws[0] - (30.0 - 45.0)) < 1e-6, f'물체 각도에 안 맞췄다: {yaws[0]}'


def test_stack_keeps_the_grasp_orientation_through_the_place() -> None:
    """놓는 자세가 `DOWN` 으로 돌아가면 쥔 채 손목이 돌아간다."""
    twin = SceneTwin(objects={'block_a': ROTATED_BLOCK, 'cylinder_a': CYLINDER},
                     gripper_by_target={'open': OPENED, 'grasp': HELD})
    stack(twin, 'block_a', 'cylinder_a', home=None)

    poses = [inputs['pose'] for name, inputs in twin.calls if name == 'move_linear']
    assert _yaw_deg(poses[-1]['orientation']) == pytest.approx(_yaw_deg(poses[0]['orientation']))


# ---------- 반복 실행 ----------
#
# 반복의 요점은 **매 회 배치가 달라지는 것**이다. 트윈은 씨앗을 생략하면 0 으로 고정하므로
# 같은 씨앗을 되쓰면 열 번을 돌려도 같은 자리에서 같은 동작을 반복하게 된다.

SCENE_NAMES = [{'name': 'block_a'}, {'name': 'cylinder_a'}]


def _repeat_twin(grasp=None):
    return ResetTwin(gripper_by_target={'open': OPENED, 'grasp': grasp or HELD},
                     scene_objects=SCENE_NAMES)


def test_iteration_seeds_are_all_different_when_random() -> None:
    seeds = iteration_seeds(None, 20)

    assert len(set(seeds)) == 20


def test_iteration_seeds_are_reproducible_when_given() -> None:
    """씨앗을 주면 실행 전체가 재현돼야 한다 — 실패한 회를 다시 돌릴 수 있게."""
    assert iteration_seeds(100, 3) == [100, 101, 102]
    assert iteration_seeds(100, 3) == iteration_seeds(100, 3)


def test_repeat_goes_home_before_every_reset() -> None:
    """리셋은 arm 자원을 함께 잡는다 — 팔이 테이블 위에 남아 있으면 물체가 그 아래 놓인다."""
    twin = _repeat_twin()
    repeat_stack(twin, 'cylinder_a', 'block_a', times=3, seed=1)

    ops = [name for name, _ in twin.calls]
    resets = [i for i, o in enumerate(ops) if o == 'reset_scene']
    assert len(resets) == 3
    for position in resets:
        assert ops[position - 1] == 'move_to_named_target', f'{position}: 복귀 직후가 아니다'


def test_repeat_uses_a_different_seed_each_time() -> None:
    twin = _repeat_twin()
    repeat_stack(twin, 'cylinder_a', 'block_a', times=4, seed=50)

    seeds = [inputs['seed'] for name, inputs in twin.calls if name == 'reset_scene']
    assert seeds == [50, 51, 52, 53]
    assert len(set(seeds)) == 4


def test_repeat_reports_every_iteration() -> None:
    twin = _repeat_twin()
    results = repeat_stack(twin, 'cylinder_a', 'block_a', times=3, seed=1)

    assert [r['index'] for r in results] == [1, 2, 3]
    assert [r['seed'] for r in results] == [1, 2, 3]


def test_repeat_keeps_going_after_a_failed_iteration() -> None:
    """한 번의 실패로 나머지를 버리면 성공률을 볼 수 없다."""
    twin = _repeat_twin(grasp=OPENED)          # 파지에서 stalled 가 안 선다 = 빈손
    results = repeat_stack(twin, 'cylinder_a', 'block_a', times=3, seed=1)

    assert len(results) == 3
    assert not any(r['ok'] for r in results)
    assert len([1 for name, _ in twin.calls if name == 'reset_scene']) == 3


def test_repeat_opens_the_gripper_after_a_failure() -> None:
    """쥔 채 다음 reset_scene 을 부르면 쥐고 있던 물체가 순간이동한다.

    **`pick` 이 시작할 때 부르는 open 과 구분해야 한다.** 그것은 실패 여부와 무관하게
    매 회 나오므로, "리셋 사이에 open 이 있다"만 보면 수습을 걷어내도 통과한다.
    실패한 **grasp 뒤에** open 이 있어야 한다.
    """
    twin = _repeat_twin(grasp=OPENED)
    repeat_stack(twin, 'cylinder_a', 'block_a', times=2, seed=1)

    def indices(target=None):
        return [i for i, (name, inputs) in enumerate(twin.calls)
                if name == 'move_gripper_to_target'
                and (target is None or inputs.get('target') == target)]

    resets = [i for i, (name, _) in enumerate(twin.calls) if name == 'reset_scene']
    failed_grasp = max(i for i in indices('grasp') if i < resets[1])
    assert any(failed_grasp < i < resets[1] for i in indices('open')), \
        '실패한 파지 뒤에 그리퍼를 열지 않고 다음 회로 갔다'
