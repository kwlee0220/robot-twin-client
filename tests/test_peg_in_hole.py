"""`peg_in_hole` 의 **판정 셋** — 기하 확인 · 앉음 확인 · 파지 오프셋.

셋 다 *"집어서 옮긴다"* 만으로는 나오지 않고, 틀렸을 때 증상이 **에러가 아니라
물체가 씬 밖으로 날아가는 것**이라 여기서 고정한다. 로봇은 쓰지 않는다 — 트윈 REST 를
흉내내는 가짜로 값만 먹인다.
"""

from __future__ import annotations

from typing import Any, Optional

import math

import pytest

from robot_twin_client.peg_in_hole import (
    FINGERTIP_OFFSET, GRASP_YAW, PegInHoleError, check_geometry, covers_xy,
    down_with_yaw, floor_at, grasp_height, other_fixture, tilt_deg, top_z,
    vertical_half_extent)

# 실측값 (2026-09-09, 빌드 MMR-Build_260907)
HOLE = {'name': 'peg_hole', 'type': 'hole',
        'position': {'x': 0.5, 'y': 0.0, 'z': 0.003},
        'entry_point': {'x': 0.5, 'y': 0.0, 'z': 0.028},
        'floor_point': {'x': 0.5, 'y': 0.0, 'z': 0.003},
        'depth': 0.025, 'inner_diameter': 0.017,
        'outer_extent_xy': [0.04, 0.04]}
# 안쪽은 2026-09-11 에 메시에서 쟀다 — 보어가 구멍과 같다 (r 8.500, 바닥 z=0).
# **입구 모따기만 다르다**: 구멍은 반경 9.473 mm 로 벌어지고 트레이는 8.498 mm 그대로다.
TRAY = {'name': 'peg_tray', 'type': 'tray',
        'position': {'x': 0.5, 'y': -0.1, 'z': 0.003},
        'entry_point': {'x': 0.5, 'y': -0.1, 'z': 0.028},
        'floor_point': {'x': 0.5, 'y': -0.1, 'z': 0.003},
        'depth': 0.025, 'inner_diameter': 0.017,
        'outer_extent_xy': [0.04, 0.04]}
# **안쪽을 모르는 자리** — 실제 파일이 아니라 합성이다. 실제 값을 쓰면 나중에 그것을
# 재서 채웠을 때 시험이 깨진다 (동작이 아니라 데이터 상태를 시험하는 것이다).
UNMEASURED = {'name': 'unmeasured', 'type': 'tray',
              'position': {'x': 0.3, 'y': 0.2, 'z': 0.003},
              'entry_point': {'x': 0.3, 'y': 0.2, 'z': 0.028},
              'outer_extent_xy': [0.04, 0.04]}
FIXTURES = {'peg_hole': HOLE, 'peg_tray': TRAY}

PEG_HALF = 0.025          # 서 있는 50 mm peg 의 수직 반높이
UPRIGHT = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
LYING = {'x': 0.0, 'y': math.sin(math.pi / 4), 'z': 0.0, 'w': math.cos(math.pi / 4)}


def peg(x: float = 0.5, y: float = 0.0, z: float = 0.028,
        orientation: Optional[dict] = None) -> dict:
    return {'type': 'cylinder', 'dimensions': [0.05, 0.008],
            'pose': {'position': {'x': x, 'y': y, 'z': z},
                     'orientation': orientation or UPRIGHT}}


# ---------------------------------------------------------------- 기하 확인


def test_normal_grasp_passes_the_geometry_check() -> None:
    """정상 파지(간격 약 −54 mm)에서는 손끝이 상면 위에 남는다."""
    target = check_geometry(HOLE, -0.054, PEG_HALF, verbose=False)

    # 바닥 0.003 + peg 반높이 0.025 − (−0.054)  → peg 바닥이 구멍 바닥에 닿는다
    assert target == pytest.approx(0.003 + 0.025 + 0.054)
    assert target - FINGERTIP_OFFSET > top_z(HOLE)


def test_ridden_up_peg_is_refused_before_moving() -> None:
    """peg 이 그리퍼 안에서 밀려 올라가면 **움직이기 전에** 거부한다.

    그대로 밀어 넣으면 손가락을 안지름 17 mm 구멍에 넣어야 하고, 실제로 그 상태에서
    peg 이 튕겨 나간 적이 있다.
    """
    with pytest.raises(PegInHoleError, match='상면 아래') as exc:
        check_geometry(HOLE, -0.010, PEG_HALF, verbose=False)

    # 아직 쥐고 서 있다 — 놓으면 안 된다.
    assert exc.value.holding is True


def test_fixture_without_a_floor_cannot_be_inserted_into() -> None:
    """**바닥을 모르면 꽂지 않는다 — 0 으로 넘겨짚지 않는다.**

    바닥을 탁자면으로 가정하면 25 mm 를 더 내려가 peg 을 눌러 버린다. `fb_move_peg.py`
    가 실제로 그 자리에 `0.003` 을 넘겨짚고 있었다 — 값은 맞았지만 근거가 없었다.
    """
    with pytest.raises(PegInHoleError, match='floor_point'):
        check_geometry(UNMEASURED, -0.054, PEG_HALF, verbose=False)


# ---------------------------------------------------------------- 실효 바닥


def test_effective_floor_is_the_fixture_top_not_the_table() -> None:
    """구멍에 꽂힌 peg 을 잡을 때 바닥은 탁자면이 아니라 **구멍 상면**이다.

    물체 바닥(구멍 속 +0.003)으로 판단하면 칼라를 15 mm 파고든다 (2026-09-10 실측).
    """
    floor, source, unknown = floor_at(FIXTURES, 0.5, 0.0)

    assert floor == pytest.approx(0.028)
    assert source == 'peg_hole'
    assert unknown == []


def test_effective_floor_falls_back_to_the_table_away_from_fixtures() -> None:
    floor, source, _ = floor_at(FIXTURES, 0.2, 0.3)

    assert floor == pytest.approx(0.0)
    assert source is None


def test_unknown_footprint_is_reported_not_treated_as_clear() -> None:
    """"안 덮는다"와 "모른다"는 다르다 — 뭉개면 손끝이 그대로 부딪힌다."""
    blind = {'name': 'blind', 'position': {'x': 0.5, 'y': 0.0, 'z': 0.0},
             'entry_point': {'x': 0.5, 'y': 0.0, 'z': 0.05}}

    assert covers_xy(blind, 0.5, 0.0) is None
    _, _, unknown = floor_at({'blind': blind}, 0.5, 0.0)
    assert unknown == ['blind']


def test_grasp_height_keeps_the_fingertips_off_the_floor() -> None:
    """손끝 여유를 먼저 확보한다 — 파지 중심을 물체 중심에 맞추면 탁자를 파고든다."""
    height = grasp_height(peg(z=0.028), floor_z=0.028)

    assert height - FINGERTIP_OFFSET >= 0.028


# ---------------------------------------------------------------- 자세


def test_lying_peg_is_out_of_scope() -> None:
    assert tilt_deg(peg(orientation=UPRIGHT)) == pytest.approx(0.0, abs=1e-9)
    assert tilt_deg(peg(orientation=LYING)) == pytest.approx(90.0, abs=1e-6)


def test_vertical_extent_shrinks_when_the_peg_lies_down() -> None:
    """누운 50 mm peg 의 수직 크기는 길이가 아니라 **지름**이다.

    길이로 계산하면 바닥을 17 mm 낮게 잡아 손끝 목표가 탁자 아래로 내려간다.
    """
    assert vertical_half_extent(peg(orientation=UPRIGHT)) == pytest.approx(0.025)
    assert vertical_half_extent(peg(orientation=LYING)) == pytest.approx(0.008)


def test_grasp_orientation_is_absolute_and_points_down() -> None:
    """⚠️ 자세는 **절대값**이다 — 현재 자세를 이어받으면 16회 만에 22.3° 기운다."""
    q = down_with_yaw(GRASP_YAW)

    assert q['z'] == 0.0 and q['w'] == 0.0          # 접근축이 정확히 연직 하향
    assert math.hypot(q['x'], q['y']) == pytest.approx(1.0)


def test_grasp_yaw_is_the_wrist_solution_away_from_the_joint7_limit() -> None:
    """**부호가 중요하다.** 2지 그리퍼는 180° 대칭이라 ±π/2 가 같은 개폐축인데

    손목 해가 다르다 — 반대 값이면 `ready` 출발 시 joint7 이 상한까지 0.5° 로 붙어
    카테시안 계획이 12.5 % 에서 끊긴다.
    """
    assert GRASP_YAW == pytest.approx(-math.pi / 2.0)


# ---------------------------------------------------------------- 목적지 선택


def test_the_other_fixture_is_chosen_unambiguously() -> None:
    assert other_fixture(FIXTURES, 'peg_hole') == 'peg_tray'
    assert other_fixture(FIXTURES, 'peg_tray') == 'peg_hole'


def test_ambiguous_destination_asks_instead_of_guessing() -> None:
    """후보가 둘이면 **고르지 않는다** — 틀린 자리에 꽂는 것보다 묻는 편이 낫다."""
    three: Any = {**FIXTURES, 'peg_hole_b': dict(HOLE, name='peg_hole_b')}

    with pytest.raises(PegInHoleError, match='목적지를 정할 수 없다'):
        other_fixture(three, 'peg_hole')


# ---------------------------------------------- 정착 (2026-09-11 실패에서 나옴)


class _FakeTwin:
    """`ee_pose` 를 미리 정한 순서대로 돌려주는 가짜."""

    def __init__(self, sequence: list) -> None:
        self.sequence = list(sequence)
        self.reads = 0

    def read(self, name: str, *, require_ok: bool = True) -> Any:
        assert name == 'ee_pose'
        self.reads += 1
        z = self.sequence[min(self.reads - 1, len(self.sequence) - 1)]
        return {'pose': {'position': {'x': 0.5, 'y': 0.0, 'z': z}}}


def test_wait_until_still_returns_only_after_motion_stops() -> None:
    """**고정 대기로는 얼마를 줘야 하는지 알 수 없다.**

    3 초를 줬더니 하강 뒤 오차를 7.55 mm 로 잘못 읽었고(실제 0.04 mm) 그 값으로 다음
    목표를 만들어 삽입이 실패했다 (2026-09-11).
    """
    from robot_twin_client.ops import wait_until_still

    twin = _FakeTwin([0.20, 0.15, 0.11, 0.0820, 0.0820, 0.0820, 0.0820, 0.0820])
    pose = wait_until_still(twin, timeout_sec=5.0, quiet_sec=0.3, poll_sec=0.05)

    assert pose is not None
    assert pose['pose']['position']['z'] == pytest.approx(0.0820)


def test_wait_until_still_gives_up_rather_than_lying() -> None:
    """멈추지 않으면 `None` 이다 — 움직이는 중의 값을 도달로 넘기지 않는다."""
    from robot_twin_client.ops import wait_until_still

    twin = _FakeTwin([0.20 - 0.01 * i for i in range(200)])
    assert wait_until_still(twin, timeout_sec=0.4, quiet_sec=0.3, poll_sec=0.05) is None


# ------------------------------------- 바닥 정렬 (2026-09-11 두 번째 실패에서 나옴)


def tilted(deg: float) -> dict:
    """+x 쪽으로 `deg` 만큼 기운 peg — y 축 둘레 회전."""
    half = math.radians(deg) / 2.0
    return peg(orientation={'x': 0.0, 'y': math.sin(half), 'z': 0.0,
                            'w': math.cos(half)})


def test_bottom_moves_away_from_the_centre_when_the_peg_tilts() -> None:
    """**구멍에 들어가는 것은 중심이 아니라 바닥 끝이다.**

    0.74° 는 눈에 안 보이지만 바닥을 **0.32 mm** 옮긴다 — 모따기 없는 트레이의 반경
    여유가 0.50 mm 뿐이라 그 하나로 한계에 닿는다.
    """
    from robot_twin_client.peg_in_hole import bottom_of

    upright = bottom_of(peg())
    assert upright[0] == pytest.approx(0.5)
    assert upright[2] == pytest.approx(0.028 - 0.025)

    b = bottom_of(tilted(0.74))
    assert abs(b[0] - 0.5) == pytest.approx(0.025 * math.sin(math.radians(0.74)), rel=1e-6)
    assert abs(b[0] - 0.5) * 1000 == pytest.approx(0.32, abs=0.01)


def test_axis_always_points_up() -> None:
    """뒤집힌 쿼터니언을 받아도 바닥을 위쪽으로 계산하지 않는다."""
    from robot_twin_client.peg_in_hole import axis_of

    assert axis_of(peg())[2] > 0
    assert axis_of(peg(orientation={'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}))[2] > 0


def test_entry_clearance_is_the_conservative_bore_value() -> None:
    """**모따기를 계산에 넣지 않는다** — 구멍에는 있고 트레이에는 없다.

    실측: 입구 z=+0.0250 에서 구멍은 반경 9.473 mm, 트레이는 8.498 mm 다. 모따기를
    믿으면 트레이에서 두 배 관대한 판정이 되어 그대로 입구에 걸린다.
    """
    from robot_twin_client.peg_in_hole import entry_clearance

    assert entry_clearance(HOLE, 0.008) == pytest.approx(0.0005)
    assert entry_clearance(TRAY, 0.008) == pytest.approx(0.0005)
    assert entry_clearance(UNMEASURED, 0.008) is None


# ------------------------------- 정렬 되먹임 (2026-09-11 구멍 성공/트레이 실패에서)


def test_align_margin_is_tighter_than_the_bare_clearance() -> None:
    """**여유 안에 드는 것만으로는 부족하다.**

    실측: 바닥 오차 **0.31 mm** 로 구멍(모따기 여유 1.47 mm)에는 들어갔고,
    **0.40 mm** 로 트레이(0.50 mm, 모따기 없음)에는 걸렸다. 0.40 은 0.50 안이지만
    날카로운 모서리에서는 못 들어간다 — 그래서 여유의 일부만 쓴다.
    """
    from robot_twin_client.peg_in_hole import ALIGN_MARGIN, entry_clearance

    allowed = entry_clearance(TRAY, 0.008) * ALIGN_MARGIN
    assert allowed == pytest.approx(0.0002)
    assert 0.00040 > allowed, '실패했던 0.40 mm 가 여전히 통과하면 안 된다'


def test_alignment_is_bounded() -> None:
    """**무한정 반복하지 않는다** — 흔들수록 peg 이 그리퍼 안에서 돌아앉는다.

    회복 시도에서 이동마다 약 2° 씩 기울었다(접촉이 없어도).
    """
    from robot_twin_client.peg_in_hole import ALIGN_PASSES

    assert 1 <= ALIGN_PASSES <= 4


# ------------------------- 주기적 정지 견디기 (2026-09-11 왕복 반복에서 나옴)


class _StallingTwin:
    """처음 몇 번은 `STALE` 을 주고 그 뒤 `OK` 를 주는 가짜 — 주기적 정지를 흉내낸다."""

    def __init__(self, stale_reads: int) -> None:
        self.remaining = stale_reads
        self.reads = 0

    def read_raw(self, name: str) -> dict:
        self.reads += 1
        if self.remaining > 0:
            self.remaining -= 1
            return {'quality': 'STALE', 'reason': 'age 2100ms > 200ms'}
        return {'quality': 'OK', 'value': {'objects': {'peg': peg()}}}


def test_a_brief_stall_does_not_abort_the_task() -> None:
    """이 시뮬레이터는 **주기적으로 멈춘다** (실측 34초마다 약 2초).

    그때마다 중단하면 긴 작업이 거의 끝나지 않는다 — 왕복 반복 2회차에서 peg 을 쥔 채
    멈췄다.
    """
    from robot_twin_client.peg_in_hole import _read_scene

    twin = _StallingTwin(stale_reads=3)
    obj = _read_scene(twin)

    assert obj['pose']['position']['x'] == pytest.approx(0.5)
    assert twin.reads == 4, '낡은 동안 기다렸다가 다시 읽어야 한다'


def test_a_persistent_stall_is_still_an_error() -> None:
    """**영영 기다리지는 않는다** — 안 돌아오면 멈춘 것이 아니라 끊긴 것이다."""
    from robot_twin_client.peg_in_hole import _read

    twin = _StallingTwin(stale_reads=10_000)
    with pytest.raises(PegInHoleError, match='끊긴 것'):
        _read(twin, 'scene_objects', timeout_sec=0.5)


def test_stall_tolerance_is_several_times_the_measured_stall() -> None:
    """실측 정지가 최대 2.2초다 — 그보다 넉넉해야 '멈춤'과 '끊김'이 갈린다."""
    from robot_twin_client.peg_in_hole import STALE_TOLERANCE_SEC

    assert STALE_TOLERANCE_SEC >= 2.2 * 4


def test_unknown_destination_is_refused_before_touching_the_robot() -> None:
    """오타 하나로 집기까지 하면 안 된다.

    실측(2026-09-11): 검증이 `place_peg` 안에만 있어서, 없는 이름을 주면 **접근·하강·
    파지까지 한 뒤** 거부하고 peg 을 쥔 채 멈췄다. 헛동작이고, 쥔 상태는 사람이 풀어야
    한다.
    """
    from robot_twin_client import peg_in_hole

    calls: list = []

    class _Twin:
        def read_raw(self, name: str) -> dict:
            value = ({'objects': {'peg': peg()}} if name == 'scene_objects'
                     else {'fixtures': FIXTURES})
            return {'quality': 'OK', 'value': value}

    def _boom(*a, **k):
        calls.append('picked')
        raise AssertionError('로봇을 건드리기 전에 멈췄어야 한다')

    original = peg_in_hole.pick_peg
    peg_in_hole.pick_peg = _boom
    try:
        with pytest.raises(PegInHoleError, match='고정물이 없다'):
            peg_in_hole.move_peg(_Twin(), 'peg_hole_2', go_home=False, verbose=False)
    finally:
        peg_in_hole.pick_peg = original

    assert calls == [], '집기를 시도했다'
