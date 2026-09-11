"""peg 을 한 고정물에서 다른 고정물로 **옮겨 넣는다** — 트윈 REST 만으로.

    robot-twin-peg                   # 반대쪽으로 (현재 위치에서 자동 판단)
    robot-twin-peg peg_tray          # 목적지를 명시
    robot-twin-peg --dry-run         # 판단만 하고 움직이지 않는다

`scripts/functionbay/fb_move_peg.py` 가 ROS 에 직접 붙어 하던 일을 옮긴 것이다.
**이 모듈은 ROS 를 모른다** — `scene_objects` · `fixtures` · `ee_pose` 를 읽고
`move_linear` · `move_gripper_to_target` 를 부를 뿐이다.

세 가지가 이 절차의 실체이며, 셋 다 *"집어서 옮긴다"* 만으로는 나오지 않는다.

**① 파지 오프셋 보정 — peg 을 정렬한다, 손이 아니다.**
그리퍼는 peg 을 정확히 중심에 물지 않는다 (실측 y 로 **−0.36 mm**). 반경 여유가
**0.5 mm** 뿐이라 그 0.36 mm 를 무시하면 안 들어간다 — 실제로 오프셋 +1.46 mm 를
무시했다가 입구 턱에 걸렸다. 쥔 상태에서 `peg − grasp_center` 를 재고
`목표 = 구멍중심 − 오프셋` 으로 지령한다.

**② 기하 사전 확인 — 손끝이 고정물 상면 위에 남는가.**
peg 이 그리퍼 안에서 밀려 올라가 있으면 완전 삽입에 **손가락을 구멍 안에 넣어야**
하는데 안지름이 17 mm 라 불가능하다. 그 상태로 밀어 넣다 peg 이 튕겨 나간 적이 있다.
**움직이기 전에** 막고 다시 잡으라고 알린다.

**③ 앉음 확인 — 놓기 전에 실제로 앉았는지 본다.**
안 앉은 채(눌렸거나 떠 있거나) 놓으면 **시뮬레이터가 물체를 씬 밖으로 튕겨낸다**
(x +188 mm, z −3900 m 까지 가속; 재현 4/4). 그리퍼 `effort` 로는 안 잡히고 물체
위치를 봐야 보인다. 펑션베이는 `/scene/reset` 이 없어 복구가 **시뮬레이터 재시작뿐**이라
미착석이면 **쥔 채로 두고** 실패를 돌려준다.

**다루는 것은 서 있는 peg 뿐이다** — 쓰러진 것은 범위 밖이며 거부한다.
"""

from __future__ import annotations

from typing import Any, Optional

import argparse
import math
import time

from .client import OperationFailed, RobotTwinClient
from .ops import move_linear_verified, move_to_named_target_verified, set_gripper

OBJECT = 'peg'

# `grasp_center` 에서 실제 손끝까지 (그리퍼 열림 실측 39~40 mm, 충돌 메시 44 mm).
# **여유는 큰 쪽으로 잡는다** — 닫을수록 손가락이 세워져 44 mm 에 가까워진다.
FINGERTIP_OFFSET = 0.044
# 파지 지점에서 손끝이 바닥에서 떠 있어야 하는 최소 높이.
FINGERTIP_CLEARANCE = 0.010
# 접근 단계에서 물체 위에 두는 높이 (`grasp_center` 기준).
APPROACH_Z = 0.120
# 쥔 채 옮길 때의 높이.
TRAVEL_Z = 0.150

# peg 이 앉았다고 볼 높이 허용치. 구멍 깊이가 25 mm 라 3 mm 면 "들어갔다"와
# "턱에 걸렸다"를 확실히 가른다.
SEAT_TOL = 0.003
# 쥔 채로 이만큼 밀려 올라가면 어딘가에 닿은 것이다.
RIDE_LIMIT = 0.004
# 원기둥이 이보다 기울어져 있으면 서 있는 것으로 보지 않는다.
UPRIGHT_LIMIT_DEG = 15.0

# 상태가 **잠깐 낡은 것**을 얼마나 기다릴 것인가. 이 시뮬레이터는 주기적으로 멈춘다
# (실측 34초마다 약 2초, 나쁜 회차에서 최대 2.2초). 그때마다 중단하면 긴 작업이 거의
# 끝나지 않는다. 10초는 그 정지의 4배 이상이라 '멈춤'과 '끊김'을 가른다.
STALE_TOLERANCE_SEC = 10.0

# 정렬을 몇 번까지 되먹일 것인가. **무한정 반복하지 않는다** — 흔들수록 peg 이 그리퍼
# 안에서 돌아앉아 오히려 나빠진다 (회복 시도에서 이동마다 약 2° 씩 기울었다).
ALIGN_PASSES = 3
# 반경 여유의 몇 %  안으로 들어와야 내려가는가. 모따기 없는 트레이의 여유가 0.50 mm
# 뿐이라, 0.40 mm(80 %)로 내렸을 때 실제로 입구에 걸렸다.
ALIGN_MARGIN = 0.4

# **개루프 백엔드에서는 정착을 기다린 뒤에 판정한다.** 스트리밍은 반환이 도달을 뜻하지
# 않아서, 끝나자마자 읽으면 아직 움직이는 중의 값을 본다 — 실측(2026-09-09) 종료
# 1.5 초 뒤 z 오차가 64.8 mm 였는데 정착 후에는 −0.6 mm 였다.
#
# ⚠️ **이것은 고정 대기가 아니라 상한이다** — 실제로는 `ee_pose` 가 멈출 때까지 기다린다.
# 고정 3 초를 줬다가 하강 뒤 오차를 7.55 mm 로 잘못 읽었고(실제 0.04 mm), 그 값으로
# 다음 목표를 만들어 **삽입이 실패하고 peg 이 그리퍼 안에서 26° 기울었다** (2026-09-11).
SETTLE_SEC = 15.0

# 개폐축(손가락 둘을 잇는 축)의 방향, base 의 +x 에서 잰 각.
#
# **`grasp_center` 기준이다** — 트윈의 `move_linear` 이 기본으로 그 프레임을 쓰고,
# 그 프레임의 x 축이 곧 개폐축이라 이 값이 그대로 개폐축 각이 된다 (실측 −90.0°).
#
# ⚠️ **`fb_move_peg.py` 의 `DEFAULT_GRASP_YAW = +π/2` 와 부호가 반대인데 같은 자세다.**
# 저쪽은 `panda_link8` 기준으로 만들면서 `yaw − 90°` 를 쓰고 이쪽은 트윈이 `+90°` 를
# 더하므로, 같은 손목 해에 이르는 입력이 180° 어긋난다. 2지 그리퍼는 180° 대칭이라
# **개폐축은 같지만 손목 해가 다르다** — 반대 값을 쓰면 `ready` 에서 출발할 때 joint7 이
# 상한(+2.897)까지 0.5° 로 붙어 **카테시안 계획이 12.5 % 에서 끊긴다.**
GRASP_YAW = -math.pi / 2.0


class PegInHoleError(RuntimeError):
    """절차가 **로봇을 움직이기 전에 또는 안전한 상태에서** 멈춘 경우.

    미착석으로 멈춘 경우에는 ``holding`` 이 True 다 — **peg 을 쥔 채**라는 뜻이며,
    놓으면 시뮬레이터가 물체를 씬 밖으로 튕겨낸다.
    """

    def __init__(self, message: str, *, holding: bool = False) -> None:
        super().__init__(message)
        self.holding = holding


# ---------------------------------------------------------------- 기하 헬퍼


def down_with_yaw(yaw_rad: float) -> dict:
    """접근축이 **정확히 연직 하향**인 자세를 z 둘레로 `yaw_rad` 돌린 것.

    ⚠️ **자세는 절대값으로 만든다 — 현재 자세를 읽어 다음 목표로 주면 안 된다.**
    처짐은 위치만이 아니라 자세도 틀어놓으므로 그 오차가 지워지지 않고 쌓인다.
    실측(2026-09-09): 그렇게 16회 이동했더니 접근축이 연직에서 **22.3° 기울었다.**
    """
    half = yaw_rad / 2.0
    return {'x': math.cos(half), 'y': math.sin(half), 'z': 0.0, 'w': 0.0}


def _pose(x: float, y: float, z: float) -> dict:
    return {'position': {'x': float(x), 'y': float(y), 'z': float(z)},
            'orientation': down_with_yaw(GRASP_YAW)}


def tilt_deg(obj: dict) -> float:
    """원기둥 축이 연직에서 벗어난 각(도). 0 이면 서 있다."""
    q = obj['pose']['orientation']
    axis_z = 1.0 - 2.0 * (q['x'] * q['x'] + q['y'] * q['y'])
    return math.degrees(math.acos(min(1.0, abs(axis_z))))


def vertical_half_extent(obj: dict) -> float:
    """물체가 중심에서 위아래로 얼마나 뻗는가 — **자세를 반영한다.**

    ⚠️ `dimensions[0]`(원기둥 길이)을 그냥 수직 크기로 쓰면 **누운 물체에서 틀린다.**
    50 mm peg 이 누우면 수직 크기는 길이가 아니라 지름(16 mm)이다. 그렇게 계산하면
    바닥을 17 mm 낮게 잡아 손끝 목표가 탁자 아래로 내려간다.
    """
    length, radius = obj['dimensions'][0], obj['dimensions'][1]
    q = obj['pose']['orientation']
    axis_z = 1.0 - 2.0 * (q['x'] * q['x'] + q['y'] * q['y'])
    horizontal = max(0.0, 1.0 - axis_z * axis_z) ** 0.5
    return abs(axis_z) * (length / 2.0) + horizontal * radius


def axis_of(obj: dict) -> tuple:
    """원기둥의 **위를 향하는 축 단위벡터**. 쿼터니언의 로컬 z 를 회전시킨 것이다."""
    q = obj['pose']['orientation']
    ax = (2 * (q['x'] * q['z'] + q['y'] * q['w']),
          2 * (q['y'] * q['z'] - q['x'] * q['w']),
          1 - 2 * (q['x'] ** 2 + q['y'] ** 2))
    return ax if ax[2] >= 0 else tuple(-v for v in ax)


def bottom_of(obj: dict) -> tuple:
    """원기둥 **아래쪽 끝면의 중심** `(x, y, z)`.

    ⚠️ **구멍에 들어가는 것은 중심이 아니라 이 점이다.** 기울기가 작아도 지렛대가
    길어서 무시할 수 없다 — 50 mm peg 이 0.74° 기울면 바닥은 중심에서 **0.32 mm**
    벗어나는데, 모따기 없는 트레이의 반경 여유가 **0.50 mm** 뿐이라 그 하나로
    한계에 닿는다 (2026-09-11 실측: 중심으로 맞춰 두 번 연속 입구에 걸렸다).
    """
    p = obj['pose']['position']
    ax = axis_of(obj)
    half = obj['dimensions'][0] / 2.0
    return (p['x'] - half * ax[0], p['y'] - half * ax[1], p['z'] - half * ax[2])


def entry_clearance(fixture: dict, peg_radius: float) -> Optional[float]:
    """입구에서 허용되는 **반경 오차**. 안지름을 모르면 `None`.

    구멍은 입구가 모따기되어 있어 실제로는 더 관대하지만(실측 반경 9.47 mm), **모따기를
    믿고 계산하지 않는다** — 트레이에는 없기 때문이다. 보수적인 쪽을 쓴다.
    """
    inner = fixture.get('inner_diameter')
    return None if inner is None else inner / 2.0 - peg_radius


def covers_xy(fixture: dict, x: float, y: float) -> Optional[bool]:
    """이 고정물이 `(x, y)` 를 덮는가. **발자국을 모르면 `None`** 이다.

    `None` 을 `False` 로 뭉개면 안 된다 — "안 덮는다"와 "모른다"는 다르고, 뒤를
    모르는 채로 손끝을 내리면 그대로 부딪힌다.
    """
    extent = fixture.get('outer_extent_xy')
    if extent is None:
        return None
    center = fixture['position']
    return (abs(x - center['x']) <= extent[0] / 2.0
            and abs(y - center['y']) <= extent[1] / 2.0)


def top_z(fixture: dict) -> Optional[float]:
    """**손끝이 닿는 윗면**의 높이 — 구멍·트레이는 입구면이다. 모르면 `None`."""
    entry = fixture.get('entry_point')
    return None if entry is None else entry['z']


def floor_at(fixtures: dict, x: float, y: float, table_z: float = 0.0) -> tuple:
    """`(x, y)` 에서 **손끝이 닿는 실효 바닥** — `(높이, 근거 이름, 판정불가 목록)`.

    **물체 자체의 바닥으로 판단하면 안 된다** — 구멍에 꽂힌 peg 은 바닥이 구멍 속
    (z=+0.003)이지만 손끝이 실제로 닿는 면은 구멍 상면(z=+0.028)이라, 그대로 내리면
    칼라를 15 mm 파고든다 (2026-09-10 실측).
    """
    floor, source, unknown = float(table_z), None, []
    for name in sorted(fixtures):
        fixture = fixtures[name]
        covered = covers_xy(fixture, x, y)
        if covered is None:
            unknown.append(name)
            continue
        top = top_z(fixture)
        if covered and top is not None and top > floor:
            floor, source = top, name
    return floor, source, unknown


def grasp_height(obj: dict, floor_z: float) -> float:
    """`grasp_center` 를 어디에 둘 것인가.

    **두 조건을 따로 본다.** 손끝이 바닥을 안 치는가, 그리고 패드가 물체를 덮는가.
    파지 중심을 물체 중심에 맞추려 하면 안 된다 — 실제로 잡는 것은 56.5 mm 짜리 패드
    면이라 물체가 그 구간에 들어오기만 하면 된다.
    """
    bottom = max(obj['pose']['position']['z'] - vertical_half_extent(obj), floor_z)
    return bottom + FINGERTIP_OFFSET + FINGERTIP_CLEARANCE


# ---------------------------------------------------------------- 관측


def _read(twin: RobotTwinClient, name: str, *, timeout_sec: float = STALE_TOLERANCE_SEC):
    """상태 변수를 읽는다 — **잠깐 낡은 것은 기다린다.**

    이 시뮬레이터는 **주기적으로 멈춘다** (실측 34초마다 약 2초, 회차에 따라 최대 2.2초).
    그동안 상태가 `STALE` 로 보고되는데, 그때마다 작업을 통째로 중단하면 **긴 작업은
    거의 끝나지 않는다** — 실제로 왕복 반복 중 2회차에서 peg 을 쥔 채 멈췄다
    (2026-09-11).

    **영영 기다리지는 않는다.** `timeout_sec` 을 넘도록 안 돌아오면 그것은 잠깐 멈춘
    것이 아니라 **끊긴 것**이므로 그대로 올린다.
    """
    deadline = time.monotonic() + timeout_sec
    while True:
        body = twin.read_raw(name)
        if body.get('quality') == 'OK':
            return body['value']
        if time.monotonic() >= deadline:
            raise PegInHoleError(
                f"상태 변수 {name!r} 이 {body.get('quality')} 로 {timeout_sec:g}초 넘게 "
                f"머물렀다 ({body.get('reason') or '사유 없음'}) — 잠깐 멈춘 것이 아니라 "
                '끊긴 것이다. 시뮬레이터와 스택을 확인한다', holding=True)
        time.sleep(0.3)


def _read_scene(twin: RobotTwinClient) -> dict:
    objects = _read(twin, 'scene_objects')['objects']
    if OBJECT not in objects:
        raise PegInHoleError(f"scene_objects 에 {OBJECT!r} 이 없다 — scene 노드를 본다")
    return objects[OBJECT]


def _read_fixtures(twin: RobotTwinClient) -> dict:
    return _read(twin, 'fixtures')['fixtures']


def _tcp(twin: RobotTwinClient) -> dict:
    """지금 `grasp_center` 위치. `move_linear` 목표와 **같은 프레임**이다."""
    return _read(twin, 'ee_pose')['pose']['position']


def wait_until_peg_still(twin: RobotTwinClient, *, timeout_sec: float = 10.0,
                         tol_m: float = 0.0002, quiet_sec: float = 0.6,
                         poll_sec: float = 0.3) -> dict:
    """peg 이 **그리퍼 안에서** 멎을 때까지 기다리고 마지막 관측을 돌려준다.

    `ops.wait_until_still` 은 팔(`ee_pose`)만 본다. **팔이 섰다고 peg 이 선 것은
    아니다** — 손끝 위쪽 15 mm 만 물린 채 35 mm 가 매달려 있어 옆으로 100 mm 옮기면
    한동안 흔들린다. 그 흔들리는 중에 바닥 위치를 재면 정렬이 그만큼 틀어진다.

    못 멎어도 마지막 관측을 준다 — 여기서는 판정이 아니라 **타이밍**이 목적이고,
    정렬 오차는 뒤의 되먹임이 다시 본다.
    """
    deadline = time.monotonic() + timeout_sec
    previous = None
    still_since = None
    obj = _read_scene(twin)
    while time.monotonic() < deadline:
        obj = _read_scene(twin)
        b = bottom_of(obj)
        if previous is not None:
            moved = max(abs(b[i] - previous[i]) for i in range(3))
            if moved <= tol_m:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since >= quiet_sec:
                    return obj
            else:
                still_since = None
        previous = b
        time.sleep(poll_sec)
    return obj


def where_is_peg(twin: RobotTwinClient) -> tuple:
    """peg 이 어느 고정물 위에 있는가 — `(물체, 고정물 목록, 이름 또는 None)`."""
    obj = _read_scene(twin)
    fixtures = _read_fixtures(twin)
    p = obj['pose']['position']
    _, source, _ = floor_at(fixtures, p['x'], p['y'])
    return obj, fixtures, source


def other_fixture(fixtures: dict, source: Optional[str]) -> str:
    """*"다른 자리"* 를 고른다. 후보가 하나가 아니면 **묻는다.**"""
    others = [n for n in sorted(fixtures) if n != source]
    if len(others) != 1:
        raise PegInHoleError(
            f'목적지를 정할 수 없다 — 고정물 {sorted(fixtures)} 중 하나를 지정한다')
    return others[0]


# ---------------------------------------------------------------- 이동


def _go(twin: RobotTwinClient, x: float, y: float, z: float, *, velocity_scaling: float,
        tol_m: float = 0.005, require_reached: bool = True) -> dict:
    """`grasp_center` 를 절대 좌표로 옮긴다. **멈출 때까지 기다린 뒤** 판정한다.

    `require_reached` 를 끄는 것은 **접촉이 예상되는 구간뿐**이다 (구멍으로의 하강).
    거기서는 못 간 것 자체가 정보이고, 앉음 확인이 뒤에서 판정한다.
    """
    check = move_linear_verified(twin, _pose(x, y, z), tol_m=tol_m,
                                 velocity_scaling=velocity_scaling,
                                 settle_sec=SETTLE_SEC)
    if check['result'].get('status') != 'COMPLETED':
        raise PegInHoleError(
            f"move_linear 이 실패했다: {check['result'].get('status')} "
            f"({check['result'].get('error') or check['result'].get('detail')})")
    if require_reached and not check.get('reached'):
        # **여기서 멈추는 것이 중요하다.** 못 간 자리를 도달한 것으로 알고 다음 목표를
        # 만들면 그 오차가 그대로 실린다 — 정렬이 틀어지면 peg 이 입구에서 기운다.
        raise PegInHoleError(
            f'({x:+.4f}, {y:+.4f}, {z:+.4f}) 에 도달하지 못했다 '
            f"(오차 {check.get('error_m', float('nan')) * 1000:.2f} mm, "
            f"{check.get('why', '경로가 잘렸거나 접촉했다')})", holding=True)
    return check


# ---------------------------------------------------------------- 절차


def pick_peg(twin: RobotTwinClient, *, verbose: bool = True) -> float:
    """peg 을 집는다. **파지 간격 `peg − grasp_center`** 를 돌려준다.

    그 간격이 뒤따르는 두 판정의 입력이다 — 기하 확인과 앉음 확인 모두 이 값을 쓴다.
    """
    obj, fixtures, source = where_is_peg(twin)
    tilt = tilt_deg(obj)
    if tilt > UPRIGHT_LIMIT_DEG:
        raise PegInHoleError(
            f'peg 이 {tilt:.1f}° 기울어져 있다 — 서 있는 물체만 다룬다')

    p = obj['pose']['position']
    floor, name, unknown = floor_at(fixtures, p['x'], p['y'])
    if unknown:
        # 발자국을 모르는 고정물이 있으면 바닥이 **낙관적일 수 있다.**
        print(f'  ⚠️ 발자국을 몰라 판정 못한 고정물: {unknown} — 바닥이 낙관적일 수 있다')
    height = grasp_height(obj, floor)
    if verbose:
        print(f'  출발 {name or "탁자"} (실효 바닥 {floor:+.4f}) → grasp_center {height:.4f}')

    set_gripper(twin, 'open')
    _go(twin, p['x'], p['y'], APPROACH_Z, velocity_scaling=0.15)
    _go(twin, p['x'], p['y'], height, velocity_scaling=0.25)

    tcp = _tcp(twin)
    error = math.hypot(tcp['x'] - p['x'], tcp['y'] - p['y'])
    if verbose:
        print(f'  접근 오차 수평 {error * 1000:.2f} mm  z {(tcp["z"] - height) * 1000:+.2f} mm')

    result = set_gripper(twin, 'grasp')
    if not result.get('at_goal'):
        raise PegInHoleError(f'파지가 성립하지 않았다 — at_goal={result.get("at_goal")}, '
                             f'stalled={result.get("stalled")}')
    time.sleep(1.5)

    held = _read_scene(twin)
    gap = held['pose']['position']['z'] - _tcp(twin)['z']
    tilt = tilt_deg(held)
    if verbose:
        print(f'  파지 간격 {gap * 1000:+.2f} mm (정상 약 −54), 기울기 {tilt:.2f}°')
    if tilt > UPRIGHT_LIMIT_DEG:
        raise PegInHoleError(
            f'집은 뒤 peg 이 {tilt:.1f}° 기울었다 — 이대로는 구멍 입구에서 걸린다',
            holding=True)
    return gap


def check_geometry(fixture: dict, gap: float, half_extent: float, *,
                   verbose: bool = True) -> float:
    """완전 삽입이 기하적으로 가능한가. 목표 `grasp_center` z 를 돌려준다.

    `half_extent` 는 **peg 의 수직 반높이**다 (고정물 깊이의 절반이 아니다). 완전히
    꽂히면 peg 바닥이 구멍 바닥에 닿으므로 peg 중심은 `바닥 + 반높이` 에 오고, 거기서
    파지 간격만큼 빼면 `grasp_center` 목표가 된다.

    Raises:
        PegInHoleError: 손끝이 고정물 상면 아래로 들어가야 할 때. peg 이 그리퍼 안에서
            밀려 올라간 것이며, **그대로 밀어 넣으면 물체가 튕겨 나간다.** 아직 움직이기
            전이므로 `holding` 은 True 다 — 쥔 채로 서 있다.
    """
    floor_point = fixture.get('floor_point')
    if floor_point is None:
        raise PegInHoleError(
            f"{fixture['name']} 은 바닥 높이를 모른다 (floor_point 가 없다) — "
            '꽂을 수 없다. 안쪽 형상을 재야 한다', holding=True)
    target = floor_point['z'] + half_extent - gap
    tip = target - FINGERTIP_OFFSET
    top = top_z(fixture)
    if verbose:
        print(f'  간격 {gap * 1000:+.2f} mm → 완전 삽입 시 grasp_center {target:.4f}, '
              f'손끝 {tip:+.4f}')
        print(f'  {fixture["name"]} 상면 {top:+.4f} → 여유 {(tip - top) * 1000:+.1f} mm')
    if top is not None and tip <= top:
        raise PegInHoleError(
            f'손끝이 {fixture["name"]} 상면 아래로 들어가야 완전 삽입이 된다 — 불가능하다.\n'
            f'  peg 이 그리퍼 안에서 밀려 올라갔다 (간격 {gap * 1000:+.1f} mm, 정상 약 −54).\n'
            f'  놓았다 다시 잡은 뒤 재시도한다.', holding=True)
    return target


def _align_bottom_over(twin: RobotTwinClient, entry: dict, clearance: Optional[float], *,
                       travel_z: float, verbose: bool = True) -> tuple:
    """peg **바닥 끝**을 `entry` 위에 올린다. `(목표 x, 목표 y, 남은 오차)`.

    한 번에 안 맞는다 — 옆으로 100 mm 옮기면 peg 이 그리퍼 안에서 조금 돌아앉아서,
    옮기기 **전에** 잰 오프셋으로 계산한 목표는 0.4 mm 쯤 빗나간다. 모따기가 있는
    구멍은 그 정도를 삼키지만(여유 1.47 mm) **모따기가 없는 트레이는 0.50 mm 뿐이라
    그대로 입구에 걸린다** (2026-09-11: 같은 정렬 품질에서 구멍 성공·트레이 실패).

    그래서 **재고 고치기를 반복한다.** 매번 peg 이 멎기를 기다린 뒤 바닥을 재고, 남은
    오차만큼 목표를 옮긴다. 줄지 않으면 더 흔들지 않고 그대로 돌려준다 — 호출자가
    여유와 대조해 판정한다.
    """
    aim_x = aim_y = 0.0
    error = float('inf')
    for attempt in range(ALIGN_PASSES):
        held = wait_until_peg_still(twin)
        bottom = bottom_of(held)
        tcp = _tcp(twin)
        # 되먹임: 지금 tcp 에서 바닥이 얼마나 빗나갔는지만큼 목표를 옮긴다.
        aim_x = tcp['x'] + (entry['x'] - bottom[0])
        aim_y = tcp['y'] + (entry['y'] - bottom[1])
        _go(twin, aim_x, aim_y, travel_z, velocity_scaling=0.15)

        held = wait_until_peg_still(twin)
        bottom = bottom_of(held)
        moved = math.hypot(bottom[0] - entry['x'], bottom[1] - entry['y'])
        if verbose:
            print(f'  정렬 {attempt + 1}회  peg 바닥 오차 '
                  f'({(bottom[0] - entry["x"]) * 1000:+.2f}, '
                  f'{(bottom[1] - entry["y"]) * 1000:+.2f}) mm = {moved * 1000:.2f} mm, '
                  f'기울기 {tilt_deg(held):.2f}°')
        if moved >= error:
            # 더 안 줄어든다. 흔들수록 peg 이 돌아앉으므로 여기서 멈춘다.
            if verbose:
                print('  → 더 줄지 않는다. 반복을 멈춘다')
            return aim_x, aim_y, min(error, moved)
        error = moved
        if clearance is not None and error < clearance * ALIGN_MARGIN:
            return aim_x, aim_y, error
    return aim_x, aim_y, error


def place_peg(twin: RobotTwinClient, destination: str, gap: float, *,
              travel_z: float = TRAVEL_Z, verbose: bool = True) -> dict:
    """쥐고 있는 peg 을 `destination` 에 꽂는다.

    **앉음 확인 전에는 놓지 않는다.** 미착석이면 쥔 채로 `PegInHoleError(holding=True)`
    를 던진다 — 놓으면 시뮬레이터가 물체를 씬 밖으로 튕겨내고 복구가 재시작뿐이다.
    """
    fixtures = _read_fixtures(twin)
    if destination not in fixtures:
        raise PegInHoleError(f'{destination!r} 라는 고정물이 없다 — '
                             f'있는 것: {sorted(fixtures)}', holding=True)
    fixture = fixtures[destination]
    entry = fixture['entry_point']

    # **peg 의 반높이**를 쓴다 — 고정물 깊이가 아니다. 누운 물체에서 둘은 크게 다르다.
    half_extent = vertical_half_extent(_read_scene(twin))
    target_z = check_geometry(fixture, gap, half_extent, verbose=verbose)

    tcp = _tcp(twin)
    _go(twin, tcp['x'], tcp['y'], travel_z, velocity_scaling=0.12)

    held = wait_until_peg_still(twin)
    clearance = entry_clearance(fixture, held['dimensions'][1])
    if verbose and clearance is not None:
        print(f'  {destination} 반경 여유 {clearance * 1000:.2f} mm '
              '(모따기는 계산에 넣지 않는다 — 트레이에는 없다)')

    aim_x, aim_y, error = _align_bottom_over(twin, entry, clearance,
                                             travel_z=travel_z, verbose=verbose)
    if clearance is not None and error >= clearance * ALIGN_MARGIN:
        raise PegInHoleError(
            f'정렬이 {error * 1000:.2f} mm 에서 멈췄다 — {destination} 의 여유 '
            f'{clearance * 1000:.2f} mm 의 {ALIGN_MARGIN:.0%} 안으로 못 들어왔다. '
            '이대로 내리면 입구에 걸린다', holding=True)
    tilt = tilt_deg(_read_scene(twin))
    if tilt > UPRIGHT_LIMIT_DEG:
        # **내려가기 전에 막는다** — 기운 채 내리면 입구에서 걸려 더 기운다.
        raise PegInHoleError(
            f'이송 중 peg 이 {tilt:.1f}° 기울었다 — 내리지 않는다', holding=True)

    began = time.time()
    _go(twin, aim_x, aim_y, target_z, velocity_scaling=0.08, require_reached=False)
    peg, tcp = _read_scene(twin)['pose']['position'], _tcp(twin)
    ride = (peg['z'] - tcp['z']) - gap
    if verbose:
        print(f'  하강 {time.time() - began:.1f}s  grasp_center {tcp["z"]:+.4f} '
              f'(오차 {(tcp["z"] - target_z) * 1000:+.2f} mm)  밀림 {ride * 1000:+.2f} mm')

    seated = abs(peg['z'] - entry['z']) < SEAT_TOL and ride <= RIDE_LIMIT
    if verbose:
        print(f'  앉음 확인 peg z={peg["z"]:+.4f} (목표 {entry["z"]:.4f} '
              f'± {SEAT_TOL * 1000:.0f} mm), 밀림 {ride * 1000:+.2f} mm → '
              f'{"앉음" if seated else "**미착석**"}')
    if not seated:
        raise PegInHoleError(
            f'peg 이 {destination} 에 앉지 않았다 '
            f'(z={peg["z"]:+.4f}, 목표 {entry["z"]:.4f}, 밀림 {ride * 1000:+.2f} mm).\n'
            '  **쥔 채로 둔다** — 놓으면 시뮬레이터가 물체를 씬 밖으로 튕겨낸다 '
            '(복구는 재시작뿐).', holding=True)

    set_gripper(twin, 'open')
    time.sleep(2.5)
    _go(twin, aim_x, aim_y, APPROACH_Z, velocity_scaling=0.15)

    peg = _read_scene(twin)['pose']['position']
    error = {'x': peg['x'] - entry['x'], 'y': peg['y'] - entry['y'],
             'z': peg['z'] - entry['z']}
    if verbose:
        print(f'  삽입 오차 ({error["x"] * 1000:+.2f}, {error["y"] * 1000:+.2f}, '
              f'{error["z"] * 1000:+.2f}) mm')
    return {'destination': destination, 'seated': True, 'error_m': error}


def move_peg(twin: RobotTwinClient, destination: Optional[str] = None, *,
             go_home: bool = True, verbose: bool = True) -> dict:
    """peg 을 집어 `destination` 에 꽂는다. 생략하면 **반대쪽**이다."""
    obj, fixtures, source = where_is_peg(twin)
    if destination is None:
        destination = other_fixture(fixtures, source)
    # **이름을 로봇을 건드리기 전에 검증한다.** 안 그러면 오타 하나에 집기까지 하고
    # 나서 거부하게 되어, 헛동작을 하고 peg 을 쥔 채로 멈춘다 (2026-09-11 실측).
    if destination not in fixtures:
        raise PegInHoleError(f'{destination!r} 라는 고정물이 없다 — '
                             f'있는 것: {sorted(fixtures)}')
    if destination == source:
        raise PegInHoleError(f'peg 이 이미 {destination} 에 있다')
    if verbose:
        p = obj['pose']['position']
        print(f'peg ({p["x"]:+.4f}, {p["y"]:+.4f}, {p["z"]:+.4f}) '
              f'{source or "탁자"} → {destination}')

    if go_home:
        set_gripper(twin, 'open')
        move_to_named_target_verified(twin, 'ready')

    if verbose:
        print('\n=== 집기 ===')
    gap = pick_peg(twin, verbose=verbose)

    if verbose:
        print(f'\n=== 꽂기 → {destination} ===')
    result = place_peg(twin, destination, gap, verbose=verbose)
    result['source'] = source

    if go_home:
        move_to_named_target_verified(twin, 'ready')
    return result


# ---------------------------------------------------------------- CLI


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description='peg 을 다른 고정물로 옮겨 꽂는다')
    parser.add_argument('destination', nargs='?', default=None,
                        help='목적지 고정물 이름. 생략하면 반대쪽')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8803)
    parser.add_argument('--twin', default='panda_functionbay')
    parser.add_argument('--dry-run', action='store_true',
                        help='판단만 하고 움직이지 않는다')
    parser.add_argument('--no-home', action='store_true',
                        help='시작·끝의 ready 복귀를 생략한다')
    args = parser.parse_args(argv)

    twin = RobotTwinClient(host=args.host, port=args.port, twin_id=args.twin)
    try:
        obj, fixtures, source = where_is_peg(twin)
        destination = args.destination or other_fixture(fixtures, source)
        p = obj['pose']['position']
        print(f'peg ({p["x"]:+.4f}, {p["y"]:+.4f}, {p["z"]:+.4f}) '
              f'기울기 {tilt_deg(obj):.1f}°  {source or "탁자"} → {destination}')
        if args.dry_run:
            print('--dry-run — 움직이지 않는다')
            return 0
        result = move_peg(twin, destination, go_home=not args.no_home)
    except PegInHoleError as exc:
        print(f'\n실패: {exc}')
        if exc.holding:
            print('  ⚠️ peg 을 **쥔 채로** 멈췄다. 놓기 전에 원인을 해결한다.')
        return 1
    except OperationFailed as exc:
        print(f'\n연산 실패: {exc}')
        return 1

    e: Any = result['error_m']
    print(f"\n완료 — {result['source']} → {result['destination']}, "
          f"삽입 오차 ({e['x'] * 1000:+.2f}, {e['y'] * 1000:+.2f}, {e['z'] * 1000:+.2f}) mm")
    return 0


__all__ = ['PegInHoleError', 'axis_of', 'bottom_of', 'check_geometry', 'covers_xy',
           'down_with_yaw', 'entry_clearance', 'floor_at', 'wait_until_peg_still',
           'grasp_height', 'main', 'move_peg', 'other_fixture', 'pick_peg', 'place_peg',
           'tilt_deg', 'top_z', 'vertical_half_extent', 'where_is_peg']
