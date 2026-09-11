"""물체를 집어 **다른 물체 위에 쌓는다**.

    pick(source) ─ 물린 위치 측정 ─ place(target 위) ─ return_home

`pick_n_place` 와 나누는 이유는 놓을 자리를 정하는 방식이 다르기 때문이다. 그쪽은
집은 자리를 베이스 z축으로 돌린 곳에 놓으므로 **높이가 집을 때와 같다**. 쌓기는
받침 물체의 상면 위로 올려야 하므로 두 물체의 **치수**를 알아야 하고, 그래서
`reset_scene` 의 배치 결과가 아니라 `scene_objects` 변수를 읽는다 (아래 참조).

**쥔 뒤에 물린 위치를 실측한다 — 이것이 이 모듈의 핵심이다.** 손끝을 물체 중심에
맞춰 닫아도 물체는 손가락 사이에서 미끄러진다. Isaac 실측에서 실린더가 **12 mm**
내려간 채 물렸다. 상수 `TCP_OFFSET_M` 만 믿고 내려놓으면 그만큼 띄워 떨어뜨리게 되고,
원통은 굴러 떨어진다. 그래서 들어올린 뒤 `ee_pose` 와 `scene_objects` 를 함께 읽어
**손끝과 물체 중심의 실제 차이**를 구하고, 놓을 좌표를 그만큼 보정한다. x·y 의
미끄러짐도 같은 방식으로 함께 잡힌다.

**`scene_objects` 변수와 `reset_scene` 의 출력은 모양이 다르다.**

    reset_scene outputs.objects   리스트    [{'name': ..., 'position': {...}}, ...]
    scene_objects 변수            딕셔너리  {'objects': {이름: {'type', 'dimensions',
                                                              'pose': {'position', ...}}}}

쌓기에는 `type` 과 `dimensions` 가 필요해서 변수 쪽을 읽는다. `reset_scene` 출력에는
치수가 없다.

**물리가 있는 백엔드가 필요하다.** mock 은 물체가 서로 얹히지 않아 "쌓았다"를 확인할
수 없다 — 그래서 기본 접속 대상이 `pick_n_place`(mock, 8801/panda01) 와 달리
Isaac 트윈(8802/panda_isaac)이다.

**반복 실행** — `--repeat N` 을 주면 N 번 돈다. 매 회 시작 전에 `home` 으로 가고
씬을 새로 배치하며, 씨앗은 회마다 다르다(`--seed` 를 주면 그것에서 파생해 실행 전체가
재현되고, 생략하면 무작위다). 한 회가 실패해도 그리퍼를 열고 다음 회로 넘어간다.

    robot-twin-stack --repeat 10
    robot-twin-stack --repeat 10 --seed 100     # 재현 가능한 10회

기본은 **`block_a` 를 `cylinder_a` 위에** 얹는다 (`--source` / `--base` 로 바꾼다).
받침이 원기둥이라 접촉면이 좁고, 실린더 상면(중심 z + 높이/2)을 정확히 잡아야 한다 —
`half_height_of` 가 타입별 `dimensions` 순서를 처리한다 (cylinder 는 [높이, 반지름]).

**이 프로그램은 실제로 로봇을 움직인다.**
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import random
import time
import urllib.error

from .client import OperationFailed, RobotTwinClient, TwinError
# `center_of` 는 물체의 두 모양(reset_scene 의 flat / scene_objects 의 nested)을 모두 받는다.
# 여기 있던 nested 전용 복사본을 걷고 그것을 쓴다 — 두 모듈이 다른 모양을 전제하면
# 한쪽에서 읽은 물체를 다른 쪽에 넘길 때 KeyError 가 난다.
from .ops import set_gripper
from .pick_n_place import (
    DOWN, center_of, grasp_pose_of, pick, place, return_home, tcp_from_arm_pose
)


# 받침 상면과 얹는 물체 사이에 두는 틈(m). 0 으로 딱 맞추면 접촉이 먼저 나서
# 물리 엔진이 밀어낸다 — 조금 띄워 떨어뜨린다.
DEFAULT_GAP_M = 0.004

# 씬 레시피 이름. Isaac 트윈 설정의 `reset_scene.backend.scenes` 에 선언돼 있으며
# `block_a`(정육면체)와 `cylinder_a`(실린더)를 흩어 놓는다.
DEFAULT_SCENE = 'two_objects'

# 이송 중 손끝 높이를 두 지점 위로 이만큼 띄운다(m). 쌓기는 놓을 자리가 받침 물체
# 높이만큼 위에 있으므로, 집는 쪽 접근 높이를 그대로 쓰면 이송 경로가 낮아진다.
DEFAULT_CLEARANCE_M = 0.15


def half_height_of(obj: dict) -> float:
    """물체 중심에서 상면(또는 하면)까지의 거리(m).

    **`dimensions` 순서가 타입마다 다르다** — `rdfp_msgs/SceneObject` 가
    `shape_msgs/SolidPrimitive` 규약을 그대로 따르기 때문이다.

        box       [x, y, z]        반높이 = z / 2
        cylinder  [높이, 반지름]    반높이 = 높이 / 2   ← 직관과 반대다
        sphere    [반지름]          반높이 = 반지름

    뒤집어 써도 값이 나오고 크래시하지 않는다 — 물체가 엉뚱한 높이에 놓일 뿐이다.
    """
    kind = obj.get('type', 'box')
    dims = obj['dimensions']
    if kind == 'box':
        return float(dims[2]) / 2.0
    if kind in ('cylinder', 'cone'):
        return float(dims[0]) / 2.0
    if kind == 'sphere':
        return float(dims[0])
    raise ValueError(f"cannot compute half height for type {kind!r}")


def scene_object(twin: RobotTwinClient, name: str) -> dict:
    """`scene_objects` 변수에서 물체 하나를 읽는다.

    없는 이름은 여기서 걸러야 한다 — 그대로 두면 좌표가 아니라 `KeyError` 로
    팔이 이미 움직인 뒤에 터진다.
    """
    objects = twin.read('scene_objects')['objects']
    if name not in objects:
        raise KeyError(f"scene has no object {name!r} (have: {sorted(objects)})")
    return objects[name]


def grasp_pose_of_center(center: dict) -> dict:
    """물체 중심을 위에서 잡는 **손끝(TCP) 기준** 파지 pose (자세는 `DOWN` 고정).

    자세를 물체에 맞춰야 하면 `pick_n_place.grasp_pose_of` 를 쓴다 — 그쪽은 물체의
    yaw 를 읽어 손목을 맞춘다. 이 함수는 좌표만 있고 물체가 없을 때를 위한 것이다.
    """
    return {'position': dict(center), 'orientation': dict(DOWN)}


def held_offset(twin: RobotTwinClient, name: str) -> dict:
    """쥔 물체의 중심이 손끝에서 얼마나 어긋나 있는지(m).

    **들어올린 뒤에 부른다.** 바닥에 닿아 있는 동안 재면 아직 미끄러지기 전이라
    0 에 가깝게 나오고, 정작 옮기는 동안 생긴 어긋남을 놓친다.

    돌려주는 값은 `손끝 - 물체중심` 이므로, 물체 중심을 좌표 ``c`` 에 놓으려면
    손끝을 ``c + offset`` 으로 보내면 된다.
    """
    tcp = tcp_from_arm_pose(twin.read('ee_pose')['pose'])['position']
    center = center_of(scene_object(twin, name))
    return {axis: tcp[axis] - center[axis] for axis in ('x', 'y', 'z')}


def stack_pose_of(base: dict, top_half_height_m: float, offset: dict, *,
                  gap_m: float = DEFAULT_GAP_M, orientation: Optional[dict] = None) -> dict:
    """받침 물체 위에 얹을 **손끝(TCP) 기준** 목표 pose.

    받침 상면 위로 얹는 물체의 반높이만큼 띄운 곳이 그 물체의 중심이고, 손끝은
    거기서 `offset` 만큼 더 간다 — 그 보정이 미끄러짐을 흡수한다.

    `orientation` 은 **집을 때 쓴 자세를 그대로 넘긴다.** 생략하면 `DOWN` 이라 손목이
    이송 중에 0 으로 돌아가고, 쥔 물체도 함께 돌아 물림이 흔들린다.
    """
    center = center_of(base)
    top = center['z'] + half_height_of(base)
    return {'position': {'x': center['x'] + offset['x'],
                         'y': center['y'] + offset['y'],
                         'z': top + top_half_height_m + gap_m + offset['z']},
            'orientation': dict(orientation or DOWN)}


def _at_height(pose: dict, z: float) -> dict:
    """같은 x·y·자세로 높이만 바꾼 pose."""
    return {'position': {**pose['position'], 'z': z}, 'orientation': dict(pose['orientation'])}


def await_fresh_scene(twin: RobotTwinClient, *, samples: int = 2,
                      timeout_sec: float = 5.0) -> bool:
    """`scene_objects` 가 **새로 발행된 값**이 될 때까지 기다린다.

    `/scene/reset` 이 끝나도 `scene_objects` 는 주기 발행이라 **직전 배치를 한동안 더
    말한다.** 그대로 읽으면 리셋 전 자리를 집으러 가고, 그 자리에는 아무것도 없다.

    표본을 둘 기다리는 이유는 하나로는 부족하기 때문이다 — 리셋이 끝나기 **직전**에
    떠난 발행이 바로 도착할 수 있다. 두 번째부터는 리셋 이후에 만들어진 값이다.

    타임스탬프를 주지 않는 트윈(테스트 대역)에서는 기다릴 근거가 없으므로 곧바로
    True 를 돌려준다.
    """
    def stamp_of() -> Optional[str]:
        body = twin.read_raw('scene_objects')
        mark = body.get('received_at') or body.get('stamp')
        return None if mark is None else repr(mark)

    first = stamp_of()
    if first is None:
        return True

    seen, last, deadline = 0, first, time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        current = stamp_of()
        if current != last:
            last, seen = current, seen + 1
            if seen >= samples:
                return True
        time.sleep(0.1)
    print(f'경고: scene_objects 가 {timeout_sec}s 안에 갱신되지 않았다 — '
          f'리셋 전 좌표를 볼 수 있다')
    return False


def reset_scene(twin: RobotTwinClient, scene: str = DEFAULT_SCENE, seed: int = 0) -> list:
    """레시피대로 물체를 다시 배치하고, 씬 상태가 따라올 때까지 기다린다.

    **`stack` 안이 아니라 밖에 둔다.** 쌓기는 겹쳐 쓸 수 있어야 한다 — A 위에 B 를
    올린 뒤 그 위에 C 를 올리는 식이다. `stack` 이 스스로 리셋하면 **두 번째 호출이
    첫 번째 결과를 무너뜨린다.**

    `reset_scene` 은 `scene` 과 `arm` 을 함께 잡으므로 팔이 움직이는 중이면 409 다.
    """
    applied = twin.outputs('reset_scene', {'scene': scene, 'seed': seed})['objects']
    await_fresh_scene(twin)
    return applied


def reset_and_stack(twin: RobotTwinClient, source_name: str, base_name: str, *,
                    scene: str = DEFAULT_SCENE, seed: int = 0, **kwargs) -> bool:
    """씬을 새로 배치하고 쌓는다.

    리셋을 붙이는 이유는 **같은 실행을 반복할 수 있게** 하기 위해서다. 리셋 없이 두 번
    돌리면 이미 얹혀 있는 물체를 다시 집어 제자리에 놓는 꼴이 된다.
    """
    placed = reset_scene(twin, scene, seed)
    names = {o['name'] for o in placed}
    for name in (source_name, base_name):
        if name not in names:
            raise KeyError(f"scene {scene!r} does not place {name!r} (places: {sorted(names)})")
    return stack(twin, source_name, base_name, **kwargs)


def stack(twin: RobotTwinClient, source_name: str, base_name: str, *,
          clearance_m: float = DEFAULT_CLEARANCE_M, gap_m: float = DEFAULT_GAP_M,
          require_grasp: bool = True, home: Optional[str] = 'ready') -> bool:
    """`source_name` 을 집어 `base_name` 위에 쌓는다.

    **접근·이송 높이를 두 지점에 공통으로 잡는다.** 집는 쪽 위와 놓는 쪽 위의 높이가
    다르면 그 사이 직선이 낮은 쪽으로 기울어 이미 놓인 물체를 스칠 수 있다.
    """
    source = scene_object(twin, source_name)
    base = scene_object(twin, base_name)
    if source_name == base_name:
        raise ValueError('cannot stack an object on itself')

    # **물체가 돌아가 있으면 손목도 돌려 면으로 잡는다.** 자세를 고정한 채 잡으면
    # 모서리를 물게 되고, 받침이 좁으면 반력이 안 생겨 파지 판정이 서지 않는다
    # (`grasp_yaw_of` 주석의 실측). 씬 레시피가 `yaw` 를 뽑으면서 필요해졌다.
    grasp = grasp_pose_of(source)

    # 놓을 높이는 물린 위치를 재기 전에는 확정할 수 없다. 이송 높이만 먼저 잡되,
    # 받침 위에 얹힌 뒤의 높이까지 고려해 넉넉한 쪽으로 정한다.
    stacked_z = (center_of(base)['z'] + half_height_of(base)
                 + half_height_of(source) + gap_m)
    transit_z = max(grasp['position']['z'], stacked_z) + clearance_m
    print(f"{source_name} ({grasp['position']['x']:.3f}, {grasp['position']['y']:.3f}, "
          f"{grasp['position']['z']:.3f}) -> {base_name} 위,  이송 높이 {transit_z:.3f}")

    if not pick(twin, _at_height(grasp, transit_z), grasp, require_grasp=require_grasp):
        return False

    offset = held_offset(twin, source_name)
    print(f"물린 오프셋 dx={offset['x'] * 1000:+.1f} dy={offset['y'] * 1000:+.1f} "
          f"dz={offset['z'] * 1000:+.1f} mm")

    # 받침은 다시 읽는다 — 집는 동안 스쳐 밀렸을 수 있다.
    # 놓을 때도 **집을 때 쓴 자세를 그대로** 유지한다 — 이송 중에 손목이 0 으로 돌아가면
    # 쥔 물체가 함께 돌아 물림이 흔들린다.
    target = stack_pose_of(scene_object(twin, base_name), half_height_of(source),
                           offset, gap_m=gap_m, orientation=grasp['orientation'])
    if not place(twin, _at_height(target, transit_z), target):
        return False

    # 복귀 실패는 **물체가 이미 놓인 뒤**라 작업 성패를 뒤집지 않는다.
    if home:
        return_home(twin, home)
    return True


def iteration_seeds(seed: Optional[int], times: int) -> list:
    """반복마다 **다른** 씨앗을 만든다.

    같은 씨앗을 되쓰면 배치가 매번 같아 반복하는 뜻이 없다 (트윈은 씨앗을 생략하면
    0 으로 고정한다). `seed` 를 주면 거기서 파생해 **실행 전체가 재현**되고, 안 주면
    매번 무작위다.
    """
    if seed is None:
        return [random.randrange(1 << 30) for _ in range(times)]
    return [seed + index for index in range(times)]


def repeat_stack(twin: RobotTwinClient, source_name: str, base_name: str, *, times: int,
                 scene: str = DEFAULT_SCENE, seed: Optional[int] = None,
                 home: Optional[str] = 'ready', **kwargs) -> list:
    """쌓기를 `times` 번 반복한다. 각 회의 결과를 리스트로 돌려준다.

    **매 회 시작 전에 `home` 으로 먼저 간다.** 두 가지 이유다 — (1) `reset_scene` 이
    `arm` 자원을 함께 잡으므로 팔이 움직이는 중이면 409 이고, (2) 팔이 테이블 위에
    남아 있으면 물체가 그리퍼 안이나 아래에 배치되어 첫 접근부터 부딪힌다.

    **한 회가 실패해도 멈추지 않는다.** 반복은 성공률을 보려고 도는 것이므로 한 번의
    실패로 나머지를 버리면 알 수 있는 것이 줄어든다. 다만 **실패한 회는 그리퍼를 열고**
    넘어간다 — 물체를 쥔 채 다음 `reset_scene` 을 부르면 쥐고 있던 것이 순간이동한다.

    각 회의 결과는 `{'index', 'seed', 'ok', 'check'|'error'}` 다.
    """
    results = []
    for index, iteration_seed in enumerate(iteration_seeds(seed, times), start=1):
        print(f'--- {index}/{times} 회  seed={iteration_seed}')
        if home and not return_home(twin, home):
            results.append({'index': index, 'seed': iteration_seed, 'ok': False,
                            'error': f"'{home}' 복귀 실패 — 이 회를 건너뛴다"})
            print(f"    {results[-1]['error']}")
            continue
        try:
            ok = reset_and_stack(twin, source_name, base_name, scene=scene,
                                 seed=iteration_seed, home=home, **kwargs)
        except (KeyError, ValueError, TwinError, OperationFailed, TimeoutError) as exc:
            results.append({'index': index, 'seed': iteration_seed, 'ok': False,
                            'error': f'{type(exc).__name__}: {exc}'})
            print(f"    실패: {results[-1]['error']}")
            _recover(twin)
            continue
        check = verify_stacked(twin, source_name, base_name) if ok else None
        results.append({'index': index, 'seed': iteration_seed,
                        'ok': bool(ok and check and check['stacked']), 'check': check})
        _describe(results[-1])
        if not results[-1]['ok']:
            _recover(twin)
    return results


def _recover(twin: RobotTwinClient) -> None:
    """다음 회를 위해 손을 비운다. 여기서 또 실패해도 반복은 이어 간다."""
    try:
        set_gripper(twin, 'open')
    except Exception as exc:                                   # noqa: BLE001
        print(f'    그리퍼를 열지 못했다: {exc}')


def _describe(result: dict) -> None:
    check = result.get('check')
    if check is None:
        print('    실패 — 쌓기 시퀀스가 중단됐다')
        return
    print(f"    가로 어긋남 {check['lateral_m'] * 1000:.1f} mm, "
          f"높이차 {check['height_gap_m'] * 1000:.1f} mm "
          f"(맞물리면 {check['expected_gap_m'] * 1000:.1f} mm) "
          f"→ {'성공' if result['ok'] else '얹히지 않았다'}")


def verify_stacked(twin: RobotTwinClient, source_name: str, base_name: str, *,
                   lateral_tol_m: float = 0.02) -> dict:
    """정말 얹혔는지 확인한다 — `place` 의 성공은 '팔이 움직였다'까지만 뜻한다.

    높이차가 두 반높이의 합이면 맞물린 것이고, 굴러떨어졌으면 그보다 작다.
    """
    source = scene_object(twin, source_name)
    base = scene_object(twin, base_name)
    sc, bc = center_of(source), center_of(base)
    want = half_height_of(source) + half_height_of(base)
    lateral = ((sc['x'] - bc['x']) ** 2 + (sc['y'] - bc['y']) ** 2) ** 0.5
    gap = sc['z'] - bc['z']
    return {'stacked': lateral <= lateral_tol_m and abs(gap - want) <= 0.008,
            'lateral_m': lateral, 'height_gap_m': gap, 'expected_gap_m': want}


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Pick one scene object and stack it on top of another')
    # 기본값이 `pick_n_place` 와 다르다 — 쌓기는 물리가 있어야 뜻이 있다 (모듈 설명 참조).
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8802)
    parser.add_argument('--twin-id', default='panda_isaac')
    # **블록을 실린더 위에 얹는 것이 기본이다.** 받침이 원기둥이라 접촉면이 좁고
    # 얹는 쪽은 평평한 바닥이라, 반대 조합(실린더를 블록 위)보다 성립 조건이 까다롭다 —
    # 쌓기 정확도를 보는 데는 이쪽이 더 말이 된다. 뒤집으려면 두 인자를 바꿔 준다.
    parser.add_argument('--source', default='block_a',
                        help='object to pick up (name in scene_objects)')
    parser.add_argument('--base', default='cylinder_a',
                        help='object to stack it on')
    parser.add_argument('--scene', default=DEFAULT_SCENE,
                        help='scene recipe to place the objects with before stacking')
    # **기본이 None(무작위)이다.** 트윈은 씨앗을 생략하면 0 으로 고정하므로, 여기서도
    # 0 을 기본으로 두면 몇 번을 돌리든 같은 배치가 나와 반복이 무의미해진다.
    parser.add_argument('--seed', type=int, default=None,
                        help='randomisation seed; 주면 실행 전체가 재현된다 '
                             '(n 회째는 seed+n-1). 생략하면 매 회 무작위')
    parser.add_argument('--repeat', type=int, default=1, metavar='N',
                        help='쌓기를 N 번 반복한다. 매 회 시작 전에 home 으로 가고 '
                             '씬을 새로 배치한다')
    # 리셋은 **겹쳐 쌓을 때** 방해가 된다 — 이미 얹은 것을 흩어 버린다.
    parser.add_argument('--no-reset', action='store_true',
                        help='skip reset_scene and stack whatever is already placed')
    parser.add_argument('--gap', type=float, default=DEFAULT_GAP_M,
                        help='clearance left above the base surface when releasing (m)')
    parser.add_argument('--clearance', type=float, default=DEFAULT_CLEARANCE_M,
                        help='transit height above the higher of the two points (m)')
    parser.add_argument('--allow-empty-grasp', action='store_true',
                        help='파지 판정(stalled)을 건너뛴다 — 물리가 없는 mock 백엔드 전용')
    parser.add_argument('--home', default='ready',
                        help="SRDF named target to return to when done ('panda_arm' group)")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    twin = RobotTwinClient(host=args.host, port=args.port, twin_id=args.twin_id)

    try:
        health = twin.health()
    except urllib.error.URLError as exc:
        print(f'트윈에 연결하지 못했다 ({twin.base}): {exc.reason}')
        return 1

    if health['move_group'] != 'READY':
        print(f"move_group 미준비: {health.get('move_group_error')}")
        return 1

    if args.repeat < 1:
        print('--repeat 은 1 이상이어야 한다')
        return 2
    if args.repeat > 1 and args.no_reset:
        # 리셋 없이 반복하면 2회째는 이미 얹혀 있는 것을 집어 제자리에 놓는 꼴이다.
        print('--repeat 과 --no-reset 은 함께 쓸 수 없다 — 반복하려면 매 회 씬을 새로 배치해야 한다')
        return 2

    common = {'clearance_m': args.clearance, 'gap_m': args.gap,
              'require_grasp': not args.allow_empty_grasp, 'home': args.home}

    if args.repeat > 1:
        results = repeat_stack(twin, args.source, args.base, times=args.repeat,
                               scene=args.scene, seed=args.seed, **common)
        good = [r for r in results if r['ok']]
        print(f"\n{len(good)}/{len(results)} 회 성공"
              + ('' if len(good) == len(results)
                 else '  실패한 회의 seed: '
                      + ', '.join(str(r['seed']) for r in results if not r['ok'])))
        return 0 if len(good) == len(results) else 1

    try:
        if args.no_reset:
            ok = stack(twin, args.source, args.base, **common)
        else:
            ok = reset_and_stack(twin, args.source, args.base,
                                 scene=args.scene, seed=args.seed, **common)
    except (KeyError, ValueError) as exc:
        print(f'대상이 잘못되었다: {exc}')
        return 1
    except TwinError as exc:
        # 400(입력) / 409(자원 점유·E-stop) / 503(백엔드 미준비) — 세션이 만들어지지 않았다.
        print(f'거절됨: {exc}')
        return 1
    except OperationFailed as exc:
        # 접수는 됐는데 실행이 실패했다. 트윈이 준 사유가 그대로 실려 있다 —
        # 대개 백엔드(시뮬레이터·씬 노드)가 안 떠 있는 경우다.
        print(f'실행 실패: {exc}')
        return 1
    except TimeoutError as exc:
        print(f'시간 초과: {exc}  (현재 자원: {twin.resources()})')
        return 1

    if not ok:
        print('실패 — 중간 자세로 남아 있을 수 있다')
        return 1

    check = verify_stacked(twin, args.source, args.base)
    print(f"가로 어긋남 {check['lateral_m'] * 1000:.1f} mm, "
          f"높이차 {check['height_gap_m'] * 1000:.1f} mm "
          f"(맞물리면 {check['expected_gap_m'] * 1000:.1f} mm)")
    if not check['stacked']:
        print('놓기는 했지만 얹히지 않았다 — 굴러떨어졌거나 빗나갔다')
        return 1
    print(f'{args.source} 를 {args.base} 위에 쌓았다')
    return 0


__all__ = ['DEFAULT_CLEARANCE_M', 'DEFAULT_GAP_M', 'DEFAULT_SCENE', 'await_fresh_scene',
           'center_of', 'grasp_pose_of_center', 'half_height_of', 'held_offset',
           'iteration_seeds', 'main', 'repeat_stack', 'reset_and_stack', 'reset_scene',
           'scene_object', 'stack', 'stack_pose_of', 'verify_stacked']


if __name__ == '__main__':
    raise SystemExit(main())
