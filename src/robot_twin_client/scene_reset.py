"""씬을 레시피대로 다시 배치한다 — `robot-twin-reset`. **로봇은 움직이지 않는다.**

    robot-twin-reset                               # 지정 위치 — 처음 배치(initial)
    robot-twin-reset --scene two_objects           # 무작위 — seed 를 뽑아 알려 준다
    robot-twin-reset --scene two_objects --seed 7  # 무작위지만 같은 배치를 재현한다
    robot-twin-reset --list                        # 이 트윈이 아는 레시피 이름

**무작위인지 지정 위치인지는 레시피가 정한다** — 트윈 설정(`reset_scene.backend.scenes`)에서
축마다 숫자면 그 자리에 고정, `[최소, 최대]` 면 그 구간에서 뽑는다. Isaac 트윈은 무작위 둘
(`two_objects` · `one_block`)과 지정 위치 하나(`initial` — `isaac_scene.json` 의 처음 배치)를 둔다.
레시피를 늘리는 것은 그 YAML 편집이고, 이 명령은 이름만 고른다.

**seed 를 안 주면 새로 뽑는다.** 라이브러리 `reset_scene()` 의 기본값 0 을 그대로 쓰면 무작위
레시피가 **매번 같은 배치**가 된다. 뽑은 값을 출력하므로 `--seed` 로 다시 만들 수 있다. 지정
위치 레시피는 seed 와 무관하다.

리셋 뒤에는 `scene_objects` 가 새 배치를 말할 때까지 기다렸다가 **실제로 놓인 자리**를 함께
보인다 — 물리 시뮬레이터에서는 명령한 자리와 안착한 자리가 몇 mm 다를 수 있고, 이후 파지
좌표의 근거는 놓인 자리다. `--no-check` 면 씬을 전혀 읽지 않는다(대기도 확인도 없다).

`reset_scene` 은 `scene` 과 함께 `arm` 자원도 잡으므로 팔이 움직이는 중에는 거부된다(409).
레시피는 떠 있는 트윈의 설정이 정한다(`--list`). 기본 레시피 `initial` 은 Isaac 트윈에만 있다.

종료 코드: 성공 0, 연산 실패 · 연결 실패 1.
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import math
import random
import sys

from .client import DEFAULT_BASE_URL, OperationFailed, RobotTwinClient, TwinError
from .geometry import center_of, yaw_of
from .stack import reset_scene

DEFAULT_SCENE = 'initial'

# seed 를 안 줬을 때 뽑는 범위. 사람이 다시 칠 수 있게 짧게 둔다.
_SEED_RANGE = 1_000_000


def scene_names(twin: RobotTwinClient) -> Optional[list]:
    """트윈 카탈로그가 밝힌 레시피 이름. 트윈이 목록을 내지 않으면 ``None``.

    트윈은 `reset_scene` 의 `scene` 입력 enum 을 레시피 표에서 만든다 — 설정이 단일 출처라
    여기서 따로 적지 않는다.
    """
    for op in twin.operations():
        if op['name'] == 'reset_scene':
            prop = ((op.get('inputs_schema') or {}).get('properties') or {}).get('scene') or {}
            names = prop.get('enum')
            return sorted(names) if names else None
    return None


def _fmt_xyz(p: dict) -> str:
    return f"({p['x']:.4f}, {p['y']:+.4f}, {p['z']:.4f})"


def _yaw_deg(obj: dict) -> float:
    """배치 결과의 yaw(도). 트윈이 `yaw_deg` 를 주면 그것을, 아니면 자세에서 구한다."""
    if 'yaw_deg' in obj:
        return float(obj['yaw_deg'])
    orientation = obj.get('orientation') or (obj.get('pose') or {}).get('orientation')
    return math.degrees(yaw_of(orientation)) if orientation else 0.0


def print_applied(objects: list) -> None:
    """트윈이 명령한 배치(`outputs.objects`)를 보인다."""
    print('  배치한 자리')
    for obj in objects:
        print(f"    {obj['name']:<12} {obj.get('type', ''):<9} {_fmt_xyz(center_of(obj))}"
              f"  yaw {_yaw_deg(obj):6.1f}°")


def print_observed(twin: RobotTwinClient, applied: list) -> None:
    """`scene_objects` 가 말하는 실제 위치와, 명령한 자리와의 차이를 보인다."""
    observed = twin.read('scene_objects')['objects']
    print('  놓인 자리 (scene_objects)')
    for obj in applied:
        name = obj['name']
        if name not in observed:
            print(f'    {name:<12} — scene_objects 에 없다')
            continue
        got, want = center_of(observed[name]), center_of(obj)
        moved = math.dist([got[a] for a in 'xyz'], [want[a] for a in 'xyz'])
        print(f'    {name:<12} {"":<9} {_fmt_xyz(got)}  차이 {moved * 1000:5.1f} mm')


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog='robot-twin-reset',
        description='씬을 레시피대로 다시 배치한다 (무작위 또는 지정 위치). 로봇은 움직이지 않는다.')
    parser.add_argument('--url', default=DEFAULT_BASE_URL,
                        help='트윈 서버 루트 URL (http(s)://호스트[:포트])')
    parser.add_argument('--twin-id', default='panda01', help='Isaac 트윈의 id')
    parser.add_argument('--scene', default=DEFAULT_SCENE,
                        help=f'레시피 이름 (기본 {DEFAULT_SCENE!r} — 처음 배치). '
                             '목록은 --list 로 본다')
    parser.add_argument('--seed', type=int, default=None,
                        help='무작위 레시피의 seed. 안 주면 새로 뽑아 출력한다')
    parser.add_argument('--list', action='store_true', help='레시피 이름만 보이고 끝낸다')
    parser.add_argument('--no-check', action='store_true',
                        help='scene_objects 를 읽지 않는다 — 갱신 대기도, 실제 위치 확인도 '
                             '하지 않는다')
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    twin = RobotTwinClient(base_url=args.url, twin_id=args.twin_id, agent='robot-twin-reset')
    where = f"{args.url.rstrip('/')}/{args.twin_id}"

    try:
        if args.list:
            names = scene_names(twin)
            if names is None:
                print(f'트윈 {where} 이 레시피 목록을 내지 않는다 (reset_scene 연산이 없거나 '
                      f'scene enum 이 비어 있다)')
                return 1
            print(f'트윈 {where} 의 레시피: {", ".join(names)}')
            return 0

        seed = args.seed if args.seed is not None else random.randrange(_SEED_RANGE)
        print(f"트윈 {where} — scene '{args.scene}' (seed {seed}"
              f"{'' if args.seed is not None else ', 새로 뽑음'})")
        if args.no_check:
            # 씬을 **전혀** 읽지 않는다 — `reset_scene()` 은 갱신을 기다리느라 읽으므로 쓰지
            # 않는다. `scene_objects` 변수가 없는 트윈에서도 리셋만은 되어야 한다.
            applied = twin.outputs('reset_scene', {'scene': args.scene, 'seed': seed})['objects']
            print_applied(applied)
        else:
            applied = reset_scene(twin, args.scene, seed)    # 새 배치가 보일 때까지 기다린다
            print_applied(applied)
            print_observed(twin, applied)
    except OperationFailed as exc:
        # 모르는 레시피는 트윈이 선언된 이름을 실어 거부한다.
        print(f'리셋 실패: {exc}')
        names = None
        try:
            names = scene_names(twin)
        except TwinError:
            pass
        if names:
            print(f'  이 트윈의 레시피: {", ".join(names)}')
        return 1
    except TwinError as exc:
        print(f'트윈 {where} 호출 실패: {exc}')
        return 1
    except OSError as exc:
        print(f'트윈 {where} 에 연결하지 못했다: {exc}')
        return 1
    return 0


__all__ = ['DEFAULT_SCENE', 'main', 'print_applied', 'print_observed', 'scene_names']


if __name__ == '__main__':
    sys.exit(main())
