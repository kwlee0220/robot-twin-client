"""쌓기를 **학습 데이터 에피소드로 수집한다** — `stack` 을 세션·에피소드 경계로 감싼 것이다.

    start_session ─┬─ reset_scene ─ return_home ─ start_episode
                   │      ─ stack ─ verify ─ stop_episode ─┬─ stop_session
                   └───────────────── N 회 ───────────────┘

**`stack` 과 나누는 이유는 경계 때문이다.** `robot-twin-stack --repeat` 은 성공률을
보려고 도는 것이라 `/session` 을 건드리지 않는다. 그 상태로 수집 계층(`rdfp_collect`)과
rosbag 을 띄워도 **에피소드가 안 잘려 학습 데이터가 한 건도 안 생긴다** — 오류가 아니라
빈 결과로 나타난다(2026-09-20 에 실제로 9회를 그렇게 돌렸다).

**ROS 의존이 없다.** 세션 경계 네 연산(`start_session` · `start_episode` ·
`stop_episode` · `stop_session`)을 트윈이 REST 로 내므로 `SessionControlClient` 가
필요 없다 — `peg_in_hole` 과 같은 성격이라 이 패키지에 둔다.

**이 스크립트만으로는 아무것도 저장되지 않는다.** 경계를 찍을 뿐이고, 실제로 담는 것은
따로 떠 있어야 하는 두 가지다.

    ros2 launch rdfp rdfp_collect.launch.py ...     세션 노드 + mp4 레코더
    RECORDING_TOPICS_FILE=... ./scripts/record_rosbag.sh

⚠️ **녹화 토픽 목록에서 카메라를 뺀다.** 안 빼면 `FrameRouter`(bag)와 레코더 등록이
같은 `(episode_id, topic_id)` 로 `image_streams` 에 넣어 `import` 가 UNIQUE 제약으로
죽는다. mp4 는 레코더가 남기므로 bag 에 또 담을 이유도 없다.

사용:

    robot-twin-collect --repeat 20
    robot-twin-collect --repeat 20 --seed 100      # 재현 가능한 20회
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import json
import urllib.error

from robot_twin_client.client import OperationFailed, RobotTwinClient, TwinError
from robot_twin_client.ops import set_gripper
from robot_twin_client.pick_n_place import return_home
from robot_twin_client.stack import (DEFAULT_CLEARANCE_M, DEFAULT_GAP_M, DEFAULT_SCENE,
                                     iteration_seeds, reset_scene, stack, verify_stacked)

DEFAULT_TASK_LABEL = 'stack_block_on_cylinder'


def collect_episode(twin: RobotTwinClient, source_name: str, base_name: str, *,
                    scene: str = DEFAULT_SCENE, seed: int = 0,
                    home: Optional[str] = 'ready', **kwargs) -> dict:
    """에피소드 하나를 수집한다. `{'ok', 'seed', 'placed', 'check'|'error'}` 를 돌려준다.

    **배치와 복귀는 에피소드 *밖*에서 한다.** `reset_scene` 은 물체를 순간이동시키고
    `return_home` 은 작업과 무관한 큰 움직임이라, 경계 안에 넣으면 그 구간이 그대로
    학습 레이블이 된다 — 정책이 "시작하면 팔을 크게 휘둘러라"를 배운다.

    **실패해도 `stop_episode` 를 반드시 부른다.** 트윈은 자동으로 닫아 주지 않으며,
    열린 채 두면 다음 `start_episode` 가 409 로 거부되어 **그 뒤 회차가 통째로 경계를
    잃는다.** 파지 실패도 유효한 학습 데이터이므로 `outcome=failure` 로 닫는 것이
    정상 경로다.
    """
    placed = reset_scene(twin, scene, seed)
    names = {o['name'] for o in placed}
    for name in (source_name, base_name):
        if name not in names:
            raise KeyError(f"scene {scene!r} does not place {name!r} (places: {sorted(names)})")

    if home and not return_home(twin, home):
        return {'ok': False, 'seed': seed, 'placed': placed,
                'error': f"'{home}' 복귀 실패 — 에피소드를 열지 않는다"}

    twin.run('start_episode')
    try:
        done = stack(twin, source_name, base_name, home=home, **kwargs)
    except (KeyError, ValueError, TwinError, OperationFailed, TimeoutError) as exc:
        done, failure = False, f'{type(exc).__name__}: {exc}'
        print(f'    쌓기 예외: {failure}')
    else:
        failure = None

    # **성공 판정은 `stack` 의 반환값이 아니라 `verify_stacked` 다** — 반환값은
    # *시퀀스 완주*이지 작업 성공이 아니다(2026-09-19 실측: "상승 실패 — 물체는 놓았다").
    check = None
    if done:
        try:
            check = verify_stacked(twin, source_name, base_name)
        except (TwinError, OperationFailed, KeyError) as exc:   # noqa: BLE001
            failure = failure or f'판정 실패: {type(exc).__name__}: {exc}'
    ok = bool(done and check and check['stacked'])

    # 재현에 필요한 것을 붙인다 — seed 만으로는 배치 추출 방식이 바뀌면 재현이 깨진다.
    metadata = {'seed': seed, 'scene': scene, 'source': source_name, 'base': base_name,
                'placed': placed, 'sequence_completed': bool(done)}
    if check is not None:
        metadata['check'] = check
    if failure:
        metadata['error'] = failure
    twin.run('stop_episode', {'outcome': 'success' if ok else 'failure',
                              'metadata': metadata})

    result = {'ok': ok, 'seed': seed, 'placed': placed, 'check': check}
    if failure:
        result['error'] = failure
    return result


def collect(twin: RobotTwinClient, source_name: str, base_name: str, *, times: int,
            task_label: str = DEFAULT_TASK_LABEL, scene: str = DEFAULT_SCENE,
            seed: Optional[int] = None, **kwargs) -> list:
    """세션을 열고 `times` 개의 에피소드를 모은 뒤 닫는다.

    **한 회가 실패해도 멈추지 않는다** — 실패한 에피소드도 학습 데이터이고, 성공률을
    알려면 끝까지 돌아야 한다. 다만 실패한 회는 그리퍼를 열고 넘어간다: 물체를 쥔 채
    다음 `reset_scene` 을 부르면 쥐고 있던 것이 순간이동한다.

    `finally` 로 세션을 닫는 것이 요점이다. 중간에 끊겨 세션이 열린 채 남으면 다음
    실행의 `start_session` 이 409 로 거부된다.
    """
    twin.run('start_session', {'task_label': task_label})
    results = []
    try:
        for index, episode_seed in enumerate(iteration_seeds(seed, times), start=1):
            print(f'--- 에피소드 {index}/{times}  seed={episode_seed}')
            try:
                result = collect_episode(twin, source_name, base_name, scene=scene,
                                         seed=episode_seed, **kwargs)
            except (KeyError, ValueError, TwinError, OperationFailed, TimeoutError) as exc:
                result = {'ok': False, 'seed': episode_seed,
                          'error': f'{type(exc).__name__}: {exc}'}
                print(f"    에피소드를 열지 못했다: {result['error']}")
            result['index'] = index
            results.append(result)
            _describe(result)
            if not result['ok']:
                _recover(twin)
    finally:
        twin.run('stop_session')
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
        print(f"    실패로 기록 — {result.get('error', '쌓기 시퀀스가 중단됐다')}")
        return
    print(f"    가로 어긋남 {check['lateral_m'] * 1000:.1f} mm, "
          f"높이차 {check['height_gap_m'] * 1000:.1f} mm "
          f"→ {'성공' if result['ok'] else '얹히지 않았다'}")


def _close_stale_session(twin: RobotTwinClient) -> None:
    """이전 실행이 남긴 세션·에피소드를 닫는다.

    **상태를 보고 필요한 것만 부른다** — `IDLE` 에서 `stop_session` 을 부르면 409 다.
    수집 중에 끊긴 뒤 다시 돌릴 때 이것이 없으면 `start_session` 이 거부된다.
    """
    state = twin.read('session_state').get('state')
    if state == 'IN_EPISODE':
        print(f'이전 에피소드가 열려 있다 ({state}) — 닫는다')
        twin.run('stop_episode', {'outcome': 'failure',
                                  'metadata': {'error': 'closed by a later collect run'}})
        state = 'IN_SESSION'
    if state == 'IN_SESSION':
        print('이전 세션이 열려 있다 — 닫는다')
        twin.run('stop_session')


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Collect stacking episodes as learning data (session/episode boundaries)')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8802)
    parser.add_argument('--twin-id', default='panda_isaac')
    parser.add_argument('--source', default='block_a', help='object to pick up')
    parser.add_argument('--base', default='cylinder_a', help='object to stack it on')
    parser.add_argument('--scene', default=DEFAULT_SCENE)
    parser.add_argument('--task-label', default=DEFAULT_TASK_LABEL,
                        help='세션의 모든 에피소드에 붙는 작업 이름')
    parser.add_argument('--seed', type=int, default=None,
                        help='주면 실행 전체가 재현된다 (n 번째는 seed+n-1). 생략하면 무작위')
    parser.add_argument('--repeat', type=int, default=1, metavar='N',
                        help='수집할 에피소드 수')
    parser.add_argument('--gap', type=float, default=DEFAULT_GAP_M)
    parser.add_argument('--clearance', type=float, default=DEFAULT_CLEARANCE_M)
    parser.add_argument('--home', default='ready')
    parser.add_argument('--summary-json', metavar='PATH',
                        help='회차별 결과를 JSON 으로 남긴다 (seed·배치·판정)')
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

    # **세션 변수가 없으면 수집 계층이 안 떠 있는 것이다.** 그대로 돌면 경계만 찍히고
    # 아무것도 저장되지 않으므로 여기서 멈춘다 — 다 돌린 뒤 빈 결과를 보는 것보다 낫다.
    if health.get('variables', {}).get('session_state') not in ('OK', None):
        print(f"session_state 를 읽을 수 없다 ({health['variables'].get('session_state')}) — "
              f'rdfp_collect 가 떠 있는지 본다')
        return 1

    try:
        _close_stale_session(twin)
        results = collect(twin, args.source, args.base, times=args.repeat,
                          task_label=args.task_label, scene=args.scene, seed=args.seed,
                          clearance_m=args.clearance, gap_m=args.gap, home=args.home)
    except TwinError as exc:
        print(f'거절됨: {exc}')
        return 1
    except KeyboardInterrupt:
        print('\n중단됨 — 세션은 닫았다')
        return 130

    good = [r for r in results if r['ok']]
    print(f'\n{len(good)}/{len(results)} 회 성공 — 에피소드는 {len(results)} 건 모두 기록됐다')
    if len(good) != len(results):
        print('실패한 회의 seed: '
              + ', '.join(str(r['seed']) for r in results if not r['ok']))
    if args.summary_json:
        with open(args.summary_json, 'w', encoding='utf-8') as handle:
            json.dump(results, handle, ensure_ascii=False, indent=2)
        print(f'요약: {args.summary_json}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
