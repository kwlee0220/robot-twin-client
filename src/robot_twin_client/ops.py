"""연산 호출 위에 얹는 검증 헬퍼 (사용 설명서 5.6 / 5.7 절).

트윈은 `COMPLETED` 를 "연산이 끝났다" 는 뜻으로만 쓴다. **도달했는지 · 물체를
쥐었는지는 클라이언트가 판단해야 한다.** 그 판단을 이 모듈에 모아 둔다.
"""

from __future__ import annotations

from typing import Optional

import time

from .client import RobotTwinClient


def wait_until_still(twin: RobotTwinClient, *, timeout_sec: float = 10.0,
                     tol_m: float = 0.0002, quiet_sec: float = 0.6,
                     poll_sec: float = 0.3):
    """`ee_pose` 가 **멈출 때까지** 기다리고 마지막 값을 돌려준다. 못 멈추면 `None`.

    개루프 백엔드에서 "다 왔다"를 아는 유일한 방법이다 — 연산의 반환도, 고정 대기도
    도달을 뜻하지 않는다. `quiet_sec` 동안 `tol_m` 안에 머물면 멈춘 것으로 본다.

    **`None` 을 마지막 값으로 뭉개지 않는다** — 아직 움직이는 중인 값으로 다음 목표를
    만드는 것이 바로 막으려는 실패라서, 호출자가 그 사실을 알아야 한다.
    """
    deadline = time.monotonic() + timeout_sec
    previous = None
    still_since = None
    while time.monotonic() < deadline:
        pose = twin.read('ee_pose', require_ok=False)
        if pose is not None:
            p = pose['pose']['position']
            if previous is not None:
                moved = max(abs(p[a] - previous[a]) for a in ('x', 'y', 'z'))
                if moved <= tol_m:
                    still_since = still_since or time.monotonic()
                    if time.monotonic() - still_since >= quiet_sec:
                        return pose
                else:
                    still_since = None
            previous = dict(p)
        time.sleep(poll_sec)
    return None


def move_linear_verified(twin: RobotTwinClient, pose: dict, *, tol_m: float = 0.005,
                         velocity_scaling: float = 0.2, frame: Optional[str] = None,
                         settle_sec: float = 0.0) -> dict:
    """직선 이동 후 실제 도달했는지 확인한다.

    반환 dict 의 `reached` 가 False 면 경로 일부만 실행된 것이다 — 트윈은 이를
    실패로 보고하지 않으므로 클라이언트가 판단해야 한다.

    ⚠️ **개루프 백엔드에서는 `settle_sec` 없이 판정하면 안 된다.** 명령 스트리밍은
    반환이 도달을 뜻하지 않아서, 끝나자마자 읽으면 아직 움직이는 중의 값을 본다 —
    펑션베이 실측(2026-09-09)에서 종료 1.5 초 뒤 z 오차가 **64.8 mm** 였는데 정착
    후에는 **−0.6 mm** 였다. 기본 0 은 **기존 호출자의 동작을 그대로 두기 위한 값**이지
    권장값이 아니다.

    `settle_sec` 은 **고정 대기가 아니라 상한**이다 — 실제로는 `ee_pose` 가 멈출 때까지
    기다린다 (`wait_until_still`). 고정 대기로는 얼마를 줘야 하는지 알 수 없다:
    2026-09-11 에 3 초를 줬더니 하강 뒤 오차가 **7.55 mm** 로 읽혔는데 더 기다린 뒤에는
    **0.04 mm** 였고, 그 잘못 읽은 값으로 다음 목표를 만들어 삽입이 실패했다.

    `frame` 은 `pose` 가 어느 프레임 기준인지다. 생략하면 트윈의 기본(EE 프레임)이라
    `ee_pose` 로 읽은 좌표를 그대로 넣을 수 있고, 아래 비교도 같은 프레임끼리가 된다.
    """
    inputs: dict = {'pose': pose, 'velocity_scaling': velocity_scaling}
    if frame is not None:
        inputs['frame'] = frame
    result = twin.run('move_linear', inputs)
    if result['status'] != 'COMPLETED':
        return {'reached': False, 'result': result}

    # 상태 변수가 비어 있으면 키 자체가 없다 (사용 설명서 4.3).
    final = result.get('outputs', {}).get('final_pose')
    if settle_sec > 0:
        # 정착을 기다린 뒤 **다시 읽는다** — outputs 의 값은 종료 시점 스냅샷이다.
        final = wait_until_still(twin, timeout_sec=settle_sec) or final
    if final is None:
        return {'reached': False, 'result': result, 'why': 'no final_pose in outputs'}

    want, got = pose['position'], final['pose']['position']   # PoseStamped → .pose
    error = max(abs(want[a] - got[a]) for a in ('x', 'y', 'z'))
    return {'reached': error <= tol_m, 'error_m': error, 'result': result}


def move_to_named_target_verified(twin: RobotTwinClient, target: str, *,
                                  velocity_scaling: float = 0.3) -> dict:
    """SRDF named target 으로 이동한 뒤 도달했는지 확인한다 (사용 설명서 4.2).

    판정 기준이 `move_linear` 과 다르다. 관절 공간 계획이라 **부분 실행이 없어**
    좌표를 비교할 필요가 없는 대신, 실행이 open loop 일 수 있다.
      closed_loop=True  (JTC)   COMPLETED 를 도달로 봐도 된다
      closed_loop=False (JGPC)  명령 스트리밍이라 정상 종료해도 도달 보장이 없다

    목표 관절값은 SRDF 안에 있어 클라이언트가 모르므로, open loop 일 때는
    `move_linear_verified` 처럼 좌표로 검증할 수단이 없다 — `reached=False` 와
    함께 `why` 를 돌려주고 판단을 호출자에게 넘긴다.

    `target` 은 **`panda_arm` 그룹의 이름**이어야 한다 (`hand` 쪽 이름은 실패한다).
    오타는 `400` 이 아니라 계획 단계까지 가서 `FAILED` 로 돌아오므로, 미리
    `twin.read('named_targets')` 로 확인하는 편이 빠르다.
    """
    result = twin.run('move_to_named_target',
                      {'target': target, 'velocity_scaling': velocity_scaling})
    if result['status'] != 'COMPLETED':
        return {'reached': False, 'result': result}

    # closed_loop 는 항상 온다 (6.3). 없다면 트윈 쪽이 규격을 벗어난 것이다.
    if not result.get('outputs', {}).get('closed_loop'):
        return {'reached': False, 'result': result,
                'why': 'open loop — COMPLETED 가 도달을 보장하지 않는다'}

    return {'reached': True, 'result': result}


def set_gripper(twin: RobotTwinClient, target: str) -> dict:
    """그리퍼를 조작하고 액션 결과를 돌려준다.

    목표는 `open` / `close` / `grasp` 세 가지다. `close` 와 `grasp` 는 목표 폭이
    같고 **힘만 다르다** — 물건을 쥘 때는 `grasp` 를 쓴다.

    반환 dict 의 판정 규칙:
      at_goal=True                 **성공 판정이다.** 목표별 판정식은 로봇 쪽
                                   `GripperNode` 가 이미 적용했다 — open/close 는
                                   목표 자세 도달, grasp 는 물체에 막혀 멈춤이다.
      stalled=True                 힘을 내는데 움직이지 않는다 = 무언가를 물었다.
                                   **관측이지 판정이 아니다** (grasp 에서만 성공과 같다).
      reached_goal                 옛 이름. 아래 호환 처리로 채운다.

    **트윈이 `reached_goal`/`position` 을 보내던 시절의 이름이 남아 있다.** 지금
    트윈은 `at_goal`/`width` 를 보낸다 (`rdfp_msgs/GripperState`). 옛 이름만 읽는
    호출자는 `KeyError` 로 죽으므로 — 실제로 그렇게 죽었다 — 여기서 채워 준다.
    새 코드는 `at_goal` 을 본다.
    """
    outputs = twin.outputs('move_gripper_to_target', {'target': target})
    if 'reached_goal' not in outputs and 'at_goal' in outputs:
        outputs = {**outputs, 'reached_goal': bool(outputs['at_goal'])}
    return outputs


__all__ = ['move_linear_verified', 'move_to_named_target_verified', 'set_gripper',
           'wait_until_still']
