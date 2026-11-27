"""연산 호출 위에 얹는 검증 헬퍼 (사용자 가이드 5.6 / 5.7 절).

트윈은 `COMPLETED` 를 "연산이 끝났다" 는 뜻으로만 쓴다. **도달했는지 · 물체를
쥐었는지는 클라이언트가 판단해야 한다.** 그 판단을 이 모듈에 모아 둔다.
"""

from __future__ import annotations

from typing import Optional

from datetime import datetime
import math
import time

from .client import RobotTwinClient

# 실패 코드별로 **다음에 할 일** (트윈 사용자 가이드 3.4). 코드만 돌려주면 호출자 — 특히
# 에이전트 — 는 같은 호출을 그대로 다시 부른다. 계획 실패와 실행 실패는 대응이 정반대라
# 갈라 둔다: 계획 실패는 로봇이 제자리라 목표를 바꿔야 하고, 실행 실패는 로봇이 경로
# 중간에 있을 수 있어 위치부터 확인해야 한다.
FAILURE_HINTS = {
    'PLANNING_FAILED': ('계획 실패 — 로봇은 움직이지 않았다. 같은 목표로 다시 불러도 결과가 '
                        '같으니 목표나 시작 자세를 바꾼다'),
    'EXECUTION_ABORTED': ('실행 실패 — 로봇이 경로 중간에 있을 수 있다. ee_pose 로 현재 위치를 '
                          '확인한 뒤 판단한다'),
    'TIMEOUT': '시간 초과 — 로봇이 아직 움직이는 중일 수 있다. ee_pose 로 현재 위치를 확인한다',
    'PRECONDITION_FAILED': ('현재 상태에서 받을 수 없는 요청이다 — 아무것도 실행되지 않았다. '
                            '재시도가 아니라 상태(session_state 등)를 먼저 맞춘다'),
    'ESTOP_ENGAGED': '비상정지로 중단됐다 — 사람이 해제하기 전에는 아무것도 움직이지 않는다',
}


# MoveIt 원본 오류(트윈 응답의 `error.details.moveit_error_name`)별 **원인과 다음에 할 일**
# (트윈 설계서 6.8). `PLANNING_FAILED` 하나로는 충돌 · IK 실패 · 시작 자세 무효가 같아 보이는데,
# 대응은 다르다 — 시작 자세가 문제면 목표를 바꿔도 소용없고, 목표가 문제면 목표를 바꿔야 한다.
_START_INVALID = ('시작 자세가 충돌(자기충돌 포함)이나 무효로 판정됐다 — 목표를 바꿔도 결과가 같다. '
                  'joint_states 로 현재 자세를 확인하고 먼저 그 자세에서 벗어난다. 벗어날 수 없으면 '
                  '사람에게 알린다')
_GOAL_INVALID = '목표가 관절 한계나 제약을 벗어난다 — 목표 값을 확인한다'
_PLAN_STALE = '계획한 경로가 실행 전에 무효가 됐다(충돌 등) — 현재 상태를 다시 읽고 다시 계획한다'
_STATE_STALE = '로봇 상태 수신이 끊겼다 — 로봇 스택(joint_states 발행)부터 확인한다'
MOVEIT_HINTS = {
    'START_STATE_IN_COLLISION': _START_INVALID,
    'START_STATE_INVALID': _START_INVALID,
    'START_STATE_VIOLATES_PATH_CONSTRAINTS': _START_INVALID,
    'GOAL_IN_COLLISION': ('목표 자세가 충돌로 판정됐다 — 목표를 물체 · 탁자에서 떨어뜨리거나 '
                          '접근 자세를 바꾼다'),
    'GOAL_STATE_INVALID': _GOAL_INVALID,
    'GOAL_CONSTRAINTS_VIOLATED': _GOAL_INVALID,
    'GOAL_VIOLATES_PATH_CONSTRAINTS': _GOAL_INVALID,
    'INVALID_GOAL_CONSTRAINTS': _GOAL_INVALID,
    'NO_IK_SOLUTION': ('목표 pose 에 IK 해가 없다 — 작업 범위 밖이거나 그 자세로는 닿을 수 없다. '
                       '위치를 로봇 쪽으로 당기거나 자세를 바꾼다'),
    'PLANNING_FAILED': ('플래너가 경로를 찾지 못했다 — 중간 경유점을 두거나 목표를 바꾼다'),
    'TIMED_OUT': '계획 시간 안에 경로를 찾지 못했다 — 중간 경유점을 두거나 목표를 바꾼다',
    'INVALID_MOTION_PLAN': _PLAN_STALE,
    'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE': _PLAN_STALE,
    'CONTROL_FAILED': ('컨트롤러가 궤적을 따르지 못했다 — 팔이 경로 중간에 있을 수 있다. ee_pose 로 '
                       '위치부터 확인한다'),
    'ROBOT_STATE_STALE': _STATE_STALE,
    'SENSOR_INFO_STALE': _STATE_STALE,
    'COMMUNICATION_FAILURE': _STATE_STALE,
}


def moveit_error_of(state: Optional[dict]) -> Optional[str]:
    """실패 응답의 MoveIt 원본 오류 이름. 이름이 없으면 ``error_code=<값>``, 둘 다 없으면 ``None``.

    트윈은 MoveIt 이 코드를 준 실패에만 `error.details` 를 싣는다 (설계서 6.8).
    """
    details = (((state or {}).get('error') or {}).get('details'))
    if not isinstance(details, dict):
        return None
    name = details.get('moveit_error_name')
    if isinstance(name, str) and name:
        return name
    code = details.get('moveit_error_code')
    if isinstance(code, int) and not isinstance(code, bool):
        return f'error_code={code}'
    return None


def failure_why(state: Optional[dict]) -> Optional[str]:
    """실패로 끝난 세션 상태에서 다음에 할 일을 찾는다. 아는 것이 없으면 ``None``.

    오류 코드의 안내 뒤에 MoveIt 원인(`moveit_error_name`)별 안내를 붙인다 — 원인이 코드보다
    구체적이라 그쪽이 다음 행동을 더 정확히 가른다.
    """
    code = ((state or {}).get('error') or {}).get('code')
    parts = []
    hint = FAILURE_HINTS.get(code or '')
    if hint:
        parts.append(hint)
    moveit = moveit_error_of(state)
    if moveit:
        cause = MOVEIT_HINTS.get(moveit)
        parts.append(f'MoveIt 원인 {moveit}' + (f': {cause}' if cause else ''))
    return '. '.join(parts) if parts else None


def _settle_sample(body: dict):
    """유효한 위치와 수신/원본 시각을 읽는다. 신선도를 확인할 수 없으면 제외한다."""
    if not isinstance(body, dict) or body.get('quality') != 'OK':
        return None
    try:
        received = datetime.fromisoformat(body['received_at'].replace('Z', '+00:00'))
        if received.tzinfo is None:
            return None
        received_at = received.timestamp()
        pose = body['value']
        position = pose['pose']['position']
        xyz = tuple(position[a] for a in ('x', 'y', 'z'))
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) for v in xyz):
            return None
        stamp = body.get('stamp')
        if stamp is not None:
            sec, nanosec = stamp['sec'], stamp['nanosec']
            if type(sec) is not int or type(nanosec) is not int or not 0 <= nanosec < 10**9:
                return None
            stamp = sec * 10**9 + nanosec
        return pose, xyz, received_at, stamp
    except (KeyError, TypeError, ValueError, AttributeError, OverflowError):
        return None


def wait_until_still(twin: RobotTwinClient, *, timeout_sec: float = 10.0,
                     tol_m: float = 0.0002, quiet_sec: float = 0.6,
                     poll_sec: float = 0.3):
    """`ee_pose` 가 **멈출 때까지** 기다리고 마지막 값을 돌려준다. 못 멈추면 `None`.

    quality=OK 이고 received_at 이 증가하는 새 표본만 사용한다. stamp 가 있으면
    원본 시각도 증가해야 한다. 시각이 없는 값/동일 캐시/재발행된 같은 원본은 정착
    근거가 아니다. 무효 표본이나 시계 역행 뒤에는 정착 구간을 새로 시작한다.

    위치가 구간 시작점의 tol_m 안에 quiet_sec 동안 머물러야 한다. 수신 시각과
    로컬 경과 시간 모두로 구간 길이를 확인하며, 자세 도달은 검사하지 않는다.

    **`None` 을 마지막 값으로 뭉개지 않는다** — 아직 움직이는 중인 값으로 다음 목표를
    만드는 것이 바로 막으려는 실패라서, 호출자가 그 사실을 알아야 한다.
    """
    for name, value in (('timeout_sec', timeout_sec), ('tol_m', tol_m),
                        ('quiet_sec', quiet_sec), ('poll_sec', poll_sec)):
        positive = name in ('quiet_sec', 'poll_sec')
        if not math.isfinite(value) or value < 0 or (positive and value == 0):
            bound = 'positive' if positive else 'nonnegative'
            raise ValueError(f'{name} must be finite and {bound}')
    deadline = time.monotonic() + timeout_sec
    anchor = None
    last_received = None
    last_stamp = None
    still_since = sample_since = None
    while time.monotonic() < deadline:
        sample = _settle_sample(twin.read_raw('ee_pose'))
        now = time.monotonic()
        if now >= deadline:
            break
        if sample is None:
            anchor = None
        else:
            pose, xyz, received_at, stamp = sample
            # 같은 캐시는 시간을 쌓거나 정착 성공을 결정하는 표본으로 쓰지 않는다.
            duplicate = (received_at == last_received or
                         (stamp is not None and stamp == last_stamp))
            if not duplicate:
                rewound = (last_received is not None and received_at < last_received)
                rewound |= (stamp is not None and last_stamp is not None and stamp < last_stamp)
                stamp_changed_kind = (stamp is None) != (last_stamp is None)
                if (anchor is None or rewound or stamp_changed_kind or
                        max(abs(a - b) for a, b in zip(xyz, anchor)) > tol_m):
                    anchor, still_since, sample_since = xyz, now, received_at
                elif now - still_since >= quiet_sec and received_at - sample_since >= quiet_sec:
                    return pose
                last_received, last_stamp = received_at, stamp
        time.sleep(min(poll_sec, max(0.0, deadline - time.monotonic())))
    return None


# 트윈이 이동 뒤 정착을 기다려 주는 입력 (트윈 설계서 6.7 "정착 대기").
SETTLE_INPUT = 'settle_timeout_sec'

_SETTLE_TIMEOUT_WHY = ('제한 시간 안에 새 유효 ee_pose 표본으로 정착을 확인하지 못했다. '
                       '움직임 또는 관측 중단일 수 있으므로 현재 상태를 확인하고 다음 동작을 멈춘다')


def _twin_settles(twin, operation: str) -> bool:
    """트윈이 그 연산에서 정착 대기를 해 주는가. 모르면 ``False`` (클라이언트가 기다린다)."""
    accepts = getattr(twin, 'accepts_input', None)
    if callable(accepts):
        return bool(accepts(operation, SETTLE_INPUT))
    try:
        ops = twin.operations()
    except AttributeError:
        return False
    for op in ops:
        if op.get('name') == operation:
            return SETTLE_INPUT in ((op.get('inputs_schema') or {}).get('properties') or {})
    return False


def move_linear_verified(twin: RobotTwinClient, pose: dict, *, tol_m: float = 0.005,
                         velocity_scaling: float = 0.2, frame: Optional[str] = None,
                         settle_sec: float = 0.0) -> dict:
    """직선 이동 후 실제 도달했는지 확인한다.

    `reached=False` 는 연산 실패, 정착 확인 실패, 관측값 없음 또는 위치 오차 초과를
    뜻한다. `why` 와 `reason` 이 있으면 먼저 확인한다. 위치 오차가 큰 경우에는
    부분 경로, 처짐, 접촉 등의 원인이 있을 수 있다. 자세 오차는 검사하지 않는다.

    ⚠️ **개루프 백엔드에서는 `settle_sec` 없이 판정하면 안 된다.** 명령 스트리밍은
    반환이 도달을 뜻하지 않아서, 끝나자마자 읽으면 아직 움직이는 중의 값을 본다 —
    펑션베이 실측(2026-09-09)에서 종료 1.5 초 뒤 z 오차가 **64.8 mm** 였는데 정착
    후에는 **−0.6 mm** 였다. 기본 0 은 **기존 호출자의 동작을 그대로 두기 위한 값**이지
    권장값이 아니다.

    `settle_sec` 은 **고정 대기가 아니라 상한**이다 — 실제로는 `ee_pose` 가 멈출 때까지
    기다린다. **트윈이 `settle_timeout_sec` 입력을 받으면 트윈이 기다린다** — 결과의
    `settled` 와 정착 시점의 `final_pose` 를 쓰고, 클라이언트는 폴링하지 않는다(판정 기준은
    백엔드마다 트윈이 갖는다). 받지 않는 옛 트윈이면 여기서 `wait_until_still` 로 기다린다.
    고정 대기로는 얼마를 줘야 하는지 알 수 없다:
    2026-09-11 에 3 초를 줬더니 하강 뒤 오차가 **7.55 mm** 로 읽혔는데 더 기다린 뒤에는
    **0.04 mm** 였고, 그 잘못 읽은 값으로 다음 목표를 만들어 삽입이 실패했다.

    정착을 확인하지 못하면 reason=SETTLE_TIMEOUT, reached=False 를 돌려준다.
    종료 시점 final_pose 로 대체하지 않는다. 이 결과는 연산 취소나 정지 명령이 아니다.

    정착을 확인했으면 **판정에 쓴 관측값을 `settled_pose` 로 함께 돌려준다**
    (`ee_pose` 변수 값 그대로라 위치는 `['pose']['position']`). `result.outputs.final_pose`
    는 종료 시점 스냅샷이라 정착 뒤 위치가 아니다 — 이 필드가 없으면 호출자가 `ee_pose` 를
    다시 읽어야 하고, 그 사이 값이 바뀌면 판정과 다른 위치로 다음 목표를 만든다.
    `settle_sec=0` 이나 정착 실패에서는 이 키가 없다.

    `frame` 은 서버가 지원할 때 명령할 링크/기준점을 선택하는 입력이다. 이 헬퍼는
    좌표를 변환하지 않는다. 제공 mock·Isaac 설정에서는 Panda Hand의 TCP 목표를
    `panda_hand.to_arm_command()`로 변환하고, 펑션베이는 기본 `grasp_center` 목표를
    그대로 전달한다. 명령 기준점과 관측 기준점의 위치가 비교 가능한지 호출자가 확인한다.
    """
    if not math.isfinite(settle_sec) or settle_sec < 0:
        raise ValueError('settle_sec must be finite and nonnegative')
    inputs: dict = {'pose': pose, 'velocity_scaling': velocity_scaling}
    if frame is not None:
        inputs['frame'] = frame
    server_settles = settle_sec > 0 and _twin_settles(twin, 'move_linear')
    if server_settles:
        inputs[SETTLE_INPUT] = settle_sec
    result = twin.run('move_linear', inputs)
    if result['status'] != 'COMPLETED':
        return _not_reached(result)

    # 상태 변수가 비어 있으면 키 자체가 없다 (사용자 가이드 4.3).
    outputs = result.get('outputs', {})
    final = outputs.get('final_pose')
    if server_settles:
        # 트윈이 정착을 판정했으면 `final_pose` 가 그 시점의 값이다 — 다시 읽지 않는다.
        if outputs.get('settled') is not True:
            return {'reached': False, 'result': result, 'reason': 'SETTLE_TIMEOUT',
                    'why': _SETTLE_TIMEOUT_WHY}
    elif settle_sec > 0:
        # 정착을 기다린 뒤 **다시 읽는다** — outputs 의 값은 종료 시점 스냅샷이다.
        final = wait_until_still(twin, timeout_sec=settle_sec)
        if final is None:
            return {'reached': False, 'result': result, 'reason': 'SETTLE_TIMEOUT',
                    'why': _SETTLE_TIMEOUT_WHY}
    if final is None:
        return {'reached': False, 'result': result, 'why': 'no final_pose in outputs'}

    want, got = pose['position'], final['pose']['position']   # PoseStamped → .pose
    error = max(abs(want[a] - got[a]) for a in ('x', 'y', 'z'))
    check = {'reached': error <= tol_m, 'error_m': error, 'result': result}
    if settle_sec > 0:
        check['settled_pose'] = final
    return check


def move_to_named_target_verified(twin: RobotTwinClient, target: str, *,
                                  velocity_scaling: float = 0.3, settle_sec: float = 0.0,
                                  tol_rad: float = 0.01) -> dict:
    """SRDF named target 으로 이동한 뒤 도달했는지 확인한다 (사용자 가이드 4.2).

    판정 기준이 `move_linear` 과 다르다. 관절 공간 계획이라 **부분 실행이 없어**
    좌표를 비교할 필요가 없는 대신, 실행이 open loop 일 수 있다.
      closed_loop=True  (JTC)   COMPLETED 를 도달로 봐도 된다
      closed_loop=False (JGPC)  명령 스트리밍이라 정상 종료해도 도달 보장이 없다

    목표 관절값은 SRDF 안에 있어 클라이언트가 모르므로, open loop 일 때는
    `move_linear_verified` 처럼 좌표로 검증할 수단이 없다 — `reached=False` 와
    함께 `why` 를 돌려주고 판단을 호출자에게 넘긴다.

    **`settle_sec > 0` 이고 트윈이 `settle_timeout_sec` 를 받으면** 트윈이 정착을 기다린 뒤
    목표 관절값과의 차(`goal_error.joint_max_rad`)를 돌려주므로, 스택과 무관하게 그것이
    `tol_rad` 안인지로 판정하고 `error_rad` 를 싣는다. 정착하지 못하면
    reason=SETTLE_TIMEOUT 이다. 옛 트윈이면 위 규칙 그대로다.

    `target` 은 **`panda_arm` 그룹의 이름**이어야 한다 (`hand` 쪽 이름은 실패한다).
    없는 이름은 트윈이 접수 단계에서 `400 INVALID_INPUT` 으로 거부하므로 `TwinError` 가
    올라온다 — 메시지에 쓸 수 있는 이름이 함께 실린다. 트윈이 이름 목록을 못 얻은
    경우에만 접수되어 `FAILED` 로 돌아온다.
    """
    for name, value in (('settle_sec', settle_sec), ('tol_rad', tol_rad)):
        if not math.isfinite(value) or value < 0:
            raise ValueError(f'{name} must be finite and nonnegative')
    inputs: dict = {'target': target, 'velocity_scaling': velocity_scaling}
    server_settles = settle_sec > 0 and _twin_settles(twin, 'move_to_named_target')
    if server_settles:
        inputs[SETTLE_INPUT] = settle_sec
    result = twin.run('move_to_named_target', inputs)
    if result['status'] != 'COMPLETED':
        return _not_reached(result)

    if server_settles:
        outputs = result.get('outputs', {})
        if outputs.get('settled') is not True:
            return {'reached': False, 'result': result, 'reason': 'SETTLE_TIMEOUT',
                    'why': _SETTLE_TIMEOUT_WHY}
        error = (outputs.get('goal_error') or {}).get('joint_max_rad')
        if not isinstance(error, (int, float)) or isinstance(error, bool):
            # 트윈이 목표 관절값을 못 구했다(SRDF 캐시 없음 등) — 도달로 낙관하지 않는다.
            return {'reached': False, 'result': result,
                    'why': '정착은 확인했지만 트윈이 비교할 관절 오차(goal_error)를 주지 않았다 '
                           '— joint_states 로 현재 관절을 직접 확인한다'}
        return {'reached': error <= tol_rad, 'error_rad': float(error), 'result': result}

    # closed_loop 는 항상 온다 (6.3). 없다면 트윈 쪽이 규격을 벗어난 것이다.
    if not result.get('outputs', {}).get('closed_loop'):
        return {'reached': False, 'result': result,
                'why': 'open loop — COMPLETED 가 도달을 보장하지 않는다'}

    return {'reached': True, 'result': result}


def _not_reached(result: dict) -> dict:
    """실패로 끝난 이동의 반환값. 코드를 알면 `why` 에 다음에 할 일을 싣는다.

    MoveIt 원인이 있으면 `moveit_error` 로도 싣는다 — 문장(`why`)을 파싱하지 않고 분기하게.
    """
    out = {'reached': False, 'result': result}
    why = failure_why(result)
    if why is not None:
        out['why'] = why
    moveit = moveit_error_of(result)
    if moveit is not None:
        out['moveit_error'] = moveit
    return out


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


__all__ = ['FAILURE_HINTS', 'MOVEIT_HINTS', 'SETTLE_INPUT', 'failure_why', 'moveit_error_of', 'move_linear_verified',
           'move_to_named_target_verified', 'set_gripper', 'wait_until_still']
