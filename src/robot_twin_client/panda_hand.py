"""손끝(TCP) ↔ `panda_link8` 변환 — **Panda Hand 그리퍼(mock · Isaac 스택) 전용 규약**이다.

:mod:`robot_twin_client.geometry` 가 만드는 pose 는 손끝 기준인데, mock · Isaac 트윈의 `move_linear`
는 planning group 의 tip link 인 `panda_link8` 을 움직인다. 둘 사이는 Panda Hand 의 기하
(손끝까지 10.34 cm, z축 45°)로 정해지므로 기하 일반이 아니라 그리퍼에 묶인 값이다 — 그래서
`geometry` 와 나눠 두고, import 하는 쪽 이름에 그 사실이 보이게 한다.

⚠️ **프레임을 스스로 변환하는 트윈에는 쓰지 않는다.** 펑션베이 트윈은 `move_linear` 에 `frame`
입력이 있고 기본이 `grasp_center` 라 파지 중심 기준 목표를 그대로 받는다. `grasp_center` 는
두 패드 사이의 중심이며 실제 손끝과는 다르다. 거기에 `to_arm_command` 를
얹으면 보정이 두 번 걸려 그리퍼가 10.3 cm 아래를 겨냥한다(실제로 그랬다). MCP 서버는 트윈
카탈로그로 갈래를 고르고(`TwinTools._twin_resolves_frames`), `peg_in_hole` 은 이 모듈을 쓰지 않는다.
같은 이유로 mock · Isaac 백엔드 프로파일에는 `frames.ee` 가 **일부러 없다** — 적으면 트윈과 이
모듈이 45° 를 한 번씩 곱해 90° 가 된다.
"""

from __future__ import annotations

import math

from .geometry import quat_mul, quat_rotate


# panda_hand 원점에서 손끝까지의 거리(m). URDF 의 finger 링크 원점(0.0584)에 손가락
# 길이를 더한 값이며, Franka 공식 description 의 `panda_hand_tcp` 프레임과 같다.
TCP_OFFSET_M = 0.1034

# panda_link8 → panda_hand 는 **이동 0, z축 -45° 회전**이다 (URDF `panda_hand_joint`).
# 명령은 link8 기준으로 나가므로 반대 부호로 되돌려 줘야 손가락이 의도한 방향에 선다.
_HAND_YAW_RAD = math.pi / 4


def to_arm_command(tcp_pose: dict) -> dict:
    """**손끝(TCP) 기준** 목표를 팔에 명령할 **`panda_link8` 기준** pose 로 바꾼다.

    이 보정이 없으면 그리퍼가 바닥을 파고든다. 트윈의 `move_linear` 는 planning
    group `panda_arm` 의 tip link, 즉 `panda_link8` 을 움직이는데 손끝은 거기서
    약 10.3 cm 더 뻗어 있다. 물체 중심을 그대로 명령하면 손끝이 그만큼 아래로
    내려가므로, 5 cm 큐브(중심 z=0.025)의 경우 손끝이 z=-0.078 이 된다.

    두 가지를 되돌린다.

    1. **위치** — 목표 자세로 회전시킨 오프셋만큼 빼서 link8 을 뒤로 물린다.
       DOWN 자세면 결과적으로 z 가 10.3 cm 올라간다.
    2. **자세** — link8 기준 명령에 z축 +45° 를 곱한다. `panda_link8` 과
       `panda_hand` 가 45° 어긋나 있어, 보정하지 않으면 손가락이 큐브 모서리
       방향으로 물린다.

    두 프레임의 **위치는 같다**(이동 0). 그래서 `move_linear_verified` 가
    `outputs.final_pose`(= `panda_hand`)와 비교하는 것은 이 변환 뒤에도 유효하다.
    """
    q = tcp_pose['orientation']
    ox, oy, oz = quat_rotate(q, (0.0, 0.0, TCP_OFFSET_M))
    p = tcp_pose['position']
    half = _HAND_YAW_RAD / 2.0
    yaw = {'x': 0.0, 'y': 0.0, 'z': math.sin(half), 'w': math.cos(half)}
    return {'position': {'x': p['x'] - ox, 'y': p['y'] - oy, 'z': p['z'] - oz},
            'orientation': quat_mul(q, yaw)}


def tcp_from_arm_pose(arm_pose: dict) -> dict:
    """팔이 보고한 pose(`ee_pose` / `outputs.final_pose`)를 **손끝(TCP) 기준**으로 바꾼다.

    입력은 PoseStamped 전체가 아니라 그 안의 `pose` dict다. 팔이 보고하는 자세는
    이미 `panda_hand` 의 것이므로 유지하고, 위치만 손끝까지 10.3 cm 더 밀어 준다.
    `to_arm_command` 의 출력은 `panda_link8` 자세이므로 그 명령 dict를 그대로 넣는
    완전한 역함수는 아니다.

    쓰임새는 **쥔 물체를 어디에 물었는지 재는 것**이다. 손끝을 물체 중심에 맞춰
    닫아도 물체는 손가락 사이에서 미끄러진다 — 실측에서 실린더가 12 mm 내려갔다.
    그 상태로 상수 `TCP_OFFSET_M` 만 믿고 내려놓으면 그만큼 띄워 떨어뜨린다.
    """
    q = arm_pose['orientation']
    ox, oy, oz = quat_rotate(q, (0.0, 0.0, TCP_OFFSET_M))
    p = arm_pose['position']
    return {'position': {'x': p['x'] + ox, 'y': p['y'] + oy, 'z': p['z'] + oz},
            'orientation': dict(q)}


__all__ = ['TCP_OFFSET_M', 'tcp_from_arm_pose', 'to_arm_command']
