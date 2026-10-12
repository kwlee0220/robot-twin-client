"""테스트용 트윈 스텁.

`pick` 과 헬퍼들은 `RobotTwinClient.run` 과 `read('ee_pose')` 만 쓰므로 HTTP 없이
대체할 수 있다.
"""

from __future__ import annotations

from typing import Optional

import pytest

from robot_twin_client import ops
from robot_twin_client.client import OperationFailed


class _FakeClock:
    """`sleep` 이 시간을 흘려보내기만 하는 시계. 실제로는 기다리지 않는다."""

    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture(autouse=True)
def _instant_settle(monkeypatch: pytest.MonkeyPatch) -> None:
    """정착 대기(`wait_until_still`)를 즉시 끝나게 한다.

    `pick` · `place` 는 이동마다 `ee_pose` 가 멈출 때까지 기다린다. 실제 시계로 두면
    한 번에 1 초 가까이 걸려 흐름 테스트 하나가 수 초가 된다. 판정 논리는 그대로이고
    시간만 가짜로 흐른다.
    """
    monkeypatch.setattr(ops, 'time', _FakeClock())


POSE = {'position': {'x': 0.4, 'y': 0.0, 'z': 0.1},
        'orientation': {'x': 1.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}

# 물체를 문 그리퍼 — 목표 폭에 못 미친 채(reached_goal=False) 힘을 내며 멈춘다.
GRASP_HELD = {'reached_goal': False, 'stalled': True, 'position': 0.02}

# `reset_scene` 이 돌려주는 실제 배치의 모양.
SCENE_OBJECTS = [{'name': 'cube_0', 'type': 'box', 'dimensions': [0.05, 0.05, 0.05],
                  'position': {'x': 0.42, 'y': -0.08, 'z': 0.025}}]


class FakeTwin:
    """호출을 기록하고 정해진 결과를 돌려주는 트윈 대역."""

    def __init__(self, *, gripper: Optional[dict] = None, pose_offset_m: float = 0.0,
                 pose_status: str = 'COMPLETED', pose_outputs: Optional[dict] = None,
                 gripper_by_target: Optional[dict] = None,
                 named_status: str = 'COMPLETED', closed_loop: bool = True,
                 scene_objects: Optional[list] = None,
                 session_state: str = 'IDLE') -> None:
        self.calls: list[tuple[str, dict]] = []
        self._scene_objects = (SCENE_OBJECTS if scene_objects is None else scene_objects)
        self._session_state = session_state
        self._gripper = gripper or {'reached_goal': True, 'stalled': False, 'position': 0.04}
        # 기본 대역은 **파지에 성공한다**. `grasp` 응답을 열림과 같은 값으로 두면
        # 기본값이 '끝까지 닫힘 = 빈손' 을 흉내내게 되어, 파지를 다루지 않는
        # 흐름 테스트들이 엉뚱하게 파지 실패로 끝난다.
        self._gripper_by_target = gripper_by_target or {'grasp': GRASP_HELD}
        self._offset = pose_offset_m
        self._pose_status = pose_status
        self._pose_outputs = pose_outputs
        self._named_status = named_status
        self._closed_loop = closed_loop
        # 마지막 `move_linear` 이 멈춘 자리. **실제 트윈처럼 `ee_pose` 가 이것을 돌려준다** —
        # 정착 대기 뒤의 도달 판정이 이 값을 보므로, 따로 놀면 판정이 엉뚱해진다.
        self._ee_pose: Optional[dict] = None

    def outputs(self, operation: str, inputs: Optional[dict] = None, **kwargs) -> dict:
        """실물 `RobotTwinClient.outputs` 와 **같은 계약** — 실패면 `OperationFailed` 다.

        대역이 이것을 흉내내지 않으면, 호출자가 실패 응답을 어떻게 다루는지 테스트가
        전혀 못 본다 (그래서 `run(...)['outputs']` 로 터지는 것을 오래 놓쳤다).
        """
        state = self.run(operation, inputs, **kwargs)
        if state.get('status') != 'COMPLETED':
            raise OperationFailed(operation, state)
        return state.get('outputs') or {}

    def run(self, operation: str, inputs: Optional[dict] = None) -> dict:
        inputs = inputs or {}
        self.calls.append((operation, inputs))

        if operation == 'move_gripper_to_target':
            target = inputs['target']
            return {'status': 'COMPLETED',
                    'outputs': self._gripper_by_target.get(target, self._gripper)}

        if operation == 'move_linear':
            if self._pose_outputs is not None:
                outputs = self._pose_outputs
            else:
                # 요청 좌표에 오차만큼 어긋난 지점에 도달했다고 응답한다.
                want = inputs['pose']['position']
                reached = {axis: value + self._offset for axis, value in want.items()}
                outputs = {'final_pose': {'pose': {'position': reached}}}
            self._ee_pose = outputs.get('final_pose') or self._ee_pose
            return {'status': self._pose_status, 'outputs': outputs}

        if operation == 'move_to_named_target':
            # 관절 공간 계획이라 좌표 비교가 없다 — closed_loop 만이 도달 근거다 (6.3).
            return {'status': self._named_status,
                    'outputs': {'closed_loop': self._closed_loop}}

        if operation == 'reset_scene':
            return {'status': 'COMPLETED',
                    'outputs': {'scene': inputs.get('scene'), 'seed': inputs.get('seed'),
                                'objects': self._scene_objects,
                                'applied_count': len(self._scene_objects)}}

        if operation in ('start_session', 'stop_session', 'start_episode', 'stop_episode'):
            return {'status': 'COMPLETED', 'outputs': {}}

        raise AssertionError(f'unexpected operation: {operation}')

    def read(self, name: str, *, require_ok: bool = True):
        if name == 'session_state':
            return {'state': self._session_state, 'task_label': ''}
        if name == 'ee_pose':
            return self._ee_pose
        raise AssertionError(f'unexpected variable: {name}')

    # ----- 카탈로그 (MCP 서버가 도구를 자동 생성할 때 쓴다) -----

    def read_raw(self, name: str) -> dict:
        return {'name': name, 'quality': 'OK', 'value': self.read(name)}

    def read_many(self, *names: str) -> dict:
        return {n: {'quality': 'OK', 'value': self.read(n)} for n in names}

    def variables(self) -> list:
        return [{'name': 'session_state', 'description': '수집 상태', 'units': {}}]

    def operations(self) -> list:
        """`GET /operations` 응답 모양. 트윈 설정의 실제 구성을 축약한 것이다."""
        return [
            {'name': 'move_linear', 'description': '직선 이동', 'kind': 'async',
             'resource': ['arm'], 'idempotent': True,
             'inputs_schema': {'type': 'object', 'properties': {'pose': {'type': 'object'}}}},
            {'name': 'move_to_named_target', 'description': '이름 자세로 이동',
             'kind': 'async', 'resource': ['arm'], 'idempotent': True,
             'inputs_schema': {'type': 'object', 'properties': {'target': {'type': 'string'}}}},
            {'name': 'move_gripper_to_target', 'description': '그리퍼 구동', 'kind': 'sync',
             'resource': ['gripper'], 'idempotent': True,
             'inputs_schema': {'type': 'object', 'properties': {'target': {'type': 'string'}}}},
            {'name': 'reset_scene', 'description': '씬 리셋', 'kind': 'sync',
             'resource': ['scene', 'arm'], 'idempotent': False,
             'inputs_schema': {'type': 'object', 'properties': {'scene': {'type': 'string'}}}},
            {'name': 'start_session', 'description': '세션 시작', 'kind': 'sync',
             'resource': [], 'idempotent': False, 'inputs_schema': {}},
            {'name': 'stop_session', 'description': '세션 종료', 'kind': 'sync',
             'resource': [], 'idempotent': False, 'inputs_schema': {}},
            {'name': 'start_episode', 'description': '에피소드 시작', 'kind': 'sync',
             'resource': [], 'idempotent': False, 'inputs_schema': {}},
            {'name': 'stop_episode', 'description': '에피소드 종료', 'kind': 'sync',
             'resource': [], 'idempotent': False, 'inputs_schema': {}},
            # 미구현 — 노출되면 안 된다.
            {'name': 'move_to_pose', 'description': '미구현이다', 'kind': 'async',
             'resource': ['arm'], 'idempotent': True, 'inputs_schema': {}},
            {'name': 'move_gripper', 'description': '미구현이다', 'kind': 'async',
             'resource': ['gripper'], 'idempotent': True, 'inputs_schema': {}},
        ]

    def resources(self) -> dict:
        return {'arm': {'busy': False}, 'gripper': {'busy': False}, 'scene': {'busy': False}}
