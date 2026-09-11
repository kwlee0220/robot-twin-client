"""로봇 트윈을 MCP 도구로 노출하는 서버.

LLM 에이전트가 프로그램을 작성하지 않고 대화만으로 로봇 상태를 확인하고 연산을
조합할 수 있게 한다. 트윈의 HTTP API 만 쓰므로 ROS 가 필요 없다.

도구 목록은 **트윈 카탈로그에서 자동 생성**한다 — `GET /operations` 의
`inputs_schema` 가 이미 JSON Schema 이고 `description` 도 설정에 있으므로, 트윈에
연산을 추가하면 MCP 도구가 저절로 늘어난다. 손으로 유지하는 중복 목록을 만들지
않기 위해서다.

다만 그대로 노출하면 안 되는 것들이 있어 세 가지 예외를 둔다.

``_OVERRIDES``
    `COMPLETED` 는 "연산이 끝났다"는 뜻일 뿐 **도달을 보장하지 않는다.** raw 연산을
    노출하면 에이전트가 거짓 성공 위에 다음 동작을 쌓으므로, `ops.py` 의 검증
    래퍼로 바꿔 끼운다.

``_HIDDEN``
    미구현 연산(호출하면 실패한다)과 **에피소드를 여는 두 연산**이다. 여는 쪽을
    감추는 이유는 아래 `begin_task` 참고 — 닫는 쪽(`stop_*`)은 복구 수단으로 남긴다.

``_EXTRA``
    카탈로그에 없는 것들 — 상태 읽기, 기하 계산, 작업 경계 래퍼.

기하 계산을 도구로 여는 이유는 **쿼터니언 연산을 에이전트에게 시키면 조용히
틀리기 때문이다.** xyzw/wxyz 순서 하나만 어긋나도 예외 없이 엉뚱한 자세로 간다.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence

import argparse
import asyncio
import json
import urllib.error

import mcp.types as types
from mcp.server.lowlevel import Server
from mcp.server.stdio import stdio_server

from .client import RobotTwinClient, TwinError
from .ops import move_linear_verified, move_to_named_target_verified
from .peg_in_hole import (
    PegInHoleError, move_peg, pick_peg, place_peg, tilt_deg, where_is_peg
)
from .pick_n_place import (
    above_of, grasp_pose_of, rotated_about_base_z, to_arm_command
)

# 호출하면 EXECUTION_ABORTED 로 실패하는 미구현 연산과, 짝을 잃기 쉬운 '여는' 연산.
# `stop_session` / `stop_episode` 는 감추지 않는다 — 이전 실행이 남긴 세션을 에이전트가
# 스스로 정리할 수 있어야 한다.
_HIDDEN = frozenset({'move_to_pose', 'move_gripper', 'start_session', 'start_episode'})

_POSE_SCHEMA = {
    'type': 'object',
    'required': ['position', 'orientation'],
    'properties': {
        'position': {
            'type': 'object', 'required': ['x', 'y', 'z'],
            'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'},
                           'z': {'type': 'number'}}
        },
        'orientation': {
            'type': 'object', 'required': ['x', 'y', 'z', 'w'],
            'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'},
                           'z': {'type': 'number'}, 'w': {'type': 'number'}}
        },
    },
}


def _obj_schema(properties: dict[str, Any], required: Sequence[str] = ()) -> dict[str, Any]:
    """도구 입력 스키마를 만든다. MCP 는 최상위가 object 인 스키마를 요구한다."""
    schema: dict[str, Any] = {'type': 'object', 'properties': properties}
    if required:
        schema['required'] = list(required)
    return schema


class TwinTools:
    """트윈 하나를 MCP 도구 집합으로 감싼다.

    작업(task) 상태를 **서버가 들고 있는 것**이 핵심이다. 에이전트는 호출 사이에
    상태를 잃어버리므로, 에피소드를 열어 놓고 닫는 것을 잊으면 다음 작업이 막힌다.
    여는 경로를 `begin_task` 하나로 좁혀 두면 서버가 그 짝을 보장할 수 있다.
    """

    def __init__(self, twin: RobotTwinClient) -> None:
        self.twin = twin
        self._catalog: Optional[list[dict[str, Any]]] = None
        # 열린 작업의 재현 정보. `end_task` 가 metadata 에 자동으로 합친다 — 에이전트가
        # 배치를 되돌려 주기를 기대하면 잊는 순간 재현 불가능한 에피소드가 남는다.
        self._task: Optional[dict[str, Any]] = None

    # ------------------------------------------------------------------
    # 카탈로그 → 도구
    # ------------------------------------------------------------------

    def _operations(self) -> list[dict[str, Any]]:
        """연산 카탈로그. 트윈 기동 후 바뀌지 않으므로 한 번만 읽는다."""
        if self._catalog is None:
            self._catalog = self.twin.operations()
        return self._catalog

    def list_tools(self) -> list[types.Tool]:
        tools = [
            types.Tool(name=op['name'],
                       description=self._describe(op),
                       input_schema=op['inputs_schema'] or _obj_schema({}))
            for op in self._operations()
            if op['name'] not in _HIDDEN and op['name'] not in _OVERRIDES
        ]
        tools += [types.Tool(name=name, description=spec['description'],
                             input_schema=spec['schema'])
                  for name, spec in {**_OVERRIDES, **_EXTRA}.items()]
        return sorted(tools, key=lambda t: t.name)

    def _describe(self, op: dict[str, Any]) -> str:
        """카탈로그 설명에 호출 규약을 덧붙인다.

        자원을 잡는 연산은 다른 연산과 동시에 실행하면 409 로 거부된다 — 그 사실이
        설정 쪽 설명에는 없으므로(트윈 공통 규약이라) 여기서 붙인다.
        """
        text = op.get('description') or f"트윈 연산 '{op['name']}'."
        if op.get('resource'):
            text += (f"\n\n점유 자원: {', '.join(op['resource'])}. "
                     '같은 자원을 쓰는 연산이 실행 중이면 RESOURCE_BUSY 로 거부되므로, '
                     '먼저 끝나기를 기다렸다가 다시 부른다.')
        return text

    # ------------------------------------------------------------------
    # 도구 실행
    # ------------------------------------------------------------------

    def call(self, name: str, args: dict[str, Any]) -> Any:
        """도구 하나를 실행한다. **블로킹 호출이므로 스레드에서 돈다.**"""
        if name in _OVERRIDES:
            return _OVERRIDES[name]['run'](self, args)
        if name in _EXTRA:
            return _EXTRA[name]['run'](self, args)
        if name in _HIDDEN or not any(op['name'] == name for op in self._operations()):
            raise LookupError(f"unknown tool '{name}'")
        return self.twin.run(name, args)

    # ----- 상태 -----

    def _read_state(self, args: dict[str, Any]) -> Any:
        name = args['name']
        # require_ok=False 로 읽는다 — 값이 낡았다는 사실 자체가 에이전트에게 필요한
        # 정보다. 예외로 바꿔 버리면 '왜' 를 알 수 없다.
        return self.twin.read_raw(name)

    def _read_states(self, args: dict[str, Any]) -> Any:
        return self.twin.read_many(*args['names'])

    def _list_state_variables(self, _args: dict[str, Any]) -> Any:
        return self.twin.variables()

    # ----- 작업 경계 -----

    def _begin_task(self, args: dict[str, Any]) -> Any:
        """씬을 리셋하고 세션·에피소드를 연다.

        순서가 중요하다 — **씬 리셋은 에피소드 밖**이다. 안에서 하면 물체가
        순간이동하는 장면이 학습 데이터에 섞인다.
        """
        warnings: list[str] = []
        if self._task is not None:
            # 이전 작업을 닫지 않고 새로 시작했다. 데이터를 버리지 않고 실패로 닫는다.
            self._end_task({'outcome': 'failure',
                            'metadata': {'aborted_by': 'begin_task'}})
            warnings.append('직전 작업이 닫히지 않아 failure 로 닫았다')

        state = self.twin.read('session_state')['state']
        if state != 'IDLE':
            # 트윈 재시작 없이 서버만 재시작한 경우 등 — 서버가 모르는 세션이 열려 있다.
            self.twin.run('stop_session')
            warnings.append(f'열려 있던 세션을 닫았다 (이전 상태: {state})')

        placement: dict[str, Any] = {}
        if args.get('scene'):
            inputs = {'scene': args['scene']}
            if args.get('seed') is not None:
                inputs['seed'] = args['seed']
            placement = self.twin.outputs('reset_scene', inputs)

        self.twin.run('start_session', {'task_label': args['task_label']})
        try:
            self.twin.run('start_episode')
        except Exception:
            # 에피소드를 못 열었으면 세션도 남기지 않는다. 반쯤 열린 상태가 가장 나쁘다.
            self.twin.run('stop_session')
            raise

        self._task = {'task_label': args['task_label'],
                      'scene': args.get('scene'), 'seed': args.get('seed'),
                      'objects': placement.get('objects', [])}
        return {'objects': placement.get('objects', []),
                'scene': args.get('scene'), 'seed': args.get('seed'),
                'warnings': warnings}

    def _end_task(self, args: dict[str, Any]) -> Any:
        """에피소드와 세션을 닫는다. **어느 한쪽이 실패해도 나머지를 시도한다.**"""
        # 재현의 근거는 seed 가 아니라 **실제 배치**다 — 추출 방식이 바뀌면 같은 seed 가
        # 다른 배치를 만들기 때문이다. 에이전트가 넘긴 metadata 에 서버가 기억한 배치를
        # 합쳐 둔다.
        metadata = dict(args.get('metadata') or {})
        if self._task is not None:
            for key in ('scene', 'seed', 'objects'):
                metadata.setdefault(key, self._task[key])

        errors: list[str] = []
        try:
            self.twin.run('stop_episode', {'outcome': args['outcome'], 'metadata': metadata})
        except Exception as exc:
            errors.append(f'stop_episode: {exc}')
        try:
            self.twin.run('stop_session')
        except Exception as exc:
            errors.append(f'stop_session: {exc}')

        self._task = None
        return {'closed': not errors, 'outcome': args['outcome'],
                'metadata': metadata, 'errors': errors}

    def close_open_task(self) -> None:
        """서버 종료 시 열린 작업을 닫는다. 실패는 삼킨다 — 종료 경로다."""
        if self._task is None:
            return
        try:
            self._end_task({'outcome': 'failure',
                            'metadata': {'aborted_by': 'mcp_server_shutdown'}})
        except Exception:
            pass

    # ----- 검증 래퍼 -----

    def _twin_resolves_frames(self) -> bool:
        """트윈이 프레임 변환을 **스스로 하는가** — `move_linear` 의 `frame` 입력으로 본다.

        백엔드 이름으로 갈래를 두지 않는다. 능력은 **카탈로그가 말하게** 한다 — 트윈이
        늘면 여기 표를 고쳐야 하는 구조는 조용히 낡는다.
        """
        for op in self._operations():
            if op['name'] == 'move_linear':
                return 'frame' in ((op.get('inputs_schema') or {}).get('properties') or {})
        return False

    def _move_linear_verified(self, args: dict[str, Any]) -> Any:
        """목표를 **손끝 기준으로** 받아 명령한다. 프레임 보정은 상황에 따라 갈린다.

        ⚠️ **보정을 두 번 하면 10 cm 어긋난다.** 트윈이 `frame` 입력을 갖는 스택은
        자기가 EE 프레임(예: `grasp_center`)을 기준으로 해석하므로 **여기서 손대면 안
        된다.** 그런 스택에 `to_arm_command` 를 얹었다가 그리퍼가 10.3 cm 아래를 겨냥한
        적이 있다 — 기하 도구가 돌려준 좌표를 그대로 넣었는데도 그렇다.

        `frame` 입력이 없는 옛 스택(mock)은 데카르트 목표를 `panda_link8` 기준으로만
        해석하므로 **여기서 보정해야 한다** — 안 하면 반대로 바닥을 파고든다.
        """
        pose = args['pose'] if self._twin_resolves_frames() else to_arm_command(args['pose'])
        return move_linear_verified(self.twin, pose,
                                    tol_m=args.get('tolerance_m', 0.005))

    def _move_to_named_target_verified(self, args: dict[str, Any]) -> Any:
        return move_to_named_target_verified(
            self.twin, args['target'],
            velocity_scaling=args.get('velocity_scaling', 0.3))


# ----------------------------------------------------------------------
# peg-in-hole — 절차는 `peg_in_hole` 이 갖고 여기서는 **설명과 스키마만** 얹는다
# ----------------------------------------------------------------------
#
# **수치를 도구 설명에 적지 않는다.** 44 mm(손끝 오프셋) · 0.028(구멍 입구) ·
# −54 mm(정상 파지 간격) · 0.5 mm(반경 여유) 같은 값은 전부 코드가 갖는다. 설명에 적으면
# 에이전트가 그것으로 산수를 하게 되고, 틀려도 조용하다. 설명이 말할 것은
# **언제 무엇을 부르고, 실패를 어떻게 읽는가** 다.


def _peg_status(self: 'TwinTools', _args: dict[str, Any]) -> dict[str, Any]:
    """peg 이 지금 어느 자리에 있고 옮길 수 있는 자리는 어디인지."""
    obj, fixtures, source = where_is_peg(self.twin)
    return {'peg': obj['pose']['position'],
            'tilt_deg': round(tilt_deg(obj), 2),
            'at': source,
            'destinations': [n for n in sorted(fixtures) if n != source],
            'all_fixtures': sorted(fixtures)}


def _pick_peg(self: 'TwinTools', _args: dict[str, Any]) -> dict[str, Any]:
    gap = pick_peg(self.twin, verbose=False)
    return {'holding': True, 'grasp_gap_m': round(gap, 5),
            'note': '이제 peg 을 쥐고 있다. place_peg 로 꽂거나, 꽂을 수 없으면 '
                    '사람에게 알린다 — 놓으면 물체가 씬을 이탈한다'}


def _place_peg(self: 'TwinTools', args: dict[str, Any]) -> dict[str, Any]:
    gap = float(args['grasp_gap_m'])
    return place_peg(self.twin, args['destination'], gap, verbose=False)


def _move_peg(self: 'TwinTools', args: dict[str, Any]) -> dict[str, Any]:
    return move_peg(self.twin, args.get('destination'),
                    go_home=bool(args.get('return_home', True)), verbose=False)


# ----------------------------------------------------------------------
# 도구 표 — raw 연산을 대체하는 것과, 카탈로그에 없는 것
# ----------------------------------------------------------------------

_OVERRIDES: dict[str, dict[str, Any]] = {
    'move_linear': {
        'description': (
            '팔 끝(손끝)을 지정한 pose 로 직선 이동한 뒤 **실제로 도달했는지 확인한다**.\n\n'
            '좌표는 **손끝 기준**이다 — 그리퍼가 실제로 그 지점에 오며, 기하 도구가 '
            '돌려주는 값을 그대로 넣으면 된다. 로봇 내부 프레임 보정은 이 도구가 한다.\n\n'
            '트윈의 raw move_linear 는 경로가 잘려도 성공으로 끝나므로 이 도구를 쓴다. '
            "반환의 `reached` 가 false 면 목표에 못 갔다는 뜻이며 `error_m` 이 오차다 "
            '— false 인데 다음 동작을 진행하면 엉뚱한 위치에서 이어진다.\n\n'
            '물체를 집을 때는 scene_objects 나 reset_scene 이 돌려준 좌표를 쓰고, '
            'grasp_pose_of_object 로 파지 자세를 만든다. 위에서 접근할 때는 '
            'pose_above 로 만든 지점을 먼저 경유한다.'),
        'schema': _obj_schema(
            {'pose': _POSE_SCHEMA,
             'tolerance_m': {'type': 'number', 'exclusiveMinimum': 0,
                             'description': '도달 판정 허용 오차(m). 기본 0.005'}},
            ['pose']),
        'run': lambda self, a: self._move_linear_verified(a),
    },
    'move_to_named_target': {
        'description': (
            '팔을 사전 정의된 이름 자세로 옮긴 뒤 도달 여부를 판정한다.\n\n'
            "쓸 수 있는 이름은 read_state('named_targets') 로 확인한다 — 이름을 지어내면 "
            '계획 단계까지 가서 FAILED 로 돌아온다.\n\n'
            '반환의 `reached` 가 false 이고 `why` 가 open loop 이면, 명령 스트리밍 방식이라 '
            '정상 종료해도 도달 보장이 없다는 뜻이다(실패가 아니다). 확실히 알아야 하면 '
            "read_state('ee_pose') 로 확인한다."),
        'schema': _obj_schema(
            {'target': {'type': 'string', 'description': "SRDF named target (예: 'ready')"},
             'velocity_scaling': {'type': 'number', 'minimum': 0.01, 'maximum': 1.0}},
            ['target']),
        'run': lambda self, a: self._move_to_named_target_verified(a),
    },
}

_EXTRA: dict[str, dict[str, Any]] = {
    'peg_status': {
        'description': (
            'peg 이 **지금 어느 자리에 있고 어디로 옮길 수 있는지** 돌려준다.\n\n'
            '*"다른 hole 로 옮겨라"* 같은 지시를 받으면 **먼저 이것을 부른다** — '
            '`destinations` 에 옮길 수 있는 자리 이름이 들어 있고, 그중에서 고른다. '
            '이름을 지어내면 안 된다.\n\n'
            '`tilt_deg` 가 크면 peg 이 쓰러졌거나 기운 것이다 — 이 도구들은 **서 있는 '
            'peg 만** 다루므로 그때는 사람에게 알린다.'),
        'schema': _obj_schema({}),
        'run': lambda self, a: _peg_status(self, a),
    },
    'move_peg': {
        'description': (
            'peg 을 **집어서 다른 자리에 꽂는 전 과정**을 한 번에 한다. '
            '*"peg 을 다른 hole 로 옮겨서 꽂아줘"* 에 해당하는 도구다.\n\n'
            '`destination` 을 생략하면 **peg 이 지금 있는 자리가 아닌 쪽**으로 간다 — '
            '후보가 둘 이상이면 거부하고 이름을 요구한다. 쓸 수 있는 이름은 '
            'peg_status 의 `destinations` 에 있다.\n\n'
            '파지 오프셋 보정·기하 확인·앉음 확인이 **안에서 일어난다** — 따로 계산하거나 '
            '좌표를 만들 필요가 없다.\n\n'
            '⚠️ **실패하면 peg 을 쥔 채로 멈출 수 있다.** 그때는 오류 문장이 알려 주며, '
            '**같은 호출을 그대로 재시도하면 안 된다** — 이유를 읽고 판단한다.'),
        'schema': _obj_schema(
            {'destination': {'type': 'string',
                             'description': '꽂을 자리 이름. 생략하면 반대쪽'},
             'return_home': {'type': 'boolean',
                             'description': '끝나고 ready 자세로 돌아갈지. 기본 true'}}),
        'run': lambda self, a: _move_peg(self, a),
    },
    'pick_peg': {
        'description': (
            'peg 을 **집기만** 한다. *"peg 을 hole 에서 뽑아줘"* 에 해당한다.\n\n'
            '반환의 `grasp_gap_m` 은 **place_peg 에 그대로 넘겨야 하는 값**이다 — '
            '그 값 없이는 어디까지 내려야 하는지 알 수 없다. 잃어버렸으면 다시 잡는다.\n\n'
            '⚠️ 집은 뒤에는 **peg 을 쥔 상태로 남는다.** 꽂지 않고 그대로 두어도 되지만 '
            '**놓으면 안 된다** — 앉지 않은 채 놓으면 물체가 씬을 이탈하고, 이 로봇에는 '
            '되돌릴 수단이 없어 사람이 시뮬레이터를 재시작해야 한다.'),
        'schema': _obj_schema({}),
        'run': lambda self, a: _pick_peg(self, a),
    },
    'place_peg': {
        'description': (
            '**쥐고 있는** peg 을 지정한 자리에 꽂는다. pick_peg 뒤에 쓴다.\n\n'
            '`grasp_gap_m` 은 pick_peg 가 돌려준 값을 그대로 넣는다.\n\n'
            '꽂기 전에 **기하적으로 가능한지 먼저 확인하고, 안 되면 움직이기 전에 '
            '거부한다.** 내려간 뒤에는 **실제로 앉았는지 확인한 뒤에만 놓는다** — '
            '안 앉았으면 쥔 채로 실패를 돌려준다.\n\n'
            '⚠️ 실패 사유가 *"밀려 올라갔다"* 이면 **재시도해도 안 된다** — peg 이 '
            '그리퍼 안에서 어긋나 있는 것이므로 놓았다 다시 잡아야 하는데, 지금 놓으면 '
            '물체를 잃는다. 사람에게 알린다.'),
        'schema': _obj_schema(
            {'destination': {'type': 'string', 'description': '꽂을 자리 이름'},
             'grasp_gap_m': {'type': 'number',
                             'description': 'pick_peg 가 돌려준 grasp_gap_m 을 그대로'}},
            ['destination', 'grasp_gap_m']),
        'run': lambda self, a: _place_peg(self, a),
    },
    'read_state': {
        'description': (
            '로봇 상태 변수 하나를 읽는다. 쓸 수 있는 이름은 list_state_variables 로 본다.\n\n'
            '**반환의 `quality` 를 먼저 본다** — OK 가 아니면 값이 낡았거나 없는 것이라 '
            '그대로 쓰면 안 된다. `reason` 에 사유가 있다.\n\n'
            '자주 쓰는 것: scene_objects(물체 좌표 — 집기 전 반드시), ee_pose(현재 위치), '
            'gripper_last_command_result(파지 성공 판정), session_state(수집 상태).'),
        'schema': _obj_schema({'name': {'type': 'string'}}, ['name']),
        'run': lambda self, a: self._read_state(a),
    },
    'read_states': {
        'description': (
            '여러 상태 변수를 한 번에 읽는다. 각 값은 "같은 시점에 읽은 최신값"이며 '
            '서로 같은 시각에 생성되었다는 뜻은 아니다. 변수마다 quality 를 확인한다.'),
        'schema': _obj_schema(
            {'names': {'type': 'array', 'items': {'type': 'string'}, 'minItems': 1}},
            ['names']),
        'run': lambda self, a: self._read_states(a),
    },
    'list_state_variables': {
        'description': '읽을 수 있는 상태 변수의 이름·설명·단위 목록을 돌려준다.',
        'schema': _obj_schema({}),
        'run': lambda self, a: self._list_state_variables(a),
    },
    'get_resource_status': {
        'description': (
            '자원(arm/gripper/scene)별 점유 상태. 연산이 RESOURCE_BUSY 로 거부됐을 때 '
            '무엇이 잡고 있는지 확인한다. 확인과 실행 사이의 경합은 남으므로 이것으로 '
            '배타성을 보장할 수는 없다.'),
        'schema': _obj_schema({}),
        'run': lambda self, a: self.twin.resources(),
    },
    # ----- 기하 (로봇을 움직이지 않는 순수 계산) -----
    'pose_above': {
        'description': (
            '주어진 pose 바로 위의 접근 지점을 만든다. 자세는 그대로 두고 z 만 띄운다.\n\n'
            '물체를 집거나 놓을 때 옆에서 들이받지 않도록 **위를 먼저 경유하기 위한** '
            '것이다. 로봇은 움직이지 않는다 — 계산만 한다.'),
        'schema': _obj_schema(
            {'pose': _POSE_SCHEMA,
             'clearance_m': {'type': 'number', 'description': '띄울 높이(m). 기본 0.10'}},
            ['pose']),
        'run': lambda self, a: above_of(a['pose'], clearance_m=a.get('clearance_m', 0.10)),
    },
    'pose_rotated_about_base_z': {
        'description': (
            '로봇 베이스 z축 둘레로 pose 를 회전시킨다. 양수 각도는 위에서 볼 때 반시계다.\n\n'
            '"물체를 N도 돌린 자리로 옮겨줘" 같은 요청에서 **목표 좌표를 만드는 도구**다. '
            '위치는 베이스 원점 둘레의 원호를 따라 돌고 자세도 같은 각만큼 함께 도므로 '
            '쥔 물체와 그리퍼의 상대 자세가 유지된다. 높이는 변하지 않는다.\n\n'
            '**쿼터니언을 직접 계산하지 말고 이 도구를 쓴다** — 성분 순서를 한 번만 '
            '틀려도 조용히 엉뚱한 자세가 된다. 로봇은 움직이지 않는다.'),
        'schema': _obj_schema(
            {'pose': _POSE_SCHEMA,
             'angle_deg': {'type': 'number', 'description': '반시계 각도(도)'}},
            ['pose', 'angle_deg']),
        'run': lambda self, a: rotated_about_base_z(a['pose'], a['angle_deg']),
    },
    'grasp_pose_of_object': {
        'description': (
            '씬 물체 하나를 받아 위에서 잡는 파지 자세를 만든다.\n\n'
            'scene_objects 로 읽었거나 reset_scene/begin_task 가 돌려준 물체 객체를 '
            '그대로 넣는다 — 두 출처의 모양이 다르지만(position 이 최상위냐 pose 안이냐) '
            '양쪽 다 받는다. 물체 중심 위치에 "아래를 보는" 그리퍼 자세를 붙여 준다. '
            '로봇은 움직이지 않는다.'),
        'schema': _obj_schema(
            {'object': {'type': 'object', 'description':
                        'scene_objects 의 objects[<이름>] 값이나 '
                        'reset_scene/begin_task 의 objects 원소'}},
            ['object']),
        'run': lambda self, a: grasp_pose_of(a['object']),
    },
    # ----- 작업 경계 -----
    'begin_task': {
        'description': (
            '데이터 수집 작업을 시작한다 — 씬 리셋 → 세션 열기 → 에피소드 열기를 한 번에.\n\n'
            '**로봇을 움직이기 전에 이것을 부르고, 끝나면 반드시 end_task 를 부른다.** '
            '경계를 직접 열 수는 없다(start_session/start_episode 는 노출하지 않는다) — '
            '짝이 맞지 않으면 다음 작업이 막히기 때문이다.\n\n'
            'scene 을 주면 물체를 그 레시피대로 무작위 배치하고 **실제 배치를 objects 로 '
            '돌려준다.** 집을 좌표는 반드시 이 값에서 가져온다 — 고정 좌표를 쓰면 그 자리에 '
            '아무것도 없다. 배치는 서버가 기억했다가 end_task 의 metadata 에 자동으로 넣는다.\n\n'
            '씬 리셋은 에피소드 밖에서 일어난다(물체가 순간이동하는 장면을 데이터에 넣지 '
            '않기 위해서다). 열린 작업이 남아 있으면 failure 로 닫고 새로 시작하며, '
            '그 사실을 warnings 로 알려준다.'),
        'schema': _obj_schema(
            {'task_label': {'type': 'string',
                            'description': '이 세션의 모든 에피소드에 기록된다. 나중에 고칠 수 없다'},
             'scene': {'type': 'string',
                       'description': "씬 레시피 이름. 생략하면 씬을 건드리지 않는다"},
             'seed': {'type': 'integer', 'description': '같은 seed 는 같은 배치를 만든다'}},
            ['task_label']),
        'run': lambda self, a: self._begin_task(a),
    },
    'end_task': {
        'description': (
            '작업을 끝낸다 — 에피소드와 세션을 닫고 성패를 기록한다.\n\n'
            '**실패했을 때도 반드시 부른다.** 파지 실패도 유효한 학습 데이터이므로 '
            "outcome='failure' 로 닫는 것이 정상 경로다. 닫지 않고 두면 다음 begin_task 가 "
            '그 에피소드를 실패로 잘라내고, 그 구간의 데이터는 경계를 잃는다.\n\n'
            'begin_task 가 배치한 물체·scene·seed 는 서버가 기억했다가 자동으로 합치므로 '
            'metadata 에 다시 넣을 필요가 없다.'),
        'schema': _obj_schema(
            {'outcome': {'type': 'string', 'enum': ['success', 'failure']},
             'metadata': {'type': 'object',
                          'description': '재현·분석에 필요한 부가 정보'}},
            ['outcome']),
        'run': lambda self, a: self._end_task(a),
    },
    # ----- 안전 -----
    'emergency_stop': {
        'description': (
            '비상 정지. 진행 중인 연산을 중단하고 새 연산을 모두 거부한다. '
            '무언가 잘못되고 있다고 판단되면 즉시 부른다. '
            '해제는 release_emergency_stop 이며, 사람이 상황을 확인한 뒤에 한다.'),
        'schema': _obj_schema({}),
        'run': lambda self, a: self.twin.estop(),
    },
    'release_emergency_stop': {
        'description': (
            '비상 정지를 해제한다. **현장이 안전한지 사람이 확인했을 때만 부른다** — '
            '스스로 판단해 해제하지 않는다.'),
        'schema': _obj_schema({}),
        'run': lambda self, a: self.twin.release_estop(),
    },
}


# ----------------------------------------------------------------------
# MCP 배선
# ----------------------------------------------------------------------

def _error_text(exc: Exception) -> str:
    """에이전트가 읽고 **복구 방법을 판단할 수 있는** 오류 문장을 만든다.

    오류 메시지가 에이전트의 유일한 피드백 채널이다 — 코드만 던지면 같은 호출을
    그대로 재시도한다.
    """
    if isinstance(exc, TwinError):
        hint = {
            'RESOURCE_BUSY': ' 같은 자원을 쓰는 연산이 실행 중이다. '
                             'get_resource_status 로 확인하고 끝난 뒤 다시 부른다.',
            'PRECONDITION_FAILED': ' 현재 상태에서 허용되지 않는 요청이다. '
                                   "read_state('session_state') 로 상태를 확인한다.",
            'INVALID_INPUT': ' 입력이 스키마에 맞지 않는다. 메시지에 허용되는 값이 있으면 '
                             '그중에서 고른다.',
            'NOT_FOUND': ' 이름이 존재하지 않는다. list_state_variables 로 확인한다.',
        }.get(exc.code or '', '')
        return f'{exc}{hint}'
    if isinstance(exc, PegInHoleError):
        # **쥔 채로 멈췄는지가 에이전트의 다음 판단을 가른다.** 놓아도 되는 상황과
        # 놓으면 물체를 잃는 상황이 겉보기로 같아서, 말해 주지 않으면 놓아 버린다.
        if exc.holding:
            return (f'{exc}\n\n**peg 을 쥔 채로 멈췄다.** 같은 호출을 그대로 재시도하지 '
                    '않는다 — 원인이 그대로면 결과도 같다. 그리퍼를 열면 물체가 씬을 '
                    '이탈해 되돌릴 수 없으므로, 판단이 서지 않으면 **사람에게 알린다.**')
        return f'{exc}\n\n아직 움직이기 전에 멈췄으므로 로봇 상태는 안전하다.'
    if isinstance(exc, TimeoutError):
        return (f'{exc} 로봇이 아직 움직이는 중일 수 있다. '
                "read_state('ee_pose') 로 현재 위치를 확인하고 판단한다.")
    if isinstance(exc, urllib.error.URLError):
        return f'트윈에 연결하지 못했다: {exc.reason}. 트윈이 떠 있는지 확인이 필요하다.'
    return f'{type(exc).__name__}: {exc}'


# ----------------------------------------------------------------------
# 프롬프트 — **판단**을 담는다. 수치는 담지 않는다.
# ----------------------------------------------------------------------
#
# 경계가 이 파일의 요점이다:
#   **코드** — 수치(손끝 오프셋 · 입구 높이 · 정상 파지 간격 · 반경 여유)와 절차.
#             에이전트가 틀릴 여지를 없앤다.
#   **프롬프트** — 언제 무엇을 읽고, 실패를 어떻게 읽고, 무엇을 하면 안 되는가.
#
# 수치를 프롬프트에 적으면 **에이전트가 산수를 하게 되고 조용히 틀린다.** 반대로 절차
# 판단을 코드에만 두면 상황에 맞게 고르지 못한다.

_PEG_IN_HOLE_PROMPT = """\
peg 을 구멍에 꽂고 빼는 작업이다. 아래 규칙을 지킨다.

## 시작하기 전에

**`peg_status` 를 먼저 부른다.** peg 이 어느 자리에 있는지, 옮길 수 있는 자리가
무엇인지 거기서 나온다. *"다른 hole"* 같은 말은 `destinations` 로 해석하고,
**이름을 지어내지 않는다.**

`tilt_deg` 가 크면 peg 이 기울었거나 쓰러진 것이다. 이 도구들은 **서 있는 peg 만**
다룬다 — 그때는 손대지 말고 사람에게 알린다.

## 무엇을 부를 것인가

- 옮겨서 꽂기까지 한 번에 → **`move_peg`**
- 뽑기만 → **`pick_peg`** (돌려주는 `grasp_gap_m` 을 반드시 간직한다)
- 쥐고 있는 것을 꽂기 → **`place_peg`** (그 `grasp_gap_m` 을 그대로 넘긴다)

좌표를 직접 만들지 않는다. 파지 위치·접근 높이·정렬 보정은 **도구 안에서** 일어난다.
`move_linear` 로 같은 일을 흉내내려 하면 보정이 빠져 들어가지 않는다.

## 반드시 지킬 것

**① 물체는 *고정물 안*에서만 놓을 수 있다. 맨 탁자 위에는 내려놓지 못한다.**
`peg_status` 의 `all_fixtures` 에 없는 자리에 두라는 지시는 **수행할 수 없다** —
시도하지 말고, 되는 것과 안 되는 것을 말한다.

이것은 절차의 한계가 아니라 **시뮬레이터에 탁자 접촉이 정의돼 있지 않아서**다 —
고정물이 잡아 주지 않으면 물체가 탁자를 통과해 떨어진다(3회 확인). 똑바로 서서
탁자에 닿아 있어도 그렇다. 사람이 시뮬레이터를 재시작해야만 복구된다.
`place_peg` 가 앉음을 확인한 뒤에야 놓는다 —
**`move_gripper_to_target open` 을 직접 부르지 않는다.**

**② 실패하면 재시도가 아니라 원인을 읽는다.**
오류 문장이 *"쥔 채로 멈췄다"* 고 하면 로봇은 peg 을 들고 서 있는 것이다. 같은 호출을
반복해도 결과는 같다. 그리퍼를 여는 것은 ①에 걸린다. **판단이 서지 않으면 사람에게
알린다** — 그것이 여기서는 올바른 종료다.

**③ 완료가 도달을 뜻하지 않는다.**
이 로봇은 명령을 개루프로 흘린다. 도구가 끝났다는 것과 팔이 그 자리에 갔다는 것은
다르다. 도구들은 안에서 확인하지만, 직접 이동을 시켰다면 `read_state('ee_pose')` 로
확인한다.

**④ 물체를 쥘 때는 `grasp` 다.** `close` 는 힘을 주지 않아 놓친다.

## 끝난 뒤

`peg_status` 로 결과를 확인하고, 무엇을 어디로 옮겼는지 사람에게 말한다.
삽입 오차를 물으면 도구가 돌려준 `error_m` 을 쓴다 — 직접 계산하지 않는다.
"""

_PROMPTS: dict[str, dict[str, Any]] = {
    'peg_in_hole': {
        'description': ('peg 을 구멍에 꽂고 빼는 작업의 규칙. '
                        '*"peg 을 다른 hole 로 옮겨서 꽂아줘"* 처럼 지시하기 전에 읽는다.'),
        'text': _PEG_IN_HOLE_PROMPT,
    },
}


def build_server(tools: TwinTools, name: str = 'robot-twin') -> Server:
    """도구 집합을 MCP 서버에 배선한다."""
    server: Server = Server(name)

    async def _list(_ctx: Any, _params: Any) -> types.ListToolsResult:
        # 카탈로그 조회도 HTTP 다 — 이벤트 루프를 막지 않는다.
        return types.ListToolsResult(tools=await asyncio.to_thread(tools.list_tools))

    async def _call(_ctx: Any, params: types.CallToolRequestParams) -> types.CallToolResult:
        try:
            # **블로킹 IO 를 스레드로 내린다.** 팔 이동은 수십 초가 걸리므로 루프에서
            # 직접 돌리면 그동안 상태 조회조차 응답하지 못한다.
            result = await asyncio.to_thread(tools.call, params.name, params.arguments or {})
        except Exception as exc:                       # noqa: BLE001 - 모두 도구 결과로 보고한다
            return types.CallToolResult(
                content=[types.TextContent(type='text', text=_error_text(exc))],
                is_error=True)
        return types.CallToolResult(
            content=[types.TextContent(
                type='text', text=json.dumps(result, ensure_ascii=False, indent=2))])

    async def _list_prompts(_ctx: Any, _params: Any) -> types.ListPromptsResult:
        return types.ListPromptsResult(prompts=[
            types.Prompt(name=name, description=spec['description'])
            for name, spec in _PROMPTS.items()])

    async def _get_prompt(_ctx: Any,
                          params: types.GetPromptRequestParams) -> types.GetPromptResult:
        spec = _PROMPTS.get(params.name)
        if spec is None:
            raise LookupError(f"unknown prompt '{params.name}'")
        return types.GetPromptResult(
            description=spec['description'],
            messages=[types.PromptMessage(
                role='user',
                content=types.TextContent(type='text', text=spec['text']))])

    server.add_request_handler('tools/list', types.PaginatedRequestParams, _list)
    server.add_request_handler('tools/call', types.CallToolRequestParams, _call)
    server.add_request_handler('prompts/list', types.PaginatedRequestParams, _list_prompts)
    server.add_request_handler('prompts/get', types.GetPromptRequestParams, _get_prompt)
    return server


async def _serve(tools: TwinTools) -> None:
    server = build_server(tools)
    options = server.create_initialization_options()
    try:
        async with stdio_server() as (read, write):
            await server.run(read, write, options)
    finally:
        # 에이전트가 end_task 를 부르지 않고 세션이 끊긴 경우다. 열린 에피소드를
        # 남기면 다음 실행이 막히므로 실패로 닫는다.
        await asyncio.to_thread(tools.close_open_task)


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='MCP server exposing a robot twin')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8801)
    parser.add_argument('--twin-id', default='panda01')
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    tools = TwinTools(RobotTwinClient(host=args.host, port=args.port, twin_id=args.twin_id))
    asyncio.run(_serve(tools))
    return 0


__all__: list[str] = ['TwinTools', 'build_server', 'main']


if __name__ == '__main__':
    raise SystemExit(main())
