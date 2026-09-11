"""로봇 트윈 최소 클라이언트 (사용 설명서 5.2 절).

표준 라이브러리만 쓴다. ROS 가 필요 없다.
"""

from __future__ import annotations

from typing import Any, Optional

import json
import time
import urllib.error
import urllib.request


class OperationFailed(Exception):
    """연산이 **접수된 뒤** 실패로 끝났다 (status 가 COMPLETED 가 아니다).

    `TwinError` 와 다르다 — 그쪽은 요청 자체가 거부된 것(4xx/5xx)이고, 이쪽은 접수되어
    실행되다가 실패한 것이라 HTTP 는 202 였다. 트윈이 세션 상태의 `error` 에 사유를
    넣어 주므로 그것을 그대로 싣는다.

    이 예외가 없던 시절 호출자들이 `run(...)['outputs']` 를 그냥 인덱싱했고, 실패
    응답에는 그 키가 없어 **`KeyError: 'outputs'`** 로 터졌다. 그러면 트윈이 준 진짜
    사유(예: "scene reset failed: '/set_entity_state' not ready — is Isaac running …")가
    통째로 버려지고 호출자에게는 `'outputs'` 한 단어만 남는다.
    """

    def __init__(self, operation: str, state: Optional[dict]) -> None:
        state = state or {}
        err = state.get('error') or {}
        self.operation = operation
        self.status = state.get('status')
        self.code = err.get('code')
        self.state = state
        detail = err.get('message') or '트윈이 사유를 주지 않았다'
        code = f'{self.code}: ' if self.code else ''
        super().__init__(f"'{operation}' {self.status}: {code}{detail}")


class TwinError(Exception):
    """트윈이 요청을 거부했을 때 (4xx / 5xx)."""

    def __init__(self, status: int, body: dict) -> None:
        err = body.get('error', {})
        super().__init__(f"HTTP {status} {err.get('code')}: {err.get('message')}")
        self.status = status
        self.code = err.get('code')
        self.body = body


class RobotTwinClient:
    """원격 로봇 트윈에 REST 로 명령을 전달하는 **대리자**.

    **트윈이 아니다.** 상태도 자원 락도 세션도 원격 쪽(`rdfp_ws` 의 `robot_twin` 노드)이
    갖는다. 이 객체가 가진 것은 URL 두 개뿐이라 **생성해도 연결하지 않고 확인도 하지
    않는다** — 트윈이 안 떠 있어도 생성은 성공하고, 첫 호출에서야 드러난다. 그래서 두 개를
    만든다고 트윈이 둘이 되는 것이 아니라 같은 트윈을 가리키는 손잡이가 둘이다.

    이름이 `RobotTwin` 이던 시절에는 그 오해를 docstring 이 되받아야 했다 (2026-09-11 개명).
    """

    def __init__(self, host: str = '127.0.0.1', port: int = 8801,
                 twin_id: str = 'panda01') -> None:
        self.base = f'http://{host}:{port}/api/v1/robot_twins/{twin_id}'
        self.root = f'http://{host}:{port}'

    # ----- 저수준 -----
    def _request(self, path: str, method: str = 'GET',
                 body: Optional[dict] = None) -> tuple[int, Any]:
        url = path if path.startswith('http') else self.base + path
        data = None if body is None else json.dumps(body).encode()
        headers = {'Content-Type': 'application/json'} if data else {}
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=10) as res:
                raw = res.read()
                return res.status, (json.loads(raw) if raw else None)
        except urllib.error.HTTPError as exc:
            raw = exc.read()
            parsed = json.loads(raw) if raw else {}
            if exc.code >= 400:
                raise TwinError(exc.code, parsed) from None
            return exc.code, parsed

    # ----- 상태 -----
    def health(self) -> dict:
        return self._request('/health')[1]

    def read(self, name: str, *, require_ok: bool = True) -> Any:
        """변수 값을 반환한다. quality 를 확인하지 않고 쓰지 않도록 강제한다."""
        body = self._request(f'/variables/{name}')[1]
        if require_ok and body['quality'] != 'OK':
            raise RuntimeError(
                f"variable '{name}' is {body['quality']}"
                f"{' (' + body['reason'] + ')' if body.get('reason') else ''}"
            )
        return body['value']

    def read_raw(self, name: str) -> dict:
        """변수를 응답 그대로 반환한다 — `quality` / `reason` 을 포함한다.

        `read` 와 달리 낮은 품질을 예외로 바꾸지 않는다. 값이 낡았다는 사실 자체가
        판단 근거인 호출자(예: MCP 도구)를 위한 것이다.
        """
        return self._request(f'/variables/{name}')[1]

    def read_many(self, *names: str) -> dict:
        return self._request(f"/variables?names={','.join(names)}")[1]['values']

    def variables(self) -> list:
        """상태 변수 카탈로그 (이름·설명·단위)."""
        return self._request('/variables')[1]['variables']

    def operations(self) -> list:
        """연산 카탈로그 (이름·설명·inputs_schema·점유 자원)."""
        return self._request('/operations')[1]['operations']

    def resources(self) -> dict:
        return self._request('/resources')[1]

    # ----- 연산 -----
    def run(self, operation: str, inputs: Optional[dict] = None, *,
            poll_sec: float = 0.5, timeout_sec: float = 120.0) -> dict:
        """연산을 실행하고 종료까지 기다린다. 동기·비동기를 모두 처리한다."""
        status, body = self._request(f'/operations/{operation}', 'POST',
                                     {'inputs': inputs or {}})
        if status == 200:                      # 동기 연산 — 이미 끝났다
            return body

        endpoint = body['session_endpoint']    # 202 — 폴링한다
        deadline = time.monotonic() + timeout_sec
        while True:
            state = self._request(self.root + endpoint)[1]
            if state['status'] != 'RUNNING':
                return state
            if time.monotonic() > deadline:
                raise TimeoutError(f"'{operation}' did not finish in {timeout_sec}s")
            time.sleep(poll_sec)

    def outputs(self, operation: str, inputs: Optional[dict] = None, **kwargs) -> dict:
        """연산을 실행하고 **`outputs` 를 돌려준다.** 실패면 `OperationFailed` 다.

        `run` 은 실패해도 세션 상태를 그대로 돌려주므로 호출자가 `status` 를 봐야 하는데,
        그것을 잊고 `run(...)['outputs']` 로 바로 들어가기 쉽다 — 실패 응답에는 그 키가
        없어 `KeyError: 'outputs'` 가 나고 사유가 사라진다. 결과를 쓸 것이라면 이쪽을
        쓴다. 상태를 직접 판단해야 하는 곳(`ops` 의 검증 헬퍼들)만 `run` 을 쓴다.
        """
        state = self.run(operation, inputs, **kwargs)
        if state.get('status') != 'COMPLETED':
            raise OperationFailed(operation, state)
        return state.get('outputs') or {}

    def cancel(self, session_endpoint: str) -> dict:
        return self._request(self.root + session_endpoint, 'DELETE')[1]

    # ----- 안전 -----
    def estop(self) -> dict:
        return self._request('/estop', 'POST')[1]

    def release_estop(self) -> dict:
        return self._request('/estop', 'DELETE')[1]


__all__ = ['OperationFailed', 'RobotTwinClient', 'TwinError']
