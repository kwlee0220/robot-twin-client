"""로봇 트윈 최소 클라이언트 (사용자 가이드 5.2 절).

표준 라이브러리만 쓴다. ROS 가 필요 없다.
"""

from __future__ import annotations

from typing import Any, Optional

import importlib.metadata
import json
import time
import urllib.error
import urllib.parse
import urllib.request


DEFAULT_BASE_URL = 'http://127.0.0.1:8801'
# 기본으로 붙는 트윈 — 펑션베이 트윈 `rdfp01` (2026-10-25). 출하 트윈은 환경마다 id 가 달라
# (`rdfp01` · `panda01` · `mock01`) 다른 환경에 붙을 때는 id 를 준다. 틀리면 404 다.
DEFAULT_TWIN_ID = 'rdfp01'

# 트윈의 감사 로그가 요청마다 `User-Agent` 를 남긴다 (트윈 설계서 8.2). urllib 기본값
# (`Python-urllib/3.x`)으로는 MCP 서버 · 스크립트 · 다른 클라이언트를 가를 수 없다.
CLIENT_NAME = 'robot-twin-client'


def _client_version() -> str:
    """설치된 배포판의 버전. 설치하지 않고 소스에서 돌리면 `unknown` 이다."""
    try:
        return importlib.metadata.version(CLIENT_NAME)
    except importlib.metadata.PackageNotFoundError:
        return 'unknown'


def user_agent(agent: Optional[str] = None) -> str:
    """`robot-twin-client/<버전>` 에 호출 주체(`agent`)를 괄호로 붙인 `User-Agent` 값."""
    base = f'{CLIENT_NAME}/{_client_version()}'
    return f'{base} ({agent})' if agent else base


def _server_root(base_url: str) -> str:
    """`base_url` 을 확인하고 끝의 `/` 를 뗀 `scheme://netloc` 을 돌려준다.

    연결은 하지 않는다 — 형식만 본다. 스킴을 빠뜨린 `127.0.0.1:8801` 을 그대로 두면 첫 호출에서
    `unknown url type` 처럼 원인이 안 보이는 오류가 난다(`localhost:8801` 은 `localhost` 가
    스킴으로 읽힌다).

    **경로는 거부한다** — 트윈은 `session_endpoint` 를 접두사 없는 절대 경로(`/api/v1/...`)로
    돌려주므로, 리버스 프록시 접두사(`https://gw/twin`)를 받으면 동기 연산은 되고 202 뒤의
    폴링만 404 로 깨진다. 접두사를 지원하려면 서버가 `root_path` 를 따라야 한다.
    """
    parts = urllib.parse.urlsplit(base_url)
    if parts.scheme not in ('http', 'https') or not parts.hostname:
        raise ValueError(f"base_url must look like 'http(s)://host[:port]', got {base_url!r}")
    parts.port                                 # 포트가 숫자가 아니면 여기서 ValueError 다
    if parts.path.strip('/') or parts.query or parts.fragment:
        raise ValueError(
            f'base_url must be the server root without a path, got {base_url!r} '
            '(path prefixes are unsupported: the twin returns absolute session endpoints)')
    return f'{parts.scheme}://{parts.netloc}'


class OperationFailed(Exception):
    """연산이 **접수된 뒤** 실패로 끝났다 (status 가 COMPLETED 가 아니다).

    `TwinError` 와 다르다 — 그쪽은 요청 자체가 거부된 것(4xx/5xx)이고, 이쪽은 접수되어
    실패 상태를 반환한 것이다. HTTP 200의 동기 실패와 202 이후 조회한 비동기 실패가
    모두 포함된다. 트윈이 결과 상태의 `error` 에 넣은 사유를 그대로 싣는다.

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


class WrongBackend(Exception):
    """떠 있는 트윈이 이 작업이 요구하는 환경(백엔드)이 아니다.

    출하 트윈은 환경마다 id 가 다르지만(`rdfp01` · `panda01` · `mock01`, 2026-10-25) id 는
    사용자가 설정에서 바꿀 수 있으므로 **환경 판정의 근거로 삼지 않는다** — 트윈 메타의
    `backend` 로 본다. 확인하지 않으면 Isaac 물리가 필요한
    쌓기가 mock 트윈에 붙어 에러 없이 엉뚱하게 돈다.
    """

    def __init__(self, purpose: str, expected: tuple[str, ...], actual: Optional[str]) -> None:
        self.purpose = purpose
        self.expected = expected
        self.actual = actual
        got = actual or '알 수 없음 (트윈이 backend 를 알려 주지 않는다 — 옛 트윈이면 rdfp_ws 를 갱신한다)'
        super().__init__(
            f"{purpose}에는 {' · '.join(expected)} 트윈이 필요한데 떠 있는 트윈은 {got} 이다 — "
            f'./scripts/run_robot_twin.sh --backend {expected[0]} 로 띄운다')


def require_backend(twin: Any, *expected: str, purpose: str) -> str:
    """떠 있는 트윈이 `expected` 중 하나의 환경인지 확인하고 그 이름을 돌려준다.

    아니면 :class:`WrongBackend`. 환경은 트윈 메타의 `backend` 로 안다 — id 는 환경을 담지
    않는다. **환경 이름이 아니라 능력으로 가를 수 있으면 그쪽이 낫다** (peg 는 `fixtures`
    변수 유무로 가른다). 이것은 쌓기처럼 능력을 변수 하나로 못 가를 때 쓴다.
    """
    actual = twin.meta().get('backend')
    if actual not in expected:
        raise WrongBackend(purpose, tuple(expected), actual)
    return actual


class RobotTwinClient:
    """원격 로봇 트윈에 REST 로 명령을 전달하는 **대리자**.

    **트윈이 아니다.** 상태도 자원 락도 세션도 원격 쪽(`rdfp_ws` 의 `robot_twin` 노드)이
    갖는다. 이 객체는 접속 URL과 HTTP 시간 제한을 보관하며 **생성해도 연결하지 않고
    확인도 하지 않는다** — 트윈이 안 떠 있어도 생성은 성공하고, 첫 호출에서야 드러난다. 그래서 두 개를
    만든다고 트윈이 둘이 되는 것이 아니라 같은 트윈을 가리키는 손잡이가 둘이다.

    이름이 `RobotTwin` 이던 시절에는 그 오해를 docstring 이 되받아야 했다 (2026-09-11 개명).
    """

    def __init__(self, base_url: str = DEFAULT_BASE_URL, twin_id: str = DEFAULT_TWIN_ID, *,
                 http_timeout_sec: float = 30.0, agent: Optional[str] = None) -> None:
        """`base_url` 은 트윈 서버의 루트(`http(s)://호스트[:포트]`)다 — API 경로는 넣지 않는다.

        `/api/v1/robot_twins/<id>` 는 이 클라이언트가 붙인다. 버전은 클라이언트 코드가 기대하는
        API 이고, id 는 어느 트윈인지를 따로 드러내야 하므로 URL 에 섞지 않는다.

        `agent` 는 호출 주체 이름(`mcp` · `robot-twin-peg` …)이며 `User-Agent` 에 실려 트윈의
        감사 로그에서 요청을 가르는 데 쓰인다.
        """
        self.base_url = _server_root(base_url)
        self.twin_id = twin_id
        self.base = f'{self.base_url}/api/v1/robot_twins/{twin_id}'
        # **동기 연산의 `sync_timeout_sec` 보다 길어야 한다** — POST 응답이 그 뒤에 온다.
        # 10초였을 때 펑션베이 그리퍼(12초)와 `reset_scene`(10초)에서 트윈이 결과를 주기
        # 전에 클라이언트가 소켓 시간 초과로 먼저 끊겼다. 동작은 서버에서 계속된다.
        self.http_timeout_sec = http_timeout_sec
        self.user_agent = user_agent(agent)

    # ----- 저수준 -----
    def _request(self, path: str, method: str = 'GET',
                 body: Optional[dict] = None) -> tuple[int, Any]:
        """트윈 기준 상대 경로(`/health` · `/operations/<이름>` …)로 요청한다."""
        return self._send(self.base + path, method, body)

    def _send(self, url: str, method: str = 'GET',
              body: Optional[dict] = None) -> tuple[int, Any]:
        data = None if body is None else json.dumps(body).encode()
        headers = {'User-Agent': self.user_agent}
        if data:
            headers['Content-Type'] = 'application/json'
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.http_timeout_sec) as res:
                raw = res.read()
                return res.status, (json.loads(raw) if raw else None)
        except urllib.error.HTTPError as exc:
            raw = exc.read()
            parsed = json.loads(raw) if raw else {}
            if exc.code >= 400:
                raise TwinError(exc.code, parsed) from None
            return exc.code, parsed

    # ----- 상태 -----
    def meta(self) -> dict:
        """트윈 메타 — id · 설명 · `backend` · 실행 모드 · 변수/연산 이름.

        **id 로는 환경을 모른다** — 출하 트윈은 환경마다 id 가 다르지만 설정에서 바꿀 수 있다.
        어느 환경에 붙었는지는 여기의 `backend` 로 본다 (:func:`require_backend`).
        """
        return self._request('')[1]

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
            state = self._send(self.base_url + endpoint)[1]
            if state['status'] != 'RUNNING':
                return state
            if time.monotonic() > deadline:
                # **취소하지 않는다** — 기다리기를 그만둘 뿐 로봇은 계속 움직인다. 멈추려면
                # 메시지의 endpoint 로 `cancel()` 한다. 트윈의 워치독은 따로 돈다.
                raise TimeoutError(f"'{operation}' did not finish in {timeout_sec}s: {endpoint}")
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
        return self._send(self.base_url + session_endpoint, 'DELETE')[1]

    # ----- 안전 -----
    def estop(self) -> dict:
        return self._request('/estop', 'POST')[1]

    def release_estop(self) -> dict:
        return self._request('/estop', 'DELETE')[1]


__all__ = ['DEFAULT_BASE_URL', 'DEFAULT_TWIN_ID', 'OperationFailed', 'RobotTwinClient', 'TwinError',
           'WrongBackend', 'require_backend']
