#!/usr/bin/env bash
#
# MCP 서버를 Claude Code 에 등록한다.
#
#   ./sbin/register_mcp_server.sh                      # 기본 (user scope — 모든 프로젝트)
#   ./sbin/register_mcp_server.sh --scope local        # 이 프로젝트에서만
#   ./sbin/register_mcp_server.sh --port 8802 --name robot-twin-02
#   ./sbin/register_mcp_server.sh --remove             # 등록 해제
#
# **덮어쓰기다.** 같은 이름이 있으면 지우고 다시 넣으므로, 인자를 바꿔 다시
# 실행하면 그대로 반영된다.
#
# 등록만 하는 스크립트다 — 트윈이나 로봇 스택이 떠 있을 필요는 없다. 다만 등록
# 직후 `claude mcp list` 는 트윈이 없으면 `tools fetch failed` 로 보이는데,
# 도구 목록을 트윈 카탈로그에서 가져오기 때문이며 정상이다.
set -euo pipefail

name='robot-twin'
# 기본을 user 로 둔다. local 은 **등록할 때의 프로젝트 디렉터리에서만** 잡히는데,
# 로봇 작업은 보통 rdfp_ws 쪽에서 하므로 거기서 도구가 보이지 않는다.
scope='user'
host='127.0.0.1'
port='8801'
twin_id='panda01'
remove_only='false'

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)     name="$2";    shift 2 ;;
        --scope)    scope="$2";   shift 2 ;;
        --host)     host="$2";    shift 2 ;;
        --port)     port="$2";    shift 2 ;;
        --twin-id)  twin_id="$2"; shift 2 ;;
        --remove)   remove_only='true'; shift ;;
        -h|--help)  sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# 프로젝트 루트는 **스크립트 위치**에서 구한다. 등록된 커맨드에 절대 경로가 박히므로
# 어느 디렉터리에서 실행하든 같은 결과가 나와야 한다.
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

command -v claude >/dev/null || { echo 'claude CLI not found in PATH' >&2; exit 3; }

# uv 는 **절대 경로**로 박는다. Claude Code 가 서버를 spawn 할 때의 PATH 는 이
# 셸과 다를 수 있고, GUI 클라이언트(Desktop)는 로그인 셸 PATH 를 보지 못한다.
uv_bin="$(command -v uv)" || { echo 'uv not found in PATH' >&2; exit 3; }

# 이미 있으면 지운다. add 는 중복 이름을 거부하므로 이 단계가 있어야 재실행된다.
if claude mcp get "$name" >/dev/null 2>&1; then
    claude mcp remove "$name" >/dev/null 2>&1 || true
    echo "removed existing registration: $name"
fi

if [[ "$remove_only" == 'true' ]]; then
    exit 0
fi

# mcp extra 가 없으면 서버가 import 단계에서 죽는다 (mcp SDK 는 optional 이다).
echo "syncing dependencies (--extra mcp) ..."
(cd "$root" && uv sync --extra mcp >/dev/null)

claude mcp add "$name" --scope "$scope" -- \
    "$uv_bin" --directory "$root" run --extra mcp robot-twin-mcp \
    --host "$host" --port "$port" --twin-id "$twin_id"

echo
claude mcp get "$name" || true
echo
echo "등록 완료. Claude Code 안에서 '/mcp' 로 상태를 확인한다."
