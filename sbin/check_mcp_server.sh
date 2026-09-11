#!/usr/bin/env bash
#
# MCP 서버의 배선을 점검한다 — 서버 기동 + 트윈 연결 + 도구 목록.
#
#   ./sbin/check_mcp_server.sh                    # 점검 (기본)
#   ./sbin/check_mcp_server.sh --port 8802        # 다른 트윈에 붙어서 점검
#   ./sbin/check_mcp_server.sh --serve            # stdio 서버로 기동 (거의 쓸 일 없다)
#
# **서버를 띄우는 스크립트가 아니다.** MCP 서버는 stdio 로 말하므로
# 클라이언트(Claude Code 등)가 프로세스를 직접 spawn 한다. 등록은
# `./sbin/register_mcp_server.sh` 를 쓴다.
#
# `--serve` 는 그 spawn 을 손으로 흉내내는 모드다. 표준입력에서 JSON-RPC 를
# 기다리며 **멈춘 것처럼 보이는데** 고장이 아니다. 사람이 확인할 목적이라면
# 인자 없이 실행하면 된다.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
serve='false'
args=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --serve)   serve='true'; shift ;;
        -h|--help) sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *)         args+=("$1"); shift ;;
    esac
done

cd "$root"

if [[ "$serve" == 'true' ]]; then
    exec uv run --extra mcp robot-twin-mcp ${args[@]+"${args[@]}"}
fi

# ── 점검 ──────────────────────────────────────────────────────────
# initialize → initialized → tools/list 를 한 번 주고받는다. 트윈까지 살아 있어야
# 도구 목록이 오므로, 이 한 번으로 배선 전체가 확인된다.
#
# stdin 을 곧바로 닫으면 서버가 응답 전에 종료하므로 잠시 열어 둔다.
out="$(mktemp)"; err="$(mktemp)"
trap 'rm -f "$out" "$err"' EXIT

{ printf '%s\n%s\n%s\n' \
    '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"check_mcp_server.sh","version":"1"}}}' \
    '{"jsonrpc":"2.0","method":"notifications/initialized"}' \
    '{"jsonrpc":"2.0","id":2,"method":"tools/list"}'
  sleep 6
} | timeout 60 uv run --extra mcp robot-twin-mcp ${args[@]+"${args[@]}"} > "$out" 2> "$err" || true

python3 - "$out" "$err" <<'PY'
import json, pathlib, sys

out, err = (pathlib.Path(p).read_text() for p in sys.argv[1:3])
msgs = {}
for line in out.splitlines():
    try:
        d = json.loads(line)
    except ValueError:
        continue
    if 'id' in d:
        msgs[d['id']] = d

if 1 not in msgs:
    print('✘ 서버가 initialize 에 응답하지 않았다.')
    print(err.strip()[-800:] or '(stderr 없음)')
    raise SystemExit(1)
print('✔ 서버 기동 OK  —', msgs[1]['result']['serverInfo']['name'])

res = msgs.get(2, {})
if 'error' in res or 'result' not in res:
    print('✘ 도구 목록을 가져오지 못했다 — 트윈이 떠 있는지 확인한다.')
    print('  ', res.get('error') or '(응답 없음)')
    raise SystemExit(2)

tools = sorted(t['name'] for t in res['result']['tools'])
print(f'✔ 도구 {len(tools)}개')
for i in range(0, len(tools), 4):
    print('   ', '  '.join(f'{n:<28}' for n in tools[i:i + 4]).rstrip())
missing = [t for t in res['result']['tools'] if not t.get('description')]
print('✘ 설명 없는 도구:', [t['name'] for t in missing]) if missing else print('✔ 모든 도구에 설명 있음')
PY
