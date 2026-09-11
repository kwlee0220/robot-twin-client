#!/usr/bin/env bash
#
# Claude Code CLI 를 설치한다. 공식 설치 스크립트를 받아서 실행한다.
#
#   ./sbin/install_claude_cli.sh                  # 없을 때만 설치
#   ./sbin/install_claude_cli.sh --force          # 이미 있어도 다시 설치
#   ./sbin/install_claude_cli.sh --version latest # stable | latest | X.Y.Z
#
# register_mcp_server.sh 가 'claude CLI not found in PATH' 로 죽을 때 먼저 실행한다.
# VSCode 확장으로만 Claude Code 를 쓰면 확장이 자체 번들 바이너리를 쓰기 때문에
# PATH 에는 claude 가 올라오지 않는다.
#
# 설치 위치는 ~/.local/bin/claude 다. sudo 로 실행하면 안 된다 — 홈 디렉터리
# 기준으로 설치되므로 root 홈에 들어가 버린다 (설치 스크립트가 직접 막는다).
set -euo pipefail

installer_url='https://claude.ai/install.sh'
target='stable'
force='false'

while [[ $# -gt 0 ]]; do
    case "$1" in
        --version)  target="$2"; shift 2 ;;
        --force)    force='true'; shift ;;
        -h|--help)  sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [[ ! "$target" =~ ^(stable|latest|[0-9]+\.[0-9]+\.[0-9]+)$ ]]; then
    echo "--version 은 stable, latest, 또는 X.Y.Z 형식이어야 한다: $target" >&2
    exit 2
fi

# 이미 있으면 그냥 끝낸다. 이 스크립트는 register_mcp_server.sh 앞에 무조건
# 붙여도 되도록 idempotent 해야 한다.
if [[ "$force" != 'true' ]] && command -v claude >/dev/null 2>&1; then
    echo "claude 가 이미 있다: $(command -v claude) ($(claude --version 2>/dev/null || echo 'version 확인 실패'))"
    echo "다시 설치하려면 --force 를 준다."
    exit 0
fi

# 파이프로 바로 bash 에 넣지 않고 받아서 실행한다. 전송이 중간에 끊기면 반쯤
# 받은 스크립트가 실행되는데, 파일로 받으면 curl 이 먼저 실패해 준다.
tmp="$(mktemp -t claude-install.XXXXXX.sh)"
trap 'rm -f "$tmp"' EXIT

echo "downloading installer: $installer_url"
if command -v curl >/dev/null 2>&1; then
    curl -fsSL "$installer_url" -o "$tmp"
elif command -v wget >/dev/null 2>&1; then
    wget -qO "$tmp" "$installer_url"
else
    echo 'curl 도 wget 도 없다' >&2
    exit 3
fi

# 받은 게 HTML 오류 페이지가 아닌지 본다. curl -f 가 4xx/5xx 는 걸러주지만,
# 200 으로 엉뚱한 걸 주는 경우까지는 막지 못한다.
head -1 "$tmp" | grep -q '^#!' || { echo '받은 파일이 셸 스크립트가 아니다' >&2; exit 3; }

echo "installing claude ($target) ..."
bash "$tmp" "$target"

# 설치 스크립트는 성공해도 이 셸의 PATH 나 command 해시는 못 고친다.
bin="$HOME/.local/bin/claude"
[[ -x "$bin" ]] || { echo "설치는 끝났는데 $bin 이 없다" >&2; exit 3; }

echo
echo "설치 완료: $bin ($("$bin" --version))"
case ":$PATH:" in
    *":$HOME/.local/bin:"*)
        echo "지금 셸에서는 'hash -r' 을 한 번 실행해야 claude 가 잡힌다 (또는 셸을 새로 연다)."
        ;;
    *)
        echo "주의: \$HOME/.local/bin 이 PATH 에 없다. ~/.bashrc 에 아래를 추가한다:"
        echo '    export PATH="$HOME/.local/bin:$PATH"'
        ;;
esac
