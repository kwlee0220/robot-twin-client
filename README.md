# robot-twin-client

로봇 트윈 HTTP API 클라이언트, **pick-and-place 시퀀스 예제**, 그리고 로봇을 LLM
에이전트에게 노출하는 **MCP 서버**.

`rdfp` 워크스페이스 문서 `docs/twin/robot_twin_user_guide.md` 의 5.9 절 예제를
독립 실행 가능한 프로젝트로 옮긴 것이다. 트윈은 HTTP 로만 접근하므로 **ROS 설치도
`source install/setup.bash` 도 필요 없다.** 클라이언트와 예제는 표준 라이브러리만
쓰고, MCP 서버만 `mcp` extra 를 요구한다.

> **패키지 이름이 `robot_twin_client` 인 이유** — `rdfp_ws` 에 `robot_twin` 이라는
> 같은 이름의 ROS 파이썬 패키지(트윈 서버 본체)가 있다. ROS 를 source 한 셸에서는
> `PYTHONPATH` 가 editable 설치의 `.pth` 보다 먼저 검색되므로 그쪽이 이 패키지를
> 가려버린다 (증상: `No module named robot_twin.mcp_server`). 이름을 겹치지 않게
> 두는 것으로 해결한다. 저장소·배포 이름도 같은 이유로 `robot-twin-client` 이고,
> 클래스도 `RobotTwinClient` 다 (2026-09-11 개명) — 이 저장소는 트윈이 아니라 트윈을
> **호출하는 쪽**이라는 것이 세 이름에 일관되게 드러난다.
>
> **명령 이름 넷과 MCP 등록 이름은 `robot-twin` 접두사 그대로다** (`robot-twin-mcp`,
> `robot-twin-pick`, `robot-twin-stack`, `robot-twin-peg`). 충돌하지 않고, 등록 이름을
> 바꾸면 이미 등록된 `claude mcp get robot-twin` 이 끊긴다.

## 구성

| 모듈 | 설명서 절 | 내용 |
|---|---|---|
| `robot_twin_client/client.py` | 5.2 | `RobotTwinClient` (변수 조회 · 카탈로그 · 연산 실행 · 취소 · E-stop), `TwinError` |
| `robot_twin_client/ops.py` | 5.6 / 5.7 / 4.2 | `move_linear_verified` (도달 검증), `move_to_named_target_verified` (SRDF 이름으로 이동), `set_gripper` (파지 판정) |
| `robot_twin_client/pick_n_place.py` | 5.9 / 5.10 | `pick` · `place` · `above_of` · `rotated_about_base_z` · **`to_arm_command`**(TCP → `panda_link8` 프레임 보정), **`reset_and_pick_and_place`**(씬 리셋 + 에피소드 경계), CLI `main` |
| `robot_twin_client/stack.py` | — | **쌓기.** 물체를 집어 다른 물체 위에 얹는다 — `half_height_of`(타입별 치수 해석) · **`held_offset`**(쥔 뒤 실측 보정) · `await_fresh_scene`(리셋 반영 대기) · `stack` · **`reset_and_stack`** · `verify_stacked`, CLI `main` |
| `robot_twin_client/peg_in_hole.py` | — | **peg-in-hole.** `pick_peg` · `place_peg` · `move_peg` — 구멍에 꽂고 빼는 절차와 **세 판정**(파지 오프셋 보정 · 기하 사전 확인 · 앉음 확인). CLI `robot-twin-peg` |
| `robot_twin_client/mcp_server.py` | — | **MCP 서버.** 트윈 카탈로그를 LLM 도구로 노출한다 (아래 참고) |

`ops.py` 가 따로 있는 이유는 **트윈의 `COMPLETED` 가 "도달했다" 는 뜻이 아니기**
때문이다. 직선 이동은 경로 일부만 실행돼도 성공으로 끝나고, 그리퍼는 물체를 물면
목표 폭에 못 미친 채(`reached_goal=False`, `stalled=True`) 끝난다. 판정을 한군데
모아 두고 `pick` / `place` 는 그 결과만 본다.

연산마다 판정 근거가 다르다는 점에 주의한다.

| 연산 | `COMPLETED` 인데 도달 못 하는 경우 | 판정 근거 |
|---|---|---|
| `move_linear` | 계획의 60~100% 만 실행돼도 성공으로 끝난다 (`fraction` 은 오지 않음) | `outputs.final_pose` 를 목표와 비교 |
| `move_to_named_target` | 관절 공간 계획이라 부분 실행은 없지만, JGPC 스택은 open loop 라 도달 보장이 없다 | `outputs.closed_loop` |

**그리퍼 결과의 이름이 바뀌었다.** 트윈은 이제 `at_goal` / `width` 를 보낸다
(`rdfp_msgs/GripperState`). 옛 이름 `reached_goal` / `position` 만 읽으면
`KeyError` 로 죽는데, 증상이 원인을 안 가리킨다 — 실제로 Isaac 트윈에서 `pick` 이
"대상이 잘못되었다" 로 끝났다. `set_gripper` 가 `at_goal` 에서 `reached_goal` 을
채워 주므로 기존 호출자는 그대로 동작하지만, **새 코드는 `at_goal` 을 본다** —
그것이 목표별 판정식을 이미 적용한 유일한 성공 신호다.

**mock 백엔드에서는 파지 판정이 성립하지 않는다.** 물리가 없어 물체를 물어도
`stalled` 가 서지 않으므로, 판정을 그대로 적용하면 파지에서 멈춰 **그 이후 단계를
전혀 검증할 수 없다.** `--allow-empty-grasp` (라이브러리에서는 `pick(...,
require_grasp=False)`) 로 건너뛴다 — 경고를 찍고 진행한다. **실제 로봇에서는 쓰지
않는다**; 빈손인 채로 이송하게 된다.

> **이 우회는 한시적이다.** 붙일 수 있는 스택이 mock 뿐이라 둔 것이고, 물리가 있는
> 스택(gazebo / isaac-sim)을 쓸 수 있게 되면 `--allow-empty-grasp` 와
> `require_grasp` 인자를 함께 제거한다. 그때부터 파지 실패는 원칙대로 오류다.

`move_to_named_target` 의 목표 관절값은 SRDF 안에 있어 클라이언트가 모른다. 그래서
`closed_loop: false` 일 때는 좌표로 검증할 방법이 없어 `reached=False` 와 `why` 를
돌려주고 판단을 호출자에게 넘긴다. 쓸 수 있는 이름은 `twin.read('named_targets')`
로 먼저 확인한다 — 오타는 `400` 이 아니라 계획 단계의 `FAILED` 로 온다.

### 좌표는 손끝(TCP) 기준이다

**트윈의 `move_linear` 는 `panda_link8`(planning group 의 tip link)을 움직이는데,
손끝은 거기서 약 10.3 cm 더 뻗어 있다.** 물체 중심을 그대로 명령하면 그리퍼가 그만큼
아래로 내려가 **바닥을 파고든다** — 5 cm 큐브(중심 z=0.025)라면 손끝이 z=-0.078 이다.

그래서 이 스크립트는 좌표를 **손끝 기준으로 다루고**, 팔에 보내기 직전에만
`to_arm_command()` 로 바꾼다. 두 가지를 되돌린다.

| 보정 | 이유 |
|---|---|
| 위치 — 목표 자세로 회전시킨 오프셋만큼 뒤로 | 손끝이 목표에 오게 한다. DOWN 자세면 z 가 10.3 cm 올라간다 |
| 자세 — z축 **+45°** | URDF 의 `panda_hand_joint` 가 link8→hand 를 -45° 돌린다. 보정하지 않으면 손가락이 큐브 모서리 방향으로 물린다 |

두 프레임의 **위치는 같으므로**(`panda_hand_joint` 의 이동이 0) `move_linear_verified`
가 `outputs.final_pose`(= `panda_hand`)와 비교하는 것은 이 변환 뒤에도 유효하다.

> 정석은 URDF 에 `panda_hand_tcp` 프레임을 두고 `link_name` 으로 지정하는 것이다.
> 그러면 이미 녹화된 `/ee_pose` 기준이 달라져 데이터셋 호환을 따져야 하므로, 우선은
> 클라이언트 쪽 보정으로 둔다.

## 설치

```bash
cd ~/development/mdtpy/robot-twin-client
uv sync
```

## 실행

먼저 로봇 스택과 트윈이 떠 있어야 한다 (사용 설명서 1.2 / 1.3).

```bash
uv run robot-twin-pick --help
uv run robot-twin-pick --scene one_cube --seed 7 --angle 30
```

**물체를 먼저 배치한다.** `--scene` 레시피대로 `reset_scene` 을 호출하고, 그 결과로
나온 **실제 배치 좌표를 집는다** — 랜덤하게 놓고 고정 좌표를 집으면 그 자리에 아무것도
없기 때문이다. `--seed` 가 같으면 배치도 같다. 축을 고정하고 싶으면 `-x/-y/-z` 로
덮어쓴다.

```
reset_scene ─ pick → place ─ return_home
```

**이 스크립트는 로봇 조작만 한다 — 세션/에피소드 경계를 다루지 않는다.** 조작과 수집을
한 함수에 묶으면 조작만 하고 싶을 때 쓸 수가 없어서다. 그래서 수집 계층(`rdfp`)이 떠
있지 않아도 되고, **제어 스택(`robot_control panda_mock`)만으로 동작한다.**

같은 실행을 학습 데이터로 남기려면 경계를 찍는 쪽이 이 흐름을 감싼다 — MCP 서버의
`begin_task` / `end_task` 가 그 역할을 한다(아래 "작업 경계는 서버가 소유한다").

집은 다음 **베이스 z축(`panda_link0`) 둘레로 `--angle` 만큼 반시계로 돈 자리**에
놓는다. 기본값은 30° 이고, 음수를 주면 시계 방향이다. 놓을 자리는 파지 지점과 같은
반경·같은 높이의 원호 위이며, 그리퍼 자세도 같은 각만큼 함께 돌아 쥔 물체와의 상대
자세가 유지된다. 놓고 상승한 뒤에는 **SRDF named target `ready` 로 복귀**한다
(`--home` 으로 바꾼다).

인자를 주지 않으면 설명서의 예시 좌표를 쓴다. 좌표는 `position`(m) + 고정 자세
`DOWN`(그리퍼가 아래를 봄)이며 좌표계는 **`panda_link0` 고정**이다.

트윈이 다른 호스트/포트에 있으면 `--host` / `--port` / `--twin-id` 로 지정한다.

> **이 프로그램은 실제로 로봇을 움직인다.** 파지 판정(`stalled`)은 그 자리에 실제로
> 물체가 있어야 성립하고, 회전한 자리가 작업 공간 안이어야 한다.
>
> **필요한 것**: 트윈과 백엔드 씬 노드(mock 은 `mock_scene_state_node`). 씬 노드는
> mock 계열 launch 가 기본으로 띄운다.

라이브러리로 쓸 때는 다음과 같다.

```python
from robot_twin_client import (
    RobotTwinClient, above_of, grasp_pose_of, pick, place, return_home, rotated_about_base_z
)

twin = RobotTwinClient(host='127.0.0.1', port=8801, twin_id='panda01')

placement = twin.run('reset_scene', {'scene': 'one_cube', 'seed': 7})['outputs']
grasp = grasp_pose_of(placement['objects'][0])   # 배치된 물체를 집는다
target = rotated_about_base_z(grasp, 30.0)       # 반시계 30°

if pick(twin, above_of(grasp), grasp):
    place(twin, above_of(target), target)
    return_home(twin)          # 복귀는 place 밖이다 — 다음 물체를 이어 집을 수도 있다
```

경계까지 포함한 전체 흐름은 `reset_and_pick_and_place(twin, args)` 하나로 돈다.

### peg-in-hole — `robot-twin-peg`

peg 을 한 고정물에서 다른 고정물로 **옮겨 꽂는다.** 펑션베이 백엔드 전용이다
(포트·트윈 이름 기본값이 그쪽이다).

```bash
uv run robot-twin-peg                 # 반대쪽으로 (지금 위치에서 자동 판단)
uv run robot-twin-peg peg_tray        # 목적지를 명시
uv run robot-twin-peg --dry-run       # 판단만 하고 움직이지 않는다
```

실측(2026-09-11): **왕복 5회 연속, 물체 이탈 0건**, 삽입 오차 0.05~0.40 mm.

**세 판정이 이 모듈의 실체다.** 셋 다 *"집어서 옮긴다"* 만으로는 나오지 않고, 틀렸을
때 증상이 **에러가 아니라 물체가 날아가는 것**이다.

| 판정 | 없으면 |
|---|---|
| 파지 오프셋 보정 — **peg 바닥 끝**을 겨냥한다 (중심이 아니다) | 0.74° 기울기가 바닥을 0.32 mm 옮겨 입구에 걸린다 (여유 0.50 mm) |
| 기하 사전 확인 — 손끝이 고정물 상면 위에 남는가 | 밀려 올라간 파지로 밀어 넣다 물체가 튕겨 나간다. **움직이기 전에** 막는다 |
| 앉음 확인 — 놓기 전에 실제로 앉았는가 | 안 앉은 채 놓으면 물체를 **영구히 잃는다** |

⚠️ **실패하면 peg 을 쥔 채로 멈춘다** (`PegInHoleError.holding`). 그리퍼를 열면 안 되고,
같은 호출을 재시도해도 안 된다 — 원인이 그대로면 결과도 같다.

⚠️ **고정물 밖에는 내려놓을 수 없다.** 시뮬레이터 씬에 `peg ↔ 탁자` 접촉이 정의돼 있지
않아, 고정물이 잡아 주지 않으면 물체가 탁자를 통과해 떨어진다(3회 확인, 낙하 가속도
−11.25 m/s² = 중력). 절차의 한계가 아니라 **씬의 한계**다 — 벤더 요청 B-14.

⚠️ **다루는 것은 서 있는 peg 뿐이다.** 쓰러진 것은 범위 밖이며 거부한다.

### 쌓기 — `robot-twin-stack`

물체를 집어 **다른 물체 위에 얹는다.**

```bash
uv run robot-twin-stack --help
uv run robot-twin-stack                                  # block_a 를 cylinder_a 위에 (기본)
uv run robot-twin-stack --source cylinder_a --base block_a   # 반대로
uv run robot-twin-stack --repeat 10                      # 10 회 반복 (매 회 ready 복귀 + 씬 무작위)
uv run robot-twin-stack --repeat 10 --seed 100           # 재현 가능한 10 회
uv run robot-twin-stack --no-reset          # 이미 놓인 것 위에 이어서 쌓는다
```

```
reset_scene ─ pick(source) ─ 물린 위치 실측 ─ place(base 위) ─ return_home ─ verify
```

**시작할 때 씬을 먼저 배치한다.** 리셋 없이 두 번 돌리면 이미 얹혀 있는 물체를 다시
집어 제자리에 놓는 꼴이 된다. `--scene` / `--seed` 로 레시피와 배치를 고르고, 같은
시드는 같은 배치를 준다.

**리셋은 `stack` 밖에 있다.** 쌓기는 겹쳐 쓸 수 있어야 하기 때문이다 — A 위에 B 를
올린 뒤 그 위에 C 를 올리는 식이다. `stack` 이 스스로 리셋하면 **두 번째 호출이 첫
번째 결과를 무너뜨린다.** 그래서 조작은 `stack`, 배치까지 묶은 것은
`reset_and_stack` 이고, CLI 는 `--no-reset` 으로 앞의 것만 쓴다.

**리셋 뒤에는 씬 변수가 따라올 때까지 기다린다.** `/scene/reset` 이 끝나도
`scene_objects` 는 주기 발행이라 **직전 배치를 한동안 더 말한다.** 그대로 읽으면
리셋 전 자리를 집으러 가고 거기엔 아무것도 없는데, 시퀀스는 끝까지 정상으로 보이고
파지만 조용히 빗나간다. `await_fresh_scene` 이 새 표본 **둘**을 기다린다 — 하나로는
리셋 직전에 떠난 발행이 바로 도착할 수 있어 부족하다.

**쥔 뒤에 물린 위치를 실측하는 것이 핵심이다.** 손끝을 물체 중심에 맞춰 닫아도 물체는
손가락 사이에서 미끄러진다 — Isaac 실측에서 실린더가 **12 mm** 내려간 채 물린 적이
있다(같은 동작을 다시 하면 1 mm 였다; 재현되지 않는다). 상수 `TCP_OFFSET_M` 만 믿고
내려놓으면 그만큼 띄워 떨어뜨리게 되고, 원통은 굴러떨어진다. 그래서 들어올린 뒤
`ee_pose` 와 `scene_objects` 를 함께 읽어 **손끝과 물체 중심의 실제 차이**를 구하고,
놓을 좌표를 x·y·z 모두 그만큼 보정한다.

**치수는 타입마다 순서가 다르다** (`shape_msgs/SolidPrimitive` 규약).

| 타입 | `dimensions` | 반높이 |
|---|---|---|
| `box` | `[x, y, z]` | `z / 2` |
| `cylinder` | **`[높이, 반지름]`** | `높이 / 2` |
| `sphere` | `[반지름]` | `반지름` |

뒤집어 읽어도 값이 나오고 크래시하지 않는다 — 물체가 엉뚱한 높이에 놓일 뿐이라
실행해 봐서는 안 갈린다. 그래서 `half_height_of` 에 모아 두고 테스트로 고정했다.

**`scene_objects` 변수와 `reset_scene` 의 출력은 모양이 다르다.** 쌓기에는 `type` 과
`dimensions` 가 필요해 변수 쪽을 읽는다 — `reset_scene` 출력에는 치수가 없다.

```
reset_scene outputs.objects   리스트    [{'name': ..., 'position': {...}}, ...]
scene_objects 변수            딕셔너리  {'objects': {이름: {'type', 'dimensions',
                                                          'pose': {'position', ...}}}}
```

**놓았다는 것이 얹혔다는 뜻은 아니다.** `place` 의 성공은 팔이 움직였다는 것까지이므로,
끝나고 `verify_stacked` 로 두 물체의 높이차와 가로 어긋남을 확인한다.

> **물리가 있는 백엔드가 필요하다.** mock 은 물체가 서로 얹히지 않아 확인할 수 없다.
> 그래서 기본 접속 대상이 `robot-twin-pick`(8801 / `panda01`) 과 달리
> **8802 / `panda_isaac`** 이다.

라이브러리로 쓸 때는 다음과 같다.

```python
from robot_twin_client import RobotTwinClient, reset_and_stack, stack, verify_stacked

twin = RobotTwinClient(host='127.0.0.1', port=8802, twin_id='panda_isaac')

# 배치부터 하고 쌓는다.
if reset_and_stack(twin, 'block_a', 'cylinder_a', scene='two_objects', seed=7):
    print(verify_stacked(twin, 'block_a', 'cylinder_a'))

# 이미 놓인 것 위에 이어서 쌓을 때는 stack 을 직접 쓴다 — 리셋이 없어야 한다.
stack(twin, 'block_b', 'cylinder_a')
```

## MCP 서버

LLM 에이전트가 프로그램을 짜지 않고 대화만으로 로봇 상태를 확인하고 연산을 조합할
수 있게 한다. *"박스를 반시계 50도로 옮겨줘"* 같은 요청을 에이전트가 도구 조합으로
푼다.

**등록은 한 번, 기동은 매번이다.**

```
[한 번만]  ./sbin/register_mcp_server.sh
[매번]     로봇 스택 → 트윈 → Claude Code 에서 /mcp 확인
```

```bash
# 터미널 A — 로봇 스택 (rdfp_ws)
rdfp_env                                       # ROS 환경 + overlay + 워크스페이스로 cd
ros2 launch robot_control panda_mock.launch.py

# 터미널 B — 트윈 (rdfp_ws)
rdfp_env
./scripts/run_robot_twin.sh
```

`rdfp_env` 는 rdfp_ws 머신의 옵트인 셸 함수다(`~/.bashrc` 가 정의만 한다). 켜지 않고
`install/setup.bash` 만 하면 `ros2` 는 돌지만 `ROS_DOMAIN_ID` 가 0 / RMW 가 fastrtps
로 떠서, 트윈과 스택이 서로를 보지 못한다.

MCP 서버는 **띄우지 않는다** — Claude Code 가 세션마다 알아서 spawn 한다. 도구
목록을 트윈 카탈로그에서 가져오므로 **트윈이 먼저 떠 있어야** 하고, 순서가 뒤바뀌면
`tools fetch failed` 로 잡힌다(그 상태면 `/mcp` 에서 재연결).

```bash
uv sync --extra mcp
./sbin/check_mcp_server.sh            # 배선 점검 — 서버 기동 + 도구 목록
```

인자 없이 실행하면 `initialize` → `tools/list` 를 한 번 주고받아 **서버·트윈·도구 목록까지
한 번에 확인**한다. 트윈이 꺼져 있으면 도구 조회 단계에서 `Connection refused` 로
멈추므로 어디가 끊겼는지 바로 보인다.

stdio 트랜스포트이므로 보통 직접 실행하지 않고 MCP 클라이언트에 등록한다. 등록
스크립트를 쓴다.

```bash
./sbin/register_mcp_server.sh                      # 기본 (user scope — 모든 프로젝트)
./sbin/register_mcp_server.sh --scope local        # 이 프로젝트에서만

# 두 번째 트윈. **`--twin-id` 를 빠뜨리지 않는다** — 포트만 바꾸면 트윈 id 가 기본값
# `panda01` 로 남아 URL 이 /api/v1/robot_twins/panda01 이 되고, 도구 목록을 가져올 때
# `HTTP 404` 로 실패한다. 등록 자체는 성공하므로 증상이 그 404 뿐이다.
./sbin/register_mcp_server.sh --name robot-twin-isaac \
    --port 8802 --twin-id panda_isaac
```

**기본이 `user` scope 인 이유** — `local` 은 등록할 때의 프로젝트 디렉터리에서만
잡힌다. 로봇 작업은 보통 `rdfp_ws` 쪽에서 하므로, `local` 로 등록하면 정작 거기서
도구가 보이지 않는다.

#### 등록 해제

**`--remove` 는 `--name` 이 가리키는 것 하나만 지운다.** 이름을 안 주면 기본값인
`robot-twin` 이 대상이므로, 다른 이름으로 등록해 둔 것은 그대로 남는다.

```bash
./sbin/register_mcp_server.sh --remove                          # robot-twin (기본 이름)
./sbin/register_mcp_server.sh --name robot-twin-isaac --remove  # 이름을 지정해서
```

**`--port` / `--twin-id` 는 해제에 아무 영향이 없다.** 등록할 때 쓰는 인자라
`--port 8802 --remove` 로 써도 지워지는 것은 여전히 `robot-twin` 이다. 대상은
**이름뿐**이다.

없는 이름을 주면 조용히 아무것도 하지 않는다 — `removed existing registration: <이름>`
이 안 찍혔다면 그 이름이 없었던 것이다. 무엇이 지워질지 미리 보려면 등록 내용을
확인한다 (커맨드 인자에 포트와 트윈 id 가 박혀 있다).

```bash
claude mcp list                    # 등록된 이름 목록
claude mcp get robot-twin          # 그 이름이 어느 포트·트윈에 붙어 있는지
```

스크립트는 **덮어쓰기**다 — 같은 이름이 있으면 지우고 다시 넣으므로 인자를 바꿔
재실행하면 그대로 반영된다. `uv sync --extra mcp` 도 함께 수행하고, `uv` 를 **절대
경로로 박는다** (Claude Code 가 서버를 spawn 할 때의 PATH 가 이 셸과 다를 수 있고,
GUI 클라이언트는 로그인 셸 PATH 를 보지 못한다).

손으로 등록한다면 다음과 같다.

```bash
claude mcp add robot-twin -- "$(command -v uv)" --directory ~/development/mdtpy/robot-twin-client \
    run --extra mcp robot-twin-mcp
```

> 등록 직후 `claude mcp list` 가 `tools fetch failed` 로 보이면 **트윈이 안 떠
> 있는 것**이다 — 도구 목록을 트윈 카탈로그에서 가져오기 때문이며, 등록 자체는
> 정상이다. `./sbin/check_mcp_server.sh` 로 같은 것을 더 자세히 볼 수 있다.

### 펑션베이 로봇에 붙기

펑션베이는 mock 과 **세 가지가 다르다** — 시뮬레이터가 Windows 쪽에 있고, 트윈 포트가
`8803`, 트윈 이름이 `panda_functionbay` 다.

#### 기동 — 매번 (아래에서 위로)

```bash
# 0. 시뮬레이터 (Windows) — MetaMotionRobot + SolverTCPServer 가 떠 있어야 한다

# 터미널 A — 로봇 스택 (rdfp_ws)
rdfp_env
ros2 launch robot_control panda_functionbay.launch.py

# 터미널 B — 트윈 :8803 (rdfp_ws)
rdfp_env
./scripts/run_robot_twin.sh --functionbay
```

⚠️ **시뮬레이터를 *프로세스째* 재시작했으면 터미널 A 도 다시 띄운다.** 앱 내 재시작은
시계가 이어지지만 프로세스 재시작은 0 으로 되감겨, 살아남은 노드의 TF 버퍼가 새 데이터를
`TF_OLD_DATA` 로 버린다. 증상은 **"도구는 성공하는데 팔이 안 움직인다"** 다
(`rdfp_ws/scripts/kill_stack.sh --ros` 후 재기동).

#### 등록 — 한 번만

**`--port 8803 --twin-id panda_functionbay` 둘 다 필수다.** 서버 기본값이
`8801` / `panda01`(mock)이라, 포트만 주면 `HTTP 404` 로 도구 조회가 실패하고 둘 다 빼면
**mock 트윈에 조용히 붙는다.**

**`claude` CLI 가 있으면** 위의 등록 스크립트를 쓴다. **mock 등록과 이름을 나눈다** —
같은 이름이면 덮어써진다.

```bash
./sbin/register_mcp_server.sh --name robot-twin-fb --port 8803 --twin-id panda_functionbay
```

**VSCode 확장처럼 `claude` 가 PATH 에 없으면** 둘 중 하나다 —
`./sbin/install_claude_cli.sh` 로 CLI 를 깔거나, 작업 워크스페이스 루트에 `.mcp.json` 을
직접 둔다(Claude Code 가 프로젝트 범위 서버로 읽는다). `rdfp_ws` 에는 이미 있다.

```jsonc
// ~/development/ros/rdfp_ws/.mcp.json
{ "mcpServers": {
    "robot-twin": {
      "command": "/home/kwlee/development/mdtpy/robot-twin-client/.venv/bin/robot-twin-mcp",
      "args": ["--port", "8803", "--twin-id", "panda_functionbay"] } } }
```

`.mcp.json` 은 **`uv run` 이 아니라 venv 콘솔 스크립트**를 가리킨다. `uv run` 은 기동마다
의존성을 확인하느라 느려 클라이언트가 타임아웃으로 읽을 수 있다. 대신 venv 는 저절로
따라오지 않으므로 **의존성을 바꾸거나 저장소를 옮기면 `uv sync --extra mcp` 를 다시
돌린다** — 안 돌리면 `ENOENT` 로 연결조차 안 된다(저장소 이름이 `robot-twin` →
`robot-twin-client` 로 바뀐 직후 실제로 그랬다).

⚠️ **user scope 등록(`~/.claude.json`)과 `.mcp.json` 에 같은 이름을 두지 않는다** — 어느
쪽이 이기는지를 기억에 의존하게 된다.

⚠️ **`.mcp.json` 이나 서버 코드를 고쳐도 돌고 있는 서버는 안 바뀐다.** 확장이 시작할 때
spawn 하므로 창을 다시 로드한다(`Developer: Reload Window`). 고친 줄 알고 옛 서버로
호출해 `move_linear` 이 10 cm 어긋난 곳으로 간 적이 있다.

#### 확인

```bash
./sbin/check_mcp_server.sh --port 8803 --twin-id panda_functionbay   # 도구 21개면 정상
```

Claude Code 에서는 `/mcp` 에 보이면 된다. peg 작업은 `peg_in_hole` 프롬프트와 도구
넷(`peg_status` · `pick_peg` · `place_peg` · `move_peg`)으로 한다 — 위
[peg-in-hole](#peg-in-hole--robot-twin-peg) 절.

⚠️ **탁자에 내려놓는 지시는 안 된다.** 시뮬레이터 씬에 `peg ↔ 탁자` 접촉이 없어, 고정물
밖에서 놓으면 물체가 탁자를 통과해 떨어지고 복구는 시뮬레이터 재시작뿐이다
(rdfp_ws 벤더 요청 B-14).

| 증상 | 원인 |
|---|---|
| `/mcp` 에 없음 · `ENOENT` | 창 리로드 안 함 / `uv sync --extra mcp` 안 함 |
| `tools fetch failed` · `Connection refused` | 트윈(:8803)이 안 떠 있음 |
| `HTTP 404` | `--twin-id` 빠짐 — 포트만 줬다 |
| 도구는 성공하는데 팔이 안 움직임 | ROS 스택 — 시뮬레이터 프로세스 재시작 뒤 스택을 안 띄움 |

### sbin/

| 스크립트 | 하는 일 |
|---|---|
| `register_mcp_server.sh` | Claude Code 에 등록/해제. 덮어쓰기라 재실행해도 된다 |
| `check_mcp_server.sh` | 배선 점검 — 서버·트윈·도구 목록을 한 번에 확인 |
| `install_claude_cli.sh` | `claude` CLI 설치. PATH 에 없을 때 등록보다 먼저 |

`check_mcp_server.sh --serve` 는 클라이언트가 하는 spawn 을 손으로 흉내내는 모드다.
**표준입력에서 JSON-RPC 를 기다리며 멈춘 것처럼 보이는데** 고장이 아니라 stdio 서버의
정상 동작이며, 사람이 쓸 일은 거의 없다.

### 도구는 트윈 카탈로그에서 자동 생성된다

`GET /operations` 의 `inputs_schema` 가 이미 JSON Schema 이고 `description` 도 트윈
설정에 있으므로, **트윈 YAML 에 연산을 추가하면 MCP 도구가 저절로 늘어난다.** 손으로
유지하는 중복 목록이 없다. 다만 그대로 내보내면 안 되는 것이 있어 예외를 둔다.

| 분류 | 대상 | 이유 |
|---|---|---|
| **검증 래퍼로 교체** | `move_linear`, `move_to_named_target` | 트윈의 `COMPLETED` 는 도달을 보장하지 않는다. raw 를 노출하면 에이전트가 거짓 성공 위에 다음 동작을 쌓는다 |
| **감춤** | `move_to_pose`, `move_gripper` | 미구현 — 호출하면 실패한다 |
| **감춤** | `start_session`, `start_episode` | 여는 경로를 `begin_task` 하나로 좁혀 짝을 보장한다. 닫는 `stop_*` 은 복구 수단으로 남긴다 |
| **추가** | 상태 읽기 · 기하 계산 · 작업 경계 | 카탈로그에 없는 것들 |

### 추가 도구

| 도구 | 하는 일 |
|---|---|
| `read_state` / `read_states` / `list_state_variables` | 상태 변수 조회. `quality` 를 그대로 넘겨 에이전트가 낡은 값을 구분한다 |
| `get_resource_status` | `RESOURCE_BUSY` 로 거부됐을 때 무엇이 잡고 있는지 |
| `pose_above` / `pose_rotated_about_base_z` / `grasp_pose_of_object` | **좌표 계산.** 로봇을 움직이지 않는다 |
| `begin_task` / `end_task` | 씬 리셋 + 세션/에피소드 경계 |
| `emergency_stop` / `release_emergency_stop` | 비상 정지 |
| `peg_status` / `pick_peg` / `place_peg` / `move_peg` | **peg-in-hole 절차.** 좌표를 만들지 않는다 — 보정·판정이 도구 안에서 일어난다 |

**기하 계산을 도구로 여는 이유는 쿼터니언을 에이전트에게 맡기면 조용히 틀리기
때문이다.** xyzw/wxyz 순서 하나만 어긋나도 예외 없이 엉뚱한 자세로 간다. "50도
회전한 자리"는 `pose_rotated_about_base_z` 가 계산한다.

### 작업 경계는 서버가 소유한다

에이전트는 호출 사이에 상태를 잃어버리므로, 에피소드를 열고 닫는 것을 잊으면 다음
작업이 막힌다. 그래서 여는 경로를 `begin_task` 로 좁히고 서버가 짝을 보장한다.

```
begin_task ─ reset_scene ─ start_session ─ start_episode
                                              (에이전트가 작업)
end_task   ─ stop_episode ─ stop_session      ← 한쪽이 실패해도 나머지를 시도
```

- **`begin_task` 가 돌려준 `objects` 가 실제 배치**다. 집을 좌표는 여기서 가져온다.
- 그 배치를 서버가 기억했다가 `end_task` 의 `metadata` 에 **자동으로 합친다** —
  에이전트가 되돌려 주기를 기대하면 잊는 순간 재현 불가능한 에피소드가 남는다.
- 닫지 않은 채 새 작업을 시작하면 이전 것을 `failure` 로 닫고 경고한다. 연결이
  끊겨도 종료 경로에서 닫는다.

### 오류는 복구 방법과 함께 돌아온다

오류 메시지가 에이전트의 유일한 피드백 채널이라, 코드만 던지면 같은 호출을 그대로
재시도한다. `RESOURCE_BUSY` → "`get_resource_status` 로 확인하라",
`PRECONDITION_FAILED` → "`session_state` 를 확인하라" 처럼 다음 행동을 붙인다.
도구 실패는 예외가 아니라 `is_error` 결과로 돌아가므로 세션이 끊기지 않는다.

> **팔 이동은 수십 초가 걸린다.** 블로킹 HTTP 호출을 스레드로 내리므로 이동 중에도
> 상태 조회가 응답한다. 다만 MCP 클라이언트 쪽 타임아웃은 별개이니, 긴 연산이 잦으면
> 클라이언트 타임아웃을 늘린다.

## 테스트

```bash
uv run pytest
```

ROS 를 source 한 쉘에서는 `PYTHONPATH` 를 타고 ROS 의 pytest 플러그인이 venv 안까지
자동 로드되어 충돌한다. 그럴 때는 다음으로 실행한다.

```bash
uv run --env-file .env pytest      # 또는 쉘에 export UV_ENV_FILE=.env
```

## 알려진 한계 (설명서와 동일)

- **자원 경합 재시도가 없다.** `409 RESOURCE_BUSY` 가 그대로 `TwinError` 로 올라와
  시퀀스가 중단된다. 경합이 잦다면 설명서 5.3 의 `run_with_retry` 로 감싼다.
- **중단 시 복구 책임이 클라이언트에 있다.** 중간에 실패하면 로봇이 어떤 자세로
  남는지 트윈은 모른다. 단계가 늘어나면 클라이언트 조합이 아니라 트윈의 복합
  연산(설명서 8.3)으로 옮기는 것이 맞다.
- **`move_to_pose` 가 미구현**이라 접근 지점까지도 직선(`move_linear`)으로 간다.
