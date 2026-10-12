"""로봇 트윈 HTTP 클라이언트와 pick-and-place 시퀀스 예제.

`rdfp` 워크스페이스의 `docs/twin/robot_twin_user_guide.md` 5.9 절 예제를
독립 프로젝트로 옮긴 것이다. 트윈은 HTTP 로만 접근하므로 ROS 가 필요 없다.

`mcp_server` 는 **여기서 import 하지 않는다** — `mcp` extra 를 설치하지 않은 환경에서
`import robot_twin_client` 자체가 실패하기 때문이다. MCP 서버가 필요하면
`from robot_twin_client.mcp_server import ...` 로 직접 가져온다.
"""

from __future__ import annotations

from .client import OperationFailed, RobotTwinClient, TwinError
from .ops import move_linear_verified, move_to_named_target_verified, set_gripper
from .pick_n_place import (
    DOWN, TCP_OFFSET_M, above_of, center_of, grasp_pose_of, pick, place,
    reset_and_pick_and_place, return_home, rotated_about_base_z,
    tcp_from_arm_pose, to_arm_command
)
from .stack import (
    half_height_of, held_offset, reset_and_stack, scene_object, stack, stack_pose_of,
    verify_stacked
)
from .collect import collect, collect_episode

__version__ = '0.1.0'

__all__ = ['DOWN', 'OperationFailed', 'RobotTwinClient', 'TCP_OFFSET_M', 'TwinError',
           'above_of', 'center_of', 'collect', 'collect_episode',
           'grasp_pose_of', 'half_height_of', 'held_offset',
           'move_linear_verified', 'move_to_named_target_verified',
           'pick', 'place', 'reset_and_pick_and_place', 'return_home',
           'reset_and_stack', 'rotated_about_base_z', 'scene_object', 'set_gripper', 'stack',
           'stack_pose_of', 'tcp_from_arm_pose', 'to_arm_command', 'verify_stacked']
