"""`python -m robot_twin_client` 진입점.

`-m robot_twin_client.pick_n_place` 로 직접 실행하면 `__init__` 이 이미 임포트해 둔 모듈을
runpy 가 `__main__` 으로 한 번 더 실행해 두 벌이 로드된다 (RuntimeWarning). 여기서는
이미 로드된 모듈의 `main` 을 가져다 쓰므로 재실행이 없다.
"""

from __future__ import annotations

from .pick_n_place import main


if __name__ == '__main__':
    raise SystemExit(main())
