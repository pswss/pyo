"""회전 중 라이다 맵핑 게이트 배선 테스트.

버그: 제자리 회전(~4.5°/스텝) 중 heading이 스텝 경계 값이라 라이다 끝점이 호 모양으로
번져 벽 쓰레기가 영구 누적됐다(클리어링 OFF). 오프라인 재현: 회전 스캔 포함 시
벽 픽셀 +53%, precision 1.0→0.82.

수정: do_mapping이 init 외 상태에서 스텝당 회전량 > max_mapping_angular_velocity(2°)면
라이다를 빼고 카메라만 업데이트한다. init의 의도적 360° 스캔은 게이트하지 않는다.

Executor.__new__ + 가짜 협력자 배선 패턴 (test_stuck_wiring.py와 동일).
"""
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

_fake_controller = types.ModuleType("controller")
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
    def __call__(self, *a, **k): return _Any()
_fake_controller.Robot = _Any
_fake_controller.__getattr__ = lambda name: _Any
sys.modules["controller"] = _fake_controller

from data_structures.angle import Angle
from executor.executor import Executor
from flow_control.state_machine import StateMachine


def make_executor(state, angular_velocity_deg, shaky=False):
    """do_mapping이 의존하는 협력자만 배선한다."""
    ex = Executor.__new__(Executor)
    ex.mapping_enabled = True
    ex.max_mapping_angular_velocity = Angle(2, Angle.DEGREES)

    sm = StateMachine(state)
    ex.state_machine = sm

    calls = types.SimpleNamespace(full=0, camera_only=0)

    def _update(*args, **kwargs):
        # 라이다 포인트클라우드가 위치 인자로 오면 풀 업데이트, kwargs만이면 카메라 전용
        if args:
            calls.full += 1
        else:
            calls.camera_only += 1

    ex.mapper = types.SimpleNamespace(update=_update)
    ex.robot = types.SimpleNamespace(
        is_shaky=lambda: shaky,
        gyroscope=types.SimpleNamespace(
            get_angular_velocity=lambda: Angle(angular_velocity_deg, Angle.DEGREES)),
        get_point_cloud=lambda: [],
        get_out_of_bounds_point_cloud=lambda: [],
        get_lidar_detections=lambda: [],
        get_camera_images=lambda: None,
        position=None, orientation=None, time=0.0,
    )
    ex._calls = calls
    return ex


def test_explore_rotating_skips_lidar():
    ex = make_executor("explore", angular_velocity_deg=4.5)
    ex.do_mapping()
    assert ex._calls.camera_only == 1 and ex._calls.full == 0, vars(ex._calls)


def test_explore_straight_uses_lidar():
    ex = make_executor("explore", angular_velocity_deg=0.5)
    ex.do_mapping()
    assert ex._calls.full == 1 and ex._calls.camera_only == 0, vars(ex._calls)


def test_init_scan_not_gated():
    """init의 의도적 360° 회전 스캔은 라이다 맵핑 유지."""
    ex = make_executor("init", angular_velocity_deg=4.5)
    ex.do_mapping()
    assert ex._calls.full == 1 and ex._calls.camera_only == 0, vars(ex._calls)


def test_shaky_still_skips_lidar():
    """기존 흔들림 게이트는 회전 게이트와 독립적으로 동작."""
    ex = make_executor("explore", angular_velocity_deg=0.5, shaky=True)
    ex.do_mapping()
    assert ex._calls.camera_only == 1 and ex._calls.full == 0, vars(ex._calls)


if __name__ == "__main__":
    test_explore_rotating_skips_lidar()
    test_explore_straight_uses_lidar()
    test_init_scan_not_gated()
    test_shaky_still_skips_lidar()
    print("ALL PASS")
