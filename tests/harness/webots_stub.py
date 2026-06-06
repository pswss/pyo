"""Webots 'controller' 모듈 스텁 — Webots 밖에서 src/ 코드를 import하기 위함.

사용법 (모든 하니스 테스트 첫 줄):
    from harness.webots_stub import install_stub
    install_stub()        # 이후 src/ import 가능
"""
import os
import sys
import types


def install_stub():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

    if "controller" in sys.modules:
        return

    fake = types.ModuleType("controller")

    class _Any:
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, n):
            return _Any()

    fake.Robot = _Any
    fake.__getattr__ = lambda n: _Any
    sys.modules["controller"] = fake
