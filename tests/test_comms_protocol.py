"""게임매니저 'G'(게임정보) 패킷 파싱 프로토콜 테스트.

버그(v26 회귀): 매니저는 'c f i i'(G, 점수, 게임시간, 실제시간)로 전송하는데
로봇은 'c f i'로 언팩 → 버퍼 크기 불일치로 struct.error → 점수/남은시간 갱신 불가.

매니저 송신 포맷 ground truth:
  erebus-26.0.1/game/controllers/MainSupervisor/MainSupervisor.py:712  struct.pack("c f i i", ...)
"""
import os
import sys
import types
import struct

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

# Webots 'controller' 모듈 stub
_fake = types.ModuleType("controller")
class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
_fake.Robot = _Any
_fake.__getattr__ = lambda n: _Any
sys.modules["controller"] = _fake

from robot.devices.comunicator import Comunicator

# 매니저 v26 송신 포맷 (MainSupervisor.py:712)
MANAGER_G_FORMAT = "c f i i"


class FakeReceiver:
    def __init__(self, packets):
        self.packets = list(packets)
    def enable(self, ts): pass
    def getQueueLength(self): return len(self.packets)
    def getBytes(self): return self.packets[0]
    def nextPacket(self):
        if self.packets:
            self.packets.pop(0)


class FakeEmitter:
    def send(self, *a, **k): pass


def test_parses_v26_game_data_packet():
    # 매니저가 보내는 v26 포맷 패킷을 로봇이 정상 파싱해야 함
    pkt = struct.pack(MANAGER_G_FORMAT, b"G", 42.5, 300, 250)
    c = Comunicator(FakeEmitter(), FakeReceiver([pkt]), 32)
    c.update()
    assert c.game_score == 42.5, f"점수 파싱 실패: {c.game_score}"
    assert c.remaining_time == 300, f"남은시간 파싱 실패: {c.remaining_time}"


def test_old_format_would_break():
    # 회귀 방지: 옛 'c f i' 포맷으로는 매니저 패킷을 언팩할 수 없음을 증명
    pkt = struct.pack(MANAGER_G_FORMAT, b"G", 1.0, 2, 3)
    try:
        struct.unpack("c f i", pkt)
        assert False, "'c f i'로 v26 패킷이 언팩되면 안 됨 (버그 재현 실패)"
    except struct.error:
        pass


if __name__ == "__main__":
    test_parses_v26_game_data_packet()
    test_old_format_would_break()
    print("ALL PASS")
