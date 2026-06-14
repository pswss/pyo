"""SubagentPriorityCombiner 플래핑 방지(히스테리시스) 테스트.

실런 로그에서 확인된 버그: FollowWalls 후보가 프레임마다 생겼다 사라져
4~6프레임마다 서브에이전트 전환 → 목표 좌표 널뜀 → 직진 불가 →
GPS heading 보정 게이트(연속직진 8프레임) 영영 미달 → 자이로 드리프트 누적 → 맵 휘어짐.

요구 동작:
- 승급(상위 우선순위로 전환): 상위 에이전트가 K프레임 연속 목표 유지할 때만.
- 강등(현재 에이전트 목표 소실): 즉시 하위로 전환.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from agent.agent import SubagentPriorityCombiner


class StubAgent:
    """스크립트된 target_position_exists 시퀀스를 재생하는 스텁."""
    def __init__(self, name, script):
        self.name = name
        self.script = list(script)  # bool 목록, 프레임마다 하나 소비
        self.exists_now = False
        self.update_calls = 0

    def update(self, force_calculation=False):
        self.update_calls += 1
        if self.script:
            self.exists_now = self.script.pop(0)

    def target_position_exists(self):
        return self.exists_now

    def get_target_position(self):
        return (0, 0)


def run_frames(combiner, agents, n):
    """n프레임 실행, 선택된 에이전트 이름 시퀀스 반환."""
    selected = []
    for _ in range(n):
        combiner.update()
        idx = combiner.current_agent_index
        selected.append(agents[idx].name)
    return selected


def count_switches(selected):
    return sum(1 for a, b in zip(selected, selected[1:]) if a != b)


def test_flapping_higher_agent_does_not_cause_switches():
    """상위 에이전트가 True/False 깜빡여도(실런 재현) 전환 없어야 함."""
    flicker = [True, False] * 30  # 프레임마다 깜빡
    high = StubAgent("high", flicker)
    low = StubAgent("low", [True] * 60)
    combiner = SubagentPriorityCombiner([high, low], switch_debounce_frames=8)
    # 첫 프레임은 high가 True라 high로 시작할 수 있음 → 이후 깜빡임에 전환 횟수 측정
    selected = run_frames(combiner, [high, low], 60)
    # 초기 안정화(앞 10프레임) 이후엔 전환 0회
    assert count_switches(selected[10:]) == 0, f"플래핑 전환 발생: {selected}"


def test_sustained_higher_agent_promotes_after_debounce():
    """상위 에이전트가 연속으로 목표 유지하면 디바운스 후 승급."""
    high = StubAgent("high", [False] * 10 + [True] * 50)
    low = StubAgent("low", [True] * 60)
    combiner = SubagentPriorityCombiner([high, low], switch_debounce_frames=8)
    selected = run_frames(combiner, [high, low], 60)
    assert selected[5] == "low"        # 초반엔 low
    assert selected[-1] == "high"      # 충분히 지속되면 high로 승급
    assert count_switches(selected) == 1  # 전환은 딱 1회


def test_current_agent_loses_target_demotes_immediately():
    """현재 에이전트 목표 소실 시 즉시 하위로 강등 (디바운스 없음)."""
    high = StubAgent("high", [True] * 20 + [False] * 40)
    low = StubAgent("low", [True] * 60)
    combiner = SubagentPriorityCombiner([high, low], switch_debounce_frames=8)
    selected = run_frames(combiner, [high, low], 60)
    assert selected[10] == "high"
    # high 소실 직후 1~2프레임 내 low로
    idx_lost = 20
    assert "low" in selected[idx_lost:idx_lost + 2], f"강등 지연: {selected[18:26]}"


if __name__ == "__main__":
    test_flapping_higher_agent_does_not_cause_switches()
    test_sustained_higher_agent_promotes_after_debounce()
    test_current_agent_loses_target_demotes_immediately()
    print("OK")
