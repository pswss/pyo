from data_structures.angle import Angle
from data_structures.vectors import Position2D

from flow_control.sequencer import Sequencer
from flow_control.state_machine import StateMachine
from flow_control.delay import DelayManager
from flow_control.step_counter import StepCounter

from executor.stuck_detector import StuckDetector

from robot.robot import Robot
from robot.drive_base import Criteria as RotationCriteria

from mapping.mapper import Mapper

from agent.agent import Agent

from fixture_detection.fixture_clasification import FixtureClasiffier

from final_matrix_creation.final_matrix_creator import FinalMatrixCreator

from utilities import ColorFilterTuner
from fixture_detection.color_filter import ColorFilter
from fixture_detection.non_fixture_filterer import NonFixtureFilter

from flags import DO_SLOW_DOWN, SLOW_DOWN_S, TUNE_FILTER

import math
import time
import numpy as np
import cv2 as cv
import skimage

class Executor:
    """로봇의 모든 모듈(센서, 모터, 매핑, 길찾기)을 조율하고 전체 미션의 흐름을 제어하는 총괄 클래스입니다."""
    
    def __init__(self, mapper: Mapper, robot: Robot) -> None:
        # --- 1. 핵심 하위 모듈 초기화 ---
        self.agent = Agent(mapper)       # 어디로 가야 할지(목표 좌표)를 계산하는 행동 대장
        self.mapper = mapper             # 지도를 그리는 역할
        self.robot = robot               # 실제 모터와 센서를 다루는 하드웨어 제어기

        self.delay_manager = DelayManager()     # 시간 지연(Delay)을 비동기적으로 관리 (Webots 화면 멈춤 방지)
        self.stuck_detector = StuckDetector()   # 로봇이 벽에 끼어서 헛돌고 있는지 감지
        self.consecutive_stuck_escapes = 0      # 같은 자리 wiggle 탈출 연속 시도 횟수
        self.max_stuck_escapes = 3              # 이 횟수만큼 wiggle 실패하면 LoP 신고(순간이동)로 전환
        self.last_stuck_position = None        # 직전 끼임 위치 (같은 자리 반복 판정용)
        self.stuck_same_spot_radius = 0.15       # 이 반경(m) 내 재끼임은 '같은 자리'로 간주. ★시뮬 튜닝

        # --- 2. 상태 머신 (State Machine) 설정 ---
        # 로봇이 현재 어떤 기분/상태인지 정의하고, 상태 간의 전환(Transition)을 관리합니다.
        self.state_machine = StateMachine("init") 
        # create_state("상태이름", 실행할함수, {이동가능한다음상태들})
        self.state_machine.create_state("init", self.state_init, {"explore",}) 
        self.state_machine.create_state("explore", self.state_explore, {"end", "report_fixture", "send_map", "stuck", "recalibrate", "avoid_hole"})
        self.state_machine.create_state("recalibrate", self.state_recalibrate, {"explore", "send_map"})
        self.state_machine.create_state("avoid_hole", self.state_avoid_hole, {"explore", "send_map"})
        self.state_machine.create_state("end", self.state_end)
        self.state_machine.create_state("report_fixture", self.state_report_fixture, {"explore", "send_map"})
        self.state_machine.create_state("send_map", self.state_send_map, {"explore", "end"})
        self.state_machine.create_state("stuck", self.state_stuck, {"explore", "send_map", "end"})

        # --- 3. 시퀀서 (Sequencer) 설정 ---
        # 로봇에게 "A하고 -> 1초 쉬고 -> B해라" 같은 연속 동작을 지시할 때 사용합니다.
        self.sequencer = Sequencer(reset_function=self.delay_manager.reset_delay) 

        self.fixture_detector = FixtureClasiffier() # 카메라 비전 기반 조난자(알파벳) 인식기

        self.final_matrix_creator = FinalMatrixCreator(mapper.tile_size, mapper.pixel_grid.resolution)
        self.reported_fixtures = []  # 최종 맵 행렬용: 보고한 토큰 (위치, 글자) 누적

        # --- 4. 플래그(상태 변수) 및 시퀀서용 함수 매핑 ---
        self.mapping_enabled = False          # 맵핑을 할지 말지 결정
        self.victim_reporting_enabled = False # 조난자 탐지를 할지 말지 결정
        self.fixture_detection_cooldown = 0  # 오탐 후 감지 억제 카운터 (>0이면 감지 스킵)

        # 자주 쓰는 함수들을 시퀀서에서 쉽게 부를 수 있게 래핑(Wrapping)합니다.
        self.seq_print =                   self.sequencer.make_simple_event( print)
        self.seq_move_wheels =             self.sequencer.make_simple_event( self.robot.move_wheels)
        self.seq_rotate_to_angle =         self.sequencer.make_complex_event(self.robot.rotate_to_angle)
        self.seq_rotate_slowly_to_angle =  self.sequencer.make_complex_event(self.robot.rotate_slowly_to_angle)
        self.seq_move_to_coords =          self.sequencer.make_complex_event(self.robot.move_to_coords)
        self.seq_delay_seconds =           self.sequencer.make_complex_event(self.delay_manager.delay_seconds)
        self.seq_align_with_fixture =      self.sequencer.make_complex_event(self.align_with_fixture)

        self.letter_to_report = None          # 발견한 조난자 글자 저장
        self.report_orientation = Angle(0)    # 조난자가 있는 방향

        self.max_time_in_run = 8 * 60         # 대회 제한 시간 (8분)
        self.map_sent = False                 # 맵 전송 여부

        self.robot.set_start_time()           # 시작 시간 기록
        self.__real_elapsed = 0.0             # 실제 실행 중에만 누적되는 리얼타임(초)
        self.__last_step_real_time = None     # 직전 스텝의 time.time() 값

        self.mini_calibrate_step_counter = StepCounter(20) # 20스텝마다 한 번씩 캘리브레이션 트리거

        # 회전 중 라이다 맵핑 게이트 임계(스텝당 회전량). 직진 <1°/스텝, 제자리 회전 ~4.5°/스텝.
        # 회전 중에는 heading이 스텝 경계 값이라 라이다 끝점이 호 모양으로 번짐
        # (오프라인 재현: 회전 스캔 포함 시 벽 픽셀 +53%, precision 1.0→0.82). ★시뮬 튜닝
        self.max_mapping_angular_velocity = Angle(2, Angle.DEGREES)

        # 능동 heading 재캘리브레이션 주기(시뮬 초). passive GPS 보정은 목표 노드가 1~3cm
        # 앞이라 직진 구간이 ≤4cm로 짧아 baseline(8cm) 영영 미달 → 4600프레임 보정 0회
        # (실런 계측 확인). explore 중 이 주기마다 멈추고 초기 캘리브와 동일한 짧은
        # 후진/전진 기동으로 자이로를 GPS heading에 재동기화한다. 후진 우선이라 안전
        # (직전에 지나온 길 = 구멍 없음 보장). ★시뮬 튜닝
        self.heading_recalib_interval = 20.0
        self.__last_recalib_time = None

        self.__avoid_hole_target = Angle(0)   # 구멍 회피 180° 회전 목표 (기동 진입 시 갱신)
        self.__swamp_clear_frames = 0         # 늪 비근접 연속 프레임 (센서 복귀 디바운스)

        self.color_filter_tuner = ColorFilterTuner(ColorFilter((0, 0, 29), (0, 0, 137)), TUNE_FILTER)

    def should_recalibrate_heading(self, now) -> bool:
        """재캘리브레이션 주기 도래 여부. 첫 호출은 타이머 시작점만 등록하고 False."""
        if self.__last_recalib_time is None:
            self.__last_recalib_time = now
            return False
        return now - self.__last_recalib_time >= self.heading_recalib_interval

    def mark_heading_recalibrated(self, now):
        """재캘리브레이션 완료 시각 기록 → 타이머 리셋."""
        self.__last_recalib_time = now


    def run(self):
        """메인 무한 루프: 매 프레임(Timestep)마다 실행되며 로봇의 모든 구성요소를 업데이트합니다."""
        
        while self.robot.do_loop():
            # 일시정지 구간은 제외하고 실제 실행 시간만 누적
            self._tick_real_elapsed()

            # 1. 하드웨어 업데이트 (센서값 읽어오기, 현재 위치 갱신)
            self.robot.update()

            # 2. 유틸리티 업데이트 (지연시간, 끼임 감지)
            self.delay_manager.update(self.robot.time)
            # 끼임 감지는 explore에서만 누적. 의도된 정지 상태(보고/재캘리브/wiggle 등)의
            # 이력이 explore 복귀 직후 창 기반 감지를 오발시키지 않도록 그 외 상태에선 리셋.
            if self.state_machine.check_state("explore"):
                self.stuck_detector.update(self.robot.position,
                                           self.robot.previous_position,
                                           self.robot.drive_base.get_wheel_average_angular_velocity())
            else:
                self.stuck_detector.reset()
            
            # 카메라 색상 필터 튜닝
            self.color_filter_tuner.tune(self.robot.center_camera.image.image)

            # 3. 맵 그리기 (로봇이 안정적일 때 센서 데이터를 지도로 변환)
            self.do_mapping()
            
            # 4. 종료 시간 임박 시 지도 전송 체크
            self.check_map_sending()

            # 4-1. 끼임(LoP) 감지 시 탈출 상태로 전환
            self.check_stuck()

            # 4-2. 늪 근처면 방향 센서를 자이로로 전환 (GPS 노이즈 회피).
            #      init 캘리브레이션의 수동 센서 제어와 충돌하지 않도록 탐색 중에만 검사.
            if self.state_machine.check_state("explore"):
                self.check_swamp_proximity()

            # 5. ★ 로봇의 핵심 두뇌 (상태 머신) 실행 ★
            # 현재 상태에 맞춰 state_init, state_explore 등의 함수 중 하나를 실행합니다.
            self.state_machine.run()

            # 디버그용 느리게 보기 모드
            if DO_SLOW_DOWN:
                time.sleep(SLOW_DOWN_S)
            
    def do_mapping(self):
        """맵핑 기능이 켜져 있을 때(True) 지도를 업데이트합니다."""
        if self.mapping_enabled:
                # 회전 중 라이다 제외: 제자리 회전 중 끝점이 호로 번져 벽 쓰레기를 영구
                # 누적시킴(클리어링 OFF라 자가수정 없음). 직진 증거만으로 커버리지 충분
                # (하니스 recall 1.0). init의 의도적 360° 스캔은 게이트하지 않음.
                rotating = (not self.state_machine.check_state("init")) and \
                           self.robot.gyroscope.get_angular_velocity() > self.max_mapping_angular_velocity
                if not self.robot.is_shaky() and not rotating: # 흔들림/회전 없을 때 (정밀도 유지)
                    # 라이다와 카메라를 모두 사용하여 지도 업데이트
                    self.mapper.update(self.robot.get_point_cloud(), 
                                    self.robot.get_out_of_bounds_point_cloud(),
                                    self.robot.get_lidar_detections(),
                                    self.robot.get_camera_images(), 
                                    self.robot.position,
                                    self.robot.orientation,
                                    self.robot.time)
                else:
                    # 흔들릴 때는 라이다를 빼고 카메라 데이터와 로봇 위치만 대략적으로 업데이트
                    self.mapper.update(camera_images=self.robot.get_camera_images(), 
                                       robot_position= self.robot.position,
                                       robot_orientation=self.robot.orientation,
                                       time=self.robot.time)

    def check_stuck(self):
        """로봇이 벽에 끼였으면 탈출을 시도합니다. ('stuck'은 'explore'에서만 진입 가능)

        1차: 'stuck' 상태(물리적 wiggle 탈출)로 전환해 스스로 빠져나옴(무패널티).
        2차: wiggle을 max_stuck_escapes회 반복해도 못 빠져나오면 게임매니저에 LoP를
             신고(send_lack_of_progress)하여 마지막 지점으로 순간이동시킨다(최후 수단, 패널티 감수).
        """
        if not self.state_machine.check_state("explore"):
            return

        if self.stuck_detector.is_stuck():
            # 연속 카운트는 '같은 자리' 기준. wiggle은 후진+제자리턴이라 거의 원위치로 복귀하므로
            # '움직였으니 성공'식 리셋은 같은 막힌 목표로 재돌입하는 한계루프(앞→뒤→턴→원위치 반복)를
            # 영영 못 끊는다(LoP까지 누적이 안 됨). 직전 끼임 위치 근처면 누적, 멀면 1로 새로 시작.
            cur = self.robot.position
            same_spot = (self.last_stuck_position is not None
                         and cur.get_distance_to(self.last_stuck_position) <= self.stuck_same_spot_radius)
            self.consecutive_stuck_escapes = self.consecutive_stuck_escapes + 1 if same_spot else 1
            self.last_stuck_position = Position2D(cur.x, cur.y)
            # 같은 자리 wiggle 반복 실패 → LoP 신고로 순간이동 (한계루프/맵 갉아먹기 차단)
            if self.consecutive_stuck_escapes >= self.max_stuck_escapes:
                print(f"[끼임 감지:executor.check_stuck] ⚠ 같은 자리 wiggle {self.max_stuck_escapes}회 실패 → LoP 신고(순간이동) "
                      f"(위치=({cur.x:.4f},{cur.y:.4f})m, 시뮬 시간={self.robot.time:.1f}s)")
                self.robot.comunicator.send_lack_of_progress()
                self.consecutive_stuck_escapes = 0
                self.last_stuck_position = None
                self.stuck_detector.reset()   # 순간이동 후 즉시 재트리거 방지 (창 이력 포함)
                return
            # 1차: 물리적 wiggle 탈출 시도
            print(f"[끼임 감지:executor.check_stuck] ⚠ 끼임 카운터 임계 초과 → stuck 전환 "
                  f"(같은자리 {self.consecutive_stuck_escapes}/{self.max_stuck_escapes}, "
                  f"위치=({cur.x:.4f},{cur.y:.4f})m, 시뮬 시간={self.robot.time:.1f}s)")
            self.state_machine.change_state("stuck")
            self.sequencer.reset_sequence()
            self.stuck_detector.reset()   # wiggle 후 새 창으로 재평가 (즉시 재트리거 방지)

    def check_swamp_proximity(self):
        """늪지대(Swamp)에 가까워졌는지 확인 (센서 오차 방지용).
        복귀는 연속 30프레임 비근접일 때만 — 경계 2cm에서 매 프레임 토글되면
        센서 모드 채터링으로 heading이 출렁여 늪 경계 왕복 진동을 유발. ★시뮬 튜닝"""
        if self.mapper.is_close_to_swamp():
            self.__swamp_clear_frames = 0
            self.robot.auto_decide_orientation_sensor = False
            self.robot.orientation_sensor = self.robot.GYROSCOPE # 늪지대에서는 GPS 에러가 커지므로 자이로만 사용
        elif not self.robot.auto_decide_orientation_sensor:
            self.__swamp_clear_frames += 1
            if self.__swamp_clear_frames >= 30:
                self.robot.auto_decide_orientation_sensor = True

    def _tick_real_elapsed(self):
        """robot.step()이 반환된 직후 호출: 일시정지 구간을 제외한 실제 실행 시간을 누적합니다."""
        now = time.time()
        if self.__last_step_real_time is not None:
            delta = now - self.__last_step_real_time
            # 한 스텝이 1초를 초과하면 일시정지로 간주하고 누적하지 않음
            if delta < 1.0:
                self.__real_elapsed += delta
        self.__last_step_real_time = now

    def check_map_sending(self):
        """대회 종료 2초 전이 되면, 상태를 강제로 'send_map'으로 바꿔서 지도를 제출하게 합니다.
        플레이타임(시뮬레이션)과 누적 리얼타임 중 더 많이 경과한 쪽을 기준으로 판단합니다.
        일시정지 구간은 리얼타임 누적에서 제외되므로 Pause 중에 오작동하지 않습니다."""
        sim_elapsed  = self.mapper.time
        real_elapsed = self.__real_elapsed
        elapsed = max(sim_elapsed, real_elapsed)
        # 마진 10초: Erebus는 맵을 '1회만' 채점하므로 게임 종료 전 제출 보장이 최우선.
        # 2초 마진은 무거운 프레임/시계 오차로 게임이 먼저 끝나 미제출되는 사례 발생. ★시뮬 튜닝
        if elapsed > self.max_time_in_run - 10 and not self.map_sent:
            print(f"[타임아웃:executor.check_map_sending] ⚠ 제한 시간 임박! 강제 지도 전송 트리거 "
                  f"(시뮬={sim_elapsed:.1f}s / 리얼={real_elapsed:.1f}s / 한계={self.max_time_in_run}s)")
            self.state_machine.change_state("send_map")
            self.sequencer.reset_sequence()

    # =====================================================================
    # 상태 (STATES) 함수들 : StateMachine에 의해 특정 상황에 맞게 호출됨
    # =====================================================================

    def state_init(self, change_state_function):
        """[초기화 상태] 프로그램 시작 시 센서 영점을 잡고 시작 위치를 등록합니다."""
        self.sequencer.start_sequence()
        self.seq_print("[초기화:executor.state_init] ▶ 로봇 초기화 시작: 캘리브레이션 및 초기 스캔")
        self.seq_delay_seconds(0.5)

        self.sequencer.simple_event(self.calibrate_position_offsets) # 시작점 오차 보정
        self.sequencer.simple_event(self.mapper.register_start, self.robot.position) # 매퍼에 시작점 알림

        self.seq_calibrate_robot_rotation() # GPS를 이용해 자이로스코프 각도 영점 조절

        # 맵핑 및 조난자 탐지 기능 활성화
        if self.sequencer.simple_event():
            print("[초기화:executor.state_init] 맵핑 및 조난자 탐지 활성화, 초기 360° 스캔 시작")
            self.mapping_enabled = True
            self.victim_reporting_enabled = True

        self.seq_delay_seconds(0.5)
        # 로봇을 제자리에서 왼쪽으로 90도, 180도 돌리며 주변 환경 스캔(초기 지도 확보)
        self.sequencer.complex_event(self.robot.rotate_to_angle, angle=Angle(90, Angle.DEGREES), direction=RotationCriteria.LEFT)
        self.sequencer.complex_event(self.robot.rotate_to_angle, angle=Angle(180, Angle.DEGREES), direction=RotationCriteria.LEFT)
        self.seq_delay_seconds(0.5)

        # 스캔이 끝나면 '탐색(explore)' 상태로 변경!
        if self.sequencer.simple_event():
            print("[초기화:executor.state_init] ✓ 초기화 완료 → explore 탐색 단계 시작")
            change_state_function("explore")
        self.sequencer.seq_reset_sequence()


    def state_explore(self, change_state_function):
        """[탐색 상태] 실제 미로를 돌아다니며 길을 찾고 미션을 수행하는 메인 모드입니다."""
        # 주기 도래 시 능동 heading 재캘리브레이션 (자이로 드리프트 → 맵 휘어짐 방지)
        if self.should_recalibrate_heading(self.robot.time):
            change_state_function("recalibrate")
            self.sequencer.reset_sequence()
            return

        # 전방 구멍(블랙홀) 감지 시: 웨이포인트가 구멍과 '겹칠 때만' 후진+180° 기동.
        # 안 겹치면 경로계획(occupied 우회)에 맡김 — 무조건 180°는 멀쩡한 웨이포인트도 버림.
        if self.mapper.is_hole_in_front(self.robot.position, self.robot.orientation):
            target = self.agent.get_target_position()
            if target is None or self.mapper.is_hole_between(self.robot.position, target):
                print(f"[구멍회피:executor.state_explore] ⚠ 전방 구멍이 웨이포인트와 겹침 → 회피 기동 "
                      f"(위치=({self.robot.position.x:.3f},{self.robot.position.y:.3f})m)")
                change_state_function("avoid_hole")
                self.sequencer.reset_sequence()
                return

        self.sequencer.start_sequence()

        self.robot.lidar.front_guard_enabled = True   # 탐색 중엔 충돌 가드 ON (보고 후 복원)

        if self.sequencer.simple_event():
            self.mapping_enabled = True

        # --- 1. 길 찾기 ---
        # Agent에게 목표 위치를 다시 계산하도록 업데이트
        self.agent.update()
        # mini_calibrate 호출 제거: 20스텝마다 강제 0.1s 정지·풀스피드 펄스가 주기적
        # 움찔(이동 끊김)의 원인이었음. 자세 교정은 recalibrate 상태가 담당.
        
        # Agent가 계산해준 다음 좌표(목표)를 향해 로봇 이동!
        self.seq_move_to_coords(self.agent.get_target_position())

        self.sequencer.seq_reset_sequence() # 명령을 다 내렸으면 시퀀서 리셋

        # 에이전트가 "탐색 다 끝났어!"라고 하면 종료 상태로 전환
        if self.agent.do_end():
            self.state_machine.change_state("end")

        # --- 2. 조난자(Fixture) 탐색 로직 ---
        cam_images = self.robot.get_camera_images()

        # 오탐 직후 쿨다운 카운트다운 (>0이면 감지 스킵)
        if self.fixture_detection_cooldown > 0:
            self.fixture_detection_cooldown -= 1

        # 아직 맵에 조난자가 안 찍혀 있고, 카메라가 켜져 있고, 쿨다운이 끝났으면 확인
        elif self.victim_reporting_enabled and cam_images is not None and not self.mapper.has_detected_victim_from_position():
            for cam_image in cam_images:
                self.robot.lidar.get_detections()
                # 카메라 이미지에서 구조물(글자) 감지
                fixtures = self.fixture_detector.find_fixtures(cam_image.image)

                if len(fixtures):
                    # 글자(H, S, U 등) 판별
                    detected_letter = self.fixture_detector.classify_fixture(fixtures[0])

                    # None = 이미 처리된(already_detected) fixture → 보고 불필요, 다음 카메라로
                    if detected_letter is None:
                        continue

                    self.letter_to_report = detected_letter
                    self.report_orientation = cam_image.data.horizontal_orientation

                    print(f"[탐색:executor.state_explore] ★ fixture 감지! 글자='{self.letter_to_report}', 방향={self.report_orientation.degrees:.1f}°, 위치=({self.robot.position.x:.3f},{self.robot.position.y:.3f})m → report_fixture 전환")
                    # 상태를 'report_fixture(조난자 보고 모드)'로 변경하고 이번 루프 탈출
                    change_state_function("report_fixture")
                    self.sequencer.reset_sequence()
                    break
                

    def state_avoid_hole(self, change_state_function):
        """[구멍 회피] 전방 블랙홀 감지 시: 정지 → 후진(~4cm) → 180° 회전 → explore 복귀.
        경로계획은 occupied로 구멍을 우회하지만, 코앞 근접 시 명시적 탈출 동작이 없어
        정지/진동하던 것을 능동 기동으로 대체."""
        self.sequencer.start_sequence()

        self.seq_move_wheels(-0.6, -0.6)
        self.seq_delay_seconds(0.5)
        self.seq_move_wheels(0, 0)

        if self.sequencer.simple_event():
            # 후진 완료 시점의 현재 방향 기준 180° 반대편으로
            self.__avoid_hole_target = Angle(self.robot.orientation.radians + math.pi)
            self.__avoid_hole_target.normalize()

        self.seq_rotate_to_angle(self.__avoid_hole_target)

        if self.sequencer.simple_event():
            print(f"[구멍회피:executor.state_avoid_hole] ✓ 후진+180° 완료 → explore 복귀")
            change_state_function("explore")
        self.sequencer.seq_reset_sequence()

    def state_recalibrate(self, change_state_function):
        """[주기 재캘리브레이션] 정지 → 짧은 후진/전진 기동으로 자이로를 GPS heading에
        재동기화. 기동 자체는 초기 캘리브(seq_calibrate_robot_rotation) 재사용."""
        self.sequencer.start_sequence()

        if self.sequencer.simple_event():
            print(f"[재캘리브:executor.state_recalibrate] ▶ heading 재동기화 시작 (시뮬 {self.robot.time:.1f}s)")
            self.mapping_enabled = False   # 기동 중 라이다 맵 오염 방지

        self.seq_move_wheels(0, 0)
        self.seq_delay_seconds(0.2)        # 완전 정지 후 측정 (관성 배제)
        self.seq_calibrate_robot_rotation()

        if self.sequencer.simple_event():
            self.mark_heading_recalibrated(self.robot.time)
            self.mapping_enabled = True
            print(f"[재캘리브:executor.state_recalibrate] ✓ 완료 → explore 복귀 (시뮬 {self.robot.time:.1f}s)")
            change_state_function("explore")
        self.sequencer.seq_reset_sequence()

    def mini_calibrate(self):
        """가끔씩 로봇이 미세하게 멈추며 자세를 교정하는 함수"""
        if self.mini_calibrate_step_counter.check():
            self.seq_move_wheels(1, 1)
            self.seq_delay_seconds(0.1)
        self.mini_calibrate_step_counter.increase()

    def align_with_fixture(self):
        """[비전 제어] 조난자를 발견했을 때 카메라 중앙에 오도록 로봇 바퀴를 미세 조정하는 함수"""
        center_image = self.robot.center_camera.image.image
        fixtures = self.fixture_detector.find_fixtures(center_image)
        
        if len(fixtures) == 0: return True
        
        fixture = fixtures[0]
        fixture_shape = Position2D(fixture["image"].shape)
        fixture_position = Position2D(fixture["position"])
        fixture_center = fixture_shape / 2 + fixture_position
        image_center = Position2D(center_image.shape) / 2

        # 화면 중앙과 글자 위치 간의 픽셀 차이(오차) 계산
        diff = image_center - fixture_center

        # Y축 오차가 크면 앞뒤로 이동
        if abs(diff.y) > 4:
            vel = diff.y * 0.1
            # 전방 벽이 가까우면 전진(vel>0) 금지 — 감지 후 벽으로 들이대 박는 것 방지.
            # 이미 감지·분류된 상태라 더 다가갈 필요 없음. 후진(vel<0)은 허용(거리 확보).
            if vel > 0 and self.robot.lidar.front_blocked:
                pass  # 전진 생략, 수직 정렬은 현 위치로 타협
            else:
                self.robot.move_wheels(vel, vel)
                return False # 아직 정렬 안 끝남

        # X축 오차가 크면 제자리 회전
        if abs(diff.x) > 6:
            sign = np.sign(diff.x)
            vel = 0.1
            self.robot.move_wheels(vel * sign, vel * -sign)
            return False # 아직 정렬 안 끝남

        return True # 정렬 완벽함!


    def state_end(self, change_state_function):
        """[미션 완료 상태] 완성된 지도를 추출하고 통신으로 본부에 보낸 후 끝냅니다."""
        final_matrix = self.final_matrix_creator.pixel_grid_to_final_grid(self.mapper.pixel_grid, self.mapper.start_position,
            area4_positions=self.mapper.area_tracker.area4_positions, reported_fixtures=self.reported_fixtures)
        self.robot.comunicator.send_map(final_matrix)
        self.robot.comunicator.send_end_of_play()

    def state_send_map(self, change_state_function):
        """[지도 강제 전송 상태] 시간이 다 되었을 때 지도를 보내고 다시 탐색으로 돌아갑니다."""
        final_matrix = self.final_matrix_creator.pixel_grid_to_final_grid(self.mapper.pixel_grid, self.mapper.start_position,
            area4_positions=self.mapper.area_tracker.area4_positions, reported_fixtures=self.reported_fixtures)
        self.robot.comunicator.send_map(final_matrix)
        self.map_sent = True
        change_state_function("explore")
        self.sequencer.seq_reset_sequence()

    def state_report_fixture(self, change_state_function):
        """[조난자 보고 상태] 조난자에게 다가가서 사진을 찍고 서버에 점수를 획득하는 모드"""
        self.sequencer.start_sequence()
        self.seq_move_wheels(0, 0) # 일단 정지

        if self.letter_to_report is not None:
            # 보고 중에는 지도를 그리지 않음
            if self.sequencer.simple_event():
                print(f"[조난자 보고:executor.state_report_fixture] ▶ 보고 시퀀스 시작: 글자='{self.letter_to_report}', 방향={self.report_orientation.degrees:.1f}°")
                self.mapping_enabled = False
                self.robot.lidar.front_guard_enabled = False   # 의도적 벽 접근 — 전진 차단 해제

            # 조난자가 있는 방향으로 정확히 회전
            if self.sequencer.simple_event():
                self.report_orientation.normalize()
            self.seq_rotate_to_angle(self.report_orientation.degrees)

            # 카메라 정중앙에 오도록 미세 정렬
            self.seq_align_with_fixture()
            self.seq_move_wheels(0, 0)

            # 다시 한 번 카메라로 확실하게 글자(H, S, U 등) 식별
            if self.sequencer.simple_event():
                center_image = self.robot.center_camera.image.image
                fixtures = self.fixture_detector.find_fixtures(center_image)
            
                # 만약 놓쳤으면 쿨하게 다시 탐색으로 복귀
                if len(fixtures) == 0:
                    # 재확인 실패 = 오탐. 이 위치를 블랙리스트에 올리고
                    # 쿨다운 동안 감지를 억제하여 로봇이 해당 구역을 벗어날 시간을 줍니다.
                    self.mapper.fixture_mapper.map_detected_fixture(self.robot.position)
                    self.fixture_detection_cooldown = 150  # ~150프레임(≈4.8초) 동안 감지 스킵
                    print(f"[조난자 보고:executor.state_report_fixture] ✗ 재확인 중 fixture 소실 → 오탐 블랙리스트 등록 + {self.fixture_detection_cooldown}프레임 쿨다운 후 explore 복귀")
                    change_state_function("explore")
                    self.sequencer.reset_sequence()
                    self.mapping_enabled = True
                    return

                self.letter_to_report = self.fixture_detector.classify_fixture(fixtures[0])
                if self.letter_to_report is None:
                    print("[조난자 보고:executor.state_report_fixture] ✗ 글자 인식 실패(already_detected) → explore 복귀")
                    change_state_function("explore")
                    self.sequencer.reset_sequence()
                    self.mapping_enabled = True
                    return

            # 조난자를 향해 앞으로 살짝 다가감 (0.6 속도로 0.1초 전진).
            # 단, 전방 벽이 이미 가까우면 더 들이대지 않는다(촬영 거리 유지, 벽 박음 방지).
            nudge_v = 0.0 if self.robot.lidar.front_blocked else 0.6
            self.seq_move_wheels(nudge_v, nudge_v)
            self.seq_delay_seconds(0.1)
            self.seq_move_wheels(0, 0)
            self.seq_delay_seconds(1.5) # 확실한 사진/보고를 위해 1.5초 대기

            # 서버 통신: "조난자 찾았습니다!"
            if self.sequencer.simple_event():
                print(f"[조난자 보고:executor.state_report_fixture] 글자='{self.letter_to_report}', 위치=({self.robot.raw_position.x:.4f}, {self.robot.raw_position.y:.4f})m, 시뮬 시간={self.robot.time:.1f}s")
                self.robot.comunicator.send_victim(self.robot.raw_position, self.letter_to_report)
                # 최종 맵 행렬에 넣기 위해 토큰(위치, 글자) 저장. 위치는 로봇 전방(벽 쪽)으로
                # ~6cm 보정 (로봇은 마진에서 보고 → 토큰/벽은 바라보는 방향에 있음).
                fx = self.robot.position.x + 0.06 * math.sin(self.report_orientation.radians)
                fy = self.robot.position.y + 0.06 * math.cos(self.report_orientation.radians)
                self.reported_fixtures.append((Position2D(fx, fy), self.letter_to_report))
                if self.letter_to_report in ('F', 'P', 'C', 'O'):
                    idx = self.mapper.pixel_grid.coordinates_to_array_index(self.robot.position)
                    rr, cc = skimage.draw.disk(idx, 4)
                    shape = self.mapper.pixel_grid.array_shape
                    valid = (rr >= 0) & (cc >= 0) & (rr < shape[0]) & (cc < shape[1])
                    self.mapper.pixel_grid.arrays["hazmats"][rr[valid], cc[valid]] = True
        
            self.seq_delay_seconds(0.1)
        
            # 지도에 조난자 마킹
            if self.sequencer.simple_event():
                self.mapper.fixture_mapper.map_detected_fixture(self.robot.position)

            # 다시 뒤로 물러나서 원래 위치 확보
            self.seq_move_wheels(-0.6, -0.6)
            self.seq_delay_seconds(0.1)
            self.seq_move_wheels(0, 0)

            if self.sequencer.simple_event():
                self.letter_to_report = None
            
        # 다 끝났으면 맵핑 다시 켜고 탐색으로 복귀
        if self.sequencer.simple_event():
            print("[조난자 보고:executor.state_report_fixture] ✓ 보고 완료, 맵핑 재개 → explore 복귀")
            self.mapping_enabled = True
        self.sequencer.simple_event(change_state_function, "explore")
        self.sequencer.seq_reset_sequence()

    def state_stuck(self, change_state_function):
        """[탈출 상태] 로봇이 벽에 끼였을 때 빠져나오기 위한 몸부림"""
        self.sequencer.start_sequence()
        if self.sequencer.simple_event():  # 탈출 시작 시 1회만 출력
            print(f"[끼임 감지:executor.state_stuck] ⚠ 로봇 끼임! 탈출 시도 시작: 위치=({self.robot.position.x:.4f},{self.robot.position.y:.4f})m, 방향={self.robot.orientation.degrees:.1f}°, 시뮬 시간={self.robot.time:.1f}s")
        # 뒤로 빼기 -> 대기 -> 오른쪽 바퀴만 뒤로 굴려 제자리 턴
        self.seq_move_wheels(-0.6, -0.6)
        self.seq_delay_seconds(0.2)
        self.seq_move_wheels(0.6, -0.6)
        self.seq_delay_seconds(1)

        # 탈출 시도 후 다시 탐색 모드로 전환
        if self.sequencer.simple_event():
            print(f"[끼임 감지:executor.state_stuck] ✓ 탈출 시도 완료 → explore 복귀, 현재 위치=({self.robot.position.x:.4f},{self.robot.position.y:.4f})m")
            change_state_function("explore")
        self.sequencer.seq_reset_sequence()

    def calibrate_position_offsets(self):
        """로봇이 처음에 정가운데 시작하지 않았을 때의 좌표 오차 계산"""
        self.robot.position_offsets = self.robot.position % (self.mapper.quarter_tile_size * 2)
        
    def seq_calibrate_robot_rotation(self):
        """[초기 캘리브레이션] 열린 방향으로 이동해 GPS heading을 측정하고 자이로 영점을 맞춘다.

        로봇은 시작 타일의 '열린 쪽'을 향해 스폰되므로(MainSupervisor), 시작 방향이
        월드마다 다르다. 한 번의 측정이 전체 런 heading 기준으로 고정되므로 여기서
        틀어지면 맵 전체가 그 각도만큼 회전한다(예: world1/4는 동쪽을 보고 시작 → 미보정 시 90° 회전).

        전략:
        1) 우선 후진으로 측정(+π). 전방 블랙홀 회피용. world3류(뒤가 열림)는 여기서 성공.
        2) 후진이 벽에 막혀 실패하면(world1/4류: 뒤가 벽, 앞이 열린 입구), 복귀용 전진 이동을
           그대로 heading 측정에 활용(+0). 즉 '열린 방향'으로 이동해 측정 → 시작 방향 무관하게 보정.
        3) 앞뒤 모두 실패하면 경고(자이로 영점 미설정 → 매핑 회전 위험).
        GPS 노이즈에 강건한 centroid 평균(get_orientation_robust)을 우선 사용한다."""
        if self.sequencer.simple_event():
            self.robot.auto_decide_orientation_sensor = False
            self.__calibrated_heading = False

        # GPS baseline 초기화 → 후진 구간만으로 방향 계산
        self.sequencer.simple_event(self.robot.gps.reset_orientation_baseline)

        # 1차: 후진 0.4s (~5cm) — 전방 블랙홀 반대 방향
        self.seq_move_wheels(-1, -1)
        self.seq_delay_seconds(0.4)
        self.seq_move_wheels(0, 0)
        if self.sequencer.simple_event():
            self.__calibrate_heading_from_gps(reverse=True)   # 후진 방향 +π → 전방

        # 2차 겸 복귀: 전진 0.4s. world3류는 단순 중앙 복귀(전방 구멍으로 가지 않음 — 후진한 만큼만 복귀).
        # world1/4류는 후진이 막혀 제자리였으므로 이 전진이 '열린 입구'로의 이동이 되어 heading 취득에 쓰인다.
        self.sequencer.simple_event(self.robot.gps.reset_orientation_baseline)
        self.seq_move_wheels(1, 1)
        self.seq_delay_seconds(0.4)
        self.seq_move_wheels(0, 0)
        if self.sequencer.simple_event():
            if not self.__calibrated_heading:
                self.__calibrate_heading_from_gps(reverse=False)   # 전진 방향 = 전방 +0
            if not self.__calibrated_heading:
                print("[초기화:executor.seq_calibrate_robot_rotation] ⚠ heading 캘리브 실패: "
                      "앞뒤 모두 이동 불가/노이즈 → 자이로 영점 미설정(매핑 회전 위험).")
            self.robot.auto_decide_orientation_sensor = True

    def __calibrate_heading_from_gps(self, reverse):
        """직전 이동 구간의 GPS heading으로 전방 heading을 산출해 자이로 영점을 세팅한다.
        reverse=True: 후진 이동(측정방향+π=전방), False: 전진 이동(측정방향=전방).
        노이즈 강건 centroid 방식 우선, 실패 시 단발, 둘 다 None이면 영점 미설정(미보정)."""
        heading = self.robot.gps.get_orientation_robust()
        if heading is None:
            heading = self.robot.gps.get_orientation()
        if heading is None:
            return
        offset = np.pi if reverse else 0.0
        true_forward = Angle(heading.radians + offset)
        true_forward.normalize()
        self.robot.gyroscope.set_orientation(true_forward)
        self.__calibrated_heading = True

