#!/usr/bin/env python3
# ROS 2 Humble + MoveIt2
# 방법 B: MoveGroup 액션 요청에 "조인트 목표 + 속도/가속도 스케일"을 코드에서 고정 설정

import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)

# ★ controller_manager 서비스 (Humble)
from controller_manager_msgs.srv import SwitchController, ListControllers
# ★ Duration 메시지 (timeout에 필요)
from builtin_interfaces.msg import Duration as DurationMsg

PLANNER_ID = "RRTConnectkConfigDefault"
PIPELINE_ID = "ompl"
PLANNING_TIME = 10.0
PLANNING_ATTEMPTS = 3
JOINT_TOL_RAD = math.radians(0.5)   # 각 조인트 목표 허용 오차

# 컨트롤러 이름 (ros2_controllers.yaml과 동일해야 함)
CTRL_LEFT  = "left_arm_controller"
CTRL_RIGHT = "right_arm_controller"
CTRL_DUAL  = "dual_arm_controller"

JOINT_GROUPS = {
    "dual_arm": [
        "L_shoulder_joint1",
        "L_shoulder_joint2",
        "L_elbow_joint",
        "L_arm_joint",
        "L_wrist_joint1",
        "L_wrist_joint2",
        "R_shoulder_joint1",
        "R_shoulder_joint2",
        "R_elbow_joint",
        "R_arm_joint",
        "R_wrist_joint1",
        "R_wrist_joint2",
    ],
}


class JointSpaceMoveGroupExecutor(Node):
    def __init__(self):
        super().__init__("joint_space_movegroup_executor")

        # MoveGroup 액션
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("MoveGroup 액션 서버 연결 대기 중...")
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("... /move_action 준비 대기")
        self.get_logger().info("MoveGroup 액션 서버 연결됨.")

        # ★ controller_manager 서비스 클라이언트 준비
        self.cm_ns = "/controller_manager"
        self.cli_switch = self.create_client(SwitchController, f"{self.cm_ns}/switch_controller")
        self.cli_list   = self.create_client(ListControllers,  f"{self.cm_ns}/list_controllers")
        for c, name in [(self.cli_switch, 'switch_controller'), (self.cli_list, 'list_controllers')]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{self.cm_ns}/{name} 서비스 대기 중...')

        # 컨트롤러가 로드될 때까지 대기 (스포너가 미리 load/configure 해둬야 함)
        self._wait_for_controllers([CTRL_LEFT, CTRL_RIGHT, CTRL_DUAL], timeout_sec=12.0, poll=0.3)

        # ★ 시작 시: left/right 비활성 + dual 활성
        self._activate_dual_only()

        # dual_arm 조인트 이름들
        self.joint_names: List[str] = JOINT_GROUPS["dual_arm"]
        self.left_joint_names  = [n for n in self.joint_names if n.startswith("L_")]
        self.right_joint_names = [n for n in self.joint_names if n.startswith("R_")]

        self.arm_info_dict = {}

    # ---------- controller_manager helper들 ----------
    def _list_controllers(self):
        req = ListControllers.Request()
        fut = self.cli_list.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def _known_controllers(self) -> set:
        res = self._list_controllers()
        if not res:
            return set()
        return {c.name for c in res.controller}

    def _is_active(self, name: str) -> bool:
        res = self._list_controllers()
        if not res:
            return False
        for c in res.controller:
            if c.name == name:
                return c.state.lower() == "active"
        return False

    def _wait_for_controllers(self, names, timeout_sec=10.0, poll=0.2) -> bool:
        deadline = time.time() + timeout_sec
        need = set(names)
        while time.time() < deadline:
            known = self._known_controllers()
            if need.issubset(known):
                return True
            time.sleep(poll)
        self.get_logger().warn(f"컨트롤러 로드 대기 시간 초과: 필요={need}, 현재={self._known_controllers()}")
        return False

    def _to_duration(self, seconds: float) -> DurationMsg:
        s = int(seconds)
        ns = int((seconds - s) * 1e9)
        return DurationMsg(sec=s, nanosec=ns)

    def _switch_controllers(self, activate_list=None, deactivate_list=None, timeout=3.0, strict=True) -> bool:
        if activate_list is None:   activate_list = []
        if deactivate_list is None: deactivate_list = []

        req = SwitchController.Request()

        # Humble: activate/deactivate, strictness, start_asap, timeout(Duration)
        req.activate_controllers = list(activate_list)
        req.deactivate_controllers = list(deactivate_list)

        # strictness: STRICT=2, BEST_EFFORT=1
        try:
            req.strictness = SwitchController.Request.STRICT if strict else SwitchController.Request.BEST_EFFORT
        except Exception:
            req.strictness = 2 if strict else 1

        req.start_asap = True
        req.timeout = self._to_duration(float(timeout))

        fut = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None:
            self.get_logger().warn("switch_controllers 응답 없음")
            return False

        if hasattr(resp, "ok"):
            return bool(resp.ok)
        if hasattr(resp, "success"):
            return bool(resp.success)
        if hasattr(resp, "return_code"):
            return int(resp.return_code) == 0

        # 응답 타입 불명 시: 활성화 대상이 진짜 active인지로 보조 확인
        return all(self._is_active(n) for n in activate_list)

    def _activate_dual_only(self) -> bool:
        """left/right 비활성화, dual 활성화"""
        self.get_logger().info("스위칭: left/right 비활성화, dual 활성화")
        ok = self._switch_controllers(
            activate_list=[CTRL_DUAL],
            deactivate_list=[CTRL_LEFT, CTRL_RIGHT],
            timeout=3.0,
            strict=True
        )
        if not ok:
            self.get_logger().warn("스위칭 실패: 1회 재시도")
            time.sleep(0.5)
            ok = self._switch_controllers(
                activate_list=[CTRL_DUAL],
                deactivate_list=[CTRL_LEFT, CTRL_RIGHT],
                timeout=3.0,
                strict=True
            )

        st_left  = self._is_active(CTRL_LEFT)
        st_right = self._is_active(CTRL_RIGHT)
        st_dual  = self._is_active(CTRL_DUAL)
        self.get_logger().info(f"[현재 상태] left={st_left}, right={st_right}, dual={st_dual}")
        if not (st_dual and not st_left and not st_right):
            self.get_logger().warn("기대 상태와 다릅니다. ros2_controllers.yaml / 스포너 구성을 점검하세요.")
        return st_dual and not st_left and not st_right

    def _ensure_dual_only_active(self, max_retry: int = 1) -> bool:
        """현재 컨트롤러 상태를 확인하고, dual_only 상태가 아니면 스위칭"""
        st_left  = self._is_active(CTRL_LEFT)
        st_right = self._is_active(CTRL_RIGHT)
        st_dual  = self._is_active(CTRL_DUAL)
        if st_dual and not st_left and not st_right:
            return True

        self.get_logger().warn("컨트롤러 상태 불일치 → dual_only로 스위칭 시도")
        ok = self._activate_dual_only()
        tries = 0
        while not ok and tries < max_retry:
            time.sleep(0.5)
            ok = self._activate_dual_only()
            tries += 1
        return ok

    def ask_joint_targets_deg(self, joint_names: List[str]) -> Optional[List[float]]:
        print("\n=== 관절 각도 입력 (deg) ===")
        print("조인트 순서:", ", ".join(joint_names))
        print("힌트: 잘못 입력하면 'q' 입력으로 취소할 수 있습니다.")
        targets_rad: List[float] = []
        for name in joint_names:
            raw = input(f"{name} (deg): ").strip()
            if raw.lower() == "q":
                print("입력을 취소했습니다.")
                return None
            try:
                deg = float(raw)
                rad = math.radians(deg)
            except ValueError:
                print("잘못된 입력입니다. 숫자를 입력하거나 'q'로 취소하세요.")
                return None
            targets_rad.append(rad)
        return targets_rad

    def build_goal_joint(self,
                         group_name: str,
                         joint_names: List[str],
                         target_positions_rad: List[float],
                         plan_only: bool = False,
                         vel=0.1,
                         acc=0.1) -> MoveGroup.Goal:

        goal = MoveGroup.Goal()
        req: MotionPlanRequest = goal.request

        req.group_name = group_name
        req.pipeline_id = PIPELINE_ID
        req.planner_id = PLANNER_ID
        req.allowed_planning_time = PLANNING_TIME
        req.num_planning_attempts = PLANNING_ATTEMPTS

        # 가속도,속도 설정
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc

        # 현재 로봇 상태에서 시작
        req.start_state.is_diff = True

        # Joint goal 제약 구성
        jcs: List[JointConstraint] = []
        for name, pos in zip(joint_names, target_positions_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = JOINT_TOL_RAD
            jc.tolerance_below = JOINT_TOL_RAD
            jc.weight = 1.0
            jcs.append(jc)

        cnst = Constraints()
        cnst.name = "joint_goal"
        cnst.joint_constraints = jcs
        req.goal_constraints = [cnst]

        goal.planning_options.plan_only = bool(plan_only)
        return goal

    def send_and_wait(self, goal: MoveGroup.Goal) -> bool:
        send_future = self.client.send_goal_async(goal)
        # 필요 시 결과 대기를 복구하세요.
        # rclpy.spin_until_future_complete(self, send_future)
        # gh = send_future.result()
        # if gh is None or not gh.accepted:
        #     self.get_logger().warn("목표가 거부되었거나 GoalHandle 획득 실패")
        #     return False
        # res_future = gh.get_result_async()
        # rclpy.spin_until_future_complete(self, res_future)
        # res_wrap = res_future.result()
        # if res_wrap is None:
        #     self.get_logger().warn("결과 수신 실패")
        #     return False
        # code = res_wrap.result.error_code.val
        # if code == MoveItErrorCodes.SUCCESS:
        #     self.get_logger().info("플래닝/실행 성공")
        #     return True
        # else:
        #     self.get_logger().warn(f"실패: MoveItErrorCodes={code}")
        return True

    def move_with_angle(self, arm_info_dict) -> bool:
        self._wait_for_controllers([CTRL_LEFT, CTRL_RIGHT, CTRL_DUAL], timeout_sec=8.0, poll=0.2)
        if not self._ensure_dual_only_active(max_retry=1):
            self.get_logger().error("dual_only 활성화 실패. 동작을 중단합니다.")
            return
        if not isinstance(arm_info_dict, dict):
            self.get_logger().error("arm_info_dict가 dict가 아닙니다.")
            return False

        speed = float(arm_info_dict.get("speed", 0.2))
        left_vals  = arm_info_dict.get("left",  None)
        right_vals = arm_info_dict.get("right", None)

        if left_vals is None or right_vals is None:
            self.get_logger().error("arm_info_dict에 'left'와 'right'가 모두 필요합니다.")
            return False

        if len(left_vals) != len(self.left_joint_names):
            self.get_logger().error(f"left 개수 불일치: 기대={len(self.left_joint_names)} 실제={len(left_vals)}")
            return False
        if len(right_vals) != len(self.right_joint_names):
            self.get_logger().error(f"right 개수 불일치: 기대={len(self.right_joint_names)} 실제={len(right_vals)}")
            return False

        try:
            left_rads  = [math.radians(float(deg)) for deg in left_vals]
            right_rads = [math.radians(float(deg)) for deg in right_vals]
        except Exception as e:
            self.get_logger().error(f"deg→rad 변환 실패: {e}")
            return False

        # dual_arm 순서에 맞춰 [왼팔 6개 + 오른팔 6개]
        targets_rad = left_rads + right_rads

        goal = self.build_goal_joint(
            group_name='dual_arm',
            joint_names=self.joint_names,
            target_positions_rad=targets_rad,
            plan_only=False,
            vel=speed,
            acc=speed
        )
        ok = self.send_and_wait(goal)
        return ok

    def run(self):
        self._wait_for_controllers([CTRL_LEFT, CTRL_RIGHT, CTRL_DUAL], timeout_sec=8.0, poll=0.2)
        if not self._ensure_dual_only_active(max_retry=1):
            self.get_logger().error("dual_only 활성화 실패. 동작을 중단합니다.")
            return

        while rclpy.ok():
            if not self._ensure_dual_only_active(max_retry=1):
                self.get_logger().error("dual_only 재보장 실패. 동작을 중단합니다.")
                break

            self.arm_info_dict = {
                "left":  [0, 0, -0, 0, 0, -0],
                "right": [0, 0, -0, 0, 0, -0],
                "speed": 0.7
            }

            ok = self.move_with_angle(self.arm_info_dict)
            if not ok:
                self.get_logger().warn("실패. 파라미터/충돌/도달범위 등을 점검하세요.")

            again = input("\n계속하시겠습니까? (y=계속 / 기타=종료): ").strip().lower()
            if again != "y":
                self.get_logger().info("프로그램 종료")
                break


def main():
    rclpy.init()
    node = JointSpaceMoveGroupExecutor()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
