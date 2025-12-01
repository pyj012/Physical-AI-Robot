#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState
import time

# ★ controller_manager 서비스 (Humble)
from controller_manager_msgs.srv import SwitchController, ListControllers
# ★ Duration 메시지 (timeout에 필요)
from builtin_interfaces.msg import Duration as DurationMsg

# 컨트롤러 이름 (ros2_controllers.yaml과 동일해야 함)
CTRL_LEFT  = "left_arm_controller"
CTRL_RIGHT = "right_arm_controller"
CTRL_DUAL  = "dual_arm_controller"

def rpy_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw

# 쿼터니언 -> 회전행렬
def quat_to_rot(qx, qy, qz, qw):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = [[0.0]*3 for _ in range(3)]
    R[0][0] = 1 - 2*(yy + zz); R[0][1] = 2*(xy - wz);   R[0][2] = 2*(xz + wy)
    R[1][0] = 2*(xy + wz);     R[1][1] = 1 - 2*(xx + zz); R[1][2] = 2*(yz - wx)
    R[2][0] = 2*(xz - wy);     R[2][1] = 2*(yz + wx);   R[2][2] = 1 - 2*(xx + yy)
    return R

def rot_apply(R, v):
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]

def normalize_xy(vx, vy, eps=1e-9):
    n = math.hypot(vx, vy)
    if n < eps:
        return 0.0, 0.0
    return vx/n, vy/n


class GadianRRTPlanner(Node):
    def __init__(self):
        super().__init__('gadian_rrt_planner')

        # ---------- MoveGroup 액션 ----------
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("MoveGroup 액션 서버 연결 대기 중...")
        while not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("... /move_action 준비 대기")
        self.get_logger().info("MoveGroup 액션 서버 연결됨.")

        # ---------- controller_manager 서비스 ----------
        self.cm_ns = "/controller_manager"
        self.cli_switch = self.create_client(SwitchController, f"{self.cm_ns}/switch_controller")
        self.cli_list   = self.create_client(ListControllers,  f"{self.cm_ns}/list_controllers")
        for c, name in [(self.cli_switch, 'switch_controller'), (self.cli_list, 'list_controllers')]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{self.cm_ns}/{name} 서비스 대기 중...')

        # 컨트롤러 로드 대기 (스포너가 미리 load/configure 되어 있어야 함)
        self._wait_for_controllers([CTRL_LEFT, CTRL_RIGHT, CTRL_DUAL], timeout_sec=12.0, poll=0.3)

        # 시작 시: dual 비활성화, left/right 활성화
        self._activate_single_arms()

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- 기본 설정 ----------
        self.base_frame = "base_link"
        self.left  = {"group": "left_arm",  "ee_link": "L_end_link"}
        self.right = {"group": "right_arm", "ee_link": "R_end_link"}

        # (참조점 — 필요시 사용)
        self.L_UP_POINT = (0.186, -0.120, 1.288)
        self.R_UP_POINT = (0.186, -0.120, 1.288)
        self.L_FORWARD_POINT = (0.165, -0.575, 0.831)
        self.R_FORWARD_POINT = (0.165, -0.575, 0.831)
        self.L_DOWN_POINT = (0.190, 0.163, 0.410)
        self.R_DOWN_POINT = (-0.191, 0.163, 0.409)

        self.max_velocity_scaling_factor = 1.0
        self.max_acceleration_scaling_factor = 1.0

        # EE 프레임에서 "정면" 축 (필요시 조정)
        self.TOOL_FWD_AXIS_EE = [0.0, -1.0, 0.0]

    # ---------- controller_manager helpers ----------
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
        req.activate_controllers = list(activate_list)
        req.deactivate_controllers = list(deactivate_list)
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

        # 응답 타입 불명 시: 활성화 대상이 active인지로 보조 확인
        return all(self._is_active(n) for n in activate_list)

    def _activate_single_arms(self) -> bool:
        """dual 비활성화, left/right 활성화"""
        self.get_logger().info("스위칭: dual 비활성화, left/right 활성화")
        ok = self._switch_controllers(
            activate_list=[CTRL_LEFT, CTRL_RIGHT],
            deactivate_list=[CTRL_DUAL],
            timeout=3.0,
            strict=True
        )
        if not ok:
            self.get_logger().warn("스위칭 실패: 1회 재시도")
            time.sleep(0.5)
            ok = self._switch_controllers(
                activate_list=[CTRL_LEFT, CTRL_RIGHT],
                deactivate_list=[CTRL_DUAL],
                timeout=3.0,
                strict=True
            )

        st_left  = self._is_active(CTRL_LEFT)
        st_right = self._is_active(CTRL_RIGHT)
        st_dual  = self._is_active(CTRL_DUAL)
        self.get_logger().info(f"[현재 상태] left={st_left}, right={st_right}, dual={st_dual}")
        if not (st_left and st_right and not st_dual):
            self.get_logger().warn("기대 상태와 다릅니다. ros2_controllers.yaml / 스포너 구성을 점검하세요.")
        return st_left and st_right and (not st_dual)

    def _ensure_single_arms_active(self, max_retry: int = 1) -> bool:
        """현재 상태 점검 후, 필요 시 single-arms(왼/오 활성, 듀얼 비활성)로 강제"""
        # 컨트롤러 목록이 아직 로드되지 않았다면 잠깐 대기
        self._wait_for_controllers([CTRL_LEFT, CTRL_RIGHT, CTRL_DUAL], timeout_sec=5.0, poll=0.2)

        st_left  = self._is_active(CTRL_LEFT)
        st_right = self._is_active(CTRL_RIGHT)
        st_dual  = self._is_active(CTRL_DUAL)
        if st_left and st_right and not st_dual:
            return True

        self.get_logger().warn("컨트롤러 상태 불일치 → single_arms로 스위칭 시도")
        ok = self._activate_single_arms()
        tries = 0
        while not ok and tries < max_retry:
            time.sleep(0.5)
            ok = self._activate_single_arms()
            tries += 1
        return ok

    # ---------- TF: EE 현재 좌표 조회 ----------
    def lookup_ee_transform(self, ee_link: str, timeout_sec: float = 0.5):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, ee_link, rclpy.time.Time(), timeout=Duration(seconds=timeout_sec)
            )
            return tf
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF 조회 실패: {self.base_frame} -> {ee_link}: {e}")
            return None

    # 간단 규칙: x ≥ 0 → 왼팔, x < 0 → 오른팔
    def choose_closest_arm(self, target_xyz):
        x = float(target_xyz[0])
        choice = self.left if x >= 0.0 else self.right
        self.get_logger().info(
            f"x={x:.3f} → 선택={choice['group']} (x>=0 → left_arm, x<0 → right_arm)"
        )
        return choice

    # 공통 목표 생성 + 전송
    def _send_goal_pose(self, group_name: str, ee_link: str, px: float, py: float, pz: float,
                        qx: float, qy: float, qz: float, qw: float,
                        box_size: float = 0.01,
                        tol_r: float = 0.1, tol_p: float = 0.1, tol_yaw: float = math.pi,
                        plan_only: bool = False) -> bool:
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = MoveGroup.Goal()
        req: MotionPlanRequest = goal.request

        req.group_name = group_name
        req.pipeline_id = "ompl"
        req.planner_id = "RRTConnectkConfigDefault"
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor

        goal.planning_options.plan_only = plan_only
        req.start_state.is_diff = False

        goal.planning_options.planning_scene_diff.is_diff = False
        goal.planning_options.planning_scene_diff.robot_state.joint_state.name = []
        goal.planning_options.planning_scene_diff.robot_state.joint_state.position = []

        constraints = Constraints()
        constraints.name = "goal_constraints"

        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base_frame
        pos_c.link_name = ee_link
        pos_c.target_point_offset.x = 0.0
        pos_c.target_point_offset.y = 0.0
        pos_c.target_point_offset.z = 0.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [box_size, box_size, box_size]
        pos_c.constraint_region.primitives = [box]
        pos_c.constraint_region.primitive_poses = [pose.pose]
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header.frame_id = self.base_frame
        ori_c.link_name = ee_link
        ori_c.orientation = pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = tol_r
        ori_c.absolute_y_axis_tolerance = tol_p
        ori_c.absolute_z_axis_tolerance = tol_yaw
        ori_c.weight = 1.0

        constraints.position_constraints.append(pos_c)
        constraints.orientation_constraints.append(ori_c)
        req.goal_constraints.append(constraints)

        self.get_logger().info(
            f"목표 전송: group={group_name}, link={ee_link}, pos=({px:.3f},{py:.3f},{pz:.3f})"
        )
        send_goal_fut = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_fut)
        gh = send_goal_fut.result()

        if gh is None or not gh.accepted:
            self.get_logger().warn("경로계획 실패(목표 미수락)")
            return False

        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        result_wrap = result_fut.result()
        if result_wrap is None:
            self.get_logger().warn("결과 수신 실패")
            return False

        error_code = result_wrap.result.error_code.val
        if error_code == 1:
            self.get_logger().info("플래닝 및 실행 성공" if not plan_only else "플래닝 성공(실행 안 함)")
            return True
        else:
            self.get_logger().warn(f"플래닝/실행 실패: error_code={error_code}")
            return False

    # 단일 IK 이동
    def move_to_ik(self, x: float, y: float, z: float) -> bool:
        # ★ 매번 보장
        if not self._ensure_single_arms_active(max_retry=1):
            self.get_logger().error("컨트롤러(single_arms) 설정 실패")
            return False

        target_xyz = (x, y, z)
        choice = self.choose_closest_arm(target_xyz)
        group_name = choice["group"]
        ee_link = choice["ee_link"]

        qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, 0.0)
        ok1 = self._send_goal_pose(
            group_name, ee_link, x, y, z,
            qx, qy, qz, qw,
            box_size=0.01,
            tol_r=0.1, tol_p=0.1, tol_yaw=math.pi,
            plan_only=False
        )
        return ok1

    # 플래너: 1) 목표 이동 → 2) EE 정면 XY로 전진
    def plan_and_execute(self, x: float, y: float, z: float) -> bool:
        # ★ 매번 보장
        if not self._ensure_single_arms_active(max_retry=1):
            self.get_logger().error("컨트롤러(single_arms) 설정 실패")
            return False

        target_xyz = (x, y, z)
        choice = self.choose_closest_arm(target_xyz)
        group_name = choice["group"]
        ee_link = choice["ee_link"]

        # 1) 입력 좌표로 이동 (R=0, P=0, Y=0)
        qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, 0.0)
        ok1 = self._send_goal_pose(
            group_name, ee_link, x, y, z,
            qx, qy, qz, qw,
            box_size=0.01,
            tol_r=0.1, tol_p=0.1, tol_yaw=math.pi,
            plan_only=False
        )
        if not ok1:
            return False, choice

        # 2) TF에서 현재 EE 자세/위치 읽고, '정면(+EE 정의축)의 XY 투영'을 따라 전진
        tf = self.lookup_ee_transform(ee_link, timeout_sec=1.0)
        if tf is None:
            self.get_logger().warn("EE TF를 읽지 못해 +X 월드 방향으로 폴백 전진합니다.")
            fwd_xy = (1.0, 0.0)
            cur_x, cur_y, cur_z = x, y, z
            cur_qx, cur_qy, cur_qz, cur_qw = qx, qy, qz, qw
        else:
            t = tf.transform.translation
            r = tf.transform.rotation
            R = quat_to_rot(r.x, r.y, r.z, r.w)
            fwd_base = rot_apply(R, self.TOOL_FWD_AXIS_EE)  # [fx, fy, fz]
            fx, fy = normalize_xy(fwd_base[0], fwd_base[1])
            if fx == 0.0 and fy == 0.0:
                self.get_logger().warn("정면 XY 성분이 0에 가까움 → +X 월드로 폴백")
                fx, fy = 1.0, 0.0
            fwd_xy = (fx, fy)
            cur_x, cur_y, cur_z = t.x, t.y, t.z
            cur_qx, cur_qy, cur_qz, cur_qw = r.x, r.y, r.z, r.w

        step = 0.1  # 8 cm
        nx = cur_x + step * fwd_xy[0]
        ny = cur_y + step * fwd_xy[1]
        nz = cur_z  # z 동일

        # 3) 2차 목표: 현재 보는 방향으로 XY 전진 (자세 유지)
        ok2 = self._send_goal_pose(
            group_name, ee_link, nx, ny, nz,
            cur_qx, cur_qy, cur_qz, cur_qw,
            box_size=0.01,
            tol_r=0.1, tol_p=0.1, tol_yaw=math.pi,
            plan_only=False
        )
        return ok2, choice

    # 직접 이동 (자세 R=0,P=0,Y=0)
    def move_direct(self, x: float, y: float, z: float) -> bool:
        # ★ 매번 보장
        if not self._ensure_single_arms_active(max_retry=1):
            self.get_logger().error("컨트롤러(single_arms) 설정 실패")
            return False

        target_xyz = (x, y, z)
        choice = self.choose_closest_arm(target_xyz)
        group_name = choice["group"]; ee_link = choice["ee_link"]
        qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, 0.0)
        return self._send_goal_pose(group_name, ee_link, x, y, z, qx, qy, qz, qw,
                                    box_size=0.01, tol_r=0.1, tol_p=0.1, tol_yaw=math.pi, plan_only=False)


# 메인
def main():
    rclpy.init()
    node = GadianRRTPlanner()
    try:
        while rclpy.ok():
            # TF 수신 등을 위해 살짝 스핀
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                x = float(input("목표 x 좌표 (m): ").strip())
                y = float(input("목표 y 좌표 (m): ").strip())
                z = float(input("목표 z 좌표 (m): ").strip())
            except ValueError:
                print("숫자를 입력하세요.")
                continue
            node.plan_and_execute(x, y, z)
            # node.move_direct(x, y, z)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
