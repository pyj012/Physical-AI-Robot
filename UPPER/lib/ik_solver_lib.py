#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import time
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from controller_manager_msgs.srv import SwitchController, ListControllers
from builtin_interfaces.msg import Duration as DurationMsg


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


class GadianRRTPlanner(Node):
    def __init__(self):
        super().__init__('gadian_rrt_planner')


        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("MoveGroup 액션 서버 연결 대기 중...")
        while not self.move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("... /move_action 준비 대기")
        self.get_logger().info("MoveGroup 액션 서버 연결됨.")


        self.cm_ns = "/controller_manager"
        self.cli_switch = self.create_client(SwitchController, f"{self.cm_ns}/switch_controller")
        self.cli_list   = self.create_client(ListControllers,  f"{self.cm_ns}/list_controllers")
        for c, name in [(self.cli_switch, 'switch_controller'), (self.cli_list, 'list_controllers')]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{self.cm_ns}/{name} 서비스 대기 중...')

        self._wait_for_controllers([CTRL_LEFT, CTRL_RIGHT, CTRL_DUAL], timeout_sec=12.0, poll=0.3)
        self._activate_single_arms()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base_frame = "base_link"
        self.left  = {"group": "left_arm",  "ee_link": "L_end_link"}
        self.right = {"group": "right_arm", "ee_link": "R_end_link"}

        self.max_velocity_scaling_factor = 1.0
        self.max_acceleration_scaling_factor = 1.0


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
        req.strictness = 2 if strict else 1
        req.start_asap = True
        req.timeout = self._to_duration(float(timeout))
        fut = self.cli_switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp is None:
            return False
        if hasattr(resp, "ok"):
            return bool(resp.ok)
        return all(self._is_active(n) for n in activate_list)

    def _activate_single_arms(self) -> bool:
        self.get_logger().info("스위칭: dual 비활성화, left/right 활성화")
        ok = self._switch_controllers(
            activate_list=[CTRL_LEFT, CTRL_RIGHT],
            deactivate_list=[CTRL_DUAL],
            timeout=3.0,
            strict=True
        )
        if not ok:
            self.get_logger().warn("스위칭 실패, 재시도")
            time.sleep(0.5)
            ok = self._switch_controllers(
                activate_list=[CTRL_LEFT, CTRL_RIGHT],
                deactivate_list=[CTRL_DUAL],
                timeout=3.0,
                strict=True
            )
        return ok

    def _ensure_single_arms_active(self) -> bool:
        st_left  = self._is_active(CTRL_LEFT)
        st_right = self._is_active(CTRL_RIGHT)
        st_dual  = self._is_active(CTRL_DUAL)
        if st_left and st_right and not st_dual:
            return True
        return self._activate_single_arms()



    def choose_closest_arm(self, target_xyz):
        x = float(target_xyz[0])
        choice = self.left if x >= 0.0 else self.right
        self.get_logger().info(
            f"x={x:.3f} → 선택={choice['group']} (x>=0 → left_arm, x<0 → right_arm)"
        )
        return choice

    def _send_goal_pose(self, group_name: str, ee_link: str,
                        px: float, py: float, pz: float,
                        qx: float, qy: float, qz: float, qw: float,
                        box_size: float = 0.01,
                        tol_r: float = 0.01, tol_p: float = 0.01, tol_yaw: float = 0.01,
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
        req.num_planning_attempts = 5
        req.allowed_planning_time = 10.0
        req.start_state.is_diff = False
        goal.request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        goal.request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor

        print(self.max_velocity_scaling_factor )
        print(self.max_acceleration_scaling_factor)
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = False

        constraints = Constraints()
        pos_c = PositionConstraint()
        pos_c.header.frame_id = self.base_frame
        pos_c.link_name = ee_link

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
            f"목표 전송: group={group_name}, pos=({px:.3f},{py:.3f},{pz:.3f})"
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


    def plan_and_execute(self, x: float, y: float, z: float) -> bool:
        """입력한 좌표로만 이동"""
        if not self._ensure_single_arms_active():
            self.get_logger().error("컨트롤러(single_arms) 설정 실패")
            return False

        choice = self.choose_closest_arm((x, y, z))
        group_name = choice["group"]
        ee_link = choice["ee_link"]

        qx, qy, qz, qw = rpy_to_quaternion(0.0, 0.0, 0.0)
        ok = self._send_goal_pose(
            group_name, ee_link, x, y, z,
            qx, qy, qz, qw,
            box_size=0.001,
            tol_r=0.05, tol_p=0.05, tol_yaw=math.pi,
            plan_only=False
        )
        return ok, choice

def main():
    rclpy.init()
    node = GadianRRTPlanner()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                x = float(input("목표 x 좌표 (m): ").strip())
                y = float(input("목표 y 좌표 (m): ").strip())
                z = float(input("목표 z 좌표 (m): ").strip())
            except ValueError:
                print("숫자를 입력하세요.")
                continue
            node.plan_and_execute(x, y, z)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

