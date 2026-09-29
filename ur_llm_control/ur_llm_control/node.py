import json
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from .config import load_world, student_mapping
from .physical_gripper import PhysicalGripper
from .moveit_cartesian_client import MoveItClient
from .planner import NineRouterPlanner, PlannerError, mock_plan
from .skills import SkillExecutor
from .validator import PlanValidationError, parse_and_validate


class LlmSkillNode(Node):
    def __init__(self):
        super().__init__("llm_skill_node")
        default_config = get_package_share_directory("ur_llm_control") + "/config/world.yaml"
        defaults = {
            "world_config": default_config,
            "student_name": "YOUR_NAME",
            "student_id": "00000000",
            "router_base_url": "http://127.0.0.1:20128/v1",
            "router_model": "oc/muse-spark-1.3-contributor-free",
            "router_api_key_env": "NINE_ROUTER_API_KEY",
            "router_timeout": 30.0,
            "mock_llm": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        self.world = load_world(self.params["world_config"])
        suffix, p_value, self.mapping = student_mapping(self.params["student_id"])

        self.raw_pub = self.create_publisher(String, "/llm/raw_response", 10)
        self.plan_pub = self.create_publisher(String, "/llm/validated_plan", 10)
        self.status_pub = self.create_publisher(String, "/execution_status", 10)
        self.command_sub = self.create_subscription(
            String, "/user_command", self._on_command, 10)

        self.gripper = PhysicalGripper(
            self, self.world["objects"], self.world.get("gripper", {})
        )
        self.moveit = MoveItClient(self, self.world)
        self.skills = SkillExecutor(self.moveit, self.gripper, self.world, self.report)
        self.planner = NineRouterPlanner(
            self.params["router_base_url"], self.params["router_model"],
            self.params["router_api_key_env"], float(self.params["router_timeout"]))
        self.busy_lock = threading.Lock()

        mapping_text = ", ".join(f"{key}={value}" for key, value in self.mapping.items())
        self.get_logger().info(
            f"Student {self.params['student_name']} ({self.params['student_id']}): "
            f"{suffix:02d} mod 6 = {p_value}; {mapping_text}")
        if self.params["student_id"] == "00000000":
            self.get_logger().warning("Set student_name and student_id before the final demo")
        mode = "offline mock (test only)" if self.params["mock_llm"] else "9Router"
        self.get_logger().info(f"Planner mode: {mode}; listening on /user_command")

    def report(self, status, message, step=None):
        payload = {"status": status, "message": message}
        if step is not None:
            payload["step"] = step
        text = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(String(data=text))
        if status in {"SUCCESS", "RUNNING", "INITIALIZING", "PLANNING", "PLAN_VALID"}:
            self.get_logger().info(text)
        else:
            self.get_logger().error(text)

    def _on_command(self, message):
        command = message.data.strip()
        if not command:
            self.report("INVALID_COMMAND", "The command is empty")
            return
        if not self.busy_lock.acquire(blocking=False):
            self.report("BUSY", "The robot is already executing a plan")
            return
        threading.Thread(target=self._process, args=(command,), daemon=True).start()

    def _process(self, command):
        try:
            self.report("PLANNING", command)
            if self.params["mock_llm"]:
                raw = mock_plan(command)
            else:
                raw = self.planner.plan(
                    command,
                    tuple(self.world["objects"]),
                    tuple(self.world["zones"]),
                    self.params["student_name"],
                    self.params["student_id"],
                    self.mapping,
                )
            self.raw_pub.publish(String(data=raw))
            plan = parse_and_validate(
                raw, set(self.world["objects"]), set(self.world["zones"]))
            normalized = json.dumps({"plan": plan}, ensure_ascii=False)
            self.plan_pub.publish(String(data=normalized))
            self.report("PLAN_VALID", normalized)
            try:
                status = self.skills.execute(plan)
            except Exception as exc:  # Robot failures are not LLM failures.
                self.report("EXECUTION_FAILED", str(exc))
                return
            if status == "SUCCESS":
                self.report("SUCCESS", "Plan completed")
        except PlannerError as exc:
            self.report("LLM_FAILED", str(exc))
        except PlanValidationError as exc:
            self.report("INVALID_PLAN", str(exc))
        except Exception as exc:  # Keep a ROS callback failure from killing the safety node.
            self.report("FAILED", str(exc))
        finally:
            self.busy_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LlmSkillNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
