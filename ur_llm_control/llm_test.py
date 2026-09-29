"""Exercise Gemini planning and validation without starting the robot stack."""

import argparse
import json

from ament_index_python.packages import get_package_share_directory

from .config import load_world, student_mapping
from .planner import GeminiPlanner, PlannerError
from .validator import PlanValidationError, parse_and_validate


DEFAULT_MODEL = "gemini-2.5-flash"


def main():
    parser = argparse.ArgumentParser(
        description="Test command -> Gemini -> JSON -> validation only."
    )
    parser.add_argument("command", help="Vietnamese or English robot command")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--api-key-env", default="GEMINI_API_KEY")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--student-name", default="YOUR_NAME")
    parser.add_argument("--student-id", default="00000000")
    parser.add_argument("--world-config")
    args = parser.parse_args()

    config_path = args.world_config or (
        get_package_share_directory("ur_llm_control") + "/config/world.yaml"
    )
    world = load_world(config_path)
    _, _, mapping = student_mapping(args.student_id)
    planner = GeminiPlanner(args.model, args.api_key_env, args.timeout)
    try:
        raw = planner.plan(
            args.command,
            tuple(world["objects"]),
            tuple(world["zones"]),
            args.student_name,
            args.student_id,
            mapping,
        )
        print("RAW RESPONSE:")
        print(raw)
        plan = parse_and_validate(
            raw, set(world["objects"]), set(world["zones"])
        )
    except (PlannerError, PlanValidationError) as exc:
        parser.exit(1, f"ERROR: {exc}\n")

    print("\nVALIDATED PLAN:")
    print(json.dumps({"plan": plan}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
