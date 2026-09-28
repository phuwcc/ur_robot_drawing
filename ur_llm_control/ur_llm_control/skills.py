"""Deterministic robot skills. No LLM output reaches MoveIt directly."""

import copy


class SkillExecutor:
    def __init__(self, moveit, gripper, config, report):
        self.moveit = moveit
        self.gripper = gripper
        self.config = config
        self.report = report
        self.positions = {
            name: list(data["position"]) for name, data in config["objects"].items()
        }
        self.held = None
        self.scene_initialized = False

    def initialize(self):
        if self.scene_initialized:
            return
        self.report("INITIALIZING", "Waiting for MoveIt and creating the planning scene")
        self.moveit.wait_ready()
        self.gripper.initialize()
        self.moveit.initialize_scene(self.positions)
        self.scene_initialized = True

    def execute(self, plan):
        self.initialize()
        for index, step in enumerate(plan):
            skill = step["skill"]
            self.report("RUNNING", f"Step {index + 1}/{len(plan)}: {skill}", step)
            if skill == "pick":
                status = self.pick(step["object"])
            elif skill == "place":
                status = self.place(step["object"], step["zone"])
            else:
                status = self.home()
            if status != "SUCCESS":
                self.report(status, f"Stopped at step {index + 1}", step)
                return status
            self.report("SUCCESS", f"Completed step {index + 1}: {skill}", step)
        return "SUCCESS"

    def pick(self, object_id):
        if object_id not in self.config["objects"]:
            return "INVALID_OBJECT"
        if self.held is not None:
            return "INVALID_STATE"

        position = self.positions[object_id]
        size = float(self.config["objects"][object_id]["size"])
        motion = self.config["motion"]
        orientation = motion["grasp_orientation"]
        tcp_offset = size / 2.0 + float(motion["tool_length"])
        above = [position[0], position[1], position[2] + tcp_offset + motion["approach_height"]]
        grasp = [position[0], position[1], position[2] + tcp_offset]

        ok, status = self.moveit.move_to_pose(above, orientation, f"move above {object_id}")
        if not ok:
            return status
        # The target cube must be removed before the TCP enters its collision volume.
        self.moveit.remove_world_object(object_id)
        ok, status = self.moveit.move_to_pose(grasp, orientation, f"descend to {object_id}")
        if not ok:
            self.moveit.restore_world_object(object_id, position, size)
            return status
        try:
            self.moveit.attach_object(object_id, size, tcp_offset)
            self.gripper.grasp(object_id, tcp_offset)
        except RuntimeError:
            self.moveit.restore_world_object(object_id, position, size)
            return "GRASP_FAILED"
        self.held = object_id
        ok, status = self.moveit.move_to_pose(above, orientation, f"lift {object_id}")
        return "SUCCESS" if ok else status

    def place(self, object_id, zone_id):
        if object_id not in self.config["objects"]:
            return "INVALID_OBJECT"
        if zone_id not in self.config["zones"]:
            return "INVALID_ZONE"
        if self.held != object_id:
            return "INVALID_STATE"

        target = list(self.config["zones"][zone_id]["position"])
        size = float(self.config["objects"][object_id]["size"])
        motion = self.config["motion"]
        orientation = motion["grasp_orientation"]
        tcp_offset = size / 2.0 + float(motion["tool_length"])
        place = [target[0], target[1], target[2] + tcp_offset]
        above = [place[0], place[1], place[2] + motion["approach_height"]]

        ok, status = self.moveit.move_to_pose(above, orientation, f"move above {zone_id}")
        if not ok:
            return status
        ok, status = self.moveit.move_to_pose(place, orientation, f"descend to {zone_id}")
        if not ok:
            return status
        try:
            self.gripper.release(target)
            self.moveit.detach_object(object_id, target, size)
        except RuntimeError:
            return "EXECUTION_FAILED"
        self.positions[object_id] = copy.deepcopy(target)
        self.held = None
        ok, status = self.moveit.move_to_pose(above, orientation, f"retreat from {zone_id}")
        return "SUCCESS" if ok else status

    def home(self):
        if self.held is not None:
            return "INVALID_STATE"
        ok, status = self.moveit.move_home()
        return "SUCCESS" if ok else status
