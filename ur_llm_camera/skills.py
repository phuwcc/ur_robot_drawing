"""Camera-grounded skills with occlusion-tolerant, eventual release checks."""
import math
from .scene import SceneError, blockers


class SkillExecutor:
    def __init__(self, moveit, gripper, config, report, camera):
        self.moveit, self.gripper, self.config = moveit, gripper, config
        self.report, self.camera = report, camera
        self.positions = {}
        self.held = None
        # Released objects stay pending until a later unobstructed camera frame
        # confirms them. More than one transfer may finish before that happens.
        self.pending_placements = {}
        self.scene_initialized = False
        self.faulted = False

    def _grasp_targets(self, position):
        motion = self.config['motion']
        offset = float(motion['tool_length']) + float(motion['grasp_height_offset'])
        target = [position[0], position[1], position[2] + offset]
        above = [target[0], target[1], target[2] + motion['approach_height']]
        return motion['grasp_orientation'], offset, above, target

    def refresh(self):
        after = self.moveit.node.get_clock().now().nanoseconds / 1e9
        self.positions = self.camera.snapshot(after=after)
        self.moveit.initialize_scene(self.positions)
        return self.positions

    def _observe_merged(self, required, excluded=(), context='motion',
                        allowed_occluded=None, max_occluded=None):
        """Merge a fresh partial frame with the last complete camera state."""
        after = self.moveit.node.get_clock().now().nanoseconds / 1e9
        visible = self.camera.snapshot(after=after, required=set(required))
        expected = set(self.config['objects']) - set(excluded)
        visible_expected = expected.intersection(visible)
        missing = expected - visible_expected
        if max_occluded is None:
            max_occluded = int(
                self.config['camera'].get('max_occluded_while_holding', 1))
        unexpected = (set() if allowed_occluded is None else
                      missing - set(allowed_occluded))
        if (not visible_expected or len(missing) > max_occluded or unexpected):
            raise SceneError(
                f'PERCEPTION_UNAVAILABLE: too many objects are occluded during '
                f'{context}; missing={sorted(missing)}'
                + (f'; unexpected={sorted(unexpected)}' if unexpected else ''))
        not_cached = missing - set(self.positions)
        if not_cached:
            raise SceneError(
                'PERCEPTION_UNAVAILABLE: no previous camera position for '
                f'{sorted(not_cached)}')
        merged = {
            name: list(visible[name] if name in visible_expected else self.positions[name])
            for name in expected
        }
        self.positions.update(merged)
        if missing:
            self.moveit.node.get_logger().warning(
                f'Camera occluded {sorted(missing)} during {context}; '
                'using the most recent camera-confirmed positions')
        observed = {name: list(visible[name]) for name in visible_expected}
        return merged, observed

    def _verify_pending_placements(self, observed, require_all=False):
        if not self.pending_placements:
            return
        missing = set(self.pending_placements) - set(observed)
        if require_all and missing:
            raise SceneError(
                'PERCEPTION_UNAVAILABLE: final camera view did not include '
                f'placed objects {sorted(missing)}')
        for object_id, (target, zone_id) in list(self.pending_placements.items()):
            if object_id not in observed:
                continue
            if (math.dist(observed[object_id], target) >
                    self.config['workspace']['placement_tolerance']):
                raise SceneError(
                    f'PLACEMENT_FAILED: camera did not confirm {object_id} at {zone_id}')
            self.report(
                'PLACEMENT_VERIFIED',
                f'Camera confirmed {object_id} at {zone_id}')
            del self.pending_placements[object_id]

    def initialize(self):
        if self.faulted:
            raise SceneError('RECOVERY_REQUIRED: restart simulation after inspecting failed grasp/motion')
        if not self.scene_initialized:
            self.report('INITIALIZING', 'Detach startup joints, observe camera, initialize MoveIt scene')
            self.moveit.wait_ready()
            self.gripper.initialize()
            self.refresh()
            status = self.home()
            if status != 'SUCCESS':
                self.faulted = True
                raise SceneError(f'INITIALIZATION_FAILED: {status}')
            self.refresh()
            self.scene_initialized = True
        else:
            self.refresh()

    def execute(self, plan):
        try:
            for i, step in enumerate(plan):
                self.report('RUNNING', f'Step {i+1}/{len(plan)}', step)
                if step['skill'] == 'pick':
                    if not self.pending_placements:
                        self.refresh()
                    else:
                        # The arm may hide the next object while retreating from
                        # the previous destination. Accept one occluded object
                        # here and retain its camera-confirmed pre-place pose.
                        # Verify previous placements whenever they reappear.
                        allowed = set(self.pending_placements) | {step['object']}
                        _, observed = self._observe_merged(
                            required=set(),
                            context=f'prepare to pick {step["object"]}',
                            allowed_occluded=allowed,
                            max_occluded=int(self.config['camera'].get(
                                'max_occluded_during_transition', 2)))
                        self.moveit.initialize_scene(self.positions)
                        self._verify_pending_placements(observed)
                    following = plan[i+1]
                    target = self.config['zones'][following['zone']]['position']
                    if blockers(target, self.positions, self.config, step['object']):
                        raise SceneError('SCENE_CHANGED: destination occupied; request a new plan')
                    status = self.pick(step['object'])
                elif step['skill'] == 'place':
                    status = self.place(step['object'], step['zone'])
                else:
                    status = self.home()
                    if status == 'SUCCESS' and self.pending_placements:
                        self.refresh()
                        self._verify_pending_placements(
                            self.positions, require_all=True)
                if status != 'SUCCESS':
                    self.faulted = True
                    self.report(status, 'Execution stopped; inspect and restart simulation', step)
                    return status
            return 'SUCCESS'
        except Exception:
            self.faulted = True
            raise

    def pick(self, object_id):
        if self.held is not None:
            return 'INVALID_STATE'
        position = self.positions[object_id]
        size = float(self.config['objects'][object_id]['size'])
        orientation, _, above, grasp = self._grasp_targets(position)
        ok, status = self.moveit.move_to_pose(above, orientation, f'move above {object_id}')
        if not ok:
            return status
        if self.pending_placements:
            # Re-check any previous placement that is now visible. A placement
            # still hidden by the arm remains pending until a later view or the
            # mandatory complete observation after the final home step.
            _, observed = self._observe_merged(
                required=set(), excluded={object_id},
                context=f'verify previous placement before picking {object_id}')
            self.moveit.initialize_scene(self.positions)
            self._verify_pending_placements(observed)
        # Preserve target geometry; only the configured gripper may contact it.
        self.moveit.allow_grasp_contact(object_id, True)
        ok, status = self.moveit.move_to_pose(grasp, orientation, f'descend to {object_id}')
        if not ok:
            self.moveit.allow_grasp_contact(object_id, False)
            return status
        # A simulator weld is only allowed after reaching the measured grasp pose.
        grasp_pose = self.moveit.verify_grasp_pose(grasp, orientation)
        self.gripper.grasp(object_id)
        # A temporary placement of this same object has now been consumed by
        # the expanded plan; its next release creates a new pending placement.
        self.pending_placements.pop(object_id, None)
        self.held = object_id
        self.moveit.attach_object(object_id, size, position, grasp_pose)
        ok, status = self.moveit.move_to_pose(above, orientation, f'lift {object_id}')
        return 'SUCCESS' if ok else status

    def place(self, object_id, zone_id):
        if self.held != object_id:
            return 'INVALID_STATE'
        target = list(self.config['zones'][zone_id]['position'])
        # Never interpret an object hidden by the held cube or arm as empty.
        others, observed = self._observe_merged(
            required=set(), excluded={object_id},
            context=f'holding {object_id}')
        self._verify_pending_placements(observed)
        if blockers(target, others, self.config, object_id):
            raise SceneError('SCENE_CHANGED: destination became occupied')
        for name, p in others.items():
            if name != object_id:
                self.moveit.restore_world_object(name, p, self.config['objects'][name]['size'])
        size = float(self.config['objects'][object_id]['size'])
        release_position = list(target)
        release_position[2] += self.config['motion']['release_clearance']
        orientation, _, above, place = self._grasp_targets(release_position)
        for p, label in [(above, f'move above {zone_id}'), (place, f'descend to {zone_id}')]:
            ok, status = self.moveit.move_to_pose(p, orientation, label)
            if not ok:
                return status
        self.gripper.release()
        self.held = None
        # Detach in MoveIt before retreat; the released block stays on the table.
        self.moveit.detach_object(object_id, target, size)
        ok, status = self.moveit.move_to_pose(above, orientation, f'retreat from {zone_id}')
        self.moveit.allow_grasp_contact(object_id, False)
        if not ok:
            return status
        self.positions[object_id] = list(target)
        self.pending_placements[object_id] = (list(target), zone_id)
        return 'SUCCESS'

    def home(self):
        if self.held is not None:
            return 'INVALID_STATE'
        ok, status = self.moveit.move_home()
        return 'SUCCESS' if ok else status
