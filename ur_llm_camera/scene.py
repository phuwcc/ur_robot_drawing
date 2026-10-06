"""Camera-derived occupancy and deterministic, geometry-checked plan expansion."""
import copy
import math

from .validator import PlanValidationError, parse_and_validate
import json


class SceneError(RuntimeError):
    pass


def inside_table(p, size, config, enforce_workspace=True):
    table = config['table']
    bounds = config['workspace']['bounds']
    return (all(math.isfinite(v) for v in p) and
            (not enforce_workspace or (bounds[0] <= p[0] <= bounds[1] and
                                       bounds[2] <= p[1] <= bounds[3])) and
            all(abs(p[i] - table['position'][i]) + size / 2 <= table['size'][i] / 2
                for i in (0, 1)))


def blockers(target, positions, config, moving=None):
    size = config['objects'][moving]['size'] if moving else 0.05
    margin = config['workspace']['clearance']
    return [name for name, p in positions.items() if name != moving and
            all(abs(p[i] - target[i]) < (size + config['objects'][name]['size']) / 2 + margin
                for i in (0, 1))]


def occupancy(positions, config):
    # Conservative square footprints include objects overlapping the zone edge.
    return {name: blockers(zone['position'], positions, config)
            for name, zone in config['zones'].items() if not name.startswith('temporary_')}


def find_free_position(positions, config, moving, reachable=None):
    bounds = config['workspace']['bounds']
    step = config['workspace']['grid_step']
    size = config['objects'][moving]['size']
    z = config['table']['position'][2] + config['table']['size'][2] / 2 + size / 2
    candidates = []
    for ix in range(int((bounds[1]-bounds[0])/step)+1):
        for iy in range(int((bounds[3]-bounds[2])/step)+1):
            p = [bounds[0]+ix*step, bounds[2]+iy*step, z]
            if not inside_table(p, size, config) or blockers(p, positions, config, moving):
                continue
            if any(math.dist(p[:2], zone['position'][:2]) < config['workspace']['zone_clearance']
                   for zone in config['zones'].values()):
                continue
            candidates.append(p)
    candidates.sort(key=lambda p: math.dist(p, positions[moving]) + 0.8 * math.hypot(*p[:2]))
    for p in candidates:
        if reachable is None or reachable(moving, p):
            return p
    raise SceneError('NO_FREE_POSITION: no free reachable temporary location on table')


def expand_plan(plan, positions, config, reachable=None):
    """Expand validated pick/place pairs, simulate occupancy, validate final sequence."""
    state = copy.deepcopy(positions)
    cfg = copy.deepcopy(config)
    cfg['zones'] = {k: v for k, v in cfg['zones'].items() if not k.startswith('temporary_')}
    result = []
    def transfer(obj, zone):
        target = cfg['zones'][zone]['position']
        if not inside_table(target, cfg['objects'][obj]['size'], cfg):
            raise SceneError('Destination is outside the configured workspace')
        if blockers(target, state, cfg, obj):
            raise SceneError('Destination remains occupied')
        result.extend([{'skill': 'pick', 'object': obj},
                       {'skill': 'place', 'object': obj, 'zone': zone}])
        state[obj] = list(target)
    for i, action in enumerate(plan):
        if action['skill'] == 'home':
            result.append(action.copy())
        elif action['skill'] == 'pick':
            if i+1 >= len(plan) or plan[i+1]['skill'] != 'place':
                raise PlanValidationError('Transfers must use adjacent pick/place steps')
            obj = action['object']; zone = plan[i+1]['zone']
            if obj not in state:
                raise SceneError(f'Object not observed: {obj}')
            for other in blockers(cfg['zones'][zone]['position'], state, cfg, obj):
                temp = f'temporary_{len(cfg["zones"])}'
                cfg['zones'][temp] = {'position': find_free_position(state, cfg, other, reachable)}
                transfer(other, temp)
            transfer(obj, zone)
    parse_and_validate(json.dumps({'plan': result}), set(cfg['objects']), set(cfg['zones']), max_steps=50)
    return result, cfg['zones']
