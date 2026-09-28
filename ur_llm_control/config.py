from pathlib import Path

import yaml


COLOR_BY_P = {
    0: ("Red", "Yellow", "Blue"),
    1: ("Red", "Blue", "Yellow"),
    2: ("Yellow", "Red", "Blue"),
    3: ("Yellow", "Blue", "Red"),
    4: ("Blue", "Red", "Yellow"),
    5: ("Blue", "Yellow", "Red"),
}


def load_world(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    required = {"frame_id", "ee_link", "group_name", "table", "objects", "zones", "motion", "gripper"}
    missing = required.difference(data)
    if missing:
        raise ValueError(f"world config is missing: {sorted(missing)}")
    return data


def student_mapping(student_id):
    value = str(student_id).strip()
    if len(value) < 2 or not value.isdigit():
        raise ValueError("student_id must contain at least two digits")
    suffix = int(value[-2:])
    p = suffix % 6
    colors = COLOR_BY_P[p]
    return suffix, p, dict(zip(("zone_a", "zone_b", "zone_c"), colors))
