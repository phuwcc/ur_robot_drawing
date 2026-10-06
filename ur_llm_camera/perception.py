"""RGB segmentation with calibrated top-down pinhole projection; no model poses."""
import math
import cv2
import numpy as np

from .scene import inside_table

# OpenCV hue is [0, 179]. Neutral zone markings never match these masks.
HUES = {'red_cube': [(0, 9), (170, 179)], 'yellow_cube': [(24, 38)],
        'blue_cube': [(100, 130)], 'green_cube': [(40, 85)],
        'purple_cube': [(132, 165)]}


def detect(bgr, config):
    camera = config['camera']
    if bgr.shape[:2] != (camera['height'], camera['width']):
        raise ValueError('Image dimensions do not match camera calibration')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    focal = camera['width'] / (2 * math.tan(camera['horizontal_fov'] / 2))
    table_top = config['table']['position'][2] + config['table']['size'][2] / 2
    positions = {}
    debug = bgr.copy()
    for name, obj in config['objects'].items():
        mask = np.zeros(bgr.shape[:2], dtype=np.uint8)
        for lo, hi in HUES[name]:
            mask |= cv2.inRange(hsv, (lo, 100, 65), (hi, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < camera['min_area']:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if not 0.6 < w/h < 1.65 or area/(w*h) < 0.65:
                continue
            depth = camera['position'][2] - (table_top + obj['size'])
            expected = (focal * obj['size'] / depth) ** 2
            if not 0.6 * expected <= area <= 1.65 * expected:
                continue
            moments = cv2.moments(contour)
            u = moments['m10'] / moments['m00']; v = moments['m01'] / moments['m00']
            # SDF camera +X looks down, +Y is image left, +Z is image up.
            p = [camera['position'][0] - (v-camera['height']/2)*depth/focal,
                 camera['position'][1] - (u-camera['width']/2)*depth/focal,
                 table_top + obj['size']/2]
            if inside_table(p, obj['size'], config, enforce_workspace=False):
                candidates.append(p)
                cv2.rectangle(debug, (x, y), (x+w, y+h), (255, 255, 255), 1)
                cv2.putText(debug, name, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
        if len(candidates) == 1:
            positions[name] = candidates[0]
    return positions, debug
