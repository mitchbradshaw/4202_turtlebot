#!/usr/bin/env python3
"""Run aruco_core over a folder of PNG frames. No ROS required.

Needs only:  pip install numpy opencv-contrib-python

Usage (from the mapping_pkg folder):
    python scripts/test_offline.py FRAMES_DIR CAMERA_INFO_JSON [--out OUT_DIR]
                                   [--marker-size 0.1] [--dictionary DICT_6X6_1000]

camera_info.json uses the same field names as sensor_msgs/CameraInfo:
    {"width": 640, "height": 480, "k": [fx,0,cx, 0,fy,cy, 0,0,1], "d": [..]}
("d" may be empty or omitted.) See scripts/camera_info_gazebo_waffle_pi.json.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

# Import aruco_core straight from the package source folder, so this works
# without installing the package and without ROS on the path.
PACKAGE_SRC = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_SRC))
from mapping_pkg import aruco_core  # noqa: E402


def load_camera_info(path):
    with open(path, 'r') as f:
        info = json.load(f)
    K = np.array(info['k'], dtype=np.float64).reshape(3, 3)
    dist = np.array(info.get('d', []), dtype=np.float64)
    return K, dist, info.get('width'), info.get('height')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('frames_dir', type=Path)
    parser.add_argument('camera_info', type=Path)
    parser.add_argument('--out', type=Path, default=None,
                        help='output folder (default: FRAMES_DIR/annotated)')
    parser.add_argument('--marker-size', type=float, default=aruco_core.DEFAULT_MARKER_SIZE_M)
    parser.add_argument('--dictionary', default=aruco_core.DEFAULT_DICTIONARY)
    args = parser.parse_args()

    out_dir = args.out if args.out is not None else args.frames_dir / 'annotated'
    out_dir.mkdir(parents=True, exist_ok=True)

    K, dist, width, height = load_camera_info(args.camera_info)
    print(f'OpenCV {cv2.__version__}  (new ArucoDetector API: {aruco_core._HAS_NEW_ARUCO_API})')
    print(f'K =\n{K}\ndist = {dist}\nmarker_size = {args.marker_size} m, '
          f'dictionary = {args.dictionary}\n')

    frames = sorted(args.frames_dir.glob('*.png'))
    if not frames:
        print(f'No .png files found in {args.frames_dir}')
        return 1

    frames_with_markers = 0
    for frame_path in frames:
        image = cv2.imread(str(frame_path), cv2.IMREAD_COLOR)
        if image is None:
            print(f'{frame_path.name}: could not read, skipped')
            continue

        if width and height and (image.shape[1], image.shape[0]) != (width, height):
            print(f'{frame_path.name}: WARNING image is {image.shape[1]}x{image.shape[0]} '
                  f'but camera_info is {width}x{height}; ranges will be wrong')

        detections = aruco_core.detect_markers(image, K, dist, args.marker_size, args.dictionary)
        annotated = aruco_core.draw_detections(image, detections, K, dist, args.marker_size)
        cv2.imwrite(str(out_dir / frame_path.name), annotated)

        if detections:
            frames_with_markers += 1
            summary = ', '.join(f'id {d.marker_id} @ {d.range_m:.2f} m' for d in detections)
        else:
            summary = 'no markers'
        print(f'{frame_path.name}: {summary}')

    print(f'\n{frames_with_markers}/{len(frames)} frames had detections. '
          f'Annotated images written to {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
