#!/usr/bin/env python3
"""Detect which ArUco dictionary a marker belongs to.

Point your camera at a marker and this will tell you:
- Which dictionary it belongs to (4x4, 5x5, 6x6, etc.)
- What ID it has

Usage:
    python scripts/detect_marker_dict.py
    python scripts/detect_marker_dict.py --camera 0
"""

import argparse
import cv2
from cv2 import aruco

# All common ArUco dictionaries to try
DICTIONARIES = [
    ("DICT_4X4_50", aruco.DICT_4X4_50),
    ("DICT_4X4_100", aruco.DICT_4X4_100),
    ("DICT_5X5_50", aruco.DICT_5X5_50),
    ("DICT_5X5_100", aruco.DICT_5X5_100),
    ("DICT_6X6_50", aruco.DICT_6X6_50),
    ("DICT_6X6_100", aruco.DICT_6X6_100),
    ("DICT_7X7_50", aruco.DICT_7X7_50),
    ("DICT_ARUCO_ORIGINAL", aruco.DICT_ARUCO_ORIGINAL),
]


def main():
    parser = argparse.ArgumentParser(description="Detect ArUco marker dictionary")
    parser.add_argument("--camera", "-c", type=int, default=0, help="Camera device ID")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Error: Could not open camera {args.camera}")
        return

    print("Point camera at ArUco marker. Press 'q' to quit.")
    print("=" * 60)

    # Create detectors for all dictionaries
    detectors = {}
    params = aruco.DetectorParameters()
    for name, dict_id in DICTIONARIES:
        dictionary = aruco.getPredefinedDictionary(dict_id)
        detectors[name] = aruco.ArucoDetector(dictionary, params)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Try each dictionary
        found = []
        for name, detector in detectors.items():
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    found.append((name, marker_id, corners[i]))

        # Display results
        display = frame.copy()

        if found:
            # Remove duplicates (same marker may be detected by multiple dicts)
            seen = set()
            unique = []
            for name, mid, corners in found:
                key = (mid, tuple(corners[0][0]))  # ID + first corner position
                if key not in seen:
                    seen.add(key)
                    unique.append((name, mid, corners))

            for name, mid, corners in unique:
                # Draw marker
                cv2.aruco.drawDetectedMarkers(display, [corners], None)

                # Draw info
                center = corners.reshape(4, 2).mean(axis=0).astype(int)
                text = f"{name}: ID {mid}"
                cv2.putText(display, text, (center[0] - 100, center[1] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                print(f"\rDetected: {text}                    ", end="", flush=True)
        else:
            cv2.putText(display, "No marker detected", (20, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Marker Detection", display)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("\nDone")


if __name__ == "__main__":
    main()
