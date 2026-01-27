"""Test XML generation from ArUco observations.

Usage:
    python scripts/test_xml_generation.py
    python scripts/test_xml_generation.py --output /tmp/test_env.xml
    python scripts/test_xml_generation.py --scale-factor 6.0  # simulation scale
"""

from __future__ import annotations

import argparse
from pathlib import Path

from robot_control.utils import NAMOXMLGenerator


def main():
    parser = argparse.ArgumentParser(description="Test XML generation")
    parser.add_argument(
        "--output", "-o",
        type=str,
        default="/tmp/namo_from_aruco.xml",
        help="Output XML file path"
    )
    parser.add_argument(
        "--scale-factor", "-s",
        type=float,
        default=6.0,
        help="Scale factor (6.0 = real 2.5cm robot -> sim 15cm robot)"
    )
    parser.add_argument(
        "--show-wavefront",
        action="store_true",
        default=True,
        help="Show wavefront visualization"
    )
    args = parser.parse_args()

    print(f"Scale factor: {args.scale_factor}x")
    generator = NAMOXMLGenerator(scale_factor=args.scale_factor)

    # Example matching objects.yaml config:
    # - Movable boxes: 5x5x5 cm cubes
    # - Static walls: 19x5.2x10 cm
    # Workspace is 60cm x 80cm, origin at bottom-left
    robot_x_cm, robot_y_cm = 15.0, 10.0

    # Define objects: (x_cm, y_cm, theta_deg, width_cm, depth_cm, height_cm, is_static)
    objects = {
        "box1": (30.0, 40.0, 0.0, 5.0, 5.0, 5.0, False),      # Movable 5cm cube
        "wall9": (30.0, 60.0, 90.0, 19.0, 5.2, 10.0, True),   # Static wall rotated 90°
    }

    # Debug: print scaled values
    s = args.scale_factor
    print(f"\nScaled coordinates (in meters):")
    print(f"  Robot: ({robot_x_cm/100*s:.3f}, {robot_y_cm/100*s:.3f})")
    print(f"  Workspace: x=[0.00, {60/100*s:.2f}] y=[0.00, {80/100*s:.2f}]")
    print(f"  Objects (scaled):")
    for name, (x, y, theta, w, d, h, static) in objects.items():
        type_str = "STATIC" if static else "MOVABLE"
        print(f"    [{type_str}] {name}: pos=({x/100*s:.3f}, {y/100*s:.3f})m, theta={theta}°, size=({w/100*s:.3f}x{d/100*s:.3f}x{h/100*s:.3f})m")

    xml_str = generator.from_observation(
        robot_x_cm=robot_x_cm,
        robot_y_cm=robot_y_cm,
        objects=objects,
        goal_x_cm=50.0,
        goal_y_cm=70.0,
        workspace_bounds_cm=(0.0, 60.0, 0.0, 80.0),
    )

    # Save to file
    output_path = Path(args.output)
    generator.save(xml_str, str(output_path))
    print(f"Generated XML saved to: {output_path}")

    # Save and show wavefront
    wavefront_path = output_path.with_suffix(".png")
    robot_pos_scaled = (robot_x_cm / 100.0 * args.scale_factor,
                        robot_y_cm / 100.0 * args.scale_factor)
    generator.save_wavefront(str(wavefront_path), robot_pos_scaled)
    print(f"Wavefront saved to: {wavefront_path}")

    if args.show_wavefront:
        import cv2
        img = cv2.imread(str(wavefront_path))
        if img is not None:
            cv2.imshow("Wavefront Grid", img)
            print("\nPress any key to close...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    # Print scale info
    print("\n" + "=" * 60)
    print(f"Scale: {args.scale_factor}x")
    print(f"  Robot radius: {generator.ROBOT_RADIUS_BASE*100:.1f}cm -> {generator.ROBOT_RADIUS*100:.1f}cm")
    print(f"  Workspace: 60x80cm -> {60*args.scale_factor:.0f}x{80*args.scale_factor:.0f}cm")
    print("=" * 60)


if __name__ == "__main__":
    main()
