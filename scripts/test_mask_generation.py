"""Test different mask generation approaches for brick detection.

Captures a frame and generates masks using different methods,
saving them for visual inspection.
"""

import cv2
import numpy as np

def main():
    # Capture a frame
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Warm up camera
    for _ in range(10):
        cap.read()

    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture frame")
        return

    print(f"Captured frame: {frame.shape}")

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Save original
    cv2.imwrite("/tmp/mask_0_original.png", frame)
    cv2.imwrite("/tmp/mask_1_grayscale.png", gray)
    print("Saved: mask_0_original.png, mask_1_grayscale.png")

    # Method 1: Simple inverse threshold (dark objects)
    # Try different thresholds
    for thresh in [80, 100, 120, 150]:
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)
        cv2.imwrite(f"/tmp/mask_2_thresh_{thresh}.png", mask)
        print(f"Saved: mask_2_thresh_{thresh}.png (dark pixels < {thresh})")

    # Method 2: Adaptive threshold (handles uneven lighting)
    adaptive = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 10
    )
    cv2.imwrite("/tmp/mask_3_adaptive.png", adaptive)
    print("Saved: mask_3_adaptive.png (adaptive threshold)")

    # Method 3: Otsu's method (automatic threshold)
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    cv2.imwrite("/tmp/mask_4_otsu.png", otsu)
    print("Saved: mask_4_otsu.png (Otsu automatic)")

    # Method 4: Edge detection (current approach)
    edges = cv2.Canny(gray, 50, 150)
    edges_dilated = cv2.dilate(edges, np.ones((5, 5), np.uint8), iterations=1)
    cv2.imwrite("/tmp/mask_5_canny.png", edges)
    cv2.imwrite("/tmp/mask_5_canny_dilated.png", edges_dilated)
    print("Saved: mask_5_canny.png, mask_5_canny_dilated.png (edge detection)")

    # Method 5: Morphological cleanup on best threshold
    _, mask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((5, 5), np.uint8)

    # Open (remove noise)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    cv2.imwrite("/tmp/mask_6_opened.png", opened)

    # Close (fill gaps)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=2)
    cv2.imwrite("/tmp/mask_6_closed.png", closed)
    print("Saved: mask_6_opened.png, mask_6_closed.png (morphological cleanup)")

    # Method 6: Show detected contours on original
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    vis = frame.copy()
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 500:  # Skip small noise
            continue

        # Approximate polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Draw contour
        cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)

        # If rectangle (4 vertices), draw bounding box
        if len(approx) == 4:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(vis, [box], 0, (255, 0, 0), 3)

            # Label
            cx, cy = int(rect[0][0]), int(rect[0][1])
            cv2.putText(vis, f"{len(approx)}v", (cx, cy),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imwrite("/tmp/mask_7_contours.png", vis)
    print("Saved: mask_7_contours.png (detected contours)")

    print("\n=== Summary ===")
    print("View masks in /tmp/mask_*.png")
    print("  0_original.png     - Original frame")
    print("  1_grayscale.png    - Grayscale")
    print("  2_thresh_*.png     - Simple threshold (different values)")
    print("  3_adaptive.png     - Adaptive threshold")
    print("  4_otsu.png         - Otsu automatic threshold")
    print("  5_canny*.png       - Edge detection")
    print("  6_opened/closed.png - Morphological cleanup")
    print("  7_contours.png     - Detected rectangles")


if __name__ == "__main__":
    main()
