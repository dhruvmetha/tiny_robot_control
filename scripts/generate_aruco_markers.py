#!/usr/bin/env python3
"""Generate ArUco markers for obstacle detection and robot tracking.

Creates a printable A4 PDF with ArUco markers.
- Objects/Walls/Goal: DICT_6X6_50, default 30mm
- Robot: DICT_4X4_50, default 36mm

Usage:
    python scripts/generate_aruco_markers.py                    # Object markers (6x6, IDs 0-11)
    python scripts/generate_aruco_markers.py --robot            # Robot marker (4x4, ID 1)
    python scripts/generate_aruco_markers.py --robot --ids 2 3  # Multiple robot markers
    python scripts/generate_aruco_markers.py --size 40 --output my_markers.pdf
"""

from __future__ import annotations

import argparse
import io
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

# A4 dimensions at 300 DPI
A4_WIDTH_PX = 2480  # 210mm at 300 DPI
A4_HEIGHT_PX = 3508  # 297mm at 300 DPI
DPI = 300
MM_TO_PX = DPI / 25.4  # Convert mm to pixels at 300 DPI


def generate_marker(dictionary: cv2.aruco.Dictionary, marker_id: int, size_px: int) -> np.ndarray:
    """Generate a single ArUco marker image."""
    marker_img = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    return marker_img


def create_marker_sheet(
    marker_ids: list[int],
    marker_size_mm: float = 50.0,
    margin_mm: float = 10.0,
    spacing_mm: float = 10.0,
) -> np.ndarray:
    """
    Create an A4 sheet with ArUco markers arranged in a grid.
    
    Args:
        marker_ids: List of marker IDs to generate
        marker_size_mm: Size of each marker in mm
        margin_mm: Page margin in mm
        spacing_mm: Spacing between markers in mm
    
    Returns:
        A4 image as numpy array (grayscale)
    """
    # Convert to pixels
    marker_size_px = int(marker_size_mm * MM_TO_PX)
    margin_px = int(margin_mm * MM_TO_PX)
    spacing_px = int(spacing_mm * MM_TO_PX)
    
    # Calculate grid layout
    usable_width = A4_WIDTH_PX - 2 * margin_px
    usable_height = A4_HEIGHT_PX - 2 * margin_px
    
    cols = max(1, (usable_width + spacing_px) // (marker_size_px + spacing_px))
    rows = max(1, (usable_height + spacing_px) // (marker_size_px + spacing_px))
    
    # Create white A4 page
    page = np.ones((A4_HEIGHT_PX, A4_WIDTH_PX), dtype=np.uint8) * 255
    
    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    
    # Place markers
    marker_idx = 0
    for row in range(rows):
        for col in range(cols):
            if marker_idx >= len(marker_ids):
                break
            
            marker_id = marker_ids[marker_idx]
            marker_img = generate_marker(aruco_dict, marker_id, marker_size_px)
            
            # Calculate position
            x = margin_px + col * (marker_size_px + spacing_px)
            y = margin_px + row * (marker_size_px + spacing_px)
            
            # Place marker on page
            page[y:y + marker_size_px, x:x + marker_size_px] = marker_img
            
            # Add label below marker
            label = f"ID: {marker_id}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.8
            thickness = 2
            text_size = cv2.getTextSize(label, font, font_scale, thickness)[0]
            text_x = x + (marker_size_px - text_size[0]) // 2
            text_y = y + marker_size_px + int(8 * MM_TO_PX)  # 8mm below marker
            cv2.putText(page, label, (text_x, text_y), font, font_scale, 0, thickness)
            
            marker_idx += 1
    
    return page


def save_as_pdf(
    marker_ids: list[int],
    marker_size_mm: float,
    margin_mm: float,
    spacing_mm: float,
    output_path: Path,
    cut_size_mm: float = 50.0,
    aruco_dict_type: int = cv2.aruco.DICT_6X6_50,
) -> None:
    """Save markers directly to PDF at correct physical size with cutting borders."""
    pdf_path = output_path.with_suffix('.pdf')

    # Create PDF canvas (A4 size)
    c = canvas.Canvas(str(pdf_path), pagesize=A4)
    page_width, page_height = A4  # in points (72 per inch)

    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    
    # Calculate border padding from cut size and marker size
    border_padding_mm = (cut_size_mm - marker_size_mm) / 2.0
    
    # Total cell size = cut size (label goes outside cut area)
    cell_width_mm = cut_size_mm
    cell_height_mm = cut_size_mm
    label_height_mm = 6  # Label below cutting border
    
    # Calculate grid layout (include label height in row spacing)
    usable_width = (page_width / mm) - 2 * margin_mm
    usable_height = (page_height / mm) - 2 * margin_mm
    
    row_height_mm = cell_height_mm + label_height_mm  # Cell + label below
    cols = max(1, int((usable_width + spacing_mm) // (cell_width_mm + spacing_mm)))
    rows = max(1, int((usable_height + spacing_mm) // (row_height_mm + spacing_mm)))
    
    # Generate high-res marker images (use 10 pixels per mm for quality)
    marker_size_px = int(marker_size_mm * 10)
    
    # Import ImageReader once
    from reportlab.lib.utils import ImageReader
    
    markers_per_page = cols * rows
    total_pages = (len(marker_ids) + markers_per_page - 1) // markers_per_page
    
    marker_idx = 0
    for page_num in range(total_pages):
        if page_num > 0:
            c.showPage()  # Start new page
        
        for row in range(rows):
            for col in range(cols):
                if marker_idx >= len(marker_ids):
                    break
                
                marker_id = marker_ids[marker_idx]
                
                # Generate marker
                marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)
                
                # Convert to PIL Image
                pil_img = Image.fromarray(marker_img)
                
                # Calculate cell position (PDF origin is bottom-left)
                cell_x_mm = margin_mm + col * (cell_width_mm + spacing_mm)
                cell_y_mm = (page_height / mm) - margin_mm - (row + 1) * (row_height_mm + spacing_mm) + spacing_mm + label_height_mm
                
                # Draw cutting border (dashed rectangle - exactly 50x50mm)
                c.setStrokeColorRGB(0.5, 0.5, 0.5)  # Gray
                c.setLineWidth(0.5)
                c.setDash(3, 3)  # Dashed line
                c.rect(
                    cell_x_mm * mm,
                    cell_y_mm * mm,
                    cell_width_mm * mm,
                    cell_height_mm * mm,
                    stroke=1,
                    fill=0,
                )
                c.setDash()  # Reset to solid line
                
                # Marker position (centered in cell)
                marker_x_mm = cell_x_mm + border_padding_mm
                marker_y_mm = cell_y_mm + border_padding_mm
                
                # Save marker to bytes buffer
                img_buffer = io.BytesIO()
                pil_img.save(img_buffer, format='PNG')
                img_buffer.seek(0)
                
                # Draw marker on PDF at exact physical size
                img_reader = ImageReader(img_buffer)
                c.drawImage(
                    img_reader,
                    marker_x_mm * mm,
                    marker_y_mm * mm,
                    width=marker_size_mm * mm,
                    height=marker_size_mm * mm,
                )
                
                # Draw label below cutting border (outside the 50x50 area)
                c.setStrokeColorRGB(0, 0, 0)
                c.setFillColorRGB(0, 0, 0)
                c.setFont("Helvetica", 8)
                label = f"ID: {marker_id}"
                text_width = c.stringWidth(label, "Helvetica", 8)
                text_x = cell_x_mm * mm + (cell_width_mm * mm - text_width) / 2
                text_y = (cell_y_mm - 4) * mm  # Below the cutting border
                c.drawString(text_x, text_y, label)
                
                marker_idx += 1
    
    c.save()
    print(f"  {total_pages} page(s), {markers_per_page} markers per page")
    print(f"Saved: {pdf_path}")
    print(f"  Print at 100% scale (no fit-to-page)")
    print(f"  Cut along dashed gray lines")


def main():
    parser = argparse.ArgumentParser(
        description="Generate ArUco markers for obstacle detection and robot tracking"
    )
    parser.add_argument(
        "--robot",
        action="store_true",
        help="Generate robot markers (DICT_4X4_50, default ID 1, 36mm)",
    )
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=None,
        help="Marker IDs to generate (default: 0-11 for objects, 1 for robot)",
    )
    parser.add_argument(
        "--size",
        type=float,
        default=None,
        help="Marker size in mm (default: 30 for objects, 36 for robot)",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=15.0,
        help="Page margin in mm (default: 15)",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=15.0,
        help="Spacing between markers in mm (default: 15)",
    )
    parser.add_argument(
        "--cut-size",
        type=float,
        default=None,
        help="Cut-out size in mm (default: marker size + 20mm border)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output filename without extension",
    )
    args = parser.parse_args()

    # Set defaults based on marker type
    if args.robot:
        aruco_dict_type = cv2.aruco.DICT_4X4_50
        dict_name = "DICT_4X4_50"
        marker_ids = args.ids if args.ids else [1]
        marker_size = args.size if args.size else 36.0
        output_name = args.output if args.output else "aruco_robot_4x4"
    else:
        aruco_dict_type = cv2.aruco.DICT_6X6_50
        dict_name = "DICT_6X6_50"
        marker_ids = args.ids if args.ids else list(range(12))
        marker_size = args.size if args.size else 30.0
        output_name = args.output if args.output else "aruco_markers_6x6"

    cut_size = args.cut_size if args.cut_size else marker_size + 20.0
    border_mm = (cut_size - marker_size) / 2.0

    marker_type = "robot" if args.robot else "object"
    print(f"Generating {len(marker_ids)} ArUco {marker_type} marker(s) ({dict_name})")
    print(f"  IDs: {marker_ids}")
    print(f"  Marker size: {marker_size}mm")
    print(f"  Cut size: {cut_size}mm x {cut_size}mm")
    print(f"  White border: {border_mm}mm on each side")
    print(f"  Page margin: {args.margin}mm")
    print()

    # Generate and save PDF
    output_path = Path(output_name)
    save_as_pdf(
        marker_ids=marker_ids,
        marker_size_mm=marker_size,
        margin_mm=args.margin,
        spacing_mm=args.spacing,
        output_path=output_path,
        cut_size_mm=cut_size,
        aruco_dict_type=aruco_dict_type,
    )

    print()
    if args.robot:
        print("Done! Robot marker configuration (config/real.yaml):")
        print(f"  marker_id: {marker_ids[0]}")
        print(f"  marker_size_mm: {marker_size}")
    else:
        print("Done! Object marker configuration (config/objects.yaml):")
        print(f"  marker_size_mm: {marker_size}")


if __name__ == "__main__":
    main()
