#!/usr/bin/env python3



# Image analysis
# Create mask for cloud cover and background sky using openCV HSV thresholding
# Find fraction of cloud cover on captured image
# Create contour image from cloud cover overlay
# Save sky and cloud mask and contour in groundstation folder

import pathlib      # For filesystem
import cv2          # OpenCV for image reading and processing
import yaml         # Library to access opencv config stored in YAML file 
import numpy as np  # Mathematical functions

root = pathlib.Path.home() / "groundstation"

def find_mask_cloud(img, filename):

    cfg = {"roi_top_frac": 0.0}
    cfg_path = root / "config" / "thresholds.yaml"

    # Find opencv config files
    if cfg_path.exists():
        from yaml import safe_load
        cfg.update(safe_load(cfg_path.read_text()) or {})


    k = np.ones((3, 3), np.uint8)

    roi_top_frac = float(cfg.get("roi_top_frac", 0.0))


    # Create ROI preview image 
    h, w = img.shape[:2]
    y0 = int(max(0.0, min(float(roi_top_frac), 0.9)) * h)
    overlay = img.copy()
    # Remove section of image outside of the ROI
    overlay[y0:, :] = (overlay[y0:, :] * 0.6 + 0.4 * 255).astype(img.dtype)
    cv2.line(overlay, (0, y0), (w, y0), (0, 255, 0), 2)
    txt = f"roi_top_frac={roi_top_frac:.2f} (kept below line)"
    cv2.putText(overlay, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(overlay, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
    
    
    image_name = pathlib.Path(filename).stem  # Get image name without extension
    folder_path = root / image_name
    folder_path.mkdir(parents=True, exist_ok=True)
    overlay_filename = folder_path / "roi_preview.jpg"
    cv2.imwrite(str(overlay_filename), overlay)
    print(f"ROI preview saved: {overlay_filename}")

    # Load threshold values for cloud and sky mask generation
    sky_h_min, sky_h_max = int(cfg['sky_h_min']), int(cfg['sky_h_max'])
    sky_s_min, sky_v_min = int(cfg['sky_s_min']), int(cfg['sky_v_min'])
    cloud_s_max, cloud_v_min = int(cfg['cloud_s_max']), int(cfg['cloud_v_min'])
    min_sky_pixels = int(cfg['min_sky_pixels'])

    # Convert image to HSV colour sapce
    roi = img[y0:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Detect sky and cloud mask based on HSV threshold value
    sky_mask = cv2.inRange(hsv, (sky_h_min, sky_s_min, sky_v_min), (sky_h_max, 255, 255))
    cloud_mask = cv2.inRange(hsv, (0, 0, cloud_v_min), (179, cloud_s_max, 255))

    # Clean masks using morphological operations
    sky_mask = cv2.morphologyEx(sky_mask, cv2.MORPH_OPEN, k, iterations=1)
    cloud_mask = cv2.morphologyEx(cloud_mask, cv2.MORPH_OPEN, k, iterations=1)

    # Count pixels in masks
    sky_px = int(np.count_nonzero(sky_mask))
    cloud_px = int(np.count_nonzero(cloud_mask))
    total_px = sky_px + cloud_px
    valid = total_px >= min_sky_pixels

    # Calculate cloud fraction
    cloud_frac = (cloud_px / total_px) if total_px > 0 else 0.0

    # Apply colour tint to sky and cloud mask pixels on image
    dbg = roi.copy()
    dbg[sky_mask > 0] = (255, 0, 0)  # sky → blue tint (BGR)
    dbg[cloud_mask > 0] = (0, 255, 255)  # clouds → yellow tint

    cv2.putText(dbg, txt, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(dbg, txt, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    # Save the image with overlays
    debug_filename = folder_path / "newoverlay.jpg"
    cv2.imwrite(str(debug_filename), dbg)

    # Contours for cloud mask
    contours, _ = cv2.findContours(cloud_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_image = np.zeros_like(roi)
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)  # Draw contours in green

    # Save the contours image
    contours_filename = folder_path / "cloud_contours.jpg"
    cv2.imwrite(str(contours_filename), contour_image)

    print(f"Cloud mask and contours saved in folder: {folder_path}")


