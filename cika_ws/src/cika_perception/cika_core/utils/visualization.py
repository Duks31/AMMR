"""
visualization.py
────────────────────────────────────────────────────────────────────────────
Draw YOLO bounding boxes + classifier labels onto a debug image for RViz.
Published on /cika/perception/debug_image — subscribe in RViz as Image.
"""

import cv2
import numpy as np

# BGR colours per label
LABEL_COLOURS = {
    "recyclable":     (0, 200, 0),    # green
    "non_recyclable": (0, 0, 220),    # red
    "unknown":        (180, 180, 180), # grey
}


def draw_detections(image: np.ndarray, detections: list[dict]) -> np.ndarray:
    """
    Draw bounding boxes and labels onto a copy of image.

    Each detection dict must contain:
        bbox                  : [x1, y1, x2, y2] float pixels
        label                 : str  "recyclable" | "non_recyclable" | "unknown"
        confidence_detector   : float
        confidence_classifier : float  (0.0 if not yet classified)
        has_3d_position       : bool
        position              : (x, y, z) metres tuple or None
    """
    out = image.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale, thickness = 0.5, 1

    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
        label       = det.get("label", "unknown")
        conf_det    = det.get("confidence_detector", 0.0)
        conf_cls    = det.get("confidence_classifier", 0.0)
        has_3d      = det.get("has_3d_position", False)
        pos         = det.get("position", None)

        colour = LABEL_COLOURS.get(label, LABEL_COLOURS["unknown"])

        # ── Bounding box ──────────────────────────────────────────────────────
        cv2.rectangle(out, (x1, y1), (x2, y2), colour, 2)

        # ── Top label: class + detector confidence ────────────────────────────
        line1 = f"{label}  det:{conf_det:.2f}"
        (tw, th), _ = cv2.getTextSize(line1, font, scale, thickness)
        # Filled background for readability
        cv2.rectangle(out, (x1, y1 - th - 8), (x1 + tw + 4, y1), colour, -1)
        cv2.putText(out, line1, (x1 + 2, y1 - 4), font, scale, (255, 255, 255), thickness)

        # ── Bottom label: classifier confidence + 3D position ────────────────
        parts = []
        if conf_cls > 0.0:
            parts.append(f"cls:{conf_cls:.2f}")
        if has_3d and pos is not None:
            parts.append(f"3D:({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})m")
        if parts:
            line2 = "  ".join(parts)
            cv2.putText(out, line2, (x1 + 2, y2 + th + 6), font, scale, colour, thickness)

    return out