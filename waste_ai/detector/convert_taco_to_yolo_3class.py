#!/usr/bin/env python3
"""
convert_taco_to_yolo_3class.py
────────────────────────────────────────────────────────────────────────────
Converts TACO dataset (COCO format) → YOLO format with 3 classes:
    0: plastic
    1: paper
    2: metal

Any TACO category not in these three groups is SKIPPED entirely.
Images where ALL annotations are skipped are also excluded from the output.

Usage (local or Kaggle):
    python convert_taco_to_yolo_3class.py

Expects:
    BASE_DIR/
        annotations.json
        batch_1/  batch_2/  ...  (TACO image folders)

Produces:
    OUTPUT_DIR/
        images/train/
        images/val/
        labels/train/
        labels/val/
        data.yaml
"""

import json
import os
import shutil
from pathlib import Path
from sklearn.model_selection import train_test_split
from tqdm import tqdm

# ── Paths (edit these for Kaggle or local) ────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_DIR = SCRIPT_DIR / "data" / "TACO" / "data"
ANNOT = BASE_DIR / "annotations.json"
OUTPUT_DIR = BASE_DIR / "taco_yolo_3class"

# ── Class mapping ─────────────────────────────────────────────────────────────
# TACO category name → class id (0=plastic, 1=paper, 2=metal)
# Anything not listed here is IGNORED entirely.

CLASS_MAP = {
    # ── Plastic (0) ──────────────────────────────────────────────────────────
    "Plastic bottle": 0,
    "Other plastic bottle": 0,
    "Plastic lid": 0,
    "Plastic film": 0,
    "Plastic bag": 0,
    "Crisp packet": 0,
    "Plastic straw": 0,
    "Plastic utensils": 0,
    "Wrapping plastic": 0,
    "Other plastic wrapper": 0,
    "Other plastic": 0,
    "Six pack rings": 0,
    "Spread tub": 0,
    "Tupperware": 0,
    # ── Paper (1) ────────────────────────────────────────────────────────────
    "Paper": 1,
    "Paper bag": 1,
    "Cardboard": 1,
    "Food carton": 1,
    "Beverage carton": 1,
    "Other paper packaging": 1,
    "Toilet tube": 1,
    "Magazine": 1,
    "Newspaper": 1,
    "Paper cup": 1,
    "Tissues": 1,
    "Tissue": 1,
    # ── Metal (2) ────────────────────────────────────────────────────────────
    "Metal can": 2,
    "Drink can": 2,
    "Can": 2,
    "Aluminium foil": 2,
    "Metal lid": 2,
    "Pop tab": 2,
    "Scrap metal": 2,
}

CLASS_NAMES = {0: "plastic", 1: "paper", 2: "metal"}


# ── Helpers ───────────────────────────────────────────────────────────────────


def coco_bbox_to_yolo(bbox, img_w, img_h):
    """Convert COCO [x, y, w, h] → YOLO [x_center, y_center, w, h] (normalised)."""
    x, y, w, h = bbox
    x_center = (x + w / 2) / img_w
    y_center = (y + h / 2) / img_h
    return x_center, y_center, w / img_w, h / img_h


def write_yaml(output_dir: Path, class_names: dict):
    yaml_path = output_dir / "data.yaml"
    names_list = [class_names[i] for i in sorted(class_names)]
    lines = [
        f"path: {output_dir.resolve()}",
        "train: images/train",
        "val:   images/val",
        "",
        f"nc: {len(names_list)}",
        "names:",
    ]
    for i, name in enumerate(names_list):
        lines.append(f"  {i}: {name}")
    yaml_path.write_text("\n".join(lines) + "\n")
    print(f"  data.yaml written → {yaml_path}")


# ── Main ──────────────────────────────────────────────────────────────────────


def main():
    # ── Create output dirs ────────────────────────────────────────────────────
    for split in ("train", "val"):
        (OUTPUT_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (OUTPUT_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)

    # ── Load annotations ──────────────────────────────────────────────────────
    print("Loading annotations...")
    with open(ANNOT, "r", encoding="utf-8") as f:
        data = json.load(f)

    images = {img["id"]: img for img in data["images"]}
    categories = {cat["id"]: cat["name"] for cat in data["categories"]}

    # ── Build img_id → annotations lookup ────────────────────────────────────
    img_to_anns: dict[int, list] = {}
    for ann in data["annotations"]:
        img_to_anns.setdefault(ann["image_id"], []).append(ann)

    # ── Pre-filter: only keep images that have at least one mappable annotation
    valid_img_ids = []
    for img_id, anns in img_to_anns.items():
        if any(categories.get(a["category_id"], "") in CLASS_MAP for a in anns):
            valid_img_ids.append(img_id)

    print(f"  Total images in dataset : {len(images)}")
    print(f"  Images with 3-class ann : {len(valid_img_ids)}")

    # ── Train / val split ─────────────────────────────────────────────────────
    train_ids, val_ids = train_test_split(valid_img_ids, test_size=0.2, random_state=42)
    print(f"  Train: {len(train_ids)}  |  Val: {len(val_ids)}")

    # ── Class distribution counters ───────────────────────────────────────────
    counts = {0: 0, 1: 0, 2: 0}

    # ── Process each split ────────────────────────────────────────────────────
    def process_split(ids, split):
        skipped_images = 0
        for img_id in tqdm(ids, desc=f"Processing {split}"):
            img_info = images[img_id]
            file_name = img_info["file_name"]  # e.g. "batch_1/000001.jpg"
            src_path = BASE_DIR / file_name

            if not src_path.exists():
                skipped_images += 1
                continue

            # ── Collect valid annotations for this image ──────────────────────
            yolo_lines = []
            for ann in img_to_anns.get(img_id, []):
                cat_name = categories.get(ann["category_id"], "")
                cls_id = CLASS_MAP.get(cat_name)
                if cls_id is None:
                    continue  # skip non-target categories

                xc, yc, w, h = coco_bbox_to_yolo(
                    ann["bbox"], img_info["width"], img_info["height"]
                )
                # Guard against degenerate boxes
                if w <= 0 or h <= 0:
                    continue

                yolo_lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")
                counts[cls_id] += 1

            if not yolo_lines:
                skipped_images += 1
                continue

            # ── Flatten nested path to a safe flat filename ───────────────────
            # "batch_3/000123.jpg" → "batch_3_000123.jpg"
            flat_name = file_name.replace("/", "_").replace("\\", "_")

            # Copy image
            dst_img = OUTPUT_DIR / "images" / split / flat_name
            shutil.copy(src_path, dst_img)

            # Write label file
            label_name = Path(flat_name).stem + ".txt"
            dst_label = OUTPUT_DIR / "labels" / split / label_name
            dst_label.write_text("\n".join(yolo_lines) + "\n")

        if skipped_images:
            print(
                f"  [{split}] Skipped {skipped_images} images (missing or no valid annotations)"
            )

    process_split(train_ids, "train")
    process_split(val_ids, "val")

    # ── Write data.yaml ───────────────────────────────────────────────────────
    write_yaml(OUTPUT_DIR, CLASS_NAMES)

    # ── Summary ───────────────────────────────────────────────────────────────
    print("\n" + "=" * 55)
    print("Conversion complete!")
    print(f"  Output → {OUTPUT_DIR}")
    print(f"  Annotation counts (train + val combined):")
    for cls_id, name in CLASS_NAMES.items():
        print(f"    {cls_id}: {name:8s}  {counts[cls_id]:>5} boxes")
    total = sum(counts.values())
    print(f"    Total               {total:>5} boxes")
    if total > 0:
        for cls_id, name in CLASS_NAMES.items():
            pct = 100 * counts[cls_id] / total
            print(f"    {name:8s}  {pct:.1f}%")
    print("=" * 55)


if __name__ == "__main__":
    main()
