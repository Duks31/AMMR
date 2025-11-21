import os
import json
from tqdm import tqdm

TACO_DIR = r"/data/TACO/data"
DATA_DIR = r"/data"
ANNOT_FILE = os.path.join(DATA_DIR, "annotations.json")
OUTPUT_IMAGES = os.path.join(DATA_DIR, "yolo_images")
OUTPUT_LABELS = os.path.join(DATA_DIR, "yolo_labels")

os.makedirs(OUTPUT_IMAGES, exist_ok=True)
os.makedirs(OUTPUT_LABELS, exist_ok=True)

with open(ANNOT_FILE, 'r') as f:
    coco = json.load(f)

images = {img['id']: img for img in coco['images']}
categories = {cat['id']: cat['name'] for cat in coco['categories']}

cat_id_to_yolo = {cid: idx for idx, cid in enumerate(categories.keys())}

print("Class mapping:")
for k, v in cat_id_to_yolo.items():
    print(f"COCO ID {k} -> YOLO class {v} ({categories[k]})")

for ann in tqdm(coco["annotations"], desc="Converting"):
    img_info = images[ann["image_id"]]

    img_path = os.path.join(DATA_DIR, img_info["file_name"])
    label_path = os.path.join(OUTPUT_LABELS, img_info["file_name"].replace(".jpg", ".txt"))

    # Ensure directory exists
    os.makedirs(os.path.dirname(label_path), exist_ok=True)

    # COCO bbox format → x, y, w, h (pixels)
    x, y, w, h = ann["bbox"]

    # Normalize coordinates
    img_w, img_h = img_info["width"], img_info["height"]
    x_center = (x + w / 2) / img_w
    y_center = (y + h / 2) / img_h
    w /= img_w
    h /= img_h

    class_id = cat_id_to_yolo[ann["category_id"]]

    # Write YOLO label
    with open(label_path, "a") as f:
        f.write(f"{class_id} {x_center} {y_center} {w} {h}\n")

    # Copy image to YOLO folder only once
    dest_img_path = os.path.join(OUTPUT_IMAGES, img_info["file_name"])
    if not os.path.exists(dest_img_path):
        os.makedirs(os.path.dirname(dest_img_path), exist_ok=True)
        try:
            import shutil
            shutil.copy(img_path, dest_img_path)
        except:
            print("Missing:", img_path)