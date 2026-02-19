import json
import os 
import shutil
from pathlib import Path
from sklearn.model_selection import train_test_split
from tqdm import tqdm

SCRIPT_DIR = Path(__file__).parent.resolve()

BASE_DIR = SCRIPT_DIR / "data" / "TACO" / "data"
ANNOT = BASE_DIR / "annotations.json"

OUTPUT_DIR = BASE_DIR / "taco_yolo"
IMAGES_OUT = OUTPUT_DIR / "images"
LABELS_OUT = OUTPUT_DIR / "labels"

for split in ["train", "val"]:
    (IMAGES_OUT / split).mkdir(parents=True, exist_ok=True)
    (LABELS_OUT / split).mkdir(parents=True, exist_ok=True)

# Class mapping: 0 -> non_recyclable, 1 -> recyclable

RECYCLABLE = {
    "Can",
    "Glass bottle",
    "Other plastic bottle",
    "Plastic bottle",
    "Glass jar",
    "Metal can",
    "Drink can",
    "Cardboard",
    "Paper",
    "Paper bag",
    "Beverage carton",
    "Food carton",
    "Plastic lid",
    "Metal lid",
    "Aluminium foil"
}

def map_class(category_name):
    return 1 if category_name in RECYCLABLE else 0

print("Loading annotations...")
with open(ANNOT, "r", encoding="utf-8") as f:
    data = json.load(f)

images = {img["id"]: img for img in data["images"]}
categories = {cat["id"]: cat["name"] for cat in data["categories"]}

img_to_anns = {}
for ann in data["annotations"]:
    img_to_anns.setdefault(ann["image_id"], []).append(ann)
    
img_ids = list(images.keys())

train_ids, val_ids = train_test_split(img_ids, test_size=0.2, random_state=42)

def coco_bbox_to_yolo(bbox, img_w, img_h):
    x, y, w, h = bbox
    x_center = (x + w/2) / img_w
    y_center = (y + h/2) / img_h
    return x_center, y_center, w/img_w, h/img_h


def process_split(ids, split="train"):
    for img_id in tqdm(ids, desc=f"Processing {split}"):
        img_info = images[img_id]
        file_name = img_info["file_name"]
        src_path = BASE_DIR / file_name

        dst_img = IMAGES_OUT / split / file_name
        dst_img.parent.mkdir(parents=True, exist_ok=True)

        shutil.copy(src_path, dst_img)

        label_path = LABELS_OUT / split / (file_name.rsplit(".", 1)[0] + ".txt")
        label_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(label_path, "w") as f:
            for ann in img_to_anns.get(img_id, []):
                cat_name = categories[ann["category_id"]]
                cls = map_class(cat_name)

                yolo_box = coco_bbox_to_yolo(
                    ann["bbox"],
                    img_info["width"],
                    img_info["height"]
                )
                
                f.write(f"{cls} " + " ".join(map(str, yolo_box)) + "\n")

process_split(train_ids, "train")
process_split(val_ids, "val")

print("\n✔ Conversion complete!")
print("YOLO dataset created at:", OUTPUT_DIR)