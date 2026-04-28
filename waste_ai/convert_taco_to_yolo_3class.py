import json
import os
import shutil
from pathlib import Path

# --- CONFIGURATION ---
TACO_JSON_PATH = 'TACO/data/annotations.json'
TACO_IMAGE_DIR = 'TACO/data'
TARGET_DIR = 'yolo_dataset'
SPLIT_RATIO = 0.8  # 80% train, 20% val

# Map TACO supercategories/categories to our 3 classes
# 0: plastic, 1: paper, 2: metal
CLASS_MAPPING = {
    'Plastic bag & wrapper': 0, 'Bottle': 0, 'Bottle cap': 0, 'Cup': 0, 'Straw': 0,
    'Paper': 1, 'Carton': 1, 'Corrugated carton': 1, 'Paper bag': 1,
    'Can': 2, 'Aluminium foil': 2, 'Pop tab': 2
}

def convert_coco_bbox_to_yolo(bbox, img_width, img_height):
    # COCO: [x_min, y_min, width, height] in pixels
    # YOLO: [x_center, y_center, width, height] normalized (0 to 1)
    x_min, y_min, w, h = bbox
    x_center = (x_min + w / 2.0) / img_width
    y_center = (y_min + h / 2.0) / img_height
    w_norm = w / img_width
    h_norm = h / img_height
    return x_center, y_center, w_norm, h_norm

def process_taco():
    print("Loading TACO annotations...")
    with open(TACO_JSON_PATH, 'r') as f:
        data = json.load(f)

    # Build category ID to our New Class ID mapping
    cat_id_to_new_class = {}
    for category in data['categories']:
        # Check if the supercategory or category name matches our mapping
        if category['supercategory'] in CLASS_MAPPING:
            cat_id_to_new_class[category['id']] = CLASS_MAPPING[category['supercategory']]
        elif category['name'] in CLASS_MAPPING:
            cat_id_to_new_class[category['id']] = CLASS_MAPPING[category['name']]

    # Group annotations by image
    img_to_anns = {}
    for ann in data['annotations']:
        if ann['category_id'] in cat_id_to_new_class:
            img_id = ann['image_id']
            if img_id not in img_to_anns:
                img_to_anns[img_id] = []
            img_to_anns[img_id].append(ann)

    # Setup directories
    for split in ['train', 'val']:
        os.makedirs(f"{TARGET_DIR}/images/{split}", exist_ok=True)
        os.makedirs(f"{TARGET_DIR}/labels/{split}", exist_ok=True)

    # Process images
    images = [img for img in data['images'] if img['id'] in img_to_anns]
    split_index = int(len(images) * SPLIT_RATIO)
    
    print(f"Found {len(images)} images containing Plastic, Paper, or Metal.")
    print("Converting and copying files...")

    for i, img in enumerate(images):
        split = 'train' if i < split_index else 'val'
        
        # Copy Image
        src_img_path = os.path.join(TACO_IMAGE_DIR, img['file_name'])
        dst_img_path = os.path.join(TARGET_DIR, 'images', split, os.path.basename(img['file_name']))
        
        # Handle TACO's subfolders by flattening them in the destination
        if os.path.exists(src_img_path):
            shutil.copy(src_img_path, dst_img_path)
        else:
            continue # Skip if image is missing from download

        # Write YOLO Label
        label_filename = os.path.splitext(os.path.basename(img['file_name']))[0] + '.txt'
        label_path = os.path.join(TARGET_DIR, 'labels', split, label_filename)
        
        with open(label_path, 'w') as f:
            for ann in img_to_anns[img['id']]:
                new_class_id = cat_id_to_new_class[ann['category_id']]
                x, y, w, h = convert_coco_bbox_to_yolo(ann['bbox'], img['width'], img['height'])
                f.write(f"{new_class_id} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n")

    print("TACO conversion complete! Ready for YOLOv8.")

if __name__ == '__main__':
    process_taco()