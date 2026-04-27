#!/usr/bin/env python3
"""
This is for exporting model to onnx, run from cika_ws

export_model.py
Usage:
    python3 export_model.py --type efficientnet --input models/classifier/efficientnetb0_finetuned.pth --output models/classifier/efficientnetb0.onnx
    python3 export_model.py --type resnet50 --input models/classifier/resnet50_finetuned.pth --output models/classifier/resnet50.onnx
    python3 export_model.py --type yolo --input models/detector/kaggle_run_100_epochs.pt --output models/detector/kaggle_run_100_epochs.onnx
"""

import argparse
import torch
import torch.nn as nn

def export_yolo(input_path, output_path):
    from ultralytics import YOLO
    model = YOLO(input_path)
    model.export(format='onnx', imgsz=640, simplify=True)
    print(f"Exported YOLO to {output_path}")

def export_efficientnet(input_path, output_path):
    from torchvision.models import efficientnet_b0
    model = efficientnet_b0(weights=None)
    model.classifier[1] = nn.Linear(model.classifier[1].in_features, 2)
    checkpoint = torch.load(input_path, map_location="cpu")
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    dummy = torch.randn(1, 3, 224, 224)
    torch.onnx.export(model, dummy, output_path,
        input_names=["input"], output_names=["output"],
        dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}})
    print(f"Exported EfficientNet to {output_path}")

def export_resnet50(input_path, output_path):
    from torchvision.models import resnet50
    model = resnet50(weights=None)
    model.fc = nn.Linear(model.fc.in_features, 2)
    checkpoint = torch.load(input_path, map_location="cpu")
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    dummy = torch.randn(1, 3, 224, 224)
    torch.onnx.export(model, dummy, output_path,
        input_names=["input"], output_names=["output"],
        dynamic_axes={"input": {0: "batch"}, "output": {0: "batch"}})
    print(f"Exported ResNet50 to {output_path}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--type", required=True, choices=["yolo", "efficientnet", "resnet50"])
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    if args.type == "yolo":
        export_yolo(args.input, args.output)
    elif args.type == "efficientnet":
        export_efficientnet(args.input, args.output)
    elif args.type == "resnet50":
        export_resnet50(args.input, args.output)

if __name__ == "__main__":
    main()