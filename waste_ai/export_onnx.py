from ultralytics import YOLO

# 1. Load your custom-trained production weights
model = YOLO("models/best_2.pt")

# 2. Export to ONNX with VPU-compatible flags
success = model.export(
    format="onnx",      
    imgsz=640,          
    dynamic=False,      # Required for OAK-D
    simplify=True,      
    opset=12            
)

print("VPU-Ready ONNX Export completed successfully!")
