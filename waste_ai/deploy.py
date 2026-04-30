import cv2
from ultralytics import YOLO

def run_vision_node():
    # 1. Load your custom trained "brain"
    # Make sure best.pt is in the models folder
    model_path = "models/300_epoch_best.pt"
    print(f"Loading model from {model_path}...")
    model = YOLO(model_path)

    # 2. Initialize the webcam (Index 0 is usually the built-in or first USB camera)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam. Check your USB connection.")
        return

    print("Camera active. Press 'q' in the video window to shut down.")

    # 3. The Inference Loop
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame. Exiting...")
            break

        # Run the YOLOv8 model on the current frame
        # Setting conf=0.4 filters out weak guesses (below 40% confidence)
        results = model(frame, conf=0.4, verbose=False)

        # Extract the frame with bounding boxes drawn on it
        annotated_frame = results[0].plot()

        # Display the live feed
        cv2.imshow("AMMR Real-Time Waste Detection", annotated_frame)

        # Listen for the 'q' key to quit cleanly
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 4. Clean up resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_vision_node()