std_msgs/Header header

# 2D detection from YOLO
float32 confidence_detector        # YOLO detection confidence
float32[4] bbox                    # [x1, y1, x2, y2] in image pixels

# Classification result
string label                       # "recyclable" or "non_recyclable"
float32 confidence_classifier      # Classifier confidence on the crop (0.0 = not yet classified)

# 3D position in base_link frame (from depth projection)
geometry_msgs/Point position       # x, y, z in metres relative to base_link

# Whether 3D position is valid (depth data available and finite)
bool has_3d_position