# cika_perception/msg/WasteDetection.msg
string label                       # "plastic", "paper", or "metal"
float32 confidence                 # Single YOLO confidence score
float32[4] bbox                    # [xmin, ymin, xmax, ymax] in pixels

# 3D position (Ready for your SLAM setup later)
geometry_msgs/Point position       
bool has_3d_position