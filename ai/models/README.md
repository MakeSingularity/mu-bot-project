# AI Models Directory

This directory contains the compiled Hailo models for the emu droid companion robot.

## Model Files

### Object Detection
- `yolov8n.hef` - YOLOv8 Nano for human detection (11MB, 30 FPS)
- `yolov8s.hef` - YOLOv8 Small for higher accuracy (22MB, 20 FPS)

### Pose Estimation  
- `yolov8n_pose.hef` - Human pose estimation with 17 keypoints (6MB, 20 FPS)

### Custom Emu Models
- `activity_classifier.hef` - Activity classification (standing/walking/running)
- `distance_estimator.hef` - Stereo distance estimation 
- `pose_tracker.hef` - Temporal pose tracking

## Usage

Models are loaded by the Hailo runtime and used by ROS nodes:

```python
from hailo_platform import HailoRTContext

# Load model
hef_file = "yolov8n.hef"
context = HailoRTContext(hef_file)

# Run inference
outputs = context.infer(input_data)
```

## Performance Targets

| Model | Size | FPS | Accuracy | Power |
|-------|------|-----|----------|-------|
| YOLOv8n | 11MB | 30 | 95% | 8W |
| YOLOv8n-pose | 6MB | 20 | 93% | 6W |
| Activity | 2MB | 50 | 98% | 2W |

## Compilation

Models are compiled using the Hailo Dataflow Compiler:

```bash
# Compile all models
python3 ../hailo/compile_hailo_models.py --all

# Verify models
hailortcli benchmark --model yolov8n.hef
```

## Model Sources

- YOLOv8 models: Ultralytics official releases
- Custom models: Trained on emu-specific dataset
- Calibration: COCO validation set + emu walking data