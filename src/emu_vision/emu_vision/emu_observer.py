#!/usr/bin/env python3
"""
Emu Observer Node - Core vision processing for emu droid companion robot

This node subscribes to camera feeds, performs AI inference using Hailo models,
and publishes detection/tracking results for human observation and reporting.

Key features:
- Stereo camera processing for depth perception
- Human pose estimation and tracking
- Speed/distance calculation for walking/running detection
- Energy-efficient processing with configurable frame rates
- Hailo AI HAT+ integration for low-latency inference

Author: Emu Bot Team
License: MIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from typing import Optional, Tuple, List

# Hailo SDK imports (placeholder - will be replaced with actual Hailo SDK)
try:
    # from hailo_sdk_client import InferenceContext
    # from hailo_platform import HailoRTContext
    HAILO_AVAILABLE = False  # Set to True when Hailo SDK is properly installed
except ImportError:
    HAILO_AVAILABLE = False


class EmuObserver(Node):
    """
    Main vision processing node for emu droid companion robot.
    
    Processes stereo camera feeds to detect and track humans for observe-and-report tasks.
    Optimized for 1-5 m/s human movement detection with energy-efficient processing.
    """
    
    def __init__(self):
        super().__init__('emu_observer')
        
        # Initialize parameters
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('processing_fps', 10.0)  # Energy-efficient processing
        self.declare_parameter('detection_confidence', 0.7)
        self.declare_parameter('tracking_distance_max', 10.0)  # meters
        self.declare_parameter('speed_threshold_walk', 1.0)    # m/s
        self.declare_parameter('speed_threshold_run', 3.0)     # m/s
        self.declare_parameter('stereo_baseline', 0.12)        # meters between cameras
        self.declare_parameter('camera_focal_length', 3.04)    # mm for OV5647
        
        # Get parameters
        self.processing_fps = self.get_parameter('processing_fps').get_parameter_value().double_value
        self.detection_confidence = self.get_parameter('detection_confidence').get_parameter_value().double_value
        self.tracking_distance_max = self.get_parameter('tracking_distance_max').get_parameter_value().double_value
        self.speed_threshold_walk = self.get_parameter('speed_threshold_walk').get_parameter_value().double_value
        self.speed_threshold_run = self.get_parameter('speed_threshold_run').get_parameter_value().double_value
        self.stereo_baseline = self.get_parameter('stereo_baseline').get_parameter_value().double_value
        self.camera_focal_length = self.get_parameter('camera_focal_length').get_parameter_value().double_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera subscribers
        self.left_camera_sub = self.create_subscription(
            Image, '/camera/left/image_raw', self.left_camera_callback, 10)
        self.right_camera_sub = self.create_subscription(
            Image, '/camera/right/image_raw', self.right_camera_callback, 10)
        self.left_info_sub = self.create_subscription(
            CameraInfo, '/camera/left/camera_info', self.left_info_callback, 10)
        self.right_info_sub = self.create_subscription(
            CameraInfo, '/camera/right/camera_info', self.right_info_callback, 10)
        
        # Publishers
        self.report_pub = self.create_publisher(String, '/emu/report', 10)
        self.human_position_pub = self.create_publisher(Point, '/emu/human_position', 10)
        self.human_velocity_pub = self.create_publisher(Twist, '/emu/human_velocity', 10)
        self.detection_image_pub = self.create_publisher(Image, '/emu/detection_image', 10)
        
        # State variables
        self.left_image = None
        self.right_image = None
        self.left_camera_info = None
        self.right_camera_info = None
        self.last_process_time = 0.0
        self.process_interval = 1.0 / self.processing_fps
        
        # Tracking state
        self.last_human_position = None
        self.last_human_time = None
        self.human_velocity = None
        
        # Initialize Hailo inference context if available
        self.hailo_context = None
        if HAILO_AVAILABLE:
            self.init_hailo_inference()
        
        self.get_logger().info(f'Emu Observer initialized - Processing at {self.processing_fps} FPS')
        self.get_logger().info(f'Hailo AI available: {HAILO_AVAILABLE}')
    
    def init_hailo_inference(self):
        """Initialize Hailo AI inference context for pose estimation and object detection."""
        try:
            # Placeholder for Hailo initialization
            # self.hailo_context = HailoRTContext()
            # Load models: pose estimation (COCO keypoints) and object detection (YOLO)
            self.get_logger().info('Hailo AI inference context initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Hailo AI: {e}')
            self.hailo_context = None
    
    def left_camera_callback(self, msg: Image):
        """Process left camera image."""
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_stereo_pair()
        except Exception as e:
            self.get_logger().error(f'Left camera callback error: {e}')
    
    def right_camera_callback(self, msg: Image):
        """Process right camera image."""
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Right camera callback error: {e}')
    
    def left_info_callback(self, msg: CameraInfo):
        """Store left camera calibration info."""
        self.left_camera_info = msg
    
    def right_info_callback(self, msg: CameraInfo):
        """Store right camera calibration info."""
        self.right_camera_info = msg
    
    def process_stereo_pair(self):
        """Process stereo camera pair for human detection and tracking."""
        current_time = time.time()
        
        # Energy-efficient processing: skip frames if processing too fast
        if current_time - self.last_process_time < self.process_interval:
            return
        
        if self.left_image is None or self.right_image is None:
            return
        
        try:
            # Perform human detection and pose estimation
            detections = self.detect_humans(self.left_image)
            
            if detections:
                # Calculate 3D position using stereo depth
                human_3d_position = self.calculate_stereo_depth(detections[0])
                
                if human_3d_position is not None:
                    # Track human movement and calculate velocity
                    self.update_human_tracking(human_3d_position, current_time)
                    
                    # Generate report based on human activity
                    self.generate_activity_report()
                    
                    # Publish detection results
                    self.publish_detection_results(human_3d_position)
                    
                    # Publish annotated image
                    annotated_image = self.annotate_detection_image(self.left_image, detections)
                    self.publish_detection_image(annotated_image)
            
            self.last_process_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Stereo processing error: {e}')
    
    def detect_humans(self, image: np.ndarray) -> List[dict]:
        """
        Detect humans in the image using Hailo AI or OpenCV fallback.
        
        Returns:
            List of detection dictionaries with bounding boxes and keypoints
        """
        detections = []
        
        if HAILO_AVAILABLE and self.hailo_context is not None:
            # Use Hailo AI for efficient inference
            detections = self.hailo_detect_humans(image)
        else:
            # Fallback to OpenCV-based detection
            detections = self.opencv_detect_humans(image)
        
        return detections
    
    def hailo_detect_humans(self, image: np.ndarray) -> List[dict]:
        """Perform human detection using Hailo AI models."""
        # Placeholder for Hailo inference
        # This would use the actual Hailo SDK to run YOLO and pose estimation
        detections = []
        
        try:
            # Example Hailo inference workflow:
            # 1. Preprocess image for Hailo model input
            # 2. Run inference on Hailo AI HAT+
            # 3. Post-process results to extract human detections and poses
            
            # For now, return mock detection for development
            if image is not None:
                h, w = image.shape[:2]
                mock_detection = {
                    'bbox': [w//4, h//4, w//2, h//2],  # x, y, width, height
                    'confidence': 0.9,
                    'keypoints': self.generate_mock_keypoints(w//2, h//2),
                    'class': 'person'
                }
                detections.append(mock_detection)
                
        except Exception as e:
            self.get_logger().error(f'Hailo detection error: {e}')
        
        return detections
    
    def opencv_detect_humans(self, image: np.ndarray) -> List[dict]:
        """Fallback human detection using OpenCV."""
        detections = []
        
        try:
            # Use OpenCV's HOG person detector as fallback
            hog = cv2.HOGDescriptorDetectMultiScale()
            hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            
            # Detect people
            boxes, weights = hog.detectMultiScale(image, winStride=(8,8), padding=(32,32), scale=1.05)
            
            for (x, y, w, h), weight in zip(boxes, weights):
                if weight > self.detection_confidence:
                    detection = {
                        'bbox': [x, y, w, h],
                        'confidence': float(weight),
                        'keypoints': None,  # HOG doesn't provide keypoints
                        'class': 'person'
                    }
                    detections.append(detection)
                    
        except Exception as e:
            self.get_logger().error(f'OpenCV detection error: {e}')
        
        return detections
    
    def generate_mock_keypoints(self, center_x: int, center_y: int) -> List[Tuple[int, int]]:
        """Generate mock keypoints for development."""
        # COCO keypoint format: 17 keypoints
        keypoints = []
        for i in range(17):
            x = center_x + (i - 8) * 10
            y = center_y + (i % 3 - 1) * 20
            keypoints.append((x, y))
        return keypoints
    
    def calculate_stereo_depth(self, detection: dict) -> Optional[Point]:
        """Calculate 3D position using stereo vision."""
        if self.right_image is None or self.left_camera_info is None:
            return None
        
        try:
            # Extract bounding box center
            bbox = detection['bbox']
            center_x = bbox[0] + bbox[2] // 2
            center_y = bbox[1] + bbox[3] // 2
            
            # Simplified stereo depth calculation
            # In practice, would use proper stereo rectification and matching
            focal_length = self.left_camera_info.k[0]  # fx
            
            # Mock disparity calculation (would be from stereo matching)
            disparity = 50.0  # pixels
            
            if disparity > 0:
                # Calculate depth: Z = (focal_length * baseline) / disparity
                depth = (focal_length * self.stereo_baseline) / disparity
                
                # Convert to 3D coordinates
                point = Point()
                point.x = (center_x - self.left_camera_info.k[2]) * depth / focal_length
                point.y = (center_y - self.left_camera_info.k[5]) * depth / focal_length
                point.z = depth
                
                return point
                
        except Exception as e:
            self.get_logger().error(f'Stereo depth calculation error: {e}')
        
        return None
    
    def update_human_tracking(self, position: Point, current_time: float):
        """Update human tracking state and calculate velocity."""
        if self.last_human_position is not None and self.last_human_time is not None:
            dt = current_time - self.last_human_time
            if dt > 0:
                # Calculate velocity
                dx = position.x - self.last_human_position.x
                dy = position.y - self.last_human_position.y
                dz = position.z - self.last_human_position.z
                
                velocity_x = dx / dt
                velocity_y = dy / dt
                velocity_z = dz / dt
                
                # Calculate speed (magnitude)
                speed = np.sqrt(velocity_x**2 + velocity_y**2)
                
                # Update velocity
                self.human_velocity = Twist()
                self.human_velocity.linear.x = velocity_x
                self.human_velocity.linear.y = velocity_y
                self.human_velocity.linear.z = velocity_z
                
                self.get_logger().debug(f'Human speed: {speed:.2f} m/s')
        
        self.last_human_position = position
        self.last_human_time = current_time
    
    def generate_activity_report(self):
        """Generate human activity report based on tracking data."""
        if self.human_velocity is None:
            return
        
        # Calculate speed
        speed = np.sqrt(self.human_velocity.linear.x**2 + self.human_velocity.linear.y**2)
        distance = self.last_human_position.z if self.last_human_position else 0.0
        
        # Generate report
        report = ""
        if speed < self.speed_threshold_walk:
            report = f"Human detected at {distance:.1f}m - Standing or slow movement ({speed:.1f} m/s)"
        elif speed < self.speed_threshold_run:
            report = f"Human detected at {distance:.1f}m - Walking ({speed:.1f} m/s)"
        else:
            report = f"Human detected at {distance:.1f}m - Running ({speed:.1f} m/s)"
        
        # Publish report
        msg = String()
        msg.data = report
        self.report_pub.publish(msg)
        
        self.get_logger().info(f'Activity report: {report}')
    
    def publish_detection_results(self, position: Point):
        """Publish human position and velocity."""
        self.human_position_pub.publish(position)
        
        if self.human_velocity is not None:
            self.human_velocity_pub.publish(self.human_velocity)
    
    def annotate_detection_image(self, image: np.ndarray, detections: List[dict]) -> np.ndarray:
        """Annotate image with detection results."""
        annotated = image.copy()
        
        for detection in detections:
            bbox = detection['bbox']
            confidence = detection['confidence']
            
            # Draw bounding box
            cv2.rectangle(annotated, 
                         (bbox[0], bbox[1]), 
                         (bbox[0] + bbox[2], bbox[1] + bbox[3]), 
                         (0, 255, 0), 2)
            
            # Draw confidence
            cv2.putText(annotated, f'Person {confidence:.2f}', 
                       (bbox[0], bbox[1] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw keypoints if available
            if detection['keypoints'] is not None:
                for kp in detection['keypoints']:
                    cv2.circle(annotated, kp, 3, (255, 0, 0), -1)
        
        return annotated
    
    def publish_detection_image(self, image: np.ndarray):
        """Publish annotated detection image."""
        try:
            msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            self.detection_image_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Image publishing error: {e}')


def main(args=None):
    """Main entry point for emu_observer node."""
    rclpy.init(args=args)
    
    try:
        node = EmuObserver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in emu_observer: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()