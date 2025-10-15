#!/usr/bin/env python3
"""
Hailo Model Compilation Script for Emu Droid

This script downloads, converts, and compiles PyTorch/ONNX models 
for deployment on the Hailo AI HAT+ (26 TOPS) accelerator.

Supported models:
- YOLOv8n for object detection (human tracking)
- YOLOv8n-pose for pose estimation
- Custom emu-optimized models

Requirements:
- Hailo Dataflow Compiler (DFC)
- Hailo Model Zoo
- Python 3.8+

Usage:
    python3 compile_hailo_models.py --model yolov8n
    python3 compile_hailo_models.py --model yolov8n_pose  
    python3 compile_hailo_models.py --all
"""

import os
import sys
import argparse
import subprocess
import urllib.request
import logging
from pathlib import Path
from typing import Dict, List, Optional

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Model configurations
MODELS_CONFIG = {
    'yolov8n': {
        'description': 'YOLOv8 Nano - Object detection (11MB)',
        'source_url': 'https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt',
        'input_shape': [1, 3, 640, 640],
        'output_format': 'detection',
        'classes': 80,  # COCO classes
        'target_fps': 30,
        'accuracy_target': 0.95  # Relative to original model
    },
    'yolov8n_pose': {
        'description': 'YOLOv8 Nano Pose - Human pose estimation (6MB)',
        'source_url': 'https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n-pose.pt',
        'input_shape': [1, 3, 640, 640],
        'output_format': 'pose',
        'keypoints': 17,  # COCO pose keypoints
        'target_fps': 20,
        'accuracy_target': 0.93
    },
    'yolov8s': {
        'description': 'YOLOv8 Small - Higher accuracy object detection (22MB)',
        'source_url': 'https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt',
        'input_shape': [1, 3, 640, 640],
        'output_format': 'detection',
        'classes': 80,
        'target_fps': 20,
        'accuracy_target': 0.97
    }
}

class HailoModelCompiler:
    """Handles compilation of models for Hailo AI HAT+."""
    
    def __init__(self, models_dir: str = './models'):
        self.models_dir = Path(models_dir)
        self.models_dir.mkdir(exist_ok=True)
        
        # Hailo paths
        self.hailo_env = self._detect_hailo_environment()
        self.dfc_path = self._find_hailo_dfc()
        
    def _detect_hailo_environment(self) -> Optional[str]:
        """Detect Hailo development environment."""
        hailo_paths = [
            '/opt/hailo',
            '/usr/local/hailo', 
            os.path.expanduser('~/hailo-ai')
        ]
        
        for path in hailo_paths:
            if os.path.exists(path):
                logger.info(f"Found Hailo environment at: {path}")
                return path
                
        logger.warning("Hailo environment not found. Please install Hailo DFC.")
        return None
    
    def _find_hailo_dfc(self) -> Optional[str]:
        """Find Hailo Dataflow Compiler executable."""
        dfc_paths = [
            'hailo',  # In PATH
            '/opt/hailo/bin/hailo',
            '/usr/local/bin/hailo'
        ]
        
        for dfc_path in dfc_paths:
            try:
                result = subprocess.run([dfc_path, '--version'], 
                                      capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    logger.info(f"Found Hailo DFC: {dfc_path}")
                    return dfc_path
            except (subprocess.TimeoutExpired, FileNotFoundError):
                continue
                
        logger.error("Hailo DFC not found in PATH or standard locations")
        return None
    
    def download_model(self, model_name: str) -> Path:
        """Download PyTorch model from Ultralytics."""
        if model_name not in MODELS_CONFIG:
            raise ValueError(f"Unknown model: {model_name}")
        
        config = MODELS_CONFIG[model_name]
        model_path = self.models_dir / f"{model_name}.pt"
        
        if model_path.exists():
            logger.info(f"Model {model_name} already exists at {model_path}")
            return model_path
        
        logger.info(f"Downloading {model_name} from {config['source_url']}")
        
        try:
            urllib.request.urlretrieve(config['source_url'], model_path)
            logger.info(f"Downloaded {model_name} to {model_path}")
            return model_path
        except Exception as e:
            logger.error(f"Failed to download {model_name}: {e}")
            raise
    
    def export_to_onnx(self, model_name: str, pytorch_path: Path) -> Path:
        """Export PyTorch model to ONNX format."""
        onnx_path = self.models_dir / f"{model_name}.onnx"
        
        if onnx_path.exists():
            logger.info(f"ONNX model already exists: {onnx_path}")
            return onnx_path
        
        logger.info(f"Exporting {model_name} to ONNX format")
        
        # Use ultralytics export if available
        try:
            from ultralytics import YOLO
            model = YOLO(str(pytorch_path))
            model.export(format='onnx', imgsz=640, simplify=True)
            
            # Move exported file to desired location
            exported_onnx = pytorch_path.with_suffix('.onnx')
            if exported_onnx.exists():
                exported_onnx.rename(onnx_path)
                logger.info(f"Exported ONNX model to {onnx_path}")
                return onnx_path
                
        except ImportError:
            logger.warning("Ultralytics not available, using torch.onnx export")
            
        # Fallback to manual ONNX export
        return self._manual_onnx_export(model_name, pytorch_path, onnx_path)
    
    def _manual_onnx_export(self, model_name: str, pytorch_path: Path, onnx_path: Path) -> Path:
        """Manual ONNX export using torch.onnx."""
        try:
            import torch
            import torch.onnx
            
            # Load PyTorch model
            model = torch.load(pytorch_path, map_location='cpu')
            model.eval()
            
            # Create dummy input
            config = MODELS_CONFIG[model_name]
            dummy_input = torch.randn(*config['input_shape'])
            
            # Export to ONNX
            torch.onnx.export(
                model, dummy_input, str(onnx_path),
                export_params=True,
                opset_version=11,
                do_constant_folding=True,
                input_names=['input'],
                output_names=['output'],
                dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
            )
            
            logger.info(f"Manual ONNX export completed: {onnx_path}")
            return onnx_path
            
        except Exception as e:
            logger.error(f"Manual ONNX export failed: {e}")
            raise
    
    def compile_hailo_model(self, model_name: str, onnx_path: Path) -> Path:
        """Compile ONNX model to Hailo HEF format."""
        if not self.dfc_path:
            raise RuntimeError("Hailo DFC not available")
        
        hef_path = self.models_dir / f"{model_name}.hef"
        
        if hef_path.exists():
            logger.info(f"HEF model already exists: {hef_path}")
            return hef_path
        
        logger.info(f"Compiling {model_name} to Hailo HEF format")
        
        config = MODELS_CONFIG[model_name]
        
        # Hailo compilation command
        cmd = [
            self.dfc_path, 'compile',
            '--ckpt', str(onnx_path),
            '--calib-data', self._get_calibration_data(model_name),
            '--yaml', self._generate_model_yaml(model_name),
            '--har', str(self.models_dir / f"{model_name}.har"),
            '--performance'
        ]
        
        try:
            logger.info(f"Running Hailo compilation: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
            
            if result.returncode == 0:
                logger.info(f"Hailo compilation successful: {hef_path}")
                return hef_path
            else:
                logger.error(f"Hailo compilation failed: {result.stderr}")
                raise RuntimeError(f"Compilation failed: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            logger.error("Hailo compilation timed out")
            raise
        except Exception as e:
            logger.error(f"Hailo compilation error: {e}")
            raise
    
    def _get_calibration_data(self, model_name: str) -> str:
        """Get calibration dataset for quantization."""
        # For now, return placeholder. In practice, would use COCO validation set
        calib_dir = self.models_dir / 'calibration'
        calib_dir.mkdir(exist_ok=True)
        
        calib_file = calib_dir / f"{model_name}_calib.npy"
        
        if not calib_file.exists():
            logger.info(f"Generating mock calibration data for {model_name}")
            import numpy as np
            
            config = MODELS_CONFIG[model_name]
            # Generate random calibration data (replace with real images)
            calib_data = np.random.randint(0, 255, (100, *config['input_shape'][1:]), dtype=np.uint8)
            np.save(calib_file, calib_data)
            
        return str(calib_file)
    
    def _generate_model_yaml(self, model_name: str) -> str:
        """Generate Hailo model configuration YAML."""
        yaml_path = self.models_dir / f"{model_name}_config.yaml"
        
        if yaml_path.exists():
            return str(yaml_path)
        
        config = MODELS_CONFIG[model_name]
        
        yaml_content = f'''
# Hailo Model Configuration for {model_name}
model_name: {model_name}
architecture: yolov8
input_shape: {config['input_shape']}
output_format: {config['output_format']}
target_fps: {config['target_fps']}
accuracy_target: {config['accuracy_target']}

preprocessing:
  - resize: [640, 640]
  - normalize: 
      mean: [0.485, 0.456, 0.406]
      std: [0.229, 0.224, 0.225]

postprocessing:
  nms_score_threshold: 0.25
  nms_iou_threshold: 0.45
  max_output_boxes: 300

quantization:
  precision: int8
  calibration_samples: 100
  
optimization:
  batch_size: 1
  target_platform: hailo8
  '''
        
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)
            
        logger.info(f"Generated model config: {yaml_path}")
        return str(yaml_path)
    
    def compile_model(self, model_name: str) -> Path:
        """Complete model compilation pipeline."""
        logger.info(f"Starting compilation pipeline for {model_name}")
        
        try:
            # Step 1: Download PyTorch model
            pytorch_path = self.download_model(model_name)
            
            # Step 2: Export to ONNX
            onnx_path = self.export_to_onnx(model_name, pytorch_path)
            
            # Step 3: Compile to HEF
            hef_path = self.compile_hailo_model(model_name, onnx_path)
            
            logger.info(f"Model compilation completed: {hef_path}")
            return hef_path
            
        except Exception as e:
            logger.error(f"Model compilation failed: {e}")
            raise
    
    def compile_all_models(self) -> Dict[str, Path]:
        """Compile all supported models."""
        results = {}
        
        for model_name in MODELS_CONFIG.keys():
            try:
                hef_path = self.compile_model(model_name)
                results[model_name] = hef_path
            except Exception as e:
                logger.error(f"Failed to compile {model_name}: {e}")
                results[model_name] = None
                
        return results
    
    def verify_model(self, model_name: str, hef_path: Path) -> bool:
        """Verify compiled model works correctly."""
        if not self.dfc_path:
            logger.warning("Cannot verify model - Hailo DFC not available")
            return False
            
        logger.info(f"Verifying model {model_name}")
        
        try:
            # Run model info command
            cmd = [self.dfc_path, 'parse', str(hef_path)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                logger.info(f"Model {model_name} verification passed")
                return True
            else:
                logger.error(f"Model verification failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Model verification error: {e}")
            return False


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Compile models for Hailo AI HAT+')
    parser.add_argument('--model', choices=list(MODELS_CONFIG.keys()), 
                       help='Model to compile')
    parser.add_argument('--all', action='store_true', 
                       help='Compile all models')
    parser.add_argument('--models-dir', default='./models',
                       help='Directory to store models')
    parser.add_argument('--verify', action='store_true',
                       help='Verify compiled models')
    
    args = parser.parse_args()
    
    if not args.model and not args.all:
        parser.error("Specify --model or --all")
    
    # Initialize compiler
    compiler = HailoModelCompiler(args.models_dir)
    
    try:
        if args.all:
            results = compiler.compile_all_models()
            
            # Print summary
            print("\nCompilation Summary:")
            for model_name, hef_path in results.items():
                status = "✓ Success" if hef_path else "✗ Failed"
                print(f"  {model_name}: {status}")
                
                if args.verify and hef_path:
                    verify_result = compiler.verify_model(model_name, hef_path)
                    verify_status = "✓ Verified" if verify_result else "✗ Verification Failed"
                    print(f"    Verification: {verify_status}")
        
        else:
            hef_path = compiler.compile_model(args.model)
            print(f"\nModel compiled successfully: {hef_path}")
            
            if args.verify:
                verify_result = compiler.verify_model(args.model, hef_path)
                print(f"Verification: {'✓ Passed' if verify_result else '✗ Failed'}")
    
    except Exception as e:
        logger.error(f"Compilation failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()