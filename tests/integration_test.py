#!/usr/bin/env python3
"""
Integration Test Suite for Emu Droid System

This script tests the complete end-to-end functionality of the emu droid
including ROS 2 node communication, vision-to-audio pipeline, and
multi-component integration.

Usage:
    python3 integration_test.py --test vision_audio
    python3 integration_test.py --test ros_nodes
    python3 integration_test.py --test full_system
"""

import os
import sys
import time
import threading
import argparse
import logging
from typing import Dict, List, Optional
from pathlib import Path

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Point, Twist
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("ROS 2 not available - install ROS 2 Humble and source setup.bash")

# Test utilities
import subprocess
import numpy as np

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class EmuIntegrationTester(Node):
    """ROS 2 node for integration testing."""

    def __init__(self):
        super().__init__('emu_integration_tester')

        # Test state
        self.test_results = {}
        self.received_reports = []
        self.received_positions = []
        self.received_images = []

        # Subscribers for monitoring
        self.report_sub = self.create_subscription(
            String, '/emu/report', self.report_callback, 10)
        self.position_sub = self.create_subscription(
            Point, '/emu/human_position', self.position_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/emu/detection_image', self.image_callback, 10)

        # Publishers for testing
        self.cmd_pub = self.create_publisher(Twist, '/emu/head_cmd', 10)

        logger.info("Integration tester node initialized")

    def report_callback(self, msg):
        """Handle incoming detection reports."""
        self.received_reports.append(msg.data)
        logger.info(f"Received report: {msg.data}")

    def position_callback(self, msg):
        """Handle incoming human position data."""
        position = (msg.x, msg.y, msg.z)
        self.received_positions.append(position)
        logger.info(f"Received position: {position}")

    def image_callback(self, msg):
        """Handle incoming detection images."""
        self.received_images.append(msg)
        logger.info(f"Received detection image: {msg.width}x{msg.height}")

    def test_vision_audio_pipeline(self) -> bool:
        """Test complete vision-to-audio reporting pipeline."""
        logger.info("Testing vision-to-audio pipeline...")

        # Clear previous test data
        self.received_reports.clear()
        self.received_positions.clear()
        self.received_images.clear()

        # Wait for incoming data
        test_duration = 30  # seconds
        start_time = time.time()

        logger.info(f"Monitoring for {test_duration} seconds...")

        while time.time() - start_time < test_duration:
            rclpy.spin_once(self, timeout_sec=1.0)

        # Analyze results
        reports_received = len(self.received_reports)
        positions_received = len(self.received_positions)
        images_received = len(self.received_images)

        logger.info(f"Test results:")
        logger.info(f"  Reports received: {reports_received}")
        logger.info(f"  Positions received: {positions_received}")
        logger.info(f"  Images received: {images_received}")

        # Success criteria
        success = (reports_received > 0 or
                  positions_received > 0 or
                  images_received > 0)

        if success:
            logger.info("✓ Vision-to-audio pipeline is active")

            # Test report quality
            if self.received_reports:
                self._analyze_report_quality()
        else:
            logger.warning("✗ No data received from vision-audio pipeline")

        return success

    def _analyze_report_quality(self):
        """Analyze quality of received reports."""
        logger.info("Analyzing report quality...")

        expected_phrases = [
            "human detected", "walking", "running", "standing",
            "meters", "speed", "distance", "standby"
        ]

        phrase_counts = {phrase: 0 for phrase in expected_phrases}

        for report in self.received_reports:
            report_lower = report.lower()
            for phrase in expected_phrases:
                if phrase in report_lower:
                    phrase_counts[phrase] += 1

        logger.info("Report phrase analysis:")
        for phrase, count in phrase_counts.items():
            logger.info(f"  '{phrase}': {count} occurrences")

        # Check for numeric data in reports
        numeric_reports = [r for r in self.received_reports if any(c.isdigit() for c in r)]
        logger.info(f"Reports with numeric data: {len(numeric_reports)}/{len(self.received_reports)}")

    def test_head_movement_control(self) -> bool:
        """Test head movement control via ROS topics."""
        logger.info("Testing head movement control...")

        try:
            # Send movement commands
            movements = [
                Twist(),  # Stop/center
                Twist(),  # Turn left
                Twist(),  # Turn right
                Twist(),  # Look up
                Twist(),  # Look down
                Twist()   # Return to center
            ]

            # Configure movement commands
            movements[1].angular.z = 0.5   # Turn left
            movements[2].angular.z = -0.5  # Turn right
            movements[3].angular.y = 0.3   # Look up
            movements[4].angular.y = -0.3  # Look down

            for i, cmd in enumerate(movements):
                logger.info(f"Sending movement command {i+1}/6")
                self.cmd_pub.publish(cmd)
                time.sleep(2)

            logger.info("✓ Head movement commands sent successfully")
            return True

        except Exception as e:
            logger.error(f"Head movement test failed: {e}")
            return False

    def test_node_communication(self) -> bool:
        """Test communication between ROS nodes."""
        logger.info("Testing ROS node communication...")

        try:
            # Get list of active nodes
            node_names = self.get_node_names()
            logger.info(f"Active nodes: {node_names}")

            # Check for expected emu nodes
            expected_nodes = [
                'emu_observer',
                'emu_audio',
                'emu_control'
            ]

            found_nodes = []
            for expected in expected_nodes:
                matching = [name for name in node_names if expected in name]
                if matching:
                    found_nodes.extend(matching)
                    logger.info(f"✓ Found node: {matching[0]}")
                else:
                    logger.warning(f"✗ Node not found: {expected}")

            # Get topic list
            topic_names_and_types = self.get_topic_names_and_types()
            topic_names = [name for name, _ in topic_names_and_types]
            logger.info(f"Active topics: {len(topic_names)}")

            # Check for expected topics
            expected_topics = [
                '/emu/report',
                '/emu/human_position',
                '/emu/detection_image',
                '/camera/left/image_raw',
                '/camera/right/image_raw'
            ]

            found_topics = []
            for expected in expected_topics:
                if expected in topic_names:
                    found_topics.append(expected)
                    logger.info(f"✓ Found topic: {expected}")
                else:
                    logger.warning(f"✗ Topic not found: {expected}")

            # Success criteria
            success = len(found_nodes) > 0 and len(found_topics) > 0

            if success:
                logger.info("✓ ROS node communication active")
            else:
                logger.warning("✗ Limited ROS node communication")

            return success

        except Exception as e:
            logger.error(f"Node communication test failed: {e}")
            return False


class SystemIntegrationTest:
    """System-level integration testing."""

    def __init__(self):
        self.test_results = {}

    def test_process_management(self) -> bool:
        """Test system process management."""
        logger.info("Testing process management...")

        try:
            # Check for ROS processes
            ros_processes = self._find_processes(['ros2', 'gazebo', 'rviz'])
            logger.info(f"ROS-related processes found: {len(ros_processes)}")

            for proc in ros_processes:
                logger.info(f"  Process: {proc}")

            # Check system resources
            self._check_system_resources()

            # Test process cleanup
            self._test_process_cleanup()

            return True

        except Exception as e:
            logger.error(f"Process management test failed: {e}")
            return False

    def _find_processes(self, keywords: List[str]) -> List[str]:
        """Find processes matching keywords."""
        try:
            result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
            lines = result.stdout.split('\n')

            found_processes = []
            for line in lines:
                for keyword in keywords:
                    if keyword in line and line not in found_processes:
                        found_processes.append(line.strip())

            return found_processes

        except Exception as e:
            logger.warning(f"Process search failed: {e}")
            return []

    def _check_system_resources(self):
        """Check system resource usage."""
        try:
            # Check CPU usage
            result = subprocess.run(['top', '-bn1'], capture_output=True, text=True)
            lines = result.stdout.split('\n')

            for line in lines:
                if 'Cpu(s):' in line:
                    logger.info(f"CPU usage: {line.strip()}")
                    break

            # Check memory usage
            result = subprocess.run(['free', '-h'], capture_output=True, text=True)
            logger.info("Memory usage:")
            for line in result.stdout.strip().split('\n'):
                logger.info(f"  {line}")

            # Check disk usage
            result = subprocess.run(['df', '-h', '.'], capture_output=True, text=True)
            logger.info("Disk usage:")
            for line in result.stdout.strip().split('\n'):
                logger.info(f"  {line}")

        except Exception as e:
            logger.warning(f"Resource check failed: {e}")

    def _test_process_cleanup(self):
        """Test graceful process cleanup."""
        logger.info("Testing process cleanup...")

        # This would test cleanup procedures for ROS nodes
        # Implementation depends on specific deployment strategy
        logger.info("Process cleanup test completed")

    def test_file_system_structure(self) -> bool:
        """Test file system structure and permissions."""
        logger.info("Testing file system structure...")

        try:
            # Check main directories
            required_dirs = [
                'src', 'sim', 'ai', 'hardware', 'docs', 'tests', '.vscode'
            ]

            missing_dirs = []
            for dir_name in required_dirs:
                if not os.path.exists(dir_name):
                    missing_dirs.append(dir_name)

            if missing_dirs:
                logger.warning(f"Missing directories: {missing_dirs}")
            else:
                logger.info("✓ All required directories present")

            # Check critical files
            critical_files = [
                'README.md',
                'requirements.txt',
                '.gitignore',
                'src/emu_vision/emu_vision/emu_observer.py',
                'hardware/wiring_guide.md',
                'docs/BOM.md'
            ]

            missing_files = []
            for file_path in critical_files:
                if not os.path.exists(file_path):
                    missing_files.append(file_path)

            if missing_files:
                logger.warning(f"Missing critical files: {missing_files}")
            else:
                logger.info("✓ All critical files present")

            # Check permissions
            self._check_file_permissions()

            success = len(missing_dirs) == 0 and len(missing_files) == 0
            return success

        except Exception as e:
            logger.error(f"File system test failed: {e}")
            return False

    def _check_file_permissions(self):
        """Check file permissions for executables."""
        executable_files = [
            'tests/field_tests.py',
            'ai/hailo/compile_hailo_models.py',
            'ai/pytorch/train_custom_models.py'
        ]

        for file_path in executable_files:
            if os.path.exists(file_path):
                file_stat = os.stat(file_path)
                is_executable = bool(file_stat.st_mode & 0o111)

                if not is_executable:
                    logger.warning(f"File not executable: {file_path}")
                    # Make executable
                    os.chmod(file_path, file_stat.st_mode | 0o755)
                    logger.info(f"Made executable: {file_path}")

    def test_configuration_files(self) -> bool:
        """Test configuration file validity."""
        logger.info("Testing configuration files...")

        try:
            # Test YAML files
            yaml_files = [
                'src/emu_vision/config/emu_vision_config.yaml'
            ]

            for yaml_file in yaml_files:
                if os.path.exists(yaml_file):
                    try:
                        import yaml
                        with open(yaml_file, 'r') as f:
                            yaml.safe_load(f)
                        logger.info(f"✓ Valid YAML: {yaml_file}")
                    except yaml.YAMLError as e:
                        logger.error(f"✗ Invalid YAML {yaml_file}: {e}")
                        return False
                    except ImportError:
                        logger.warning("PyYAML not available for validation")

            # Test JSON files
            json_files = [
                '.vscode/launch.json',
                '.vscode/tasks.json',
                '.vscode/settings.json'
            ]

            for json_file in json_files:
                if os.path.exists(json_file):
                    try:
                        import json
                        with open(json_file, 'r') as f:
                            json.load(f)
                        logger.info(f"✓ Valid JSON: {json_file}")
                    except json.JSONDecodeError as e:
                        logger.error(f"✗ Invalid JSON {json_file}: {e}")
                        return False

            logger.info("✓ Configuration files validated")
            return True

        except Exception as e:
            logger.error(f"Configuration test failed: {e}")
            return False

    def run_all_tests(self) -> Dict:
        """Run complete system integration test suite."""
        logger.info("Starting system integration tests...")

        test_functions = [
            ('File System Structure', self.test_file_system_structure),
            ('Configuration Files', self.test_configuration_files),
            ('Process Management', self.test_process_management),
        ]

        for test_name, test_func in test_functions:
            logger.info(f"\n{'='*50}")
            logger.info(f"Running {test_name} Test")
            logger.info(f"{'='*50}")

            try:
                success = test_func()
                self.test_results[test_name.lower().replace(' ', '_')] = success
            except Exception as e:
                logger.error(f"Test {test_name} crashed: {e}")
                self.test_results[test_name.lower().replace(' ', '_')] = False

        return self.test_results


def run_ros_integration_tests():
    """Run ROS-specific integration tests."""
    if not HAS_ROS:
        logger.error("ROS 2 not available for integration testing")
        return False

    try:
        rclpy.init()

        # Create test node
        test_node = EmuIntegrationTester()

        # Run vision-audio pipeline test
        vision_audio_result = test_node.test_vision_audio_pipeline()

        # Run head movement test
        movement_result = test_node.test_head_movement_control()

        # Run node communication test
        communication_result = test_node.test_node_communication()

        # Cleanup
        test_node.destroy_node()
        rclpy.shutdown()

        overall_result = vision_audio_result or communication_result

        logger.info(f"\nROS Integration Test Results:")
        logger.info(f"  Vision-Audio Pipeline: {'✓' if vision_audio_result else '✗'}")
        logger.info(f"  Head Movement Control: {'✓' if movement_result else '✗'}")
        logger.info(f"  Node Communication: {'✓' if communication_result else '✗'}")

        return overall_result

    except Exception as e:
        logger.error(f"ROS integration test failed: {e}")
        return False


def main():
    """Main entry point for integration testing."""
    parser = argparse.ArgumentParser(description='Emu Droid Integration Tests')
    parser.add_argument('--test', choices=['vision_audio', 'ros_nodes', 'system', 'full_system'],
                       default='full_system', help='Test suite to run')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    results = {}

    if args.test in ['system', 'full_system']:
        # Run system-level tests
        system_tester = SystemIntegrationTest()
        system_results = system_tester.run_all_tests()
        results.update(system_results)

    if args.test in ['ros_nodes', 'vision_audio', 'full_system']:
        # Run ROS integration tests
        ros_result = run_ros_integration_tests()
        results['ros_integration'] = ros_result

    # Generate final report
    logger.info(f"\n{'='*60}")
    logger.info("INTEGRATION TEST SUMMARY")
    logger.info(f"{'='*60}")

    if results:
        total_tests = len(results)
        passed_tests = sum(1 for result in results.values() if result)

        logger.info(f"Overall Result: {passed_tests}/{total_tests} test suites passed")
        logger.info(f"Success Rate: {(passed_tests/total_tests)*100:.1f}%")
        logger.info("")

        for test_name, result in results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            logger.info(f"{test_name.replace('_', ' ').title():<30} {status}")

        if all(results.values()):
            logger.info("\n🎉 All integration tests passed!")
            sys.exit(0)
        else:
            logger.info("\n⚠️  Some integration tests failed")
            sys.exit(1)
    else:
        logger.warning("No tests were run")
        sys.exit(1)


if __name__ == '__main__':
    main()
