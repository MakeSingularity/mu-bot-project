#!/usr/bin/env python3
"""
Field Test Scripts for Emu Droid Hardware Validation

This script provides comprehensive testing for all emu droid hardware components
including camera stereo pair, servo control, audio input/output, and HAT stack.

Usage:
    python3 field_tests.py --test all
    python3 field_tests.py --test cameras
    python3 field_tests.py --test servos
    python3 field_tests.py --test audio
    python3 field_tests.py --test hailo
"""

import os
import sys
import time
import argparse
import logging
import subprocess
import numpy as np
from typing import Optional, Tuple, List
from pathlib import Path

# Hardware interface imports
try:
    import cv2
    import pyaudio
    import pyttsx3
    import speech_recognition as sr
    from adafruit_pca9685 import PCA9685
    import board
    import busio
    HAS_HARDWARE_LIBS = True
except ImportError as e:
    HAS_HARDWARE_LIBS = False
    print(f"Hardware libraries not available: {e}")
    print("Install with: pip install opencv-python pyaudio pyttsx3 speechrecognition adafruit-circuitpython-pca9685")

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class EmuFieldTester:
    """Comprehensive field testing for emu droid hardware components."""

    def __init__(self):
        self.test_results = {}
        self.pca9685 = None
        self.audio_engine = None
        self.microphone = None

        # Initialize hardware interfaces if available
        if HAS_HARDWARE_LIBS:
            self._init_hardware()

    def _init_hardware(self):
        """Initialize hardware interfaces."""
        try:
            # Initialize I2C for servo control
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca9685 = PCA9685(i2c)
            self.pca9685.frequency = 50  # 50Hz for servos
            logger.info("PCA9685 servo controller initialized")
        except Exception as e:
            logger.warning(f"Failed to initialize PCA9685: {e}")

        try:
            # Initialize TTS engine
            self.audio_engine = pyttsx3.init()
            self.audio_engine.setProperty('rate', 150)
            self.audio_engine.setProperty('volume', 0.8)
            logger.info("TTS engine initialized")
        except Exception as e:
            logger.warning(f"Failed to initialize TTS: {e}")

        try:
            # Initialize microphone
            self.microphone = sr.Microphone()
            logger.info("Microphone initialized")
        except Exception as e:
            logger.warning(f"Failed to initialize microphone: {e}")

    def test_camera_stereo_pair(self) -> bool:
        """Test stereo camera pair functionality."""
        logger.info("Testing stereo camera pair...")

        try:
            # Test left camera (index 0)
            cap_left = cv2.VideoCapture(0)
            if not cap_left.isOpened():
                logger.error("Failed to open left camera (index 0)")
                return False

            # Test right camera (index 1)
            cap_right = cv2.VideoCapture(1)
            if not cap_right.isOpened():
                logger.error("Failed to open right camera (index 1)")
                cap_left.release()
                return False

            # Capture test frames
            ret_left, frame_left = cap_left.read()
            ret_right, frame_right = cap_right.read()

            if not (ret_left and ret_right):
                logger.error("Failed to capture frames from stereo cameras")
                cap_left.release()
                cap_right.release()
                return False

            # Validate frame properties
            logger.info(f"Left camera frame: {frame_left.shape}")
            logger.info(f"Right camera frame: {frame_right.shape}")

            if frame_left.shape != frame_right.shape:
                logger.warning("Stereo camera frame shapes don't match")

            # Save test images
            cv2.imwrite('test_left_camera.jpg', frame_left)
            cv2.imwrite('test_right_camera.jpg', frame_right)
            logger.info("Test images saved: test_left_camera.jpg, test_right_camera.jpg")

            # Test autofocus (if supported)
            self._test_camera_autofocus(cap_left, "left")
            self._test_camera_autofocus(cap_right, "right")

            cap_left.release()
            cap_right.release()

            self.test_results['camera_stereo'] = True
            logger.info("✓ Stereo camera pair test passed")
            return True

        except Exception as e:
            logger.error(f"Stereo camera test failed: {e}")
            self.test_results['camera_stereo'] = False
            return False

    def _test_camera_autofocus(self, cap, camera_name: str):
        """Test camera autofocus functionality."""
        try:
            # Try to set autofocus
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
            autofocus_enabled = cap.get(cv2.CAP_PROP_AUTOFOCUS)
            logger.info(f"{camera_name} camera autofocus: {'enabled' if autofocus_enabled else 'disabled'}")

            # Test focus values
            focus_value = cap.get(cv2.CAP_PROP_FOCUS)
            logger.info(f"{camera_name} camera focus value: {focus_value}")

        except Exception as e:
            logger.warning(f"Autofocus test failed for {camera_name} camera: {e}")

    def test_servo_control(self) -> bool:
        """Test servo motor control via PCA9685."""
        logger.info("Testing servo control...")

        if not self.pca9685:
            logger.error("PCA9685 not initialized - servo test skipped")
            self.test_results['servo_control'] = False
            return False

        try:
            # Test all 12 servos used in emu droid
            servo_channels = range(12)  # Channels 0-11

            for channel in servo_channels:
                logger.info(f"Testing servo on channel {channel}")

                # Move servo to center position (1.5ms pulse)
                self.pca9685.channels[channel].duty_cycle = 0x7FFF
                time.sleep(0.5)

                # Move to minimum position (1ms pulse)
                self.pca9685.channels[channel].duty_cycle = 0x4CCC
                time.sleep(0.5)

                # Move to maximum position (2ms pulse)
                self.pca9685.channels[channel].duty_cycle = 0x9999
                time.sleep(0.5)

                # Return to center
                self.pca9685.channels[channel].duty_cycle = 0x7FFF
                time.sleep(0.5)

                # Turn off servo
                self.pca9685.channels[channel].duty_cycle = 0

            logger.info("✓ All servo channels tested successfully")

            # Test eye platform movement (Stewart platform simulation)
            self._test_eye_platform_movement()

            # Test neck cable control
            self._test_neck_cable_control()

            self.test_results['servo_control'] = True
            return True

        except Exception as e:
            logger.error(f"Servo control test failed: {e}")
            self.test_results['servo_control'] = False
            return False

    def _test_eye_platform_movement(self):
        """Test Stewart platform eye movement."""
        logger.info("Testing eye platform movement...")

        # Left eye platform (channels 0-2)
        left_eye_channels = [0, 1, 2]
        # Right eye platform (channels 3-5)
        right_eye_channels = [3, 4, 5]

        try:
            # Coordinate eye movement pattern
            for step in range(5):
                angle = step * 0.3  # Small movements

                for i, channel in enumerate(left_eye_channels):
                    pulse_width = 0x7FFF + int(0x1000 * np.sin(angle + i * 2.0944))  # 120° phase
                    pulse_width = max(0x4CCC, min(0x9999, pulse_width))  # Clamp to safe range
                    self.pca9685.channels[channel].duty_cycle = pulse_width

                for i, channel in enumerate(right_eye_channels):
                    pulse_width = 0x7FFF + int(0x1000 * np.sin(angle + i * 2.0944))
                    pulse_width = max(0x4CCC, min(0x9999, pulse_width))
                    self.pca9685.channels[channel].duty_cycle = pulse_width

                time.sleep(0.8)

            # Return to center
            for channel in left_eye_channels + right_eye_channels:
                self.pca9685.channels[channel].duty_cycle = 0x7FFF

            logger.info("✓ Eye platform movement test completed")

        except Exception as e:
            logger.warning(f"Eye platform test failed: {e}")

    def _test_neck_cable_control(self):
        """Test neck cable-driven movement."""
        logger.info("Testing neck cable control...")

        # Neck servos (channels 6-11)
        neck_channels = [6, 7, 8, 9, 10, 11]

        try:
            # Test coordinated neck movement
            for movement in range(3):
                # Simulate cable pull patterns
                for i, channel in enumerate(neck_channels):
                    if i < 3:  # Upper neck cables
                        pulse_width = 0x7FFF + int(0x800 * np.sin(movement * 0.5))
                    else:  # Lower neck cables
                        pulse_width = 0x7FFF - int(0x800 * np.sin(movement * 0.5))

                    pulse_width = max(0x4CCC, min(0x9999, pulse_width))
                    self.pca9685.channels[channel].duty_cycle = pulse_width

                time.sleep(1.0)

            # Return to neutral position
            for channel in neck_channels:
                self.pca9685.channels[channel].duty_cycle = 0x7FFF

            logger.info("✓ Neck cable control test completed")

        except Exception as e:
            logger.warning(f"Neck cable test failed: {e}")

    def test_audio_pipeline(self) -> bool:
        """Test complete audio input/output pipeline."""
        logger.info("Testing audio pipeline...")

        # Test Text-to-Speech output
        tts_result = self._test_tts_output()

        # Test microphone input
        mic_result = self._test_microphone_input()

        # Test speech recognition
        speech_result = self._test_speech_recognition()

        # Test observe-and-report callback
        callback_result = self._test_observe_callback()

        overall_result = tts_result and mic_result
        self.test_results['audio_pipeline'] = overall_result

        if overall_result:
            logger.info("✓ Audio pipeline test passed")
        else:
            logger.error("✗ Audio pipeline test failed")

        return overall_result

    def _test_tts_output(self) -> bool:
        """Test text-to-speech output."""
        if not self.audio_engine:
            logger.warning("TTS engine not available")
            return False

        try:
            test_messages = [
                "Emu droid initialization complete",
                "Human detected at 3.2 meters - walking at 2.1 meters per second",
                "No human activity detected - entering standby mode",
                "System test in progress"
            ]

            logger.info("Testing TTS output with sample messages...")
            for message in test_messages:
                logger.info(f"Speaking: {message}")
                self.audio_engine.say(message)
                self.audio_engine.runAndWait()
                time.sleep(1)

            logger.info("✓ TTS output test completed")
            return True

        except Exception as e:
            logger.error(f"TTS test failed: {e}")
            return False

    def _test_microphone_input(self) -> bool:
        """Test microphone input recording."""
        if not self.microphone:
            logger.warning("Microphone not available")
            return False

        try:
            # Test microphone recording
            recognizer = sr.Recognizer()

            logger.info("Testing microphone input (5 second recording)...")
            with self.microphone as source:
                recognizer.adjust_for_ambient_noise(source, duration=1)
                logger.info("Listening...")
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)

            # Save recorded audio for analysis
            with open("test_microphone_recording.wav", "wb") as f:
                f.write(audio.get_wav_data())

            logger.info("✓ Microphone recording saved: test_microphone_recording.wav")
            return True

        except Exception as e:
            logger.error(f"Microphone test failed: {e}")
            return False

    def _test_speech_recognition(self) -> bool:
        """Test speech recognition functionality."""
        logger.info("Testing speech recognition (say something)...")

        if not self.microphone:
            logger.warning("Microphone not available for speech recognition")
            return False

        try:
            recognizer = sr.Recognizer()

            with self.microphone as source:
                recognizer.adjust_for_ambient_noise(source, duration=1)
                logger.info("Say something...")
                audio = recognizer.listen(source, timeout=10, phrase_time_limit=5)

            # Attempt speech recognition
            try:
                text = recognizer.recognize_google(audio)
                logger.info(f"✓ Speech recognized: '{text}'")
                return True
            except sr.UnknownValueError:
                logger.warning("Speech not understood")
                return False
            except sr.RequestError as e:
                logger.warning(f"Speech recognition service error: {e}")
                return False

        except Exception as e:
            logger.error(f"Speech recognition test failed: {e}")
            return False

    def _test_observe_callback(self) -> bool:
        """Test observe-and-report callback function."""
        logger.info("Testing observe-and-report callback...")

        try:
            # Simulate detection events
            detection_events = [
                {"type": "human_detected", "distance": 3.2, "speed": 2.1, "activity": "walking"},
                {"type": "human_detected", "distance": 5.8, "speed": 4.3, "activity": "running"},
                {"type": "human_lost", "last_seen": 2.5},
                {"type": "standby", "timeout": 30}
            ]

            for event in detection_events:
                report = self._generate_report(event)
                logger.info(f"Generated report: {report}")

                if self.audio_engine:
                    self.audio_engine.say(report)
                    self.audio_engine.runAndWait()

                time.sleep(2)

            logger.info("✓ Observe-and-report callback test completed")
            return True

        except Exception as e:
            logger.error(f"Observe callback test failed: {e}")
            return False

    def _generate_report(self, event: dict) -> str:
        """Generate human-readable report from detection event."""
        if event["type"] == "human_detected":
            distance = event["distance"]
            speed = event["speed"]
            activity = event["activity"]
            return f"Human detected at {distance:.1f} meters - {activity} at {speed:.1f} meters per second"

        elif event["type"] == "human_lost":
            last_seen = event["last_seen"]
            return f"Human tracking lost - last seen {last_seen:.1f} seconds ago"

        elif event["type"] == "standby":
            timeout = event["timeout"]
            return f"No human activity detected - entering standby mode for {timeout} seconds"

        else:
            return "Unknown detection event"

    def test_hailo_ai_hat(self) -> bool:
        """Test Hailo AI HAT+ functionality."""
        logger.info("Testing Hailo AI HAT+...")

        try:
            # Test Hailo device detection
            result = subprocess.run(['hailortcli', 'scan'],
                                  capture_output=True, text=True, timeout=30)

            if result.returncode == 0:
                logger.info("✓ Hailo device detected:")
                logger.info(result.stdout)

                # Test model benchmark if available
                self._test_hailo_benchmark()

                self.test_results['hailo_ai'] = True
                return True
            else:
                logger.error("✗ Hailo device not detected")
                logger.error(result.stderr)
                self.test_results['hailo_ai'] = False
                return False

        except subprocess.TimeoutExpired:
            logger.error("Hailo scan timed out")
            self.test_results['hailo_ai'] = False
            return False
        except FileNotFoundError:
            logger.warning("hailortcli not found - Hailo SDK not installed")
            self.test_results['hailo_ai'] = False
            return False
        except Exception as e:
            logger.error(f"Hailo test failed: {e}")
            self.test_results['hailo_ai'] = False
            return False

    def _test_hailo_benchmark(self):
        """Run Hailo model benchmark if models are available."""
        model_dir = Path("ai/models")
        if model_dir.exists():
            hef_files = list(model_dir.glob("*.hef"))

            if hef_files:
                logger.info(f"Found {len(hef_files)} compiled Hailo models")

                # Test first available model
                test_model = hef_files[0]
                logger.info(f"Testing model: {test_model}")

                try:
                    result = subprocess.run(['hailortcli', 'benchmark', str(test_model)],
                                          capture_output=True, text=True, timeout=60)

                    if result.returncode == 0:
                        logger.info("✓ Hailo model benchmark completed")
                        logger.info(result.stdout[:500])  # First 500 chars
                    else:
                        logger.warning("Hailo benchmark failed")
                        logger.warning(result.stderr)

                except subprocess.TimeoutExpired:
                    logger.warning("Hailo benchmark timed out")
                except Exception as e:
                    logger.warning(f"Hailo benchmark error: {e}")
            else:
                logger.info("No compiled Hailo models found for testing")
        else:
            logger.info("Hailo models directory not found")

    def test_i2c_devices(self) -> bool:
        """Test I2C device detection and communication."""
        logger.info("Testing I2C devices...")

        try:
            # Run i2cdetect to scan for devices
            result = subprocess.run(['i2cdetect', '-y', '1'],
                                  capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                logger.info("I2C device scan:")
                logger.info(result.stdout)

                # Look for expected devices
                output = result.stdout.lower()
                devices_found = {}

                # Check for common I2C addresses
                if '1a' in output:
                    devices_found['WM8960 Audio'] = '0x1a'
                if '40' in output:
                    devices_found['PCA9685 PWM'] = '0x40'
                if '48' in output:
                    devices_found['Possible sensor'] = '0x48'

                if devices_found:
                    logger.info("✓ I2C devices detected:")
                    for device, addr in devices_found.items():
                        logger.info(f"  {device} at {addr}")
                else:
                    logger.warning("No expected I2C devices found")

                self.test_results['i2c_devices'] = True
                return True
            else:
                logger.error("I2C scan failed")
                logger.error(result.stderr)
                self.test_results['i2c_devices'] = False
                return False

        except FileNotFoundError:
            logger.warning("i2cdetect not found - install i2c-tools")
            self.test_results['i2c_devices'] = False
            return False
        except Exception as e:
            logger.error(f"I2C test failed: {e}")
            self.test_results['i2c_devices'] = False
            return False

    def run_all_tests(self) -> dict:
        """Run complete hardware validation test suite."""
        logger.info("Starting complete emu droid hardware validation...")

        test_functions = [
            ('I2C Devices', self.test_i2c_devices),
            ('Camera Stereo Pair', self.test_camera_stereo_pair),
            ('Servo Control', self.test_servo_control),
            ('Audio Pipeline', self.test_audio_pipeline),
            ('Hailo AI HAT+', self.test_hailo_ai_hat),
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

        # Generate test report
        self._generate_test_report()

        return self.test_results

    def _generate_test_report(self):
        """Generate comprehensive test report."""
        logger.info(f"\n{'='*60}")
        logger.info("EMU DROID HARDWARE VALIDATION REPORT")
        logger.info(f"{'='*60}")

        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result)

        logger.info(f"Overall Result: {passed_tests}/{total_tests} tests passed")
        logger.info(f"Success Rate: {(passed_tests/total_tests)*100:.1f}%")
        logger.info("")

        for test_name, result in self.test_results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            logger.info(f"{test_name.replace('_', ' ').title():<25} {status}")

        # System readiness assessment
        critical_tests = ['camera_stereo_pair', 'servo_control', 'audio_pipeline']
        critical_passed = all(self.test_results.get(test, False) for test in critical_tests)

        logger.info("")
        if critical_passed:
            logger.info("🎉 System Status: READY FOR FIELD DEPLOYMENT")
        else:
            logger.info("⚠️  System Status: REQUIRES ATTENTION")
            failed_critical = [test for test in critical_tests if not self.test_results.get(test, False)]
            logger.info(f"Critical failures: {', '.join(failed_critical)}")

        logger.info(f"{'='*60}")


def main():
    """Main entry point for field testing."""
    parser = argparse.ArgumentParser(description='Emu Droid Hardware Field Tests')
    parser.add_argument('--test', choices=['all', 'cameras', 'servos', 'audio', 'hailo', 'i2c'],
                       default='all', help='Test to run')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    if not HAS_HARDWARE_LIBS:
        logger.error("Required hardware libraries not available")
        logger.error("Install with: pip install -r requirements.txt")
        sys.exit(1)

    # Initialize tester
    tester = EmuFieldTester()

    # Run specified tests
    if args.test == 'all':
        results = tester.run_all_tests()
    elif args.test == 'cameras':
        results = {'cameras': tester.test_camera_stereo_pair()}
    elif args.test == 'servos':
        results = {'servos': tester.test_servo_control()}
    elif args.test == 'audio':
        results = {'audio': tester.test_audio_pipeline()}
    elif args.test == 'hailo':
        results = {'hailo': tester.test_hailo_ai_hat()}
    elif args.test == 'i2c':
        results = {'i2c': tester.test_i2c_devices()}

    # Exit with appropriate code
    if all(results.values()):
        logger.info("All tests passed! 🎉")
        sys.exit(0)
    else:
        logger.error("Some tests failed! ⚠️")
        sys.exit(1)


if __name__ == '__main__':
    main()
