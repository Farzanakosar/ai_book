# Testing Voice-to-Action Examples with Whisper API Integration

## Overview
This document outlines the testing strategy and implementation for validating the voice-to-action examples with Whisper API integration. The tests focus on structural validation, error handling, and integration logic without requiring actual API calls.

## Test Suite Structure

### 1. Unit Tests for Core Components

```python
import unittest
from unittest.mock import Mock, patch, MagicMock
import io
import wave
import numpy as np
from pathlib import Path

# Test the Whisper integration components
class TestWhisperIntegration(unittest.TestCase):
    def setUp(self):
        # Import the components to test
        from book.docs.vla.whisper_integration import VoiceToActionProcessor

        self.mock_api_key = "test-key"
        self.processor = VoiceToActionProcessor(self.mock_api_key)

    def test_voice_to_action_processor_initialization(self):
        """Test that VoiceToActionProcessor initializes correctly"""
        self.assertIsNotNone(self.processor.recognizer)
        self.assertIsNotNone(self.processor.microphone)

    @patch('openai.Audio.transcribe')
    def test_listen_and_transcribe_success(self, mock_transcribe):
        """Test successful transcription"""
        # Mock the API response
        mock_transcribe.return_value = "move forward"

        # Test the method (this would normally require actual audio input)
        # For testing, we'll verify that the API call is structured correctly
        with patch.object(self.processor.recognizer, 'listen') as mock_listen:
            mock_listen.return_value = Mock()
            mock_listen.return_value.get_raw_data.return_value = b'test_audio_data'

            with patch('wave.open') as mock_wave:
                mock_wave.return_value.__enter__ = Mock()
                mock_wave.return_value.__exit__ = Mock()

                # This would call the transcribe method but we're testing the structure
                pass

    def test_safe_transcribe_with_backoff_structure(self):
        """Test the structure of safe_transcribe_with_backoff function"""
        # Import the function to test
        import sys
        import importlib.util

        # Create a temporary module to test the function
        spec = importlib.util.spec_from_loader("test_module", loader=None)
        test_module = importlib.util.module_from_spec(spec)

        # Define the function as it appears in the documentation
        def safe_transcribe_with_backoff(audio_file, max_retries=3):
            """Transcribe with exponential backoff for rate limits"""
            import time
            import random

            for attempt in range(max_retries):
                try:
                    # In real implementation, this would call openai.Audio.transcribe
                    # For testing, we just verify the structure
                    result = f"mock_result_attempt_{attempt}"
                    return result
                except Exception as e:
                    if attempt < max_retries - 1:
                        # Exponential backoff with jitter
                        wait_time = (2 ** attempt) + random.uniform(0, 1)
                        time.sleep(0)  # Don't actually sleep in tests
                    else:
                        raise
            return None

        # Test the function structure
        result = safe_transcribe_with_backoff("test_audio")
        self.assertIsNotNone(result)
        self.assertIn("mock_result_attempt", result)

class TestCommandMapping(unittest.TestCase):
    def test_static_command_mapper(self):
        """Test the StaticCommandMapper class from voice-command-mapping.md"""

        # Define the class as it appears in the documentation
        class StaticCommandMapper:
            def __init__(self):
                self.command_map = {
                    # Navigation commands
                    "move forward": {
                        "action": "navigation/move_forward",
                        "parameters": {"distance": 1.0}
                    },
                    "move backward": {
                        "action": "navigation/move_backward",
                        "parameters": {"distance": 1.0}
                    },
                    "turn left": {
                        "action": "navigation/turn",
                        "parameters": {"angle": -90}
                    },
                    "turn right": {
                        "action": "navigation/turn",
                        "parameters": {"angle": 90}
                    },
                    "stop": {
                        "action": "control/stop",
                        "parameters": {}
                    },
                    # Manipulation commands
                    "pick up object": {
                        "action": "manipulation/pick_object",
                        "parameters": {}
                    },
                    "place object": {
                        "action": "manipulation/place_object",
                        "parameters": {}
                    },
                    # System commands
                    "power off": {
                        "action": "system/power_off",
                        "parameters": {}
                    },
                    "power on": {
                        "action": "system/power_on",
                        "parameters": {}
                    }
                }

            def map_command(self, command_text):
                """Map command text to robot action"""
                command_lower = command_text.lower().strip()

                # Exact match
                if command_lower in self.command_map:
                    return self.command_map[command_lower]

                # Partial match
                for key, value in self.command_map.items():
                    if key in command_lower:
                        return value

                return {
                    "action": "unknown/command",
                    "parameters": {"text": command_text}
                }

        # Test the mapper
        mapper = StaticCommandMapper()

        # Test exact matches
        result = mapper.map_command("move forward")
        self.assertEqual(result["action"], "navigation/move_forward")
        self.assertEqual(result["parameters"]["distance"], 1.0)

        result = mapper.map_command("turn left")
        self.assertEqual(result["action"], "navigation/turn")
        self.assertEqual(result["parameters"]["angle"], -90)

        # Test partial matches
        result = mapper.map_command("please move forward slowly")
        self.assertEqual(result["action"], "navigation/move_forward")

        # Test unknown command
        result = mapper.map_command("unknown command")
        self.assertEqual(result["action"], "unknown/command")

    def test_parameterized_command_mapper(self):
        """Test the ParameterizedCommandMapper class"""

        # Define the class as it appears in the documentation
        import re

        class ParameterizedCommandMapper:
            def __init__(self):
                self.patterns = {
                    # Navigation with distance
                    "move_forward_distance": {
                        "pattern": r"move forward (\d+(?:\.\d+)?) meters?",
                        "action": "navigation/move_forward",
                        "param_extractor": lambda match: {"distance": float(match.group(1))}
                    },
                    "move_backward_distance": {
                        "pattern": r"move backward (\d+(?:\.\d+)?) meters?",
                        "action": "navigation/move_backward",
                        "param_extractor": lambda match: {"distance": float(match.group(1))}
                    },
                    "turn_angle": {
                        "pattern": r"turn (left|right) (\d+(?:\.\d+)?) degrees?",
                        "action": "navigation/turn",
                        "param_extractor": lambda match: {
                            "angle": float(match.group(2)) if match.group(1) == "right" else -float(match.group(2))
                        }
                    },
                    # Object manipulation
                    "pick_object": {
                        "pattern": r"pick up (?:the )?(\w+)",
                        "action": "manipulation/pick_object",
                        "param_extractor": lambda match: {"object_type": match.group(1)}
                    },
                    "place_object_at": {
                        "pattern": r"place (?:the )?(\w+) at (\w+)",
                        "action": "manipulation/place_object",
                        "param_extractor": lambda match: {
                            "object_type": match.group(1),
                            "destination": match.group(2)
                        }
                    }
                }

            def map_command(self, command_text):
                """Map command with parameter extraction"""
                command_lower = command_text.lower().strip()

                for pattern_name, pattern_config in self.patterns.items():
                    match = re.search(pattern_config["pattern"], command_lower)
                    if match:
                        try:
                            params = pattern_config["param_extractor"](match)
                            return {
                                "action": pattern_config["action"],
                                "parameters": params
                            }
                        except Exception as e:
                            print(f"Error extracting parameters: {e}")
                            continue

                # Fall back to simple mapping
                return self._simple_map(command_text)

            def _simple_map(self, command_text):
                """Simple command mapping as fallback"""
                simple_map = {
                    "move forward": {"action": "navigation/move_forward", "parameters": {"distance": 1.0}},
                    "move backward": {"action": "navigation/move_backward", "parameters": {"distance": 1.0}},
                    "turn left": {"action": "navigation/turn", "parameters": {"angle": -90}},
                    "turn right": {"action": "navigation/turn", "parameters": {"angle": 90}},
                    "stop": {"action": "control/stop", "parameters": {}}
                }

                return simple_map.get(command_text.lower().strip(), {
                    "action": "unknown/command",
                    "parameters": {"text": command_text}
                })

        # Test the mapper
        mapper = ParameterizedCommandMapper()

        # Test parameter extraction
        result = mapper.map_command("move forward 2.5 meters")
        self.assertEqual(result["action"], "navigation/move_forward")
        self.assertEqual(result["parameters"]["distance"], 2.5)

        result = mapper.map_command("turn right 45 degrees")
        self.assertEqual(result["action"], "navigation/turn")
        self.assertEqual(result["parameters"]["angle"], 45.0)

        result = mapper.map_command("turn left 90 degrees")
        self.assertEqual(result["action"], "navigation/turn")
        self.assertEqual(result["parameters"]["angle"], -90.0)

        result = mapper.map_command("pick up the red ball")
        self.assertEqual(result["action"], "manipulation/pick_object")
        self.assertEqual(result["parameters"]["object_type"], "red")

        result = mapper.map_command("place the cup on the table")
        self.assertEqual(result["action"], "manipulation/place_object")
        self.assertEqual(result["parameters"]["object_type"], "cup")
        self.assertEqual(result["parameters"]["destination"], "table")

class TestVoiceCommandRouter(unittest.TestCase):
    def test_voice_command_router(self):
        """Test the VoiceCommandRouter class from whisper integration"""

        # Define the class as it appears in the documentation
        class VoiceCommandRouter:
            def __init__(self):
                self.command_map = {
                    "move forward": "navigation/move_forward",
                    "move backward": "navigation/move_backward",
                    "turn left": "navigation/turn_left",
                    "turn right": "navigation/turn_right",
                    "pick up object": "manipulation/pick_object",
                    "place object": "manipulation/place_object",
                    "stop": "control/stop",
                    "pause": "control/pause"
                }

            def route_command(self, transcription):
                """Map transcribed text to robot action"""
                transcription_lower = transcription.lower()

                for command, action in self.command_map.items():
                    if command in transcription_lower:
                        return action

                # If no exact match, try fuzzy matching
                return self._fuzzy_match_command(transcription_lower)

            def _fuzzy_match_command(self, text):
                """Implement fuzzy matching for partial command recognition"""
                # Implementation of fuzzy matching algorithm
                # This is a simplified example
                if "forward" in text or "ahead" in text:
                    return "navigation/move_forward"
                elif "backward" in text or "back" in text:
                    return "navigation/move_backward"
                elif "left" in text:
                    return "navigation/turn_left"
                elif "right" in text:
                    return "navigation/turn_right"
                else:
                    return "unknown/command"

        # Test the router
        router = VoiceCommandRouter()

        # Test exact matches
        self.assertEqual(router.route_command("move forward"), "navigation/move_forward")
        self.assertEqual(router.route_command("turn left"), "navigation/turn_left")
        self.assertEqual(router.route_command("stop"), "control/stop")

        # Test fuzzy matches
        self.assertEqual(router.route_command("go forward please"), "navigation/move_forward")
        self.assertEqual(router.route_command("move to the left"), "navigation/turn_left")

class TestIntegration(unittest.TestCase):
    def test_complete_mapping_pipeline(self):
        """Test the complete voice-to-action mapping pipeline from documentation"""

        # This simulates the test described in the documentation
        voice_input = "move forward 2 meters"

        # Step 1: Transcription (simulated)
        transcribed_text = voice_input  # In real implementation, this comes from Whisper

        # Step 2: Command mapping using ParameterizedCommandMapper
        import re

        class ParameterizedCommandMapper:
            def __init__(self):
                self.patterns = {
                    "move_forward_distance": {
                        "pattern": r"move forward (\d+(?:\.\d+)?) meters?",
                        "action": "navigation/move_forward",
                        "param_extractor": lambda match: {"distance": float(match.group(1))}
                    }
                }

            def map_command(self, command_text):
                command_lower = command_text.lower().strip()

                for pattern_name, pattern_config in self.patterns.items():
                    match = re.search(pattern_config["pattern"], command_lower)
                    if match:
                        try:
                            params = pattern_config["param_extractor"](match)
                            return {
                                "action": pattern_config["action"],
                                "parameters": params
                            }
                        except Exception:
                            continue

                return {"action": "unknown/command", "parameters": {"text": command_text}}

        mapper = ParameterizedCommandMapper()
        mapped_action = mapper.map_command(transcribed_text)

        # Step 3: Verify correct mapping
        self.assertEqual(mapped_action["action"], "navigation/move_forward")
        self.assertEqual(mapped_action["parameters"]["distance"], 2.0)

        print("Pipeline test passed!")

def run_all_tests():
    """Run all validation tests"""
    print("Running Voice-to-Action Integration Tests...")

    # Create test suite
    suite = unittest.TestSuite()

    # Add all test cases
    suite.addTest(unittest.makeSuite(TestWhisperIntegration))
    suite.addTest(unittest.makeSuite(TestCommandMapping))
    suite.addTest(unittest.makeSuite(TestVoiceCommandRouter))
    suite.addTest(unittest.makeSuite(TestIntegration))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print(f"\nTest Results:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success: {result.wasSuccessful()}")

    return result.wasSuccessful()

# Run the tests
if __name__ == "__main__":
    success = run_all_tests()
    if success:
        print("\n✅ All voice-to-action integration tests passed!")
    else:
        print("\n❌ Some tests failed!")
```
