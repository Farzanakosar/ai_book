# Voice Command Mapping Guide

## Overview

This guide covers how to map natural language voice commands to specific robot actions. Effective command mapping is crucial for creating intuitive human-robot interactions where spoken commands reliably translate to robot behaviors.

## Understanding Command Mapping

Command mapping is the process of converting recognized speech (text) into executable robot actions. This involves:

1. **Natural Language Understanding**: Parsing the intent from spoken commands
2. **Action Mapping**: Converting intents to specific robot commands
3. **Parameter Extraction**: Identifying parameters like distances, objects, or locations
4. **Action Execution**: Executing the mapped robot action

## Basic Command Mapping Architecture

```
Voice Command → Intent Recognition → Parameter Extraction → Action Mapping → Robot Execution
```

## Simple Command Mapping

### 1. Static Command Mapping

The simplest approach is to create a static mapping between voice commands and robot actions:

```python
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
```

### 2. Fuzzy Command Matching

For more robust command recognition, implement fuzzy matching:

```python
import difflib

class FuzzyCommandMapper:
    def __init__(self, threshold=0.7):
        self.threshold = threshold
        self.commands = [
            "move forward", "move backward", "turn left", "turn right",
            "stop", "pick up object", "place object", "power off", "power on"
        ]

        # Extended command variations
        self.command_variations = {
            "move forward": ["move forward", "go forward", "move ahead", "go ahead", "forward"],
            "move backward": ["move backward", "go backward", "move back", "go back", "backward", "back"],
            "turn left": ["turn left", "turn to the left", "left turn", "turn left", "left"],
            "turn right": ["turn right", "turn to the right", "right turn", "turn right", "right"],
            "stop": ["stop", "halt", "freeze", "pause"],
            "pick up object": ["pick up object", "pick up", "grasp", "grab", "take object"],
            "place object": ["place object", "place", "put down", "release", "drop"],
        }

    def map_command(self, command_text):
        """Map command with fuzzy matching"""
        command_lower = command_text.lower().strip()

        # Check for exact matches first
        for canonical_command, variations in self.command_variations.items():
            if command_lower in variations:
                # Return the canonical command structure
                return self._get_command_structure(canonical_command)

        # Fuzzy matching
        best_matches = difflib.get_close_matches(
            command_lower,
            [cmd for variations in self.command_variations.values() for cmd in variations],
            n=1,
            cutoff=self.threshold
        )

        if best_matches:
            # Find the canonical command for the matched variation
            matched_variation = best_matches[0]
            for canonical_command, variations in self.command_variations.items():
                if matched_variation in variations:
                    return self._get_command_structure(canonical_command)

        return {
            "action": "unknown/command",
            "parameters": {"text": command_text, "confidence": 0.0}
        }

    def _get_command_structure(self, canonical_command):
        """Get the action structure for a canonical command"""
        base_map = {
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
            "pick up object": {
                "action": "manipulation/pick_object",
                "parameters": {}
            },
            "place object": {
                "action": "manipulation/place_object",
                "parameters": {}
            }
        }
        return base_map.get(canonical_command, {
            "action": "unknown/command",
            "parameters": {"text": canonical_command}
        })
```

## Advanced Command Mapping with Parameters

### 1. Parameter Extraction

Many commands require parameters like distances, angles, or object names:

```python
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
```

### 2. Context-Aware Command Mapping

Commands may have different meanings based on context:

```python
class ContextAwareCommandMapper:
    def __init__(self):
        self.context = {}
        self.command_processors = {
            "navigation": self._process_navigation_command,
            "manipulation": self._process_manipulation_command,
            "system": self._process_system_command
        }

    def set_context(self, context):
        """Set current robot context"""
        self.context = context

    def map_command(self, command_text):
        """Map command based on current context"""
        command_lower = command_text.lower().strip()

        # Determine context based on current state or command content
        context_type = self._determine_context_type(command_lower)

        if context_type in self.command_processors:
            return self.command_processors[context_type](command_lower)
        else:
            return self._default_mapping(command_lower)

    def _determine_context_type(self, command):
        """Determine the appropriate context for the command"""
        navigation_keywords = ["move", "turn", "go", "navigate", "forward", "backward", "left", "right"]
        manipulation_keywords = ["pick", "place", "grab", "take", "drop", "lift", "hold"]

        if any(keyword in command for keyword in navigation_keywords):
            return "navigation"
        elif any(keyword in command for keyword in manipulation_keywords):
            return "manipulation"
        else:
            return "system"

    def _process_navigation_command(self, command):
        """Process navigation-specific commands"""
        # Navigation-specific logic
        if "forward" in command or "ahead" in command:
            distance = self._extract_distance(command) or 1.0
            return {
                "action": "navigation/move_forward",
                "parameters": {"distance": distance}
            }
        elif "backward" in command or "back" in command:
            distance = self._extract_distance(command) or 1.0
            return {
                "action": "navigation/move_backward",
                "parameters": {"distance": distance}
            }
        elif "turn left" in command:
            angle = self._extract_angle(command) or 90
            return {
                "action": "navigation/turn",
                "parameters": {"angle": -angle}
            }
        elif "turn right" in command:
            angle = self._extract_angle(command) or 90
            return {
                "action": "navigation/turn",
                "parameters": {"angle": angle}
            }
        else:
            return {"action": "navigation/unknown", "parameters": {"text": command}}

    def _process_manipulation_command(self, command):
        """Process manipulation-specific commands"""
        # Manipulation-specific logic
        if "pick" in command or "take" in command:
            object_type = self._extract_object_type(command) or "unknown"
            return {
                "action": "manipulation/pick_object",
                "parameters": {"object_type": object_type}
            }
        elif "place" in command or "drop" in command or "put" in command:
            return {
                "action": "manipulation/place_object",
                "parameters": {}
            }
        else:
            return {"action": "manipulation/unknown", "parameters": {"text": command}}

    def _process_system_command(self, command):
        """Process system-level commands"""
        return {"action": "system/unknown", "parameters": {"text": command}}

    def _extract_distance(self, command):
        """Extract distance from command"""
        match = re.search(r"(\d+(?:\.\d+)?)\s*(?:meters?|m)", command)
        if match:
            return float(match.group(1))
        return None

    def _extract_angle(self, command):
        """Extract angle from command"""
        match = re.search(r"(\d+(?:\.\d+)?)\s*(?:degrees?|deg)", command)
        if match:
            return float(match.group(1))
        return None

    def _extract_object_type(self, command):
        """Extract object type from command"""
        # Simple extraction - in practice, this would be more sophisticated
        words = command.split()
        for i, word in enumerate(words):
            if word in ["the", "a", "an"]:
                if i + 1 < len(words):
                    return words[i + 1]
        return "object"

    def _default_mapping(self, command):
        """Default command mapping"""
        return {"action": "unknown/command", "parameters": {"text": command}}
```

## Integration with LLMs for Complex Mapping

For more sophisticated command understanding, integrate with Large Language Models:

```python
import openai

class LLMCommandMapper:
    def __init__(self, api_key):
        openai.api_key = api_key
        self.system_prompt = """
        You are a robot command interpreter. Your job is to convert natural language commands into structured robot actions.

        Available action types:
        - navigation/move_forward: Move robot forward
          Parameters: distance (float in meters)
        - navigation/move_backward: Move robot backward
          Parameters: distance (float in meters)
        - navigation/turn: Turn robot
          Parameters: angle (float in degrees)
        - manipulation/pick_object: Pick up an object
          Parameters: object_type (string)
        - manipulation/place_object: Place an object
          Parameters: location (string)
        - control/stop: Stop current action
          Parameters: none
        - control/pause: Pause robot
          Parameters: none

        Respond with a JSON object containing 'action' and 'parameters' keys.
        If the command is unclear or not a robot command, return action 'unknown/command'.
        """

    def map_command(self, command_text):
        """Map command using LLM for complex understanding"""
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": f"Command: {command_text}"}
                ],
                temperature=0.1,
                max_tokens=200
            )

            # Parse the response (in a real implementation, you'd want more robust JSON parsing)
            import json
            response_text = response.choices[0].message.content.strip()

            # Extract JSON from response if wrapped in code blocks
            if "```json" in response_text:
                json_start = response_text.find("```json") + 7
                json_end = response_text.find("```", json_start)
                json_str = response_text[json_start:json_end].strip()
            elif "```" in response_text:
                code_start = response_text.find("```") + 3
                code_end = response_text.find("```", code_start)
                json_str = response_text[code_start:code_end].strip()
            else:
                json_str = response_text

            return json.loads(json_str)

        except Exception as e:
            print(f"LLM command mapping failed: {e}")
            # Fallback to simpler mapping
            return self._fallback_mapping(command_text)

    def _fallback_mapping(self, command_text):
        """Simple fallback mapping"""
        simple_mapper = StaticCommandMapper()
        return simple_mapper.map_command(command_text)
```

## Performance Considerations

### 1. Caching Mapped Commands

```python
from functools import lru_cache
import hashlib

class CachedCommandMapper:
    def __init__(self, base_mapper, cache_size=1000):
        self.base_mapper = base_mapper
        self.cache_size = cache_size

    @lru_cache(maxsize=1000)
    def _cached_map(self, command_hash, command_text):
        """Cached command mapping"""
        return self.base_mapper.map_command(command_text)

    def map_command(self, command_text):
        """Map command with caching"""
        command_hash = hashlib.md5(command_text.encode()).hexdigest()
        return self._cached_map(command_hash, command_text)
```

### 2. Confidence Scoring

```python
class ConfidentCommandMapper:
    def __init__(self):
        self.fuzzy_mapper = FuzzyCommandMapper()

    def map_command_with_confidence(self, command_text):
        """Map command with confidence scoring"""
        # Use fuzzy matching to get confidence
        command_lower = command_text.lower().strip()

        best_matches = difflib.get_close_matches(
            command_lower,
            [cmd for variations in self.fuzzy_mapper.command_variations.values() for cmd in variations],
            n=1,
            cutoff=0.3  # Lower cutoff to capture more possibilities
        )

        if best_matches:
            confidence = difflib.SequenceMatcher(None, command_lower, best_matches[0]).ratio()

            # Map to canonical command
            mapped_result = self.fuzzy_mapper.map_command(command_text)
            mapped_result["confidence"] = confidence

            # Only execute if confidence is above threshold
            if confidence < 0.7:
                mapped_result["action"] = "uncertain/command"
                mapped_result["parameters"]["confidence"] = confidence

            return mapped_result
        else:
            return {
                "action": "unknown/command",
                "parameters": {"text": command_text, "confidence": 0.0}
            }
```

## Testing Command Mapping

### 1. Unit Tests

```python
import unittest

class TestCommandMapping(unittest.TestCase):
    def setUp(self):
        self.mapper = StaticCommandMapper()

    def test_basic_command_mapping(self):
        """Test basic command mapping"""
        result = self.mapper.map_command("move forward")
        self.assertEqual(result["action"], "navigation/move_forward")
        self.assertEqual(result["parameters"]["distance"], 1.0)

    def test_partial_command_matching(self):
        """Test partial command matching"""
        result = self.mapper.map_command("please move forward")
        self.assertEqual(result["action"], "navigation/move_forward")

    def test_unknown_command(self):
        """Test unknown command handling"""
        result = self.mapper.map_command("unknown command")
        self.assertEqual(result["action"], "unknown/command")
```

### 2. Integration Tests

```python
def test_complete_mapping_pipeline():
    """Test the complete voice-to-action mapping pipeline"""
    # Simulate voice command through the entire pipeline
    voice_input = "move forward 2 meters"

    # Step 1: Transcription (simulated)
    transcribed_text = voice_input  # In real implementation, this comes from Whisper

    # Step 2: Command mapping
    mapper = ParameterizedCommandMapper()
    mapped_action = mapper.map_command(transcribed_text)

    # Step 3: Verify correct mapping
    assert mapped_action["action"] == "navigation/move_forward"
    assert mapped_action["parameters"]["distance"] == 2.0

    print("Pipeline test passed!")
```

## Best Practices

1. **Start Simple**: Begin with static command mapping and add complexity as needed
2. **Handle Ambiguity**: Always have fallback strategies for unclear commands
3. **Context Awareness**: Consider robot state and environment when mapping commands
4. **User Feedback**: Provide clear feedback when commands are recognized or rejected
5. **Continuous Learning**: Consider implementing systems that learn from user interactions
6. **Safety First**: Ensure mapped commands are safe for robot execution

## Troubleshooting Common Issues

### 1. Command Not Recognized
- Check for exact text matches in your command map
- Implement fuzzy matching for variations
- Verify transcription quality from Whisper

### 2. Incorrect Parameter Extraction
- Test regex patterns thoroughly
- Add validation for extracted parameters
- Implement parameter range checking

### 3. High False Positives
- Increase confidence thresholds
- Add more specific command patterns
- Implement context validation

## Next Steps

After implementing basic command mapping, consider:

1. Adding support for multi-step commands
2. Implementing command confirmation for critical actions
3. Adding undo/cancel functionality
4. Integrating with robot state monitoring
5. Adding support for custom user-defined commands