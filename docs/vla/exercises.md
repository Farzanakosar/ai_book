# Voice-to-Action Exercises

## Exercise 1: Basic Voice Command Recognition

### Objective
Implement a basic voice command system that can recognize simple commands and map them to actions.

### Prerequisites
- OpenAI API key configured
- Python environment with required packages installed
- Working microphone

### Instructions

1. **Set up the basic voice processing system:**
   ```python
   import openai
   import speech_recognition as sr
   import os
   from dotenv import load_dotenv

   # Load environment variables
   load_dotenv()
   openai.api_key = os.getenv("OPENAI_API_KEY")

   # Initialize speech recognizer
   recognizer = sr.Recognizer()
   microphone = sr.Microphone()

   # Adjust for ambient noise
   with microphone as source:
       recognizer.adjust_for_ambient_noise(source)
   ```

2. **Create a simple voice command function:**
   ```python
   def listen_for_command():
       """Listen for a voice command and return the transcribed text"""
       print("Listening for command...")

       with microphone as source:
           audio = recognizer.listen(source, timeout=5)

       try:
           # Use Whisper API for transcription
           import io
           import wave

           # Convert audio to appropriate format for Whisper
           wav_data = io.BytesIO()
           with wave.open(wav_data, 'wb') as wav_file:
               wav_file.setnchannels(1)  # Mono
               wav_file.setsampwidth(2)  # 16-bit
               wav_file.setframerate(16000)  # 16kHz

               # Get raw audio data and write to buffer
               raw_data = audio.get_raw_data()
               wav_file.writeframes(raw_data)

           wav_data.seek(0)

           transcription = openai.Audio.transcribe("whisper-1", wav_data)
           return transcription.text

       except sr.WaitTimeoutError:
           print("No speech detected within timeout period")
           return None
       except Exception as e:
           print(f"Error during transcription: {e}")
           return None
   ```

3. **Create a simple command mapping function:**
   ```python
   def map_command_to_action(command_text):
       """Map recognized command to robot action"""
       if command_text is None:
           return "unknown"

       command_lower = command_text.lower()

       if "move forward" in command_lower or "go forward" in command_lower:
           return "navigation/move_forward"
       elif "move backward" in command_lower or "go backward" in command_lower:
           return "navigation/move_backward"
       elif "turn left" in command_lower:
           return "navigation/turn_left"
       elif "turn right" in command_lower:
           return "navigation/turn_right"
       elif "stop" in command_lower:
           return "control/stop"
       else:
           return "unknown"
   ```

4. **Create the main loop to test the system:**
   ```python
   def main():
       print("Voice Command System - Ready")
       print("Say commands like 'move forward', 'turn left', 'stop'")
       print("Press Ctrl+C to exit")

       try:
           while True:
               command_text = listen_for_command()

               if command_text:
                   print(f"Heard: {command_text}")
                   action = map_command_to_action(command_text)
                   print(f"Mapped to action: {action}")

                   # In a real robot system, you would execute the action here
                   # For this exercise, we just print it

                   print("-" * 40)
               else:
                   print("No command recognized, listening again...")

       except KeyboardInterrupt:
           print("\nExiting voice command system...")

   if __name__ == "__main__":
       main()
   ```

### Exercise Tasks

1. **Implementation:**
   - Create a new Python file called `voice_exercise.py`
   - Implement the code above to create a basic voice command system
   - Test that the system can capture your voice and send it to Whisper

2. **Extension:**
   - Add at least 3 more commands to the mapping function (e.g., "pick up object", "place object", "turn around")
   - Add error handling for API rate limits
   - Implement a simple confidence score display

3. **Testing:**
   - Test your system with different commands
   - Try the same command multiple times to see consistency
   - Test in different noise environments

### Expected Output
```
Voice Command System - Ready
Say commands like 'move forward', 'turn left', 'stop'
Press Ctrl+C to exit
Listening for command...
Heard: move forward
Mapped to action: navigation/move_forward
----------------------------------------
Listening for command...
Heard: turn left
Mapped to action: navigation/turn_left
----------------------------------------
```

### Solution Hints
- Make sure your microphone is working and permissions are granted
- Check that your OpenAI API key is valid and has sufficient credits
- If getting rate limit errors, add delays between requests or implement retry logic

## Exercise 2: Whisper Integration Exercise

### Objective
Integrate OpenAI Whisper API for speech-to-text conversion and optimize the integration for robotics applications.

### Prerequisites
- Basic voice command system from Exercise 1
- OpenAI API key with Whisper access
- Understanding of audio formats and processing

### Instructions

1. **Enhanced Whisper Integration:**
   ```python
   import openai
   import io
   import wave
   import time
   from typing import Optional, Dict, Any

   class WhisperIntegrator:
       def __init__(self, api_key: str, model: str = "whisper-1"):
           openai.api_key = api_key
           self.model = model
           self.transcription_history = []

       def transcribe_audio_buffer(self, audio_data: bytes,
                                 sample_rate: int = 16000,
                                 channels: int = 1) -> Optional[str]:
           """Transcribe raw audio bytes using Whisper API"""
           try:
               # Create WAV buffer from raw audio data
               wav_buffer = io.BytesIO()

               with wave.open(wav_buffer, 'wb') as wav_file:
                   wav_file.setnchannels(channels)
                   wav_file.setsampwidth(2)  # 16-bit
                   wav_file.setframerate(sample_rate)
                   wav_file.writeframes(audio_data)

               # Reset buffer position
               wav_buffer.seek(0)

               # Transcribe using Whisper
               start_time = time.time()
               result = openai.Audio.transcribe(
                   model=self.model,
                   file=wav_buffer,
                   response_format="text"
               )

               # Record performance metrics
               transcription_time = time.time() - start_time
               self.transcription_history.append({
                   "text": result,
                   "processing_time": transcription_time,
                   "timestamp": time.time()
               })

               return result

           except Exception as e:
               print(f"Whisper transcription error: {e}")
               return None

       def transcribe_with_timing_options(self, audio_data: bytes) -> Dict[str, Any]:
           """Transcribe with additional options for timing control"""
           try:
               wav_buffer = io.BytesIO()

               with wave.open(wav_buffer, 'wb') as wav_file:
                   wav_file.setnchannels(1)
                   wav_file.setsampwidth(2)
                   wav_file.setframerate(16000)
                   wav_file.writeframes(audio_data)

               wav_buffer.seek(0)

               # Get detailed response with timing info
               result = openai.Audio.transcribe(
                   model=self.model,
                   file=wav_buffer,
                   response_format="verbose_json",  # More detailed response
                   timestamp_granularities=["segment"]
               )

               return {
                   "text": result.text,
                   "segments": result.segments,
                   "processing_time": result.duration
               }

           except Exception as e:
               print(f"Detailed transcription error: {e}")
               return {"text": None, "segments": [], "processing_time": 0}

       def get_performance_metrics(self) -> Dict[str, float]:
           """Get performance metrics for Whisper integration"""
           if not self.transcription_history:
               return {"avg_time": 0, "count": 0}

           times = [item["processing_time"] for item in self.transcription_history]
           avg_time = sum(times) / len(times)

           return {
               "avg_time": avg_time,
               "count": len(times),
               "min_time": min(times),
               "max_time": max(times)
           }
   ```

2. **Audio Preprocessing for Better Whisper Results:**
   ```python
   import numpy as np
   from scipy import signal

   class AudioPreprocessor:
       def __init__(self):
           self.sample_rate = 16000

       def preprocess_audio(self, raw_audio: bytes) -> bytes:
           """Preprocess audio to optimize for Whisper"""
           # Convert bytes to numpy array
           audio_array = np.frombuffer(raw_audio, dtype=np.int16)

           # Apply noise reduction (simple spectral gating)
           audio_filtered = self._apply_noise_reduction(audio_array)

           # Normalize audio levels
           audio_normalized = self._normalize_audio(audio_filtered)

           # Convert back to bytes
           return audio_normalized.tobytes()

       def _apply_noise_reduction(self, audio_array: np.ndarray) -> np.ndarray:
           """Apply basic noise reduction"""
           # Simple noise reduction by spectral gating
           # This is a simplified version - real implementation would be more sophisticated
           return audio_array  # Placeholder for actual noise reduction

       def _normalize_audio(self, audio_array: np.ndarray) -> np.ndarray:
           """Normalize audio to optimal levels for Whisper"""
           # Find max amplitude
           max_amplitude = np.max(np.abs(audio_array))

           # Normalize to 70% of maximum to prevent clipping
           if max_amplitude > 0:
               normalization_factor = 0.7 * (2**15) / max_amplitude
               audio_array = audio_array * normalization_factor

           return audio_array.astype(np.int16)
   ```

3. **Integration and Testing:**
   ```python
   import speech_recognition as sr
   import time

   def test_whisper_integration():
       """Test the complete Whisper integration"""
       # Initialize components
       whisper_integrator = WhisperIntegrator(api_key=os.getenv("OPENAI_API_KEY"))
       audio_preprocessor = AudioPreprocessor()
       recognizer = sr.Recognizer()
       microphone = sr.Microphone()

       with microphone as source:
           recognizer.adjust_for_ambient_noise(source)

       print("Testing Whisper integration...")
       print("Say 'Move forward' to test the system")

       try:
           with microphone as source:
               audio = recognizer.listen(source, timeout=5)

           # Get raw audio data
           raw_audio = audio.get_raw_data()

           # Preprocess audio
           processed_audio = audio_preprocessor.preprocess_audio(raw_audio)

           # Transcribe with Whisper
           start_time = time.time()
           result = whisper_integrator.transcribe_audio_buffer(processed_audio)
           end_time = time.time()

           print(f"Transcription: {result}")
           print(f"Processing time: {end_time - start_time:.2f} seconds")

           # Check performance metrics
           metrics = whisper_integrator.get_performance_metrics()
           print(f"Average processing time: {metrics['avg_time']:.2f} seconds")

       except sr.WaitTimeoutError:
           print("No speech detected")
       except Exception as e:
           print(f"Error during testing: {e}")

   # Run the test
   test_whisper_integration()
   ```

### Exercise Tasks

1. **Implementation:**
   - Create the `WhisperIntegrator` class with proper error handling
   - Implement the `AudioPreprocessor` with noise reduction and normalization
   - Test the integration with your microphone input

2. **Optimization:**
   - Add caching for repeated transcriptions
   - Implement rate limiting to handle API quotas
   - Add retry logic for failed requests

3. **Performance Testing:**
   - Measure transcription times for different audio lengths
   - Test with different audio quality inputs
   - Compare results with and without preprocessing

### Expected Output
```
Testing Whisper integration...
Say 'Move forward' to test the system
Transcription: Move forward
Processing time: 1.25 seconds
Average processing time: 1.25 seconds
```

### Advanced Challenge
- Implement a streaming audio processor that can send partial audio to Whisper for faster response
- Add support for multiple Whisper models and compare quality vs. speed
- Create a fallback system that uses a local ASR model when Whisper is unavailable

## Exercise 3: Command Mapping Exercise

### Objective
Implement sophisticated command mapping that converts natural language to robot actions with context awareness and parameter extraction.

### Prerequisites
- Completed Whisper integration from Exercise 2
- Understanding of natural language processing concepts
- Basic knowledge of regular expressions

### Instructions

1. **Advanced Command Mapper with Context Awareness:**
   ```python
   import re
   from typing import Dict, List, Optional, Any
   from dataclasses import dataclass

   @dataclass
   class RobotAction:
       action_type: str
       parameters: Dict[str, Any]
       confidence: float = 1.0
       context: Dict[str, Any] = None

   class AdvancedCommandMapper:
       def __init__(self):
           self.context = {}
           self.command_patterns = [
               # Navigation patterns
               {
                   "pattern": r"move\s+(forward|backward|ahead|back)\s+(\d+(?:\.\d+)?)\s*(?:meters?|m|units?)",
                   "action_type": "navigation/move_distance",
                   "extractors": [
                       lambda m: ("direction", "forward" if m.group(1) in ["forward", "ahead"] else "backward"),
                       lambda m: ("distance", float(m.group(2)))
                   ]
               },
               {
                   "pattern": r"turn\s+(left|right)\s+(\d+(?:\.\d+)?)\s*(?:degrees?|deg)",
                   "action_type": "navigation/turn_angle",
                   "extractors": [
                       lambda m: ("angle", float(m.group(2)) if m.group(1) == "right" else -float(m.group(2)))
                   ]
               },
               {
                   "pattern": r"go\s+to\s+(?:the\s+)?(\w+)",
                   "action_type": "navigation/go_to_location",
                   "extractors": [
                       lambda m: ("location", m.group(1))
                   ]
               },
               # Manipulation patterns
               {
                   "pattern": r"(?:pick\s+up|grasp|take)\s+(?:the\s+)?(\w+)",
                   "action_type": "manipulation/pick_object",
                   "extractors": [
                       lambda m: ("object_type", m.group(1))
                   ]
               },
               {
                   "pattern": r"place\s+(?:the\s+)?(\w+)\s+(?:on|at|in)\s+(?:the\s+)?(\w+)",
                   "action_type": "manipulation/place_object",
                   "extractors": [
                       lambda m: ("object_type", m.group(1)),
                       lambda m: ("destination", m.group(2))
                   ]
               },
               # Simple commands
               {
                   "pattern": r"stop",
                   "action_type": "control/stop",
                   "extractors": []
               },
               {
                   "pattern": r"pause",
                   "action_type": "control/pause",
                   "extractors": []
               },
               {
                   "pattern": r"continue|resume",
                   "action_type": "control/resume",
                   "extractors": []
               }
           ]

       def map_command(self, command_text: str) -> Optional[RobotAction]:
           """Map command text to robot action with parameter extraction"""
           command_lower = command_text.lower().strip()

           for pattern_config in self.command_patterns:
               match = re.search(pattern_config["pattern"], command_lower)
               if match:
                   parameters = {}
                   for extractor in pattern_config["extractors"]:
                       key, value = extractor(match)
                       parameters[key] = value

                   return RobotAction(
                       action_type=pattern_config["action_type"],
                       parameters=parameters,
                       confidence=0.9  # High confidence for pattern matches
                   )

           # If no pattern matches, try fuzzy matching
           return self._fuzzy_match_command(command_lower)

       def _fuzzy_match_command(self, command_text: str) -> Optional[RobotAction]:
           """Use fuzzy matching for commands that don't match patterns exactly"""
           # Simple fuzzy matching - in practice, use more sophisticated NLP
           known_commands = {
               "move forward": ("navigation/move_forward", {"distance": 1.0}),
               "move backward": ("navigation/move_backward", {"distance": 1.0}),
               "turn left": ("navigation/turn", {"angle": -90}),
               "turn right": ("navigation/turn", {"angle": 90}),
               "stop": ("control/stop", {}),
               "pick object": ("manipulation/pick_object", {}),
               "place object": ("manipulation/place_object", {})
           }

           best_match = None
           best_score = 0

           for known_cmd, (action_type, params) in known_commands.items():
               score = self._calculate_similarity(command_text, known_cmd)
               if score > best_score and score > 0.7:  # 70% similarity threshold
                   best_match = (action_type, params)
                   best_score = score

           if best_match:
               return RobotAction(
                   action_type=best_match[0],
                   parameters=best_match[1],
                   confidence=best_score
               )

           return RobotAction(
               action_type="unknown/command",
               parameters={"text": command_text},
               confidence=0.0
           )

       def _calculate_similarity(self, str1: str, str2: str) -> float:
           """Calculate similarity between two strings"""
           from difflib import SequenceMatcher
           return SequenceMatcher(None, str1, str2).ratio()

       def set_context(self, context: Dict[str, Any]):
           """Set current robot context for contextual command mapping"""
           self.context = context

       def get_context_aware_mapping(self, command_text: str) -> Optional[RobotAction]:
           """Map command considering current context"""
           # Apply context-specific rules
           action = self.map_command(command_text)

           # Modify action based on context if needed
           if self.context.get("location") == "kitchen" and "go to" in command_text.lower():
               # In kitchen context, "go to" might mean different things
               if "fridge" in command_text.lower():
                   action.parameters["location"] = "kitchen_fridge"
               elif "counter" in command_text.lower():
                   action.parameters["location"] = "kitchen_counter"

           return action
   ```

2. **Confidence-Based Command Processing:**
   ```python
   class ConfidenceBasedProcessor:
       def __init__(self, threshold: float = 0.7):
           self.threshold = threshold
           self.command_mapper = AdvancedCommandMapper()

       def process_command_with_confidence(self, command_text: str) -> Dict[str, Any]:
           """Process command with confidence scoring and validation"""
           action = self.command_mapper.map_command(command_text)

           if action.confidence < self.threshold:
               return {
                   "status": "uncertain",
                   "action": action,
                   "suggestions": self._generate_suggestions(command_text),
                   "confidence": action.confidence
               }

           # Validate parameters
           validation_result = self._validate_action_parameters(action)
           if not validation_result["valid"]:
               return {
                   "status": "invalid_parameters",
                   "action": action,
                   "errors": validation_result["errors"],
                   "confidence": action.confidence
               }

           return {
               "status": "confirmed",
               "action": action,
               "confidence": action.confidence
           }

       def _validate_action_parameters(self, action: RobotAction) -> Dict[str, Any]:
           """Validate action parameters based on action type"""
           errors = []

           if action.action_type == "navigation/move_distance":
               if "distance" not in action.parameters:
                   errors.append("Distance parameter is required for move commands")
               elif not isinstance(action.parameters["distance"], (int, float)):
                   errors.append("Distance must be a number")
               elif action.parameters["distance"] <= 0:
                   errors.append("Distance must be positive")

           elif action.action_type == "navigation/turn_angle":
               if "angle" not in action.parameters:
                   errors.append("Angle parameter is required for turn commands")
               elif not isinstance(action.parameters["angle"], (int, float)):
                   errors.append("Angle must be a number")

           elif action.action_type == "navigation/go_to_location":
               if "location" not in action.parameters:
                   errors.append("Location parameter is required for navigation commands")

           return {
               "valid": len(errors) == 0,
               "errors": errors
           }

       def _generate_suggestions(self, command_text: str) -> List[str]:
           """Generate suggestions for uncertain commands"""
           suggestions = []

           # Common command patterns that might be similar
           common_commands = [
               "move forward",
               "move backward",
               "turn left",
               "turn right",
               "go to kitchen",
               "pick up object",
               "place object",
               "stop"
           ]

           for cmd in common_commands:
               similarity = self.command_mapper._calculate_similarity(command_text.lower(), cmd)
               if similarity > 0.5:  # 50% similarity
                   suggestions.append(cmd)

           return suggestions
   ```

3. **Integration and Testing:**
   ```python
   def test_command_mapping():
       """Test the command mapping system"""
       mapper = AdvancedCommandMapper()
       processor = ConfidenceBasedProcessor(threshold=0.6)

       test_commands = [
           "move forward 2 meters",
           "turn left 90 degrees",
           "pick up the red ball",
           "place the cup on the table",
           "go to the kitchen",
           "stop immediately",
           "move backwards 1.5 meters"
       ]

       print("Testing Command Mapping System")
       print("=" * 40)

       for command in test_commands:
           print(f"\nCommand: '{command}'")
           result = processor.process_command_with_confidence(command)

           if result["status"] == "confirmed":
               print(f"  Action: {result['action'].action_type}")
               print(f"  Parameters: {result['action'].parameters}")
               print(f"  Confidence: {result['action'].confidence:.2f}")
           elif result["status"] == "uncertain":
               print(f"  Status: Uncertain (confidence: {result['action'].confidence:.2f})")
               if result["suggestions"]:
                   print(f"  Suggestions: {', '.join(result['suggestions'])}")
           elif result["status"] == "invalid_parameters":
               print(f"  Status: Invalid parameters")
               print(f"  Errors: {result['errors']}")

   # Run the test
   test_command_mapping()
   ```

4. **Context-Aware Command Processing:**
   ```python
   def test_context_aware_mapping():
       """Test command mapping with context awareness"""
       mapper = AdvancedCommandMapper()

       # Set initial context
       context = {
           "location": "kitchen",
           "objects_nearby": ["cup", "plate", "apple"],
           "robot_state": "idle"
       }
       mapper.set_context(context)

       print("\nTesting Context-Aware Command Mapping")
       print("=" * 40)

       contextual_commands = [
           "pick up the apple",  # Should recognize apple is nearby
           "go to the counter",  # Context-aware navigation
           "place the apple on the plate"  # Multi-object interaction
       ]

       for command in contextual_commands:
           print(f"\nCommand: '{command}'")
           print(f"Context: {mapper.context}")
           action = mapper.get_context_aware_mapping(command)
           print(f"  Action: {action.action_type}")
           print(f"  Parameters: {action.parameters}")
           print(f"  Confidence: {action.confidence:.2f}")
   ```

### Exercise Tasks

1. **Implementation:**
   - Create the `AdvancedCommandMapper` class with pattern matching
   - Implement the `ConfidenceBasedProcessor` with validation
   - Test with various command formats and edge cases

2. **Extension:**
   - Add support for temporal commands (e.g., "wait for 5 seconds")
   - Implement command chaining (e.g., "move forward then turn left")
   - Add natural language quantifiers ("a little", "a lot", "very")

3. **Validation:**
   - Create a test suite with various command formats
   - Test edge cases and error conditions
   - Validate parameter ranges and types

### Expected Output
```
Testing Command Mapping System
========================================

Command: 'move forward 2 meters'
  Action: navigation/move_distance
  Parameters: {'direction': 'forward', 'distance': 2.0}
  Confidence: 0.90

Command: 'turn left 90 degrees'
  Action: navigation/turn_angle
  Parameters: {'angle': -90.0}
  Confidence: 0.90

Command: 'pick up the red ball'
  Action: manipulation/pick_object
  Parameters: {'object_type': 'red ball'}
  Confidence: 0.90
```

### Advanced Challenge
- Implement a learning system that improves command mapping based on user corrections
- Add support for multi-language commands
- Create a command history system that can learn user preferences
- Implement safety validation for dangerous commands

## Exercise 4: Performance Optimization and Troubleshooting

### Objective
Optimize the voice-to-action pipeline for performance and implement comprehensive troubleshooting strategies.

### Instructions
1. Implement performance monitoring and optimization techniques
2. Create comprehensive error handling and troubleshooting guides
3. Develop system health checks and diagnostic tools

This exercise focuses on making the system robust and performant in real-world conditions.