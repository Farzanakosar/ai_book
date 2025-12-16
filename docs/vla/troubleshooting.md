# Voice-to-Action Troubleshooting Guide

## Overview

This guide provides solutions to common issues encountered when implementing and using voice-to-action pipelines with OpenAI Whisper in robotics applications. It covers problems related to audio capture, transcription, command mapping, and robot execution.

## Common Issues and Solutions

### 1. Audio Capture Issues

#### Problem: "No audio input detected"
**Symptoms:**
- System doesn't respond to voice commands
- Microphone indicator shows no activity
- Error messages about audio device access

**Solutions:**
1. **Check hardware connections:**
   ```bash
   # On Linux, check available audio devices
   arecord -l

   # On Windows, check Device Manager for audio devices
   # On macOS, check System Preferences > Sound
   ```

2. **Verify microphone permissions:**
   - Windows: Settings > Privacy > Microphone
   - macOS: System Preferences > Security & Privacy > Privacy > Microphone
   - Linux: Check if running in a container that has audio access

3. **Test audio input directly:**
   ```python
   import pyaudio
   import wave

   def test_microphone():
       CHUNK = 1024
       FORMAT = pyaudio.paInt16
       CHANNELS = 1
       RATE = 44100
       RECORD_SECONDS = 5
       WAVE_OUTPUT_FILENAME = "test.wav"

       p = pyaudio.PyAudio()

       stream = p.open(format=FORMAT,
                       channels=CHANNELS,
                       rate=RATE,
                       input=True,
                       frames_per_buffer=CHUNK)

       print("Recording...")
       frames = []

       for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
           data = stream.read(CHUNK)
           frames.append(data)

       print("Done recording.")

       stream.stop_stream()
       stream.close()
       p.terminate()

       wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
       wf.setnchannels(CHANNELS)
       wf.setsampwidth(p.get_sample_size(FORMAT))
       wf.setframerate(RATE)
       wf.writeframes(b''.join(frames))
       wf.close()

   test_microphone()
   ```

#### Problem: "Poor audio quality or excessive noise"
**Symptoms:**
- High Word Error Rate (WER) in transcriptions
- Commands frequently misunderstood
- Background noise interference

**Solutions:**
1. **Optimize audio settings:**
   ```python
   import speech_recognition as sr

   def optimize_audio_settings():
       r = sr.Recognizer()
       m = sr.Microphone()

       # Adjust for ambient noise
       with m as source:
           r.adjust_for_ambient_noise(source, duration=1.0)

       # Set energy threshold (default is 300)
       r.energy_threshold = 4000  # Increase for noisy environments

       # Set dynamic energy threshold
       r.dynamic_energy_threshold = True

       return r, m
   ```

2. **Use noise reduction techniques:**
   ```python
   import numpy as np
   from scipy import signal

   def reduce_noise(audio_data, sample_rate=16000):
       """Apply basic noise reduction to audio data"""
       # Convert bytes to numpy array
       audio_array = np.frombuffer(audio_data, dtype=np.int16)

       # Apply a simple low-pass filter to remove high-frequency noise
       nyquist = sample_rate / 2
       cutoff = 0.8 * nyquist  # 80% of nyquist frequency

       # Design and apply filter
       b, a = signal.butter(4, cutoff / nyquist, btype='low')
       filtered_audio = signal.filtfilt(b, a, audio_array)

       # Convert back to bytes
       return filtered_audio.astype(np.int16).tobytes()
   ```

### 2. Whisper API Issues

#### Problem: "API connection failures or timeouts"
**Symptoms:**
- Timeout errors when calling Whisper API
- "Connection refused" or "Network error" messages
- Slow response times

**Solutions:**
1. **Check network connectivity:**
   ```bash
   # Test connectivity to OpenAI
   curl -I https://api.openai.com/v1/models
   ```

2. **Verify API key:**
   ```python
   import openai

   def test_api_key():
       try:
           # Test with a simple API call
           models = openai.Model.list()
           print("API key is valid")
           return True
       except openai.error.AuthenticationError:
           print("Invalid API key")
           return False
       except Exception as e:
           print(f"API test failed: {e}")
           return False
   ```

3. **Implement retry logic:**
   ```python
   import time
   import random
   from functools import wraps

   def retry_with_backoff(max_retries=3, base_delay=1, max_delay=60):
       def decorator(func):
           @wraps(func)
           def wrapper(*args, **kwargs):
               for attempt in range(max_retries):
                   try:
                       return func(*args, **kwargs)
                   except openai.error.RateLimitError:
                       if attempt < max_retries - 1:
                           delay = min(base_delay * (2 ** attempt) + random.uniform(0, 1), max_delay)
                           time.sleep(delay)
                           continue
                       raise
                   except openai.error.APIConnectionError:
                       if attempt < max_retries - 1:
                           delay = min(base_delay * (2 ** attempt) + random.uniform(0, 1), max_delay)
                           time.sleep(delay)
                           continue
                       raise
                   except Exception:
                       if attempt < max_retries - 1:
                           time.sleep(1)
                           continue
                       raise
               return None
           return wrapper
       return decorator

   @retry_with_backoff(max_retries=3)
   def transcribe_with_retry(audio_file):
       return openai.Audio.transcribe("whisper-1", audio_file)
   ```

#### Problem: "High transcription error rates"
**Symptoms:**
- Commands frequently misrecognized
- Inconsistent transcription results
- Poor understanding of specific terminology

**Solutions:**
1. **Optimize audio format for Whisper:**
   ```python
   import io
   import wave

   def prepare_audio_for_whisper(raw_audio, sample_rate=16000):
       """Prepare audio in optimal format for Whisper API"""
       wav_buffer = io.BytesIO()

       with wave.open(wav_buffer, 'wb') as wav_file:
           wav_file.setnchannels(1)  # Mono
           wav_file.setsampwidth(2)  # 16-bit
           wav_file.setframerate(sample_rate)  # 16kHz
           wav_file.writeframes(raw_audio)

       wav_buffer.seek(0)
       return wav_buffer
   ```

2. **Preprocess audio for better results:**
   ```python
   def enhance_audio_for_transcription(audio_data):
       """Enhance audio quality before sending to Whisper"""
       # Convert to numpy array
       audio_array = np.frombuffer(audio_data, dtype=np.int16)

       # Normalize audio levels (Whisper works best with normalized audio)
       max_amplitude = np.max(np.abs(audio_array))
       if max_amplitude > 0:
           # Normalize to reasonable level
           normalization_factor = 0.8 * (2**15) / max_amplitude
           audio_array = audio_array * normalization_factor

       # Apply gentle compression to improve clarity
       # This is a simple implementation - consider using proper audio processing libraries
       audio_array = np.clip(audio_array, -0.9 * (2**15), 0.9 * (2**15))

       # Convert back to bytes
       return audio_array.astype(np.int16).tobytes()
   ```

### 3. Command Mapping Issues

#### Problem: "Commands not recognized or mapped incorrectly"
**Symptoms:**
- Voice commands result in "unknown" actions
- Similar commands mapped to different actions inconsistently
- Expected actions not being triggered

**Solutions:**
1. **Debug command mapping:**
   ```python
   def debug_command_mapping(command_text, mapper):
       """Debug the command mapping process"""
       print(f"Input command: '{command_text}'")
       print(f"Lowercase: '{command_text.lower()}'")

       # Test each pattern
       for i, pattern_config in enumerate(mapper.command_patterns):
           match = re.search(pattern_config["pattern"], command_text.lower())
           if match:
               print(f"Pattern {i} matched: {pattern_config['pattern']}")
               print(f"Groups: {match.groups()}")
               break
       else:
           print("No patterns matched")

       # Test fuzzy matching
       fuzzy_result = mapper._fuzzy_match_command(command_text.lower())
       print(f"Fuzzy match result: {fuzzy_result.action_type}")
   ```

2. **Improve pattern matching:**
   ```python
   class ImprovedCommandMapper(AdvancedCommandMapper):
       def __init__(self):
           super().__init__()
           # Add more flexible patterns
           self.command_patterns.extend([
               # More flexible navigation patterns
               {
                   "pattern": r"(?:move|go|step)\s+(forward|backward|ahead|back)\s*(?:by\s+)?(\d+(?:\.\d+)?)?",
                   "action_type": "navigation/move_distance",
                   "extractors": [
                       lambda m: ("direction", "forward" if m.group(1) in ["forward", "ahead"] else "backward"),
                       lambda m: ("distance", float(m.group(2)) if m.group(2) else 1.0)
                   ]
               },
               # Flexible manipulation patterns
               {
                   "pattern": r"(?:pick|take|grab)\s+(?:up\s+)?(?:the\s+)?(\w+)\s*(?:from\s+(?:the\s+)?(\w+))?",
                   "action_type": "manipulation/pick_object",
                   "extractors": [
                       lambda m: ("object_type", m.group(1)),
                       lambda m: ("source_location", m.group(2)) if m.group(2) else ("source_location", "unknown")
                   ]
               }
           ])
   ```

#### Problem: "Context-aware commands not working properly"
**Symptoms:**
- Context information not being used in command processing
- Same commands producing different results in different contexts
- Context state not being maintained properly

**Solutions:**
1. **Verify context management:**
   ```python
   class ContextManager:
       def __init__(self):
           self.context = {}
           self.history = []

       def update_context(self, new_context):
           """Update context with new information"""
           self.context.update(new_context)
           self.history.append({
               "timestamp": time.time(),
               "context": new_context.copy()
           })

       def get_relevant_context(self, max_age=300):  # 5 minutes
           """Get context that's not too old"""
           current_time = time.time()
           relevant_context = {}

           # Include recent context
           for item in reversed(self.history):
               if current_time - item["timestamp"] <= max_age:
                   relevant_context.update(item["context"])
               else:
                   break

           return relevant_context

       def clear_old_context(self, max_age=300):
           """Remove old context history"""
           current_time = time.time()
           self.history = [item for item in self.history
                          if current_time - item["timestamp"] <= max_age]
   ```

### 4. Performance Issues

#### Problem: "High latency in voice-to-action pipeline"
**Symptoms:**
- Long delay between speaking command and robot response
- Timeout errors in real-time applications
- Poor user experience due to delays

**Solutions:**
1. **Optimize the pipeline:**
   ```python
   import asyncio
   import concurrent.futures
   from threading import Thread

   class OptimizedVoiceProcessor:
       def __init__(self, api_key):
           openai.api_key = api_key
           self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)
           self.cache = {}
           self.cache_size_limit = 50

       def process_voice_command(self, audio_data):
           """Process voice command with optimization"""
           # Preprocessing in background
           processed_audio = self._preprocess_audio(audio_data)

           # Check cache first
           audio_hash = hash(processed_audio)
           if audio_hash in self.cache:
               return self.cache[audio_hash]

           # Submit to executor for non-blocking processing
           future = self.executor.submit(self._process_audio, processed_audio)
           result = future.result(timeout=10.0)  # 10 second timeout

           # Update cache
           self._update_cache(audio_hash, result)

           return result

       def _update_cache(self, audio_hash, result):
           """Update cache with new result"""
           if len(self.cache) >= self.cache_size_limit:
               # Remove oldest entry
               oldest_key = next(iter(self.cache))
               del self.cache[oldest_key]

           self.cache[audio_hash] = result
   ```

2. **Implement streaming or partial processing:**
   ```python
   class StreamingVoiceProcessor:
       def __init__(self, api_key):
           openai.api_key = api_key
           self.audio_buffer = b""
           self.min_audio_length = 8000  # Minimum bytes before processing

       def add_audio_chunk(self, chunk):
           """Add audio chunk to buffer"""
           self.audio_buffer += chunk

           # Process if we have enough audio
           if len(self.audio_buffer) >= self.min_audio_length:
               # Check if we have a potential end-of-sentence
               if self._likely_end_of_command():
                   self._process_buffered_audio()
                   self.audio_buffer = b""  # Reset buffer

       def _likely_end_of_command(self):
           """Check if audio likely contains complete command"""
           # Simple heuristic - look for periods of silence
           # In practice, use more sophisticated audio analysis
           silence_threshold = 100  # Adjust based on your audio format
           audio_array = np.frombuffer(self.audio_buffer[-1000:], dtype=np.int16)
           return np.mean(np.abs(audio_array)) < silence_threshold

       def _process_buffered_audio(self):
           """Process the buffered audio"""
           if len(self.audio_buffer) < self.min_audio_length:
               return

           # Process the audio
           # This would call Whisper and command mapping
           pass
   ```

### 5. Integration Issues

#### Problem: "Robot doesn't execute mapped actions"
**Symptoms:**
- Commands properly mapped but robot doesn't respond
- Action execution fails silently
- Robot in wrong state for action execution

**Solutions:**
1. **Implement action execution verification:**
   ```python
   class ActionExecutor:
       def __init__(self, robot_interface):
           self.robot = robot_interface

       def execute_action(self, robot_action):
           """Execute robot action with verification"""
           try:
               print(f"Executing action: {robot_action.action_type}")
               print(f"Parameters: {robot_action.parameters}")

               # Verify robot is in correct state
               if not self._verify_robot_state(robot_action):
                   raise ValueError(f"Robot not in appropriate state for {robot_action.action_type}")

               # Execute the action
               result = self._execute_by_type(robot_action)

               # Verify execution was successful
               if not self._verify_execution_success(robot_action):
                   raise RuntimeError("Action execution failed verification")

               print(f"Action completed successfully: {robot_action.action_type}")
               return True

           except Exception as e:
               print(f"Action execution failed: {e}")
               return False

       def _verify_robot_state(self, robot_action):
           """Verify robot is in appropriate state for action"""
           current_state = self.robot.get_state()

           # Define state requirements for different action types
           state_requirements = {
               "navigation/move_forward": ["idle", "ready"],
               "manipulation/pick_object": ["idle", "ready", "navigated"],
               "control/stop": ["*"]  # Any state
           }

           required_states = state_requirements.get(robot_action.action_type, ["idle"])
           return required_states[0] == "*" or current_state in required_states

       def _execute_by_type(self, robot_action):
           """Execute action based on type"""
           action_type = robot_action.action_type

           if action_type.startswith("navigation/"):
               return self._execute_navigation(robot_action)
           elif action_type.startswith("manipulation/"):
               return self._execute_manipulation(robot_action)
           elif action_type.startswith("control/"):
               return self._execute_control(robot_action)
           else:
               raise ValueError(f"Unknown action type: {action_type}")

       def _verify_execution_success(self, robot_action):
           """Verify that action was executed successfully"""
           # Implementation depends on your robot platform
           # This is a placeholder
           time.sleep(0.1)  # Simulate execution time
           return True
   ```

## Diagnostic Tools

### 1. System Health Check
```python
def run_system_health_check():
    """Run comprehensive system health check"""
    results = {
        "audio_system": False,
        "api_connection": False,
        "command_mapping": False,
        "robot_interface": False
    }

    # Check audio system
    try:
        import pyaudio
        p = pyaudio.PyAudio()
        device_count = p.get_device_count()
        p.terminate()
        results["audio_system"] = device_count > 0
        print(f"Audio devices: {device_count}")
    except Exception as e:
        print(f"Audio system check failed: {e}")

    # Check API connection
    try:
        import openai
        models = openai.Model.list()
        results["api_connection"] = True
        print("API connection: OK")
    except Exception as e:
        print(f"API connection failed: {e}")

    # Check command mapping
    try:
        mapper = AdvancedCommandMapper()
        test_action = mapper.map_command("move forward")
        results["command_mapping"] = test_action.action_type != "unknown/command"
        print(f"Command mapping: {'OK' if results['command_mapping'] else 'FAILED'}")
    except Exception as e:
        print(f"Command mapping check failed: {e}")

    # Check robot interface (if available)
    # This would depend on your specific robot platform
    results["robot_interface"] = True  # Placeholder
    print("Robot interface: Assuming OK (implementation specific)")

    return results
```

### 2. Performance Monitoring
```python
import time
import statistics

class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            "transcription_times": [],
            "command_mapping_times": [],
            "total_processing_times": [],
            "success_counts": {"transcription": 0, "mapping": 0, "execution": 0},
            "error_counts": {"transcription": 0, "mapping": 0, "execution": 0}
        }

    def start_timer(self, operation):
        """Start timing for an operation"""
        self.metrics[f"{operation}_start"] = time.time()

    def end_timer(self, operation):
        """End timing for an operation"""
        start_time = self.metrics.get(f"{operation}_start")
        if start_time:
            elapsed = time.time() - start_time
            self.metrics[f"{operation}_times"].append(elapsed)
            del self.metrics[f"{operation}_start"]
            return elapsed
        return None

    def record_success(self, operation):
        """Record successful operation"""
        self.metrics["success_counts"][operation] += 1

    def record_error(self, operation):
        """Record failed operation"""
        self.metrics["error_counts"][operation] += 1

    def get_performance_report(self):
        """Generate performance report"""
        report = {
            "transcription": {},
            "command_mapping": {},
            "total_processing": {}
        }

        for op in ["transcription", "command_mapping", "total_processing"]:
            times = self.metrics[f"{op}_times"]
            if times:
                report[op] = {
                    "avg_time": statistics.mean(times),
                    "p95_time": statistics.quantiles(times, n=20)[-1] if len(times) > 1 else times[0],
                    "min_time": min(times),
                    "max_time": max(times),
                    "count": len(times)
                }
            else:
                report[op] = {"avg_time": 0, "count": 0}

        report["success_rates"] = {}
        for op in ["transcription", "mapping", "execution"]:
            total = self.metrics["success_counts"][op] + self.metrics["error_counts"][op]
            if total > 0:
                report["success_rates"][op] = self.metrics["success_counts"][op] / total
            else:
                report["success_rates"][op] = 0

        return report
```

## Prevention Strategies

### 1. Robust Error Handling
```python
def robust_voice_processing(audio_data, fallback_timeout=5.0):
    """Process voice with multiple fallback strategies"""
    results = []

    # Strategy 1: Standard processing
    try:
        result = standard_voice_processor(audio_data)
        results.append(("standard", result, "success"))
    except Exception as e:
        results.append(("standard", None, str(e)))

    # Strategy 2: Fallback processing (simpler, more robust)
    try:
        result = fallback_voice_processor(audio_data)
        results.append(("fallback", result, "success"))
    except Exception as e:
        results.append(("fallback", None, str(e)))

    # Strategy 3: Local processing if available
    try:
        result = local_voice_processor(audio_data)
        results.append(("local", result, "success"))
    except Exception as e:
        results.append(("local", None, str(e)))

    # Return best available result
    for strategy, result, status in results:
        if result and status == "success":
            return result

    # If all failed, return error information
    return {"error": "All processing strategies failed", "details": results}
```

### 2. Configuration Validation
```python
def validate_configuration(config):
    """Validate system configuration before starting"""
    errors = []

    # Check API key
    if not config.get("openai_api_key"):
        errors.append("OpenAI API key is missing")

    # Check audio device
    try:
        import pyaudio
        p = pyaudio.PyAudio()
        if p.get_device_count() == 0:
            errors.append("No audio devices found")
        p.terminate()
    except Exception as e:
        errors.append(f"Audio system error: {e}")

    # Check required packages
    required_packages = ["openai", "speechrecognition", "pyaudio"]
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            errors.append(f"Missing required package: {package}")

    return {
        "valid": len(errors) == 0,
        "errors": errors
    }
```

## Quick Reference

### Common Error Messages and Solutions

| Error Message | Likely Cause | Solution |
|---------------|--------------|----------|
| "No speech detected" | Microphone not working or environment too quiet | Check microphone connections, increase energy threshold |
| "Invalid API key" | Incorrect or expired API key | Verify API key in environment variables |
| "Rate limit exceeded" | Too many API requests | Implement rate limiting and retry logic |
| "Connection timeout" | Network connectivity issues | Check internet connection, increase timeout values |
| "Unknown command" | Command not in pattern list | Add command pattern or improve fuzzy matching |
| "Action failed" | Robot state or execution error | Check robot state, verify action parameters |

### Performance Optimization Checklist

- [ ] Implement audio preprocessing
- [ ] Add caching for repeated commands
- [ ] Use connection pooling for API calls
- [ ] Implement asynchronous processing
- [ ] Monitor and log performance metrics
- [ ] Set appropriate timeout values
- [ ] Validate inputs before processing

## Next Steps

If you're still experiencing issues after following this guide:

1. Check the [Performance Optimization Guide](./performance.md) for advanced optimization techniques
2. Review your specific robot platform documentation for integration issues
3. Consider implementing more sophisticated error recovery mechanisms
4. Set up monitoring and alerting for production systems