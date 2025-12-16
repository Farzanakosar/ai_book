# Voice Recognition Performance Optimization Guide

## Overview

This guide provides strategies and techniques to optimize the performance of voice recognition systems using OpenAI Whisper in robotics applications. The focus is on reducing latency, improving accuracy, and ensuring reliable operation in real-time environments.

## Performance Metrics

### Key Performance Indicators (KPIs)

1. **Latency Metrics**
   - **Audio Capture Latency**: Time from audio input to processing start
   - **Transcription Latency**: Time from audio to text conversion
   - **Command Mapping Latency**: Time from text to action mapping
   - **End-to-End Latency**: Total time from audio capture to action execution

2. **Accuracy Metrics**
   - **Word Error Rate (WER)**: Percentage of incorrectly transcribed words
   - **Command Recognition Rate**: Percentage of correctly identified commands
   - **Action Success Rate**: Percentage of successfully executed robot actions
   - **System Availability**: Percentage of time system is responsive

3. **Resource Utilization**
   - **CPU Usage**: Processing power required for audio and transcription
   - **Memory Usage**: RAM consumption during operation
   - **Network Usage**: Bandwidth consumed for API calls
   - **API Cost**: Financial cost of Whisper API usage

### Baseline Performance Targets

For real-time robotics applications:
- End-to-End Latency: < 2.0 seconds (ideally < 1.0 second)
- WER: < 15% for clear audio, < 30% for noisy environments
- Command Recognition Rate: > 90%
- System Availability: > 99%

## Audio Capture Optimization

### 1. Audio Quality Enhancement

```python
import pyaudio
import numpy as np
from scipy import signal
import threading
import queue

class OptimizedAudioCapture:
    def __init__(self, sample_rate=16000, chunk_size=1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.audio_queue = queue.Queue()
        self.is_recording = False

        # Audio enhancement parameters
        self.energy_threshold = 4000
        self.dynamic_energy_threshold = True
        self.pause_threshold = 0.8
        self.phrase_threshold = 0.3

    def start_capture(self):
        """Start optimized audio capture"""
        self.is_recording = True

        # Initialize PyAudio
        p = pyaudio.PyAudio()

        # Find best input device
        best_device_index = self._find_best_input_device(p)

        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=best_device_index if best_device_index is not None else 0
        )

        def capture_thread():
            while self.is_recording:
                try:
                    data = stream.read(self.chunk_size, exception_on_overflow=False)
                    self.audio_queue.put(data)
                except Exception as e:
                    print(f"Audio capture error: {e}")
                    break

            stream.stop_stream()
            stream.close()
            p.terminate()

        threading.Thread(target=capture_thread, daemon=True).start()

    def _find_best_input_device(self, p):
        """Find the best audio input device"""
        best_device = None
        best_score = 0

        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:  # Is an input device
                # Score based on device name (prioritize USB, external devices)
                name_score = 0
                name = info['name'].lower()
                if 'usb' in name or 'external' in name or 'headset' in name:
                    name_score = 10
                elif 'internal' in name or 'built-in' in name:
                    name_score = 5

                # Score based on sample rate capability
                rate_score = 0
                if info['defaultSampleRate'] >= 16000:
                    rate_score = 5

                total_score = name_score + rate_score
                if total_score > best_score:
                    best_score = total_score
                    best_device = i

        return best_device

    def get_audio_chunk(self, timeout=1.0):
        """Get audio chunk with timeout"""
        try:
            return self.audio_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def preprocess_audio_chunk(self, audio_chunk):
        """Apply preprocessing to enhance audio quality"""
        # Convert bytes to numpy array
        audio_array = np.frombuffer(audio_chunk, dtype=np.int16)

        # Apply noise reduction (simple spectral gating approach)
        audio_filtered = self._apply_noise_reduction(audio_array)

        # Apply automatic gain control
        audio_gain_controlled = self._apply_agc(audio_filtered)

        # Convert back to bytes
        return audio_gain_controlled.astype(np.int16).tobytes()

    def _apply_noise_reduction(self, audio_array):
        """Apply basic noise reduction"""
        # Calculate noise profile (simple approach)
        noise_threshold = np.std(audio_array) * 0.5

        # Simple noise gate - attenuate low-amplitude signals
        amplitude = np.abs(audio_array)
        mask = amplitude < noise_threshold
        audio_array[mask] = audio_array[mask] * 0.1  # Attenuate noise by 90%

        return audio_array

    def _apply_agc(self, audio_array):
        """Apply automatic gain control"""
        target_amplitude = 0.7 * (2**15)  # 70% of maximum
        current_amplitude = np.max(np.abs(audio_array))

        if current_amplitude > 0:
            gain_factor = target_amplitude / current_amplitude
            # Limit gain to prevent excessive amplification
            gain_factor = min(gain_factor, 2.0)  # Maximum 2x amplification
            audio_array = audio_array * gain_factor

        return audio_array
```

### 2. Adaptive Audio Configuration

```python
class AdaptiveAudioConfig:
    def __init__(self):
        self.energy_threshold = 3000  # Base energy threshold
        self.adjustment_factor = 1.2  # How much to adjust threshold
        self.min_threshold = 500      # Minimum acceptable threshold
        self.max_threshold = 8000     # Maximum acceptable threshold

        # Performance history
        self.transcription_successes = []
        self.current_ambient_noise = 0

    def update_for_environment(self, recent_audio_samples):
        """Adapt audio settings based on current environment"""
        # Calculate current noise level
        avg_amplitude = np.mean([np.std(np.frombuffer(sample, dtype=np.int16))
                                for sample in recent_audio_samples])

        # Adjust energy threshold based on ambient noise
        if avg_amplitude > self.current_ambient_noise * 1.5:
            # Environment has become noisier
            self.energy_threshold = min(
                self.energy_threshold * self.adjustment_factor,
                self.max_threshold
            )
        elif avg_amplitude < self.current_ambient_noise / 1.5:
            # Environment has become quieter
            self.energy_threshold = max(
                self.energy_threshold / self.adjustment_factor,
                self.min_threshold
            )

        self.current_ambient_noise = avg_amplitude

        return {
            "energy_threshold": self.energy_threshold,
            "ambient_noise_level": avg_amplitude
        }

    def optimize_for_speaker(self, speaker_voice_profile):
        """Optimize settings for a specific speaker"""
        # Adjust based on speaker's voice characteristics
        voice_amplitude = speaker_voice_profile.get("avg_amplitude", 1000)
        voice_clarity = speaker_voice_profile.get("clarity_score", 0.8)

        # Lower threshold for quieter speakers
        self.energy_threshold = max(
            self.min_threshold,
            min(self.max_threshold, 3000 * (1000 / voice_amplitude))
        )

        return {
            "energy_threshold": self.energy_threshold,
            "optimized_for_speaker": True
        }
```

## Whisper API Optimization

### 1. Efficient API Usage

```python
import asyncio
import aiohttp
import time
from typing import Optional, Dict, Any
import json

class OptimizedWhisperClient:
    def __init__(self, api_key: str, max_concurrent_requests: int = 3):
        self.api_key = api_key
        self.max_concurrent = max_concurrent_requests
        self.semaphore = asyncio.Semaphore(max_concurrent_requests)
        self.session: Optional[aiohttp.ClientSession] = None
        self.request_cache = {}
        self.cache_size_limit = 100

        # Rate limiting
        self.request_times = []
        self.max_requests_per_minute = 50  # Adjust based on your plan

    async def __aenter__(self):
        self.session = aiohttp.ClientSession(
            headers={"Authorization": f"Bearer {self.api_key}"}
        )
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()

    async def transcribe_audio(self, audio_data: bytes,
                             model: str = "whisper-1",
                             language: Optional[str] = None) -> str:
        """Transcribe audio with optimization and caching"""
        # Create cache key
        import hashlib
        cache_key = hashlib.md5(audio_data).hexdigest()

        # Check cache first
        if cache_key in self.request_cache:
            return self.request_cache[cache_key]

        # Enforce rate limiting
        await self._enforce_rate_limit()

        async with self.semaphore:  # Limit concurrent requests
            url = "https://api.openai.com/v1/audio/transcriptions"

            data = aiohttp.FormData()
            data.add_field('file', audio_data, filename='audio.wav', content_type='audio/wav')
            data.add_field('model', model)
            data.add_field('response_format', 'text')

            if language:
                data.add_field('language', language)

            start_time = time.time()

            try:
                async with self.session.post(url, data=data) as response:
                    if response.status == 200:
                        result = await response.text()
                        transcription_time = time.time() - start_time

                        print(f"Whisper API call took {transcription_time:.2f}s")

                        # Add to cache
                        self._add_to_cache(cache_key, result)

                        return result
                    else:
                        error_text = await response.text()
                        raise Exception(f"API Error {response.status}: {error_text}")

            except Exception as e:
                print(f"Whisper transcription failed: {e}")
                raise

    async def _enforce_rate_limit(self):
        """Enforce rate limiting to avoid API penalties"""
        current_time = time.time()

        # Remove old requests (older than 1 minute)
        self.request_times = [req_time for req_time in self.request_times
                             if current_time - req_time < 60]

        if len(self.request_times) >= self.max_requests_per_minute:
            # Need to wait
            sleep_time = 60 - (current_time - self.request_times[0])
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

        # Record this request
        self.request_times.append(current_time)

    def _add_to_cache(self, cache_key: str, result: str):
        """Add result to cache with size management"""
        if len(self.request_cache) >= self.cache_size_limit:
            # Remove oldest entry (FIFO)
            oldest_key = next(iter(self.request_cache))
            del self.request_cache[oldest_key]

        self.request_cache[cache_key] = result
```

### 2. Audio Format Optimization

```python
import io
import wave
from typing import Tuple

class AudioFormatOptimizer:
    @staticmethod
    def optimize_for_whisper(audio_data: bytes,
                           sample_rate: int = 16000,
                           channels: int = 1,
                           bit_depth: int = 16) -> bytes:
        """
        Optimize audio format specifically for Whisper API
        Whisper performs best with: 16kHz, 16-bit, mono WAV
        """
        # Convert raw audio data to optimal format
        wav_buffer = io.BytesIO()

        with wave.open(wav_buffer, 'wb') as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(bit_depth // 8)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio_data)

        return wav_buffer.getvalue()

    @staticmethod
    def estimate_transcription_cost(file_size_bytes: int,
                                 whisper_model: str = "whisper-1") -> float:
        """Estimate API cost for transcription"""
        # Whisper pricing (as of knowledge cutoff): $0.006 per minute
        # File size to duration estimation
        duration_minutes = file_size_bytes / (16000 * 2 * 1 * 60)  # 16kHz, 16-bit, mono
        cost = duration_minutes * 0.006

        return cost

    @staticmethod
    def chunk_audio_for_optimal_performance(audio_data: bytes,
                                          max_chunk_duration: float = 30.0) -> list:
        """
        Split long audio into chunks for optimal Whisper performance
        Whisper works best with shorter audio segments
        """
        # Calculate chunk size based on desired duration
        bytes_per_second = 16000 * 2  # 16kHz, 16-bit
        chunk_size_bytes = int(max_chunk_duration * bytes_per_second)

        chunks = []
        for i in range(0, len(audio_data), chunk_size_bytes):
            chunk = audio_data[i:i + chunk_size_bytes]
            chunks.append(chunk)

        return chunks
```

## Command Mapping Optimization

### 1. Fast Pattern Matching

```python
import re
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass

@dataclass
class OptimizedCommandPattern:
    pattern: re.Pattern
    action_type: str
    extractors: List[callable]
    priority: int = 0  # Higher priority patterns are checked first

class FastCommandMapper:
    def __init__(self):
        self.patterns: List[OptimizedCommandPattern] = []
        self.compiled_patterns = {}
        self.setup_patterns()

        # Performance cache
        self.cache = {}
        self.cache_size_limit = 1000

    def setup_patterns(self):
        """Set up optimized command patterns"""
        pattern_configs = [
            # High priority: exact commands
            (r"^\s*stop\s*$", "control/stop", [], 10),
            (r"^\s*pause\s*$", "control/pause", [], 10),
            (r"^\s*continue\s*$", "control/resume", [], 10),

            # Navigation commands
            (r"move\s+(forward|backward|ahead|back)\s+(\d+(?:\.\d+)?)\s*(?:meters?|m|units?)",
             "navigation/move_distance", [
                 lambda m: ("direction", "forward" if m.group(1) in ["forward", "ahead"] else "backward"),
                 lambda m: ("distance", float(m.group(2)))
             ], 5),

            (r"turn\s+(left|right)\s+(\d+(?:\.\d+)?)\s*(?:degrees?|deg)",
             "navigation/turn_angle", [
                 lambda m: ("angle", float(m.group(2)) if m.group(1) == "right" else -float(m.group(2)))
             ], 5),

            # Manipulation commands
            (r"(?:pick\s+up|grasp|take)\s+(?:the\s+)?(\w+)",
             "manipulation/pick_object", [
                 lambda m: ("object_type", m.group(1))
             ], 5),
        ]

        for pattern_str, action_type, extractors, priority in pattern_configs:
            pattern = re.compile(pattern_str, re.IGNORECASE)
            self.patterns.append(OptimizedCommandPattern(
                pattern=pattern,
                action_type=action_type,
                extractors=extractors,
                priority=priority
            ))

        # Sort by priority (highest first)
        self.patterns.sort(key=lambda p: p.priority, reverse=True)

    def map_command(self, command_text: str) -> Dict[str, Any]:
        """Fast command mapping with caching"""
        # Check cache first
        cache_key = command_text.lower().strip()
        if cache_key in self.cache:
            return self.cache[cache_key]

        start_time = time.time()

        for cmd_pattern in self.patterns:
            match = cmd_pattern.pattern.search(command_text)
            if match:
                # Extract parameters
                parameters = {}
                for extractor in cmd_pattern.extractors:
                    try:
                        key, value = extractor(match)
                        parameters[key] = value
                    except Exception as e:
                        print(f"Parameter extraction error: {e}")
                        continue

                result = {
                    "action_type": cmd_pattern.action_type,
                    "parameters": parameters,
                    "confidence": 0.9,
                    "processing_time": time.time() - start_time
                }

                # Add to cache
                self._add_to_cache(cache_key, result)
                return result

        # If no pattern matches, return unknown
        result = {
            "action_type": "unknown/command",
            "parameters": {"text": command_text},
            "confidence": 0.0,
            "processing_time": time.time() - start_time
        }

        self._add_to_cache(cache_key, result)
        return result

    def _add_to_cache(self, key: str, result: Dict[str, Any]):
        """Add result to cache with size management"""
        if len(self.cache) >= self.cache_size_limit:
            # Remove oldest entries (in insertion order)
            oldest_key = next(iter(self.cache))
            del self.cache[oldest_key]

        self.cache[key] = result

    def benchmark_performance(self, test_commands: List[str], iterations: int = 100):
        """Benchmark command mapping performance"""
        times = []

        for _ in range(iterations):
            for cmd in test_commands:
                start = time.time()
                self.map_command(cmd)
                times.append(time.time() - start)

        return {
            "avg_time": sum(times) / len(times),
            "min_time": min(times),
            "max_time": max(times),
            "p95_time": sorted(times)[int(0.95 * len(times))] if times else 0
        }
```

### 2. Context-Aware Optimization

```python
class ContextAwareOptimizer:
    def __init__(self):
        self.context_scorer = ContextScorer()
        self.command_predictor = CommandPredictor()

    def optimize_for_context(self, command_text: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """Optimize command mapping based on current context"""
        # Score command relevance based on context
        context_score = self.context_scorer.score_command_context(
            command_text, context
        )

        # Predict likely next commands to pre-cache
        predicted_commands = self.command_predictor.predict_next_commands(
            context
        )

        # If command seems contextually relevant, boost confidence
        base_result = FastCommandMapper().map_command(command_text)

        if context_score > 0.7:
            base_result["confidence"] = min(base_result["confidence"] + 0.1, 1.0)

        return {
            **base_result,
            "context_score": context_score,
            "predicted_next_commands": predicted_commands
        }

class ContextScorer:
    def score_command_context(self, command: str, context: Dict[str, Any]) -> float:
        """Score how well command fits current context"""
        score = 0.5  # Base score

        # Check location relevance
        if "location" in context and context["location"] in command.lower():
            score += 0.2

        # Check available objects
        if "objects_nearby" in context:
            for obj in context["objects_nearby"]:
                if obj.lower() in command.lower():
                    score += 0.15

        # Check robot state
        if "robot_state" in context:
            state = context["robot_state"].lower()
            if state == "navigating" and any(word in command.lower() for word in ["stop", "pause", "wait"]):
                score += 0.2
            elif state == "manipulating" and "place" in command.lower():
                score += 0.15

        return min(score, 1.0)

class CommandPredictor:
    def __init__(self):
        self.command_sequences = {}
        self.max_sequence_length = 5

    def predict_next_commands(self, context: Dict[str, Any]) -> List[str]:
        """Predict likely next commands based on context"""
        # This is a simplified prediction model
        # In practice, you'd use more sophisticated ML models

        predictions = []

        if context.get("location") == "kitchen":
            predictions.extend(["pick up cup", "go to counter", "open fridge"])
        elif context.get("location") == "living_room":
            predictions.extend(["turn on light", "adjust temperature", "play music"])

        # Add predictions based on robot state
        if context.get("robot_state") == "idle":
            predictions.extend(["move forward", "turn left", "go to kitchen"])
        elif context.get("robot_state") == "navigating":
            predictions.extend(["stop", "pause", "continue"])

        return predictions[:5]  # Limit to top 5 predictions
```

## Real-Time Performance Management

### 1. Performance Monitoring

```python
import statistics
import time
from collections import deque
import threading

class RealTimePerformanceMonitor:
    def __init__(self, history_size: int = 100):
        self.history_size = history_size
        self.latency_history = deque(maxlen=history_size)
        self.accuracy_history = deque(maxlen=history_size)
        self.cpu_usage_history = deque(maxlen=history_size)
        self.memory_usage_history = deque(maxlen=history_size)

        self.start_times = {}
        self.lock = threading.Lock()

    def start_timing(self, event_id: str):
        """Start timing for a specific event"""
        self.start_times[event_id] = time.time()

    def end_timing(self, event_id: str) -> float:
        """End timing and record latency"""
        if event_id in self.start_times:
            latency = time.time() - self.start_times[event_id]
            with self.lock:
                self.latency_history.append(latency)
            del self.start_times[event_id]
            return latency
        return 0.0

    def record_accuracy(self, success: bool):
        """Record accuracy result"""
        with self.lock:
            self.accuracy_history.append(1 if success else 0)

    def record_resource_usage(self, cpu_percent: float, memory_mb: float):
        """Record resource usage"""
        with self.lock:
            self.cpu_usage_history.append(cpu_percent)
            self.memory_usage_history.append(memory_mb)

    def get_current_metrics(self) -> Dict[str, Any]:
        """Get current performance metrics"""
        with self.lock:
            metrics = {}

            if self.latency_history:
                metrics["latency"] = {
                    "avg": statistics.mean(self.latency_history),
                    "p95": statistics.quantiles(self.latency_history, n=20)[-1] if len(self.latency_history) > 1 else 0,
                    "min": min(self.latency_history),
                    "max": max(self.latency_history),
                    "count": len(self.latency_history)
                }

            if self.accuracy_history:
                metrics["accuracy"] = {
                    "rate": statistics.mean(self.accuracy_history),
                    "count": len(self.accuracy_history)
                }

            if self.cpu_usage_history:
                metrics["cpu"] = {
                    "avg": statistics.mean(self.cpu_usage_history),
                    "max": max(self.cpu_usage_history),
                    "count": len(self.cpu_usage_history)
                }

            if self.memory_usage_history:
                metrics["memory"] = {
                    "avg": statistics.mean(self.memory_usage_history),
                    "max": max(self.memory_usage_history),
                    "count": len(self.memory_usage_history)
                }

            return metrics

    def should_adapt_performance(self) -> bool:
        """Determine if performance adaptation is needed"""
        metrics = self.get_current_metrics()

        # Define adaptation thresholds
        latency_threshold = 2.0  # seconds
        accuracy_threshold = 0.7  # 70% accuracy

        latency_issue = ("latency" in metrics and
                        metrics["latency"]["p95"] > latency_threshold)
        accuracy_issue = ("accuracy" in metrics and
                         metrics["accuracy"]["rate"] < accuracy_threshold)

        return latency_issue or accuracy_issue
```

### 2. Adaptive Resource Management

```python
import psutil
import gc
from typing import Literal

class AdaptiveResourceManager:
    def __init__(self, max_cpu_percent: float = 80.0, max_memory_mb: int = 1000):
        self.max_cpu_percent = max_cpu_percent
        self.max_memory_mb = max_memory_mb
        self.current_mode: Literal["performance", "balanced", "conservative"] = "balanced"

        # Component controllers
        self.audio_quality_level = 1.0  # 0.0 to 1.0
        self.api_batch_size = 1  # Number of concurrent API calls
        self.cache_enabled = True

    def assess_system_load(self) -> Dict[str, float]:
        """Assess current system resource usage"""
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_info = psutil.virtual_memory()
        memory_mb = memory_info.used / (1024 * 1024)

        return {
            "cpu_percent": cpu_percent,
            "memory_mb": memory_mb,
            "memory_percent": memory_info.percent
        }

    def adjust_for_load(self) -> str:
        """Adjust system settings based on current load"""
        load = self.assess_system_load()

        if load["cpu_percent"] > self.max_cpu_percent or load["memory_mb"] > self.max_memory_mb:
            # High load - switch to conservative mode
            if self.current_mode != "conservative":
                self._switch_to_conservative_mode()
                return "conservative"
        elif load["cpu_percent"] < 50 and load["memory_mb"] < self.max_memory_mb * 0.5:
            # Low load - switch to performance mode
            if self.current_mode != "performance":
                self._switch_to_performance_mode()
                return "performance"
        else:
            # Balanced load - maintain balanced mode
            if self.current_mode != "balanced":
                self._switch_to_balanced_mode()
                return "balanced"

        return self.current_mode

    def _switch_to_performance_mode(self):
        """Optimize for maximum performance"""
        self.audio_quality_level = 1.0
        self.api_batch_size = 3
        self.cache_enabled = True
        self.current_mode = "performance"

        print("Switched to performance mode: Higher quality audio, more concurrent API calls")

    def _switch_to_balanced_mode(self):
        """Balance performance and resource usage"""
        self.audio_quality_level = 0.7
        self.api_batch_size = 2
        self.cache_enabled = True
        self.current_mode = "balanced"

        print("Switched to balanced mode: Moderate quality, moderate resource usage")

    def _switch_to_conservative_mode(self):
        """Minimize resource usage"""
        self.audio_quality_level = 0.5
        self.api_batch_size = 1
        self.cache_enabled = False

        # Force garbage collection
        gc.collect()

        self.current_mode = "conservative"

        print("Switched to conservative mode: Lower quality, reduced resource usage")

    def get_optimization_recommendations(self) -> List[str]:
        """Get recommendations for performance optimization"""
        recommendations = []

        load = self.assess_system_load()

        if load["cpu_percent"] > 80:
            recommendations.append("Reduce audio processing complexity")
            recommendations.append("Limit concurrent API calls")
            recommendations.append("Implement more aggressive caching")

        if load["memory_mb"] > self.max_memory_mb * 0.8:
            recommendations.append("Reduce cache size")
            recommendations.append("Implement memory pooling")
            recommendations.append("Use more efficient data structures")

        if self.current_mode == "conservative" and load["cpu_percent"] < 30:
            recommendations.append("Consider switching to performance mode for better user experience")

        return recommendations
```

## Testing and Validation

### 1. Performance Testing Framework

```python
import asyncio
import time
import random
from typing import List, Dict, Any

class PerformanceTestFramework:
    def __init__(self, voice_processor):
        self.processor = voice_processor
        self.monitor = RealTimePerformanceMonitor()

    async def run_latency_tests(self, test_scenarios: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Run comprehensive latency tests"""
        results = {
            "scenarios": {},
            "summary": {}
        }

        for scenario in test_scenarios:
            scenario_name = scenario["name"]
            print(f"Running latency test: {scenario_name}")

            scenario_results = []

            for _ in range(scenario.get("iterations", 10)):
                # Simulate audio input
                audio_input = self._generate_test_audio(scenario)

                self.monitor.start_timing(f"end_to_end_{scenario_name}")

                try:
                    result = await self.processor.process_audio_async(audio_input)
                    latency = self.monitor.end_timing(f"end_to_end_{scenario_name}")

                    scenario_results.append({
                        "latency": latency,
                        "success": result is not None,
                        "result": result
                    })

                    self.monitor.record_accuracy(result is not None)

                except Exception as e:
                    self.monitor.end_timing(f"end_to_end_{scenario_name}")
                    scenario_results.append({
                        "latency": float('inf'),
                        "success": False,
                        "error": str(e)
                    })
                    self.monitor.record_accuracy(False)

            results["scenarios"][scenario_name] = {
                "avg_latency": statistics.mean([r["latency"] for r in scenario_results if r["latency"] != float('inf')]),
                "success_rate": sum(1 for r in scenario_results if r["success"]) / len(scenario_results),
                "total_tests": len(scenario_results)
            }

        # Generate summary
        all_latencies = []
        all_success_rates = []

        for scenario_results in results["scenarios"].values():
            all_latencies.append(scenario_results["avg_latency"])
            all_success_rates.append(scenario_results["success_rate"])

        results["summary"] = {
            "overall_avg_latency": statistics.mean(all_latencies) if all_latencies else 0,
            "overall_success_rate": statistics.mean(all_success_rates) if all_success_rates else 0,
            "test_count": len(test_scenarios)
        }

        return results

    def _generate_test_audio(self, scenario: Dict[str, Any]) -> bytes:
        """Generate test audio based on scenario"""
        # This would generate realistic test audio
        # For now, returning dummy audio
        duration = scenario.get("duration", 2.0)  # seconds
        sample_rate = 16000
        samples = int(duration * sample_rate)

        # Generate simple test signal
        import numpy as np
        t = np.linspace(0, duration, samples)
        signal = np.sin(2 * np.pi * 1000 * t)  # 1kHz tone
        audio_data = (signal * 0.5 * (2**15)).astype(np.int16)

        return audio_data.tobytes()

    def run_stress_test(self, duration_minutes: int = 5,
                       concurrent_users: int = 3) -> Dict[str, Any]:
        """Run stress test to evaluate system limits"""
        print(f"Running stress test: {duration_minutes}min, {concurrent_users} concurrent users")

        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)

        results = {
            "requests": 0,
            "successful": 0,
            "failed": 0,
            "avg_latency": 0,
            "max_latency": 0,
            "p95_latency": 0
        }

        latencies = []

        while time.time() < end_time:
            # Simulate concurrent requests
            for _ in range(concurrent_users):
                try:
                    # Process dummy audio
                    dummy_audio = self._generate_test_audio({"duration": 2.0})

                    self.monitor.start_timing("stress_test")
                    result = self.processor.process_audio(dummy_audio)
                    latency = self.monitor.end_timing("stress_test")

                    results["requests"] += 1
                    latencies.append(latency)

                    if result:
                        results["successful"] += 1
                        self.monitor.record_accuracy(True)
                    else:
                        results["failed"] += 1
                        self.monitor.record_accuracy(False)

                except Exception:
                    results["failed"] += 1
                    results["requests"] += 1
                    self.monitor.record_accuracy(False)

            # Small delay to prevent overwhelming the system
            time.sleep(0.1)

        if latencies:
            results["avg_latency"] = statistics.mean(latencies)
            results["max_latency"] = max(latencies)
            if len(latencies) > 1:
                results["p95_latency"] = statistics.quantiles(latencies, n=20)[-1]

        return results
```

### 2. Continuous Performance Monitoring

```python
import threading
import json
import os
from datetime import datetime

class ContinuousPerformanceMonitor:
    def __init__(self, output_dir: str = "performance_logs"):
        self.output_dir = output_dir
        self.is_monitoring = False
        self.monitor_thread = None

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # Initialize monitors
        self.performance_monitor = RealTimePerformanceMonitor()
        self.resource_manager = AdaptiveResourceManager()

    def start_monitoring(self, log_interval: int = 60):
        """Start continuous performance monitoring"""
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(log_interval,),
            daemon=True
        )
        self.monitor_thread.start()

        print(f"Started continuous performance monitoring (logging every {log_interval}s)")

    def stop_monitoring(self):
        """Stop continuous performance monitoring"""
        self.is_monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()

        print("Stopped continuous performance monitoring")

    def _monitor_loop(self, log_interval: int):
        """Main monitoring loop"""
        while self.is_monitoring:
            try:
                # Collect metrics
                metrics = self.performance_monitor.get_current_metrics()
                system_load = self.resource_manager.assess_system_load()

                # Combine metrics
                combined_metrics = {
                    "timestamp": datetime.now().isoformat(),
                    "performance": metrics,
                    "system_load": system_load,
                    "resource_mode": self.resource_manager.current_mode
                }

                # Log to file
                self._log_metrics(combined_metrics)

                # Adjust resources if needed
                adaptation_needed = self.performance_monitor.should_adapt_performance()
                if adaptation_needed:
                    new_mode = self.resource_manager.adjust_for_load()
                    print(f"Adapted to {new_mode} mode based on performance metrics")

                # Wait for next interval
                time.sleep(log_interval)

            except Exception as e:
                print(f"Error in monitoring loop: {e}")
                time.sleep(10)  # Brief pause before continuing

    def _log_metrics(self, metrics: Dict[str, Any]):
        """Log metrics to file"""
        timestamp = datetime.fromisoformat(metrics["timestamp"].replace("Z", "+00:00"))
        filename = f"performance_{timestamp.strftime('%Y%m%d_%H%M%S')}.json"
        filepath = os.path.join(self.output_dir, filename)

        with open(filepath, 'w') as f:
            json.dump(metrics, f, indent=2)
```

## Optimization Best Practices

### 1. Preprocessing Pipeline Optimization

```python
class OptimizedProcessingPipeline:
    def __init__(self):
        self.audio_optimizer = AudioFormatOptimizer()
        self.command_mapper = FastCommandMapper()
        self.performance_monitor = RealTimePerformanceMonitor()

    def process_voice_command_optimized(self, raw_audio: bytes) -> Dict[str, Any]:
        """Optimized end-to-end voice command processing"""
        self.performance_monitor.start_timing("total_process")

        try:
            # Step 1: Optimize audio format
            self.performance_monitor.start_timing("audio_optimization")
            optimized_audio = self.audio_optimizer.optimize_for_whisper(raw_audio)
            audio_opt_time = self.performance_monitor.end_timing("audio_optimization")

            # Step 2: Simulate Whisper transcription (in practice, call actual API)
            self.performance_monitor.start_timing("transcription_simulation")
            # This would be the actual Whisper call
            transcribed_text = self._simulate_transcription(optimized_audio)
            transcribe_time = self.performance_monitor.end_timing("transcription_simulation")

            # Step 3: Fast command mapping
            self.performance_monitor.start_timing("command_mapping")
            command_result = self.command_mapper.map_command(transcribed_text)
            mapping_time = self.performance_monitor.end_timing("command_mapping")

            # Record success
            self.performance_monitor.record_accuracy(command_result["action_type"] != "unknown/command")

            # Calculate total time
            total_time = self.performance_monitor.end_timing("total_process")

            return {
                **command_result,
                "processing_times": {
                    "audio_optimization": audio_opt_time,
                    "transcription": transcribe_time,
                    "command_mapping": mapping_time,
                    "total": total_time
                },
                "optimized": True
            }

        except Exception as e:
            total_time = self.performance_monitor.end_timing("total_process")
            return {
                "action_type": "error/processing",
                "parameters": {"error": str(e)},
                "confidence": 0.0,
                "processing_times": {"total": total_time}
            }

    def _simulate_transcription(self, audio_data: bytes) -> str:
        """Simulate Whisper transcription - replace with actual API call"""
        # In real implementation, this would call Whisper API
        # For simulation, return a simple transcription
        return "move forward"
```

### 2. Resource-Efficient Caching

```python
import weakref
from functools import lru_cache
import pickle
import hashlib

class ResourceEfficientCache:
    def __init__(self, max_size: int = 1000, max_memory_mb: float = 50.0):
        self.max_size = max_size
        self.max_memory_bytes = max_memory_mb * 1024 * 1024
        self.cache = {}
        self.access_times = {}
        self.current_memory = 0

    def get(self, key: str, default=None):
        """Get value from cache with memory management"""
        if key in self.cache:
            # Update access time
            self.access_times[key] = time.time()
            return self.cache[key]
        return default

    def set(self, key: str, value):
        """Set value in cache with size and memory management"""
        # Estimate memory usage of value
        value_size = len(pickle.dumps(value))

        # Check if adding this value would exceed memory limits
        if self.current_memory + value_size > self.max_memory_bytes:
            # Remove least recently used items
            self._evict_old_items(value_size)

        # Check if cache is at size limit
        if len(self.cache) >= self.max_size:
            # Remove least recently used item
            oldest_key = min(self.access_times.keys(), key=lambda k: self.access_times[k])
            self._remove_key(oldest_key)

        # Add new item
        self.cache[key] = value
        self.access_times[key] = time.time()
        self.current_memory += value_size

    def _evict_old_items(self, needed_space: int):
        """Evict old items to free up memory"""
        current_time = time.time()

        # Sort by access time (oldest first)
        sorted_keys = sorted(self.access_times.keys(), key=lambda k: self.access_times[k])

        for key in sorted_keys:
            if self.current_memory + needed_space <= self.max_memory_bytes:
                break

            self._remove_key(key)

    def _remove_key(self, key: str):
        """Remove key and update memory tracking"""
        if key in self.cache:
            # Estimate and remove memory usage
            value_size = len(pickle.dumps(self.cache[key]))
            self.current_memory -= value_size

            del self.cache[key]
            if key in self.access_times:
                del self.access_times[key]

# Global cache instance
global_voice_cache = ResourceEfficientCache()
```

## Performance Tuning Checklist

### Configuration Optimization
- [ ] Set appropriate energy threshold for environment
- [ ] Configure audio capture parameters (sample rate, bit depth)
- [ ] Optimize Whisper API request batching
- [ ] Configure appropriate timeout values
- [ ] Set up proper error handling and retries

### Resource Management
- [ ] Monitor CPU and memory usage
- [ ] Implement adaptive resource allocation
- [ ] Set up caching for common operations
- [ ] Configure connection pooling for API calls
- [ ] Implement garbage collection for long-running processes

### Performance Monitoring
- [ ] Set up continuous performance monitoring
- [ ] Implement alerting for performance degradation
- [ ] Track key performance metrics over time
- [ ] Log performance data for analysis
- [ ] Set up automated performance testing

### Quality Assurance
- [ ] Test with various audio quality levels
- [ ] Validate performance under load
- [ ] Test error recovery mechanisms
- [ ] Verify accuracy metrics
- [ ] Conduct user experience testing

## Troubleshooting Performance Issues

### High Latency Issues
1. **Check API connection speed**: Use faster network or consider edge deployment
2. **Optimize audio preprocessing**: Reduce computational overhead
3. **Implement caching**: Cache common transcriptions and command mappings
4. **Review system resources**: Ensure sufficient CPU and memory

### High Error Rates
1. **Improve audio quality**: Better microphones or preprocessing
2. **Adjust sensitivity settings**: Fine-tune energy thresholds
3. **Expand command patterns**: Add more variations to pattern matching
4. **Implement retry logic**: Handle transient API failures

### Resource Exhaustion
1. **Implement rate limiting**: Control API call frequency
2. **Optimize memory usage**: Use efficient data structures
3. **Reduce concurrent operations**: Limit parallel processing
4. **Enable conservative mode**: Switch to resource-saving mode

## Next Steps

After implementing these optimizations:

1. **Monitor performance**: Use the continuous monitoring tools to track improvements
2. **Conduct user testing**: Validate improvements with real users
3. **Fine-tune settings**: Adjust parameters based on usage patterns
4. **Plan for scaling**: Consider how optimizations will work at larger scale
5. **Document findings**: Record what optimizations worked best for your specific use case