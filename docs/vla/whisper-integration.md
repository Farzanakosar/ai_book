# OpenAI Whisper Integration Guide

## Overview

This guide covers how to integrate OpenAI's Whisper API for speech recognition in your robotics applications. Whisper is a state-of-the-art automatic speech recognition (ASR) system that can convert spoken language into written text with high accuracy.

## Prerequisites

Before implementing Whisper integration, ensure you have:

- An OpenAI API key
- Python 3.8 or higher installed
- The `openai` Python package installed (`pip install openai`)
- Basic understanding of REST API integration

## Basic Whisper Integration

### 1. Setting Up the OpenAI Client

First, configure the OpenAI client with your API key:

```python
import openai
import os

# Set your OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

# Alternative: directly assign the API key
# openai.api_key = "your-api-key-here"
```

### 2. Transcribing Audio Files

Whisper can transcribe audio files in several formats. Here's a basic example:

```python
import openai
from pathlib import Path

# Transcribe an audio file
audio_file = open("path/to/your/audio.wav", "rb")
transcript = openai.Audio.transcribe("whisper-1", audio_file)

print(transcript.text)
```

### 3. Supported Audio Formats

Whisper supports the following audio formats:
- MP3
- MP4
- M4A
- WAV
- MPEG
- MPGA
- WEBM
- FLAC

## Robotics-Specific Implementation

### 1. Real-time Audio Processing

For robotics applications, you'll often need to process audio in real-time. Here's an example implementation:

```python
import openai
import speech_recognition as sr
import io
import wave
import pyaudio

class VoiceToActionProcessor:
    def __init__(self, api_key):
        openai.api_key = api_key
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def listen_and_transcribe(self):
        """Listen to microphone input and transcribe using Whisper"""
        try:
            print("Listening for command...")
            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=5)

            # Convert audio to WAV format for Whisper
            wav_data = io.BytesIO()
            with wave.open(wav_data, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)  # 16-bit
                wav_file.setframerate(16000)  # 16kHz

                # Write the audio data
                wav_file.writeframes(audio.get_raw_data())

            # Reset buffer position
            wav_data.seek(0)

            # Send to Whisper API
            transcription = openai.Audio.transcribe(
                model="whisper-1",
                file=wav_data,
                response_format="text"
            )

            return transcription.strip()

        except sr.WaitTimeoutError:
            print("No speech detected within timeout period")
            return None
        except Exception as e:
            print(f"Error during transcription: {e}")
            return None
```

### 2. Batch Processing for Better Performance

For applications that can batch process commands, consider this approach:

```python
import openai
import asyncio
from concurrent.futures import ThreadPoolExecutor

class BatchVoiceProcessor:
    def __init__(self, api_key):
        openai.api_key = api_key
        self.executor = ThreadPoolExecutor(max_workers=4)

    async def process_audio_batch(self, audio_files):
        """Process multiple audio files concurrently"""
        loop = asyncio.get_event_loop()

        tasks = []
        for audio_file in audio_files:
            task = loop.run_in_executor(
                self.executor,
                self._transcribe_single_file,
                audio_file
            )
            tasks.append(task)

        results = await asyncio.gather(*tasks, return_exceptions=True)
        return results

    def _transcribe_single_file(self, audio_file_path):
        """Transcribe a single audio file"""
        with open(audio_file_path, "rb") as file:
            result = openai.Audio.transcribe("whisper-1", file)
        return result
```

## Error Handling and Best Practices

### 1. Rate Limiting

Whisper API has rate limits. Implement proper error handling:

```python
import time
import random

def safe_transcribe_with_backoff(audio_file, max_retries=3):
    """Transcribe with exponential backoff for rate limits"""
    for attempt in range(max_retries):
        try:
            result = openai.Audio.transcribe("whisper-1", audio_file)
            return result
        except openai.error.RateLimitError:
            if attempt < max_retries - 1:
                # Exponential backoff with jitter
                wait_time = (2 ** attempt) + random.uniform(0, 1)
                time.sleep(wait_time)
            else:
                raise
        except Exception as e:
            print(f"Transcription failed: {e}")
            raise
```

### 2. Audio Quality Considerations

For optimal Whisper performance in robotics:

- Use 16kHz sample rate for audio input
- Ensure clear, noise-free audio capture
- Consider preprocessing audio to reduce background noise
- Use directional microphones when possible

## Performance Optimization

### 1. Caching Common Commands

For frequently used commands, consider caching:

```python
from functools import lru_cache

class OptimizedVoiceProcessor:
    def __init__(self, api_key):
        openai.api_key = api_key
        self.command_cache = {}

    @lru_cache(maxsize=100)
    def _cached_transcribe(self, audio_hash):
        """Cached transcription for repeated audio"""
        # Implementation depends on your specific use case
        pass
```

### 2. Model Selection

Whisper offers different models with trade-offs between speed and accuracy:

- `whisper-1`: The default model, balancing speed and accuracy
- Consider using different models based on your application's requirements

## Integration with Robot Actions

Here's how to connect Whisper output to robot actions:

```python
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
```

## Testing Your Integration

### 1. Unit Tests

```python
import unittest
from unittest.mock import patch, MagicMock

class TestWhisperIntegration(unittest.TestCase):
    @patch('openai.Audio.transcribe')
    def test_transcription_success(self, mock_transcribe):
        # Mock the API response
        mock_transcribe.return_value = MagicMock(text="move forward")

        # Test your voice processing logic
        processor = VoiceToActionProcessor("test-key")
        result = processor._transcribe_audio("test.wav")

        self.assertEqual(result, "move forward")
```

### 2. Integration Tests

Test the complete voice-to-action pipeline:

```python
def test_voice_command_pipeline():
    """Test complete pipeline from voice input to robot action"""
    # This would typically involve:
    # 1. Providing audio input (could be pre-recorded files)
    # 2. Running through Whisper transcription
    # 3. Mapping to robot action
    # 4. Verifying correct action was generated
    pass
```

## Troubleshooting Common Issues

### 1. API Connection Errors

- Verify your OpenAI API key is correct and active
- Check your internet connection
- Ensure you have sufficient API quota

### 2. Poor Transcription Quality

- Check audio quality and noise levels
- Verify audio format is supported
- Consider preprocessing audio to improve quality

### 3. High Latency

- Optimize audio capture and processing pipeline
- Consider using smaller Whisper models for faster processing
- Implement caching for common commands

## Next Steps

After implementing basic Whisper integration, consider:

1. Adding confidence scoring to transcription results
2. Implementing custom wake word detection
3. Adding support for multiple languages
4. Integrating with your specific robot platform (ROS 2, etc.)