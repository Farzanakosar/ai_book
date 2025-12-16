# Voice-to-Action Prerequisites Guide

## Overview

This guide outlines the prerequisites needed to implement and run voice-to-action pipelines using OpenAI Whisper for robotics applications. Following these prerequisites ensures your system can properly process voice commands and map them to robot actions.

## System Requirements

### Hardware Requirements

#### For Development and Testing
- **Processor**: 64-bit processor with 4+ cores (Intel i5 or AMD Ryzen 5 equivalent or better)
- **RAM**: 8GB+ (16GB recommended for simulation environments)
- **Storage**: 10GB+ free disk space
- **Network**: Stable internet connection (required for OpenAI API access)

#### For Audio Capture
- **Microphone**: USB microphone or built-in laptop/desktop microphone
- **Audio Quality**: Support for 16kHz sample rate, 16-bit depth preferred
- **Noise Environment**: Quiet environment preferred for best results

### Software Requirements

#### Operating System
- **Windows**: Windows 10/11 (64-bit)
- **macOS**: macOS 10.14 or later
- **Linux**: Ubuntu 18.04 or later, other distributions with Python 3.8+ support

#### Core Software Dependencies
1. **Python**: Version 3.8 or higher
2. **pip**: Python package installer (usually included with Python)
3. **Git**: Version control system
4. **OpenAI API Key**: Required for Whisper access

#### Python Packages
```bash
# Core packages
pip install openai
pip install python-dotenv  # For environment management

# Audio processing
pip install pyaudio
pip install speechrecognition
pip install sounddevice
pip install numpy

# For file handling
pip install pydub
pip install wave

# Testing and utilities
pip install pytest
pip install requests
```

## Development Environment Setup

### 1. Python Virtual Environment

Create and activate a virtual environment to isolate dependencies:

```bash
# Create virtual environment
python -m venv vla_env

# Activate virtual environment
# On Windows:
vla_env\Scripts\activate
# On macOS/Linux:
source vla_env/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 2. OpenAI API Configuration

1. **Get OpenAI API Key**:
   - Visit [OpenAI Platform](https://platform.openai.com/)
   - Create an account or log in
   - Navigate to "API Keys" section
   - Create a new secret key

2. **Set Up Environment Variables**:
   ```bash
   # Create .env file in your project root
   echo "OPENAI_API_KEY=your-api-key-here" > .env
   ```

3. **Python Configuration**:
   ```python
   import openai
   import os
   from dotenv import load_dotenv

   # Load environment variables
   load_dotenv()

   # Set OpenAI API key
   openai.api_key = os.getenv("OPENAI_API_KEY")
   ```

### 3. Audio System Configuration

#### Installing PyAudio (may require additional steps)

PyAudio can be tricky to install on some systems. Try these approaches:

```bash
# Method 1: Direct pip install
pip install pyaudio

# Method 2: If Method 1 fails, try pre-compiled wheels
pip install --only-binary=all pyaudio

# Method 3: On Windows, you might need to download from:
# https://www.lfd.uci.edu/~gohlke/pythonlibs/#pyaudio
# Then install with: pip install path/to/downloaded/wheel
```

#### Testing Audio Setup

```python
import pyaudio
import speech_recognition as sr

def test_audio_setup():
    """Test if audio input/output is properly configured"""
    print("Testing audio setup...")

    # Test PyAudio
    try:
        p = pyaudio.PyAudio()
        print(f"PyAudio: Available input devices: {p.get_device_count()}")
        p.terminate()
        print("✓ PyAudio is working correctly")
    except Exception as e:
        print(f"✗ PyAudio error: {e}")
        return False

    # Test SpeechRecognition
    try:
        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("✓ SpeechRecognition can access microphone")
    except Exception as e:
        print(f"✗ SpeechRecognition error: {e}")
        print("This might be due to missing audio drivers or PyAudio issues")
        return False

    return True

# Run the test
if test_audio_setup():
    print("\n✓ All audio prerequisites are met!")
else:
    print("\n✗ Some audio prerequisites need attention.")
```

## Robotics Environment Setup

### ROS 2 Integration (Optional but Recommended)

If integrating with ROS 2 for robot control:

#### 1. Install ROS 2
- **Ubuntu**: Follow the official ROS 2 Humble Hawksbill installation guide
- **Windows**: Use the Windows development guide
- **macOS**: Limited support, Ubuntu VM recommended

#### 2. Required ROS 2 Packages
```bash
# Install ROS 2 desktop
sudo apt install ros-humble-desktop

# Install Python interfaces
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

#### 3. Python ROS 2 Client
```bash
pip install ros2
pip install rclpy
```

## Network and API Prerequisites

### 1. OpenAI API Access
- Verify your OpenAI account has sufficient credits
- Check API usage limits and quotas
- Ensure your network allows HTTPS connections to OpenAI endpoints

### 2. Rate Limits and Quotas
- Understand Whisper API rate limits (typically requests per minute)
- Plan for cost management based on expected usage
- Implement retry logic for rate limit handling

### 3. Firewall and Proxy Settings
If behind a corporate firewall:
```python
import openai

# Configure proxy if needed
openai.proxy = "http://your-proxy:port"
```

## Testing Prerequisites

### 1. Basic API Test
```python
import openai
import os
from dotenv import load_dotenv

load_dotenv()

def test_openai_connection():
    """Test OpenAI API connection"""
    try:
        # Test with a simple completion
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": "Hello"}],
            max_tokens=5
        )
        print("✓ OpenAI API connection successful")
        return True
    except Exception as e:
        print(f"✗ OpenAI API connection failed: {e}")
        return False

# Run the test
test_openai_connection()
```

### 2. Audio Test
```python
import speech_recognition as sr

def test_audio_input():
    """Test audio input capabilities"""
    recognizer = sr.Recognizer()

    try:
        with sr.Microphone() as source:
            print("Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source)
            print("✓ Audio input test successful")
            return True
    except Exception as e:
        print(f"✗ Audio input test failed: {e}")
        print("This could be due to:")
        print("- No microphone detected")
        print("- Microphone permissions not granted")
        print("- PyAudio not installed correctly")
        return False

test_audio_input()
```

## Optional Enhancements

### 1. Docker Setup (For Isolated Environment)
```dockerfile
FROM python:3.9-slim

# Install system dependencies for audio
RUN apt-get update && apt-get install -y \
    build-essential \
    portaudio19-dev \
    python3-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install Python packages
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

WORKDIR /app
COPY . .

CMD ["python", "voice_robot.py"]
```

### 2. Development Tools
```bash
# Additional tools for development
pip install black          # Code formatting
pip install flake8         # Code linting
pip install pytest         # Testing framework
pip install pytest-cov     # Coverage reporting
```

## Troubleshooting Common Issues

### 1. PyAudio Installation Issues
- **Problem**: `pip install pyaudio` fails
- **Solution**: Use pre-compiled wheels or install system packages:
  ```bash
  # Ubuntu/Debian
  sudo apt-get install python3-pyaudio

  # Or try conda
  conda install pyaudio
  ```

### 2. Microphone Access Issues
- **Problem**: "No Microphone" or permission errors
- **Solution**: Check system audio settings and application permissions

### 3. API Connection Issues
- **Problem**: "Invalid API key" or connection errors
- **Solution**: Verify API key and network connectivity

### 4. Audio Quality Issues
- **Problem**: Poor transcription quality
- **Solution**: Use external microphone, reduce background noise, check audio format

## Verification Checklist

Before proceeding with voice-to-action implementation, verify:

- [ ] Python 3.8+ installed and accessible
- [ ] OpenAI API key configured and valid
- [ ] Microphone accessible and functional
- [ ] Required Python packages installed
- [ ] Internet connection available for API calls
- [ ] Audio system properly configured
- [ ] (Optional) ROS 2 environment set up if needed

## Next Steps

After verifying all prerequisites:

1. Proceed to the [OpenAI Whisper Integration Guide](./whisper-integration.md) to implement speech recognition
2. Move to [Voice Command Mapping](./voice-command-mapping.md) to connect voice commands to robot actions
3. Review [Latency and Accuracy Considerations](./latency-accuracy.md) for performance optimization
4. Begin implementing the voice-to-action pipeline with the foundation established by these prerequisites