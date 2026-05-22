# Building ASR + LLM + TTS Pipeline

## Introduction

Imagine talking to your Jetson device and receiving spoken responses—just like talking to a friend. This is exactly what an **ASR + LLM + TTS** pipeline enables. By combining three AI capabilities:
- **ASR** (Automatic Speech Recognition): Listen to your voice and convert it to text
- **LLM** (Large Language Model): Understand and generate intelligent responses
- **TTS** (Text-to-Speech): Convert the text response back to natural speech

You create a complete **voice assistant** that runs entirely offline on your Jetson device.

<p align="center">
  <img src="../images/5-6-asr-llm-tts-01.png" alt="ASR+LLM+TTS Pipeline" width="800">
  <br>
  <sub>Complete Voice Assistant Pipeline Architecture</sub>
</p>

## Pipeline Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Voice Assistant Pipeline                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  🎤 Microphone ──→ [ASR] ──→ Text ──→ [LLM] ──→ Text ──→ [TTS] ──→ 🔊 Speaker
│                                                                  │
│  Speech-to-Text    Whisper    Response    Llama/Qwen    Speech   Parler-TTS
│                                                    Generation   │
└─────────────────────────────────────────────────────────────────┘
```

### How It Works

1. **Speech Capture**: The system listens through a microphone and captures audio
2. **ASR Processing**: The speech recognition model converts audio to text
3. **LLM Generation**: The language model processes the text and generates a response
4. **TTS Synthesis**: The text-to-speech model converts the response back to audio
5. **Audio Output**: The system plays the generated speech through speakers

## System Requirements

| Component | Minimum | Recommended |
|:----------|:--------|:------------|
| **Device** | Jetson Orin Nano 8GB | Jetson Orin NX 16GB |
| **RAM** | 16GB | 32GB |
| **Storage** | 50GB free | 100GB+ |
| **Audio** | USB microphone | USB microphone + speakers |
| **Network** | Initial setup only | Stable for model downloads |

### Hardware Setup

You'll need:
- **USB Microphone**: For capturing speech input
- **Speakers/Headphones**: For audio output
- **Jetson Device**: Orin Nano 8GB+ recommended

## Component Deep Dive

### ASR: Whisper (Automatic Speech Recognition)

**OpenAI Whisper** is a state-of-the-art speech recognition model that supports multiple languages with high accuracy.

#### Installation

```bash
# Pull the Whisper Docker image via jetson-examples
reComputer run whisper

# Or install manually
pip install openai-whisper

# For Jetson, install with CUDA support
pip install openai-whisper --no-cache-dir
```

#### Download Models

```bash
# Available model sizes
# tiny:   39M parameters  - Fastest, less accurate
# base:   74M parameters  - Good balance
# small:  244M parameters - Better accuracy
# medium: 769M parameters - High accuracy
# large:  1550M parameters - Best accuracy, slowest

# For Jetson, use base or small
python3 -c "import whisper; whisper.load_model('base')"
```

#### Basic Usage

```python
import whisper

# Load the model
model = whisper.load_model("base")

# Transcribe an audio file
result = model.transcribe("audio.wav")
print(f"Detected language: {result['language']}")
print(f"Transcription: {result['text']}")
```

#### Real-time Transcription

```python
import whisper
import pyaudio
import numpy as np

def real_time_transcription():
    """Capture audio from microphone and transcribe in real-time."""
    model = whisper.load_model("base")
    
    # Initialize PyAudio
    p = pyaudio.PyAudio()
    stream = p.open(
        format=pyaudio.paFloat32,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=4096
    )
    
    print("Listening... (press Ctrl+C to stop)")
    
    try:
        while True:
            # Capture audio chunk
            audio_data = np.frombuffer(stream.read(4096), dtype=np.float32)
            
            # Transcribe (simplified - real implementation needs buffering)
            # For production, use whisper's streaming API
    except KeyboardInterrupt:
        print("Stopped listening.")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == "__main__":
    real_time_transcription()
```

### LLM: Language Model Response Generation

We'll use Ollama (from Module 5.2) for language model inference:

```bash
# Ensure Ollama is running
ollama serve &

# Pull a fast, conversational model
ollama pull llama3.2:3b
```

#### LLM Integration

```python
import requests

def get_llm_response(text, model="llama3.2:3b"):
    """Send transcribed text to LLM and get response."""
    response = requests.post(
        "http://localhost:11434/api/chat",
        json={
            "model": model,
            "messages": [
                {
                    "role": "system",
                    "content": "You are a helpful voice assistant. Keep responses concise and conversational. Limit responses to 2-3 sentences."
                },
                {"role": "user", "content": text}
            ],
            "stream": False
        }
    )
    return response.json()["message"]["content"]

# Example usage
user_text = "What's the weather like today?"
response = get_llm_response(user_text)
print(f"LLM Response: {response}")
```

### TTS: Text-to-Speech

**Parler-TTS** is an open-source text-to-speech model that generates natural-sounding speech.

#### Installation

```bash
# Pull the Parler-TTS image via jetson-examples
reComputer run parler-tts

# Or install manually
pip install parler-tts
```

#### Basic Usage

```python
import torch
from parler_tts import ParlerTTSForConditionalGeneration
from transformers import AutoTokenizer
import soundfile as sf

# Load model
model = ParlerTTSForConditionalGeneration.from_pretrained(
    "parler-tts/parler_tts_v1"
)
tokenizer = AutoTokenizer.from_pretrained(
    "parler-tts/parler_tts_v1"
)

def text_to_speech(text, output_file="output.wav"):
    """Convert text to speech using Parler-TTS."""
    # Description of the voice
    description = "A friendly, clear voice speaking at a moderate pace."
    
    # Tokenize
    input_ids = tokenizer(description, return_tensors="pt").input_ids
    prompt_input_ids = tokenizer(text, return_tensors="pt").input_ids
    
    # Generate audio
    with torch.no_grad():
        generation = model.generate(
            input_ids=input_ids,
            prompt_input_ids=prompt_input_ids,
            max_new_tokens=256
        )
    
    audio = generation.cpu().numpy().squeeze()
    
    # Save to file
    sf.write(output_file, audio, model.config.sampling_rate)
    print(f"Audio saved to {output_file}")

# Example usage
text_to_speech("Hello! I'm your AI assistant running on Jetson.")
```

## Building the Complete Pipeline

Now let's combine all components into a complete voice assistant:

### main.py

```python
#!/usr/bin/env python3
"""
Voice Assistant Pipeline: ASR + LLM + TTS
Runs entirely offline on NVIDIA Jetson
"""

import whisper
import requests
import torch
import soundfile as sf
from parler_tts import ParlerTTSForConditionalGeneration
from transformers import AutoTokenizer
import pyaudio
import numpy as np
import threading
import queue
import time

class VoiceAssistant:
    def __init__(self):
        print("Initializing Voice Assistant Pipeline...")
        
        # Load ASR model (Whisper)
        print("[1/3] Loading ASR model (Whisper)...")
        self.asr_model = whisper.load_model("base")
        
        # Initialize LLM (Ollama)
        print("[2/3] Connecting to LLM (Ollama)...")
        self.llm_url = "http://localhost:11434/api/chat"
        
        # Load TTS model (Parler-TTS)
        print("[3/3] Loading TTS model (Parler-TTS)...")
        self.tts_model = ParlerTTSForConditionalGeneration.from_pretrained(
            "parler-tts/parler_tts_v1"
        )
        self.tts_tokenizer = AutoTokenizer.from_pretrained(
            "parler-tts/parler_tts_v1"
        )
        
        print("Voice Assistant ready!")
        print("=" * 50)
    
    def speech_to_text(self, audio_path):
        """Convert speech to text using Whisper."""
        print("  [ASR] Transcribing audio...")
        result = self.asr_model.transcribe(audio_path)
        return result["text"]
    
    def generate_response(self, text):
        """Generate response using LLM."""
        print(f"  [LLM] Processing: '{text}'")
        response = requests.post(
            self.llm_url,
            json={
                "model": "llama3.2:3b",
                "messages": [
                    {
                        "role": "system",
                        "content": "You are a helpful voice assistant on a Jetson device. "
                                  "Keep responses concise (2-3 sentences max). "
                                  "Be friendly and helpful."
                    },
                    {"role": "user", "content": text}
                ],
                "stream": False
            }
        )
        return response.json()["message"]["content"]
    
    def text_to_speech(self, text, output_path="response.wav"):
        """Convert text to speech using Parler-TTS."""
        print(f"  [TTS] Generating speech for: '{text[:50]}...'")
        
        description = "A friendly, clear voice speaking at a moderate pace."
        input_ids = self.tts_tokenizer(description, return_tensors="pt").input_ids
        prompt_input_ids = self.tts_tokenizer(text, return_tensors="pt").input_ids
        
        with torch.no_grad():
            generation = self.tts_model.generate(
                input_ids=input_ids,
                prompt_input_ids=prompt_input_ids,
                max_new_tokens=256
            )
        
        audio = generation.cpu().numpy().squeeze()
        sf.write(output_path, audio, self.tts_model.config.sampling_rate)
        return output_path
    
    def play_audio(self, audio_path):
        """Play audio file through speakers."""
        data, samplerate = sf.read(audio_path)
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=samplerate,
            output=True
        )
        stream.write(data.tobytes())
        stream.stop_stream()
        stream.close()
        p.terminate()
    
    def record_audio(self, output_path="input.wav", duration=5):
        """Record audio from microphone."""
        print("  [MIC] Listening... (speak now)")
        
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024
        )
        
        frames = []
        for _ in range(0, int(16000 / 1024 * duration)):
            data = stream.read(1024)
            frames.append(data)
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Save recording
        audio_data = np.frombuffer(b''.join(frames), dtype=np.float32)
        sf.write(output_path, audio_data, 16000)
        print(f"  [MIC] Recording saved: {output_path}")
        return output_path
    
    def process_query(self):
        """Process a single voice query through the full pipeline."""
        # Step 1: Record audio
        audio_path = self.record_audio(duration=5)
        
        # Step 2: ASR - Speech to Text
        text = self.speech_to_text(audio_path)
        print(f"  You said: '{text}'")
        
        # Step 3: LLM - Generate response
        response = self.generate_response(text)
        print(f"  Response: '{response}'")
        
        # Step 4: TTS - Text to Speech
        speech_path = self.text_to_speech(response)
        
        # Step 5: Play response
        print("  [AUDIO] Playing response...")
        self.play_audio(speech_path)
    
    def run(self):
        """Run the voice assistant in a loop."""
        print("\nVoice Assistant Started!")
        print("Speak to interact. Press Ctrl+C to exit.\n")
        
        try:
            while True:
                input("Press Enter to start listening...")
                self.process_query()
        except KeyboardInterrupt:
            print("\nShutting down voice assistant.")

if __name__ == "__main__":
    assistant = VoiceAssistant()
    assistant.run()
```

### Install Dependencies

```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install required packages
pip install whisper parler-tts torch torchvision torchaudio
pip install pyaudio soundfile numpy requests

# Ensure Ollama is running
ollama serve &
ollama pull llama3.2:3b
```

### Run the Assistant

```bash
# Make sure Ollama is running
ollama serve &

# Run the voice assistant
python main.py
```

## Optimization for Jetson

### Reduce Memory Usage

```python
# Use smaller models
ASR_MODEL = "tiny"      # Instead of "base"
LLM_MODEL = "llama3.2:1b"  # Instead of 3b
# Use Parler-TTS small variant

# Enable GPU acceleration where possible
import torch
device = "cuda" if torch.cuda.is_available() else "cpu"
```

### Pipeline Optimization

```python
# Process in parallel where possible
# While ASR processes current query, TTS can process previous response

def optimized_pipeline(assistant, audio_queue, response_queue):
    """Optimized pipeline with parallel processing."""
    while True:
        # Get audio from queue
        audio_path = audio_queue.get()
        
        # ASR
        text = assistant.speech_to_text(audio_path)
        response_queue.put(("asr_done", text))
        
        # LLM
        response = assistant.generate_response(text)
        response_queue.put(("llm_done", response))
        
        # TTS
        speech_path = assistant.text_to_speech(response)
        response_queue.put(("tts_done", speech_path))
```

### Performance Monitoring

```bash
# Monitor GPU usage
watch -n 0.5 nvidia-smi

# Check memory
free -h

# Profile the pipeline
python -m cProfile main.py
```

## Customization Options

### Change Voice Characteristics

```python
# Different voice descriptions for Parler-TTS
VOICES = {
    "default": "A friendly, clear voice speaking at a moderate pace.",
    "professional": "A professional, authoritative voice speaking clearly.",
    "casual": "A relaxed, casual voice speaking conversationally.",
    "energetic": "An energetic, enthusiastic voice speaking quickly."
}

def text_to_speech(text, voice="default"):
    description = VOICES[voice]
    # ... rest of TTS code
```

### Change LLM Behavior

```python
# Different system prompts for different use cases
PROMPTS = {
    "general": "You are a helpful voice assistant. Keep responses concise.",
    "coding": "You are a programming assistant. Explain code briefly.",
    "learning": "You are a patient teacher. Explain concepts simply.",
    "creative": "You are a creative storyteller. Be engaging and fun."
}

def get_llm_response(text, mode="general"):
    # ... use PROMPTS[mode] as system prompt
```

### Add Wake Word Detection

```python
# Simple wake word detection (before processing)
def detect_wake_word(audio, wake_word="hey assistant"):
    """Detect if the audio contains a wake word."""
    text = asr_model.transcribe(audio)["text"].lower()
    return wake_word in text
```

## Common Issues and Solutions

### Issue 1: Audio Device Not Found

**Problem**: `OSError: No default input device found`

**Solution**:
```bash
# Check available audio devices
python3 -c "import pyaudio; p=pyaudio.PyAudio(); print([p.get_device_info_by_index(i) for i in range(p.get_device_count())])"

# Install ALSA utilities
sudo apt-get install alsa-utils

# Test microphone
arecord -d 5 test.wav
aplay test.wav
```

### Issue 2: TTS Generation Too Slow

**Problem**: Long delays in speech generation

**Solution**:
```python
# Use streaming TTS if available
# Or pre-generate common phrases
# Or use a smaller TTS model

# Reduce output quality for speed
generation = model.generate(
    input_ids=input_ids,
    max_new_tokens=100,  # Instead of 256
    num_beams=1,          # Instead of 4
    do_sample=False       # Faster, deterministic
)
```

### Issue 3: Pipeline Latency Too High

**Problem**: Too much delay between input and response

**Solution**:
```python
# Use smaller models across the board
# ASR: tiny model instead of base
# LLM: 1B model instead of 3B
# TTS: Use faster synthesis method

# Or implement streaming:
# - Start TTS as soon as first sentence is complete
# - Don't wait for full LLM response
```

## Practice Exercise

1. **Set up hardware**: Connect USB microphone and speakers
2. **Install dependencies**: Set up all three components (Whisper, Ollama, Parler-TTS)
3. **Test each component individually** before combining
4. **Run the complete pipeline** and test with different queries
5. **Customize** the voice and LLM behavior
6. **Optimize** for better performance on your Jetson device
7. **Add features**: Wake word detection, conversation history, or skills

## Advanced Extensions

### Add Conversation Memory

```python
class VoiceAssistantWithMemory(VoiceAssistant):
    def __init__(self):
        super().__init__()
        self.conversation_history = []
    
    def process_query(self):
        # ... existing pipeline ...
        
        # Store in history
        self.conversation_history.append({
            "user": text,
            "assistant": response
        })
        
        # Include history in LLM context
        messages = [
            {"role": "system", "content": "You are a helpful voice assistant."},
        ] + [
            {"role": "user", "content": h["user"]}
            for h in self.conversation_history[-5:]  # Last 5 exchanges
        ] + [
            {"role": "assistant", "content": h["assistant"]}
            for h in self.conversation_history[-5:]
        ]
```

### Add Skills/Actions

```python
def handle_command(text):
    """Handle special commands before sending to LLM."""
    text_lower = text.lower()
    
    if "what time" in text_lower:
        import datetime
        return f"The current time is {datetime.datetime.now().strftime('%I:%M %p')}"
    
    elif "what date" in text_lower:
        import datetime
        return f"Today is {datetime.datetime.now().strftime('%A, %B %d, %Y')}"
    
    elif "shut down" in text_lower:
        return "SHUTDOWN"  # Signal to exit
    
    return None  # No special command, use LLM
```

## References

- [OpenAI Whisper](https://github.com/openai/whisper) - Speech recognition model
- [Parler-TTS](https://github.com/huggingface/parler-tts) - Text-to-speech model
- [Ollama](https://ollama.com/) - Local LLM inference
- [Jetson Audio](https://developer.nvidia.com/embedded/jetson) - Audio setup guides
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/) - Audio I/O library

---

**Congratulations!** You've completed Module 5 and learned how to run LLMs offline on your Jetson device using multiple frameworks, and build a complete voice assistant pipeline!

**Next**: Proceed to [Chapter 6: Generative AI Applications](../../6-Generative-AI/README.md) or return to [Chapter 5 Overview](../README.md).
