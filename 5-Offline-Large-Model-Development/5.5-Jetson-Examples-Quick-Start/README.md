# Jetson Examples Quick Start

## Introduction

The **jetson-examples** repository by Seeed Studios provides pre-built, ready-to-run AI examples for NVIDIA Jetson devices. Instead of spending hours setting up environments and dependencies, you can deploy complex AI models with a single command using Docker containers.

This module shows you how to use the `jetson-examples` CLI tool to quickly deploy LLMs and other AI models on your Jetson device.

<p align="center">
  <img src="../images/5-5-jetson-examples-01.png" alt="Jetson Examples" width="800">
  <br>
  <sub>jetson-examples - One-Command AI Deployment</sub>
</p>

## What is jetson-examples?

**jetson-examples** is a command-line tool that:
- Provides **one-line deployment** for popular AI models
- Uses **Docker containers** for isolated, reproducible environments
- Includes **pre-optimized configurations** for Jetson hardware
- Supports **multiple frameworks**: PyTorch, TensorRT, Ollama, and more

| Feature | Description |
|:--------|:------------|
| **Easy Installation** | Install with `pip3 install jetson-examples` |
| **One-Command Run** | `reComputer run <example>` to deploy |
| **Docker-based** | Consistent environments across devices |
| **Auto-Configuration** | Pre-tuned for Jetson performance |
| **Regular Updates** | New examples added regularly |

## Installation

### Install the CLI Tool

```bash
# Install jetson-examples via pip
pip3 install jetson-examples

# Verify installation
reComputer --help
```

### Alternative: Manual Docker Setup

If you prefer manual control:

```bash
# Install Docker (if not already installed)
sudo apt-get update
sudo apt-get install -y docker.io

# Add user to docker group (requires logout/login)
sudo usermod -aG docker $USER

# Verify Docker
docker --version
```

## Available LLM Examples

Here are the LLM-related examples available in the jetson-examples repository:

### Text-Only LLMs

| Example | Type | Model Size | Command |
|:--------|:-----|:-----------|:--------|
| **llama3** | Text (LLM) | 4.9GB model | `reComputer run llama3` |
| **gemma4** | Text (LLM) | 2.5GB model | `reComputer run gemma4` |
| **qwen3.5-4b** | Text (LLM) | 2.5GB model | `reComputer run qwen3.5-4b` |
| **qwen3.6-35b** | Text (LLM) | 28GB model | `reComputer run qwen3.6-35b` |
| **nemotron-3-nano** | Text (LLM) | 24.5GB model | `reComputer run nemotron-3-nano` |
| **text-generation-webui** | Text (LLM) | 3.9GB model | `reComputer run text-generation-webui` |

### Vision-Language Models (VLM)

| Example | Type | Model Size | Command |
|:--------|:-----|:-----------|:--------|
| **llava-v1.5** | VLM | 13GB model | `reComputer run llava-v1.5-7b` |
| **llava-v1.6** | VLM | 13GB model | `reComputer run llava-v1.6-vicuna-7b` |
| **live-vlm-webui** | VLM | 13GB model | `reComputer run live-vlm-webui` |
| **gemma4** | VLM | 2.5GB model | `reComputer run gemma4` |

### Audio & Speech

| Example | Type | Model Size | Command |
|:--------|:-----|:-----------|:--------|
| **whisper** | ASR (Speech-to-Text) | 1.5GB model | `reComputer run whisper` |
| **parler-tts** | TTS (Text-to-Speech) | 6.9GB image | `reComputer run parler-tts` |

### Inference Server

| Example | Type | Model Size | Command |
|:--------|:-----|:-----------|:--------|
| **ollama** | Inference Server | 10.5GB image | `reComputer run ollama` |

## Getting Started

### Example 1: Running Ollama

The quickest way to start with LLMs on Jetson:

```bash
# Deploy Ollama with Docker
reComputer run ollama
```

Once running, you can pull and run models:
```bash
# Enter the Docker container
docker exec -it ollama bash

# Pull and run a model
ollama run llama3.2:3b
```

### Example 2: Running a Vision-Language Model

```bash
# Deploy LLaVA for image understanding
reComputer run llava-v1.5-7b
```

This sets up a model that can understand and describe images.

### Example 3: Deploying Text Generation WebUI

For a full-featured web interface to interact with LLMs:

```bash
# Deploy text-generation-webui (Oobabooga)
reComputer run text-generation-webui
```

Access the web interface at `http://localhost:5000`.

### Example 4: Deploying Whisper for Speech Recognition

```bash
# Deploy OpenAI Whisper for speech-to-text
reComputer run whisper
```

## Practical Example: Building a Chatbot

Let's put it all together to create a simple chatbot using jetson-examples:

### Step 1: Deploy Ollama

```bash
reComputer run ollama
```

### Step 2: Pull a Model

```bash
# Check running containers
docker ps

# Enter the Ollama container
docker exec -it ollama ollama pull llama3.2:3b
```

### Step 3: Test via API

```bash
# Test the chat API
curl http://localhost:11434/api/chat -d '{
  "model": "llama3.2:3b",
  "messages": [
    {"role": "user", "content": "What can you help me with?"}
  ],
  "stream": false
}'
```

### Step 4: Python Integration

```python
import requests

def chat_with_model(user_message, model="llama3.2:3b"):
    response = requests.post(
        "http://localhost:11434/api/chat",
        json={
            "model": model,
            "messages": [{"role": "user", "content": user_message}],
            "stream": False
        }
    )
    return response.json()["message"]["content"]

# Interactive chat
while True:
    user_input = input("You: ")
    if user_input.lower() in ['quit', 'exit', 'q']:
        break
    response = chat_with_model(user_input)
    print(f"AI: {response}\n")
```

## Managing Containers

### Common Docker Commands

```bash
# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# View container logs
docker logs <container-name>

# Stop a container
docker stop <container-name>

# Restart a container
docker restart <container-name>

# Remove a container
docker rm <container-name>

# Enter a running container
docker exec -it <container-name> bash
```

### Resource Monitoring

```bash
# Monitor container resource usage
docker stats

# Check GPU usage
nvidia-smi

# Check disk usage
docker system df
```

## Customizing Deployments

### Environment Variables

Many examples support configuration via environment variables:

```bash
# Example: Setting model parameters for Ollama
docker run -d \
  --name ollama \
  --gpus all \
  -e OLLAMA_HOST=0.0.0.0:11434 \
  -p 11434:11434 \
  ollama/ollama:latest
```

### Volume Mounts

Persist data between container restarts:

```bash
# Mount a local directory for model storage
docker run -d \
  --name ollama \
  --gpus all \
  -v /path/to/models:/root/.ollama \
  -p 11434:11434 \
  ollama/ollama:latest
```

### Network Configuration

Access services from other devices on your network:

```bash
# Bind to all interfaces (not just localhost)
docker run -d \
  --name ollama \
  --gpus all \
  -p 0.0.0.0:11434:11434 \
  ollama/ollama:latest
```

## Performance Tips

### 1. Use GPU Passthrough

Always ensure Docker has GPU access:

```bash
# Verify GPU access in container
docker exec -it <container-name> nvidia-smi
```

### 2. Allocate Appropriate Memory

```bash
# Limit container memory (optional, for multi-container setups)
docker run -d \
  --name ollama \
  --gpus all \
  --memory=12g \
  --memory-swap=16g \
  ollama/ollama:latest
```

### 3. Use SSD Storage

Models load faster from SSD than SD card or eMMC.

### 4. Monitor and Optimize

```bash
# Check what's consuming resources
htop        # CPU and memory
nvidia-smi  # GPU
df -h       # Disk
```

## Common Issues and Solutions

### Issue 1: Docker Permission Denied

**Problem**: `Got permission denied while trying to connect to the Docker daemon socket`

**Solution**:
```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Log out and log back in for changes to take effect
# Or run in current session:
newgrp docker
```

### Issue 2: GPU Not Available in Container

**Problem**: `nvidia-smi` fails inside the container

**Solution**:
```bash
# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Restart Docker
sudo systemctl restart docker
```

### Issue 3: Container Won't Start

**Problem**: Container exits immediately after starting

**Solution**:
```bash
# Check container logs for errors
docker logs <container-name>

# Check available resources
free -h
df -h
nvidia-smi
```

## Example Repositories and Projects

Explore these examples from the jetson-examples repository:

### llama-factory
Fine-tune LLMs on your Jetson device:
```bash
reComputer run llama-factory
```
Model/data size: 13.5GB

### live-vlm-webui
Real-time vision-language model interface:
```bash
reComputer run live-vlm-webui
```
Features: Live camera feed processing with AI understanding

### deep-live-cam
Real-time face swap and processing:
```bash
reComputer run deep-live-cam
```

### ultralytics-yolo
Object detection with YOLO:
```bash
reComputer run ultralytics-yolo
```

## Practice Exercise

1. **Install jetson-examples** tool
2. **Deploy Ollama** and run `llama3.2:3b`
3. **Deploy LLaVA** for image understanding
4. **Create a Python script** that interacts with the deployed model
5. **Set up Open WebUI** for a graphical interface
6. **Explore** other examples in the repository

## References

- [jetson-examples Repository](https://github.com/Seeed-Projects/jetson-examples) - Official repository
- [jetson-examples Documentation](https://github.com/Seeed-Projects/jetson-examples/blob/main/docs/examples.md) - Complete example list
- [Docker Documentation](https://docs.docker.com/) - Docker fundamentals
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) - GPU support in Docker
- [Ollama Documentation](https://ollama.com/docs) - Ollama usage guide

---

**Next**: Continue to [Module 5.6: Building ASR + LLM + TTS Pipeline](../5.6-ASR-LLM-TTS-Pipeline/README.md) to create a complete voice assistant!
