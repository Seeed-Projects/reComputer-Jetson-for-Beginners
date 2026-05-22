# Chapter 5 Image Generation Guide

## Seeed Studio 风格说明

参考图风格特征：
- **Seeed 吉祥物机器人**：绿色+白色配色的可爱机器人角色，手持设备/工具
- **打开的门/窗口**：作为主视觉引导元素，象征"开启新世界"
- **Seeed Studio Logo** + **NVIDIA Logo** 在画面中
- **reComputer 设备实体**：展示实际硬件产品
- **城市/科技场景**：从门口望出去的未来科技城市场景
- **柔和的天空蓝背景**：整体明亮、温暖的色调
- **插画风格**：3D 渲染感，偏卡通但专业

### 通用风格 Prompt 模板

所有图片生成时，建议在 prompt 末尾添加以下风格后缀：

```
--style suffix to add at the end of every prompt:
"Cute green and white Seeed robot mascot character visible somewhere in the scene. Seeed Studio logo placement area in upper portion. reComputer Jetson device shown as hardware element. Bright, optimistic color palette with sky blue gradient background. 3D rendered illustration style, professional tech documentation quality, warm and inviting atmosphere. 1920x1080 resolution, widescreen format."
```

---

## 5.0 Main Chapter README

### Image 1: Chapter Cover - "Opening the LLM World with Seeed Studio"
**Filename:** `5-chapter-cover.png`  
**Location:** Top of main README.md  
**Purpose:** Visual overview of offline LLM deployment on Jetson

**Generation Prompt:**
```
A cute green and white Seeed robot mascot character standing in front of large open double doors, looking through them with excitement. Through the doors, a glowing futuristic AI cityscape is visible with floating holographic elements: a brain icon representing LLMs, chat bubbles, code symbols, and neural network patterns. On the ground near the doorway sits a reComputer Jetson device with green LED glow. Seeed Studio logo floating in the sky above. NVIDIA logo in the corner. Bright blue sky with soft clouds visible through the doors. Warm golden light streaming through. The text "Offline LLM World with Seeed Studio" arching above the door frame. 3D rendered illustration style, professional tech documentation quality, warm optimistic atmosphere, 1920x1080 widescreen format.
```

---

## 5.1 Introduction to LLMs - Transformer Architecture (新增4张)

### Image 5: Transformer Architecture Overview
**Filename:** `5-1-introduction-to-llms-05.png`  
**Location:** In "The Transformer Architecture" section - after intro paragraph  
**Purpose:** Visual overview of Transformer architecture

**Generation Prompt:**
```
A cute green and white robot mascot standing next to a large glowing holographic diagram of a Transformer neural network architecture. The diagram shows stacked blocks labeled "Self-Attention" and "Feed-Forward Network" connected by arrows, with input tokens on the left flowing through to output on the right. The robot points at the diagram with a laser pointer, looking like a teacher. A reComputer Jetson device sits at the base of the diagram powering it. Seeed Studio logo upper left. Clean blue gradient background with soft tech glow. 3D rendered illustration style, educational and professional, 1920x1080 format.
```

### Image 6: RNN vs Transformer Comparison
**Filename:** `5-1-introduction-to-llms-06.png`  
**Location:** In "Self-Attention: Words Paying Attention to Each Other" section  
**Purpose:** Visualize self-attention connecting words

**Generation Prompt:**
```
A cute green and white robot mascot holding a magnifying glass, examining floating words "The robot picked up the ball because it was heavy" in the air. Bright glowing connection lines between the word "it" and "ball" (thick line showing high attention), and thinner lines to other words. A small brain icon at the center of the connections. A reComputer Jetson device visible below. Seeed Studio logo upper left. Soft purple and blue gradient background with floating data particles. 3D rendered illustration style, educational and clear, 1920x1080 format.
```

### Image 7: Multi-Head Attention
**Filename:** `5-1-introduction-to-llms-07.png`  
**Location:** In "Multi-Head Attention" section  
**Purpose:** Show multiple attention heads processing differently

**Generation Prompt:**
```
A cute green and white robot mascot wearing multiple colorful visors/goggles (red, blue, green, orange), each representing a different attention head. Above the robot, four parallel panels show the same sentence "The cat sat on the mat" but with different words highlighted in different colors (grammatical, positional, semantic, reference). Each panel has a label: "Grammar", "Position", "Meaning", "Reference". A reComputer Jetson device connects all four panels. Seeed Studio logo upper left. Bright, colorful but organized layout, white background with subtle grid. 3D rendered illustration style, 1920x1080 format.
```

### Image 8: Encoder vs Decoder Architecture
**Filename:** `5-1-introduction-to-llms-08.png`  
**Location:** In "Encoder vs Decoder" section  
**Purpose:** Show difference between encoder and decoder architectures

**Generation Prompt:**
```
A cute green and white robot mascot standing between two large transparent pillars. Left pillar labeled "Encoder (Reader)" shows text flowing in from both sides, representing bidirectional processing. Right pillar labeled "Decoder (Writer)" shows text generating from left to right with a cursor. The robot holds a book in one hand (encoder/reading) and a pen in the other (decoder/writing). Between the pillars, a smaller combined pillar labeled "Both" with arrows. A reComputer Jetson device at the base. Seeed Studio logo upper left. Clean educational diagram style, blue and green color scheme, 1920x1080 format.
```

---

### Image 1: "What is an LLM?" - Robot Learning to Talk
**Filename:** `5-1-introduction-to-llms-01.png`  
**Location:** After "How LLMs Work" section  
**Purpose:** Visualize token prediction process

**Generation Prompt:**
```
A cute green and white Seeed robot mascot sitting at a desk, looking at a computer screen showing text "The capital of France is..." with a thinking bubble containing "Paris?". Above the robot, floating holographic icons show: books (training data), a brain (neural network), and speech bubbles with tokens like "The", "capital", "of", "France", "is". A reComputer Jetson device sits on the desk as the computing power. Seeed Studio logo in upper left. Soft blue gradient background with floating data particles. 3D rendered illustration style, warm and educational tone, 1920x1080 format.
```

### Image 2: Transformer "Attention" Concept
**Filename:** `5-1-introduction-to-llms-02.png`  
**Location:** In "The Transformer Architecture" section  
**Purpose:** Simplified transformer attention mechanism

**Generation Prompt:**
```
A cute green and white Seeed robot mascot holding a magnifying glass, examining floating text tokens connected by glowing attention lines. The words "The cat sat on the mat" float in the air with bright connection lines between related words (cat-sat, mat-on). A simplified neural network diagram glows in the background. The robot looks curious and engaged. reComputer Jetson device visible on a small stand. Seeed Studio logo upper left. Soft gradient background transitioning from blue to purple. 3D rendered illustration style, educational and approachable, 1920x1080 format.
```

### Image 3: Model Size Guide - "Which Model Fits Your Jetson?"
**Filename:** `5-1-introduction-to-llms-03.png`  
**Location:** In "Model Sizes and Local Deployment" section  
**Purpose:** Visual comparison of model parameter sizes

**Generation Prompt:**

```
A cute green and whiterobot mascot standing next to a row of four progressively larger glowing orbs labeled "1B", "3B", "7B", "70B". The robot is easily lifting the small 1B orb, hugging the 3B orb, standing on tiptoes for the 7B orb, and looking up at the massive 70B orb with a dizzy expression. Below each orb, a reComputer Jetson device with a green checkmark for 1B/3B/7B and a red X for 70B. A shelf-like display with warm lighting. Seeed Studio logo in upper right. Bright, friendly color palette with blue and green accents. 3D rendered illustration style, 1920x1080 format.
```

### Image 4: Framework Comparison - "Three Paths to LLM"
**Filename:** `5-1-introduction-tollms-04.png`  
**Location:** In "Inference Frameworks Overview" section  
**Purpose:** Compare Ollama, llama.cpp, and vLLM

**Generation Prompt:**
```
A cute green and white robot mascot standing at a crossroads with three paths leading forward, each labeled with a framework name. Left path: "Ollama" with a friendly llama icon, path is smooth and easy. Center path: "llama.cpp" with C++ code brackets icon, path has gears and tools (lightweight). Right path: "vLLM" with a rocket/speedometer icon, path has server racks (high performance). The robot is looking at a guidebook/map. reComputer Jetson device visible at the intersection. Seeed Studio logo in upper left. Each path has its own color theme (green, orange, blue). Sunny outdoor tech park setting. 3D rendered illustration style, 1920x1080 format.
```

---

## 5.2 Getting Started with Ollama

### Image 1: "Meet Ollama" - Your AI Pet Llama
**Filename:** `5-2-ollama-01.png`  
**Location:** Introduction section  
**Purpose:** Visual introduction to Ollama

**Generation Prompt:**
```
A cute green and white robot mascot sitting on the floor, petting a friendly cartoon llama character (representing Ollama). The llama has a glowing brain icon above its head and chat bubbles floating around it. A reComputer Jetson device sits between them with a warm glow. The robot and llama look happy together. A cozy tech workspace setting with bookshelves and plants in background. Seeed Studio logo in upper left. Soft warm lighting, inviting atmosphere. Text area: "Ollama - Your Local AI Companion". 3D rendered illustration style, 1920x1080 format.
```

### Image 2: Installation Journey
**Filename:** `5-2-ollama-04.png`  
**Location:** In "Installing Ollama" section  
**Purpose:** Visualize the installation steps

**Generation Prompt:**

```
A cute green and white robot mascot walking along a 4-step path/staircase. Step 1: Robot typing on a terminal screen (Download). Step 2: Robot watching a progress bar fill up (Install). Step 3: Robot giving thumbs up next to a green checkmark (Ready). Step 4: Robot chatting with a friendly llama character (Chat!). Each step is a glowing platform with numbered circles. reComputer Jetson device at the base of the stairs. Seeed Studio logo in upper left. Upward progression feeling with warm lighting. Pastel gradient background. 3D rendered illustration style, 1920x1080 format.
```

### Image 3: Chat with AI - The First Conversation
**Filename:** `5-2-ollama-02.png`  
**Location:** In "Open WebUI Profile" section  
**Purpose:** Show the chat interface concept

**Generation Prompt:**
```
A cute green and white robot mascot sitting at a desk, chatting on a large monitor displaying a ChatGPT-like interface. The screen shows a conversation between the robot and an AI. Floating holographic chat bubbles around the monitor with friendly messages. A reComputer Jetson device powering the monitor, glowing green. Cozy workspace setting with plants and warm lighting. Seeed Studio logo in upper right. NVIDIA logo subtle on the monitor bezel. The robot looks pleased with the conversation. 3D rendered illustration style, 1920x1080 format.
```

---

## 5.3 Running LLMs with llama.cpp

### Image 1: "Under the Hood" - llama.cpp Engine
**Filename:** `5-3-llama-cpp-01.png`  
**Location:** Introduction section  
**Purpose:** Show C++ implementation concept

**Generation Prompt:**
```
A cute green and white robot mascot with a mechanic's cap, opening the hood of a sleek car (representing llama.cpp). Inside the "engine", glowing C++ code symbols ({}, ;, #) are visible alongside neural network patterns. The robot holds a wrench with a lightning bolt (GPU acceleration). A reComputer Jetson device is visible as the car's dashboard. Tools and gears float around suggesting optimization. Seeed Studio logo upper left. Garage/workshop setting with warm lighting. Tech-meets-mechanic aesthetic. 3D rendered illustration style, 1920x1080 format.
```

### Image 2: GGUF Quantization - "Shrinking the Model"
**Filename:** `5-3-llama-cpp-02.png`  
**Location:** In "Understanding GGUF Quantization" section  
**Purpose:** Visualize quantization concept

**Generation Prompt:**

```
A cute green and white robot mascot standing between a giant heavy box labeled "FP32 - 28GB" (sweating, struggling) and a small compact box labeled "Q4 - 3.5GB" (easily carrying it). The robot is using a glowing "quantization machine" (a funnel/hopper device) in the center to transform the big box into the small box. Sparkles and compression effects around the funnel. A reComputer Jetson device sits nearby, happy to receive the small box. Seeed Studio logo upper left. Clean white background with blue accents. 3D rendered illustration style, 1920x1080 format.
```

### Image 3: Build Process - "Constructing Your LLM Engine"
**Filename:** `5-3-llama-cpp-03.png`  
**Location:** In "Clone and Build llama.cpp" section  
**Purpose:** Show compilation process

**Generation Prompt:**

```
A cute green and white robot mascot building/assembling a robot (representing llama.cpp build process). The robot is connecting pieces: a GitHub cloud piece, a terminal/code piece, a compiler gear piece, and a final running robot piece. Assembly line style with sparks of light at connection points. A reComputer Jetson device serves as the workbench. Progress indicators show completion at each stage. Seeed Studio logo upper left. Factory/workshop setting with warm industrial lighting. 3D rendered illustration style, 1920x1080 format.
```

### Image 4: GPU Acceleration - "Turbo Mode"
**Filename:** `5-3-llama-cpp-04.png`  
**Location:** In "GPU Offloading" section  
**Purpose:** Show CPU vs GPU layer distribution

**Generation Prompt:**
```
A cute green and white robot mascot driving two different vehicles side by side. Left: A slow bicycle labeled "CPU Only" with neural network layers stacked on a basket (looking tired). Right: A fast motorcycle with NVIDIA GPU logo, labeled "GPU Accelerated" with the same layers flying through a jet stream (looking excited and fast). A reComputer Jetson device with GPU chip highlighted glows in the background. Speed lines and motion blur on the motorcycle. Seeed Studio logo upper left. Racing track setting with tech elements. 3D rendered illustration style, 1920x1080 format.
```

---

## 5.4 High-Performance Inference with vLLM

### Image 1: vLLM - "The Traffic Controller"
**Filename:** `5-4-vllm-01.png`  
**Location:** Introduction section  
**Purpose:** Show PagedAttention and high-throughput serving

**Generation Prompt:**
```
A cute green and white robot mascot wearing an air traffic controller outfit, standing in a control tower. Below, multiple request airplanes (labeled with user icons) are being efficiently guided to landing pads (memory pages) using a smart scheduling system. The robot uses multiple screens showing "PagedAttention" memory management with organized blocks. A reComputer Jetson device powers the control tower. Seeed Studio logo upper left. Airport/tech hub setting with blue sky. NVIDIA logo on the control tower. 3D rendered illustration style, 1920x1080 format.
```

### Image 2: Server Setup - "Your AI Server Room"
**Filename:** `5-4-vllm-02.png`  
**Location:** In "Running a Model with vLLM" section  
**Purpose:** Show server-client architecture

**Generation Prompt:**
```
A cute green and white robot mascot standing proudly next to a server rack (representing vLLM) with a "vLLM Server" label and glowing green status lights. Multiple smaller devices (laptop, phone, tablet) are connected to the server by glowing data streams. The robot is operating a control panel. A reComputer Jetson device sits at the base of the rack as the server hardware. API connection icons float between devices. Seeed Studio logo upper right. Clean server room setting with cool blue lighting. 3D rendered illustration style, 1920x1080 format.
```

### Image 3: Framework Speed Race
**Filename:** `5-4-vllm-03.png`  
**Location:** In "Comparison: vLLM vs Other Frameworks" section  
**Purpose:** Visual performance comparison

**Generation Prompt:**

```
A cute green and white robot mascot at a finish line, watching three race cars zoom by on a track. Car 1 (blue, "Ollama") is steady and reliable. Car 2 (orange, "llama.cpp") is light and nimble. Car 3 (green, "vLLM") is a high-speed racer pulling ahead with multiple passenger seats (multi-user). Speedometer readings show different speeds. A reComputer Jetson device is at the finish line. Seeed Studio logo upper left. Racing stadium setting with cheering crowd silhouettes. 3D rendered illustration style, 1920x1080 format.
```

---

## 5.5 Jetson Examples Quick Start

### Image 1: "One Command to Deploy" - Magic Deployment
**Filename:** `5-5-jetson-examples-01.png`  
**Location:** Introduction section  
**Purpose:** Show one-command deployment concept

**Generation Prompt:**

```
A cute green and white robot magician character, casting a spell with a magic wand labeled "reComputer run". From the wand, multiple AI model boxes (LLaVA, Ollama, Whisper, Llama3) are materializing from sparkles around a reComputer Jetson device. The robot wears a magician hat. A terminal screen shows the single command. Seeed Studio logo in the magical light above. Magical particle effects and warm golden lighting. Fantasy-meets-tech aesthetic. 3D rendered illustration style, 1920x1080 format.
```

### Image 2: Docker Container World
**Filename:** `5-5-jetson-examples-02.png`  
**Location:** In "What is jetson-examples?" section  
**Purpose:** Show containerized architecture

**Generation Prompt:**

```
A cute green and white robot mascot standing next to a stack of colorful transparent containers (like glass boxes) on top of a reComputer Jetson device. Each container holds a different AI model represented by an icon: a brain (LLM), an eye (vision), a microphone (ASR), a speaker (TTS). The robot is placing a new container on top. Docker whale logo visible on the Jetson device. Each container glows in its own color. Seeed Studio logo upper left. Clean isometric view, bright lab setting. 3D rendered illustration style, 1920x1080 format.
```

### Image 3: Model Marketplace
**Filename:** `5-5-jetson-examples-03.png`  
**Location:** In "Available LLM Examples" section  
**Purpose:** Visual menu of available examples

**Generation Prompt:**
```
A cute green and white robot mascot browsing a glowing holographic marketplace/store. Shelves display different AI model "products" as glowing orbs: Llama3, Gemma4, Qwen, Whisper, LLaVA, Parler-TTS. Each orb has a label and size indicator. The robot holds a shopping cart. A reComputer Jetson device powers the marketplace display. "jetson-examples" storefront sign above. Seeed Studio logo upper left. Warm marketplace lighting with neon tech accents. 3D rendered illustration style, 1920x1080 format.
```

### Image 4: Quick Start Workflow
**Filename:** `5-5-jetson-examples-04.png`  
**Location:** In "Building a Chatbot" section  
**Purpose:** Step-by-step deployment workflow

**Generation Prompt:**
```
A cute green and white robot mascot on a 4-step adventure path. Step 1: Robot installing a package (pip icon). Step 2: Robot typing a command on terminal (rocket launching). Step 3: Robot receiving a model download (cloud raining data). Step 4: Robot chatting happily with an AI on screen (speech bubbles). Each step is an illustrated station with warm lighting. reComputer Jetson device visible throughout as companion. Seeed Studio logo upper left. Progressive color gradient from cool to warm. 3D rendered illustration style, 1920x1080 format.
```

---

## 5.6 ASR + LLM + TTS Pipeline

### Image 1: Voice Assistant Overview - "Talk to Your Jetson"
**Filename:** `5-6-asr-llm-tts-01.png`  
**Location:** Introduction section  
**Purpose:** Complete pipeline visualization

**Generation Prompt:**
```
A cute green and white robot mascot wearing headphones, speaking into a microphone on the left side. Speech bubbles flow through a glowing pipeline: first through an ear icon (Whisper ASR), then through a brain icon (LLM), then through a speaker icon (TTS). The robot on the right side hears the response through speakers. A reComputer Jetson device sits in the center powering the whole pipeline with green energy beams. Seeed Studio logo upper left. Flowing data particles along the pipeline. Warm blue and purple gradient background. 3D rendered illustration style, 1920x1080 format.
```

### Image 2: Pipeline Components Deep Dive
**Filename:** `5-6-asr-llm-tts-02.png`  
**Location:** In "Pipeline Architecture" section  
**Purpose:** Technical architecture with data flow

**Generation Prompt:**
```
A cute green and white robot mascot presenting three large glowing stages on a stage/platform. Left stage: Ear/listening icon labeled "Whisper" with audio waveform flowing in. Center stage: Brain icon labeled "LLM" with text flowing through. Right stage: Speaker icon labeled "TTS" with sound waves coming out. Conveyor belt connects all three stages. A reComputer Jetson device sits backstage powering everything. Seeed Studio logo upper left. Presentation/stage lighting with spotlights on each component. Data flow arrows connecting stages. 3D rendered illustration style, 1920x1080 format.
```

### Image 3: Speech Recognition Visualized
**Filename:** `5-6-asr-llm-tts-03.png`  
**Location:** In "ASR: Whisper" section  
**Purpose:** Show speech recognition process

**Generation Prompt:**
```
A cute green and white robot mascot cupping its ear, listening to sound waves coming from a person's speech bubble. Inside the robot's head, a magical transformation happens: sound waves are being converted into glowing text characters. The word "Hello" emerges from the process. A microphone sends audio to the robot. A reComputer Jetson device nearby shows the Whisper model loading. Seeed Studio logo upper left. Soft blue and purple gradient background with floating sound/text particles. 3D rendered illustration style, 1920x1080 format.
```

### Image 4: LLM Response Generation
**Filename:** `5-6-asr-llm-tts-04.png`  
**Location:** In "LLM Generation" section  
**Purpose:** Show language model thinking and responding

**Generation Prompt:**
```
A cute green and white robot mascot in a thinking pose (hand on chin), with a thought bubble showing text being processed. Inside the thought bubble: neural network connections, gears turning, and text generating word by word. The robot sits on a reComputer Jetson device like a thinking throne. Multiple books and knowledge icons float around. "Processing..." indicator glows. Seeed Studio logo upper left. Warm golden thinking/contemplation lighting. Brain glow effect around the robot's head. 3D rendered illustration style, 1920x1080 format.
```

### Image 5: Text-to-Speech Output
**Filename:** `5-6-asr-llm-tts-05.png`  
**Location:** In "TTS: Text-to-Speech" section  
**Purpose:** Show speech synthesis process

**Generation Prompt:**
```
A cute green and white robot mascot speaking enthusiastically into a megaphone/speaker. Text characters ("Hello! I'm your AI assistant!") flow into one side of a magical speaker device, and beautiful musical/sound waves come out the other side. Sound visualization bars dance to the speech rhythm. A reComputer Jetson device powers the speaker. Musical notes and sound particles float in the air. Seeed Studio logo upper left. Warm, lively stage lighting with golden and blue accents. 3D rendered illustration style, 1920x1080 format.
```

### Image 6: Complete Voice Assistant in Action
**Filename:** `5-6-asr-llm-tts-06.png`  
**Location:** In "Practice Exercise" section  
**Purpose:** Final comprehensive scene

**Generation Prompt:**
```
A cute green and white robot mascot and a human character having a natural conversation in a cozy tech living room. The human speaks into a USB microphone, and the robot responds through speakers. A holographic display shows the full pipeline: microphone → ASR → LLM → TTS → speaker. A reComputer Jetson device sits on the desk, glowing warmly. Both characters look engaged and happy. A complete voice assistant setup visible: mic, speakers, Jetson device, monitor. Seeed Studio logo upper right. Warm, comfortable home/office lighting. NVIDIA logo on the Jetson. 3D rendered illustration style, 1920x1080 format.
```

---

## Image Generation Tips

### Recommended AI Image Generators:
1. **Midjourney v6** - Best for Seeed-style 3D rendered illustrations
2. **DALL-E 3** - Good for consistent character designs
3. **Stable Diffusion XL + ControlNet** - Good for batch consistency

### Midjourney Specific Settings:
```
--ar 16:9          (widescreen)
--style raw        (cleaner, less artistic distortion)
--stylize 250-400  (balanced style)
--q 2              (higher quality)
```

### Style Consistency Tips:
- Always include "Seeed robot mascot" in every prompt
- Keep color palette consistent: green (#76B900), white, sky blue, warm gold
- Always include reComputer Jetson device as hardware element
- Use "3D rendered illustration style" consistently
- Include Seeed Studio logo placement area

### Resolution Requirements:
- **1920x1080** (16:9 widescreen) - Primary format
- **1200x800** - Minimum acceptable
- Format: **PNG** with transparency when applicable
- Max file size: **500KB** for web

### Naming Convention:
```
5-{module}-{descriptive-name}-{sequence}.png

Examples:
- 5-chapter-cover.png
- 5-1-introduction-to-llms-01.png
- 5-2-ollama-01.png
- 5-3-llama-cpp-02.png
- 5-6-asr-llm-tts-03.png
```

---

## Screenshot Requirements (NOT AI Generated)

These images should be actual screenshots:

1. **5-2-ollama-03.png** - Terminal showing ollama run command
2. **5-3-llama-cpp-05.png** - llama.cpp compilation output
3. **5-4-vllm-04.png** - vLLM API response in terminal
4. **5-5-jetson-examples-05.png** - Docker container list
5. **5-6-asr-llm-tts-07.png** - Python script running voice assistant

---

## Total Image Count

| Module | AI Generated | Screenshots | Total |
|--------|-------------|-------------|-------|
| 5.0 Main | 1 | 0 | 1 |
| 5.1 LLM Intro (+ Transformer) | 8 (+4 new) | 0 | 8 |
| 5.2 Ollama | 3 | 1 | 4 |
| 5.3 llama.cpp | 4 | 1 | 5 |
| 5.4 vLLM | 3 | 1 | 4 |
| 5.5 Jetson Examples | 4 | 1 | 5 |
| 5.6 ASR+LLM+TTS | 6 | 1 | 7 |
| **TOTAL** | **29 (+4 new)** | **5** | **34** |
| 5.0 Main | 1 | 0 | 1 |
| 5.1 LLM Intro | 4 | 0 | 4 |
| 5.2 Ollama | 3 | 1 | 4 |
| 5.3 llama.cpp | 4 | 1 | 5 |
| 5.4 vLLM | 3 | 1 | 4 |
| 5.5 Jetson Examples | 4 | 1 | 5 |
| 5.6 ASR+LLM+TTS | 6 | 1 | 7 |
| **TOTAL** | **25** | **5** | **30** |

---

*Generate images in order of importance: 5.1 → 5.6 → 5.2 → 5.3 → 5.4 → 5.5 → 5.0*
