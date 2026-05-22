# Offline Vision-Language Models and Applications

## 03 Large offline visual multimodal models and applications

| Name | Owner | Modified | Created |
| --- | --- | --- | --- |
| 11.03-01 Aliyun: Qwen2.5vl Visual Multimodal Model | Yujiang! | 2026-01-09 14:53 | 2026-01-09 14:52 |
| 11.03-02 Google: Gemma3 Visual Multimodal Model | Yujiang! | 2026-01-09 14:53 | 2026-01-09 14:53 |
| 11.03-03 Llava Visual Multimodal Model | Yujiang! | 2026-01-09 14:53 | 2026-01-09 14:53 |
| 11.03-04 MiniCPM-V Visual Multimodal Model | Yujiang! | 2026-01-09 14:54 | 2026-01-09 14:53 |
| 11.03-05 Multimodal Visual Understanding Application | Yujiang! | 2026-01-16:53 | 2026-01-09 14:54 |
| 11.03-06 Multimodal Textile Graphic Applications | Yujiang! | 2026-01-14 16:37 | 2026-01-09 14:55 |
| 11.03-07 Multimodal Video Analysis Application | Yujiang! | 2026-01-14 16:44 | 2026-01-09 14:55 |
| 11.03-08 Multimodal Visual Positioning Application | Yujiang! | 2026-01-14:48 | 2026-01-09 14:56 |
| 11.03-09 Multimodal Table Scan Application | Yujiang! | 2026-01-16:05 | 2026-01-09 14:56 |
| 11.03-10 Multimodal Auto-Agent Applications | Yujiang! | 2026-01-14:49 | 2026-01-09 14:56 |

### 11.03-01 Aliyun: Qwen2.5vl Visual Multimodal Model

## Introduction

Qwen 2.5 VL is an advanced visual-language multi-modular model launched by the Qwen team, which not only inherits the high performance of Qwen 2.5 in terms of language understanding and generation, but also adds a strong image and video understanding capability, capable of processing text, picture and even long video input at the same time, with visual questions and answers, image analysis, structured data extraction and event positioning tasks. Compared to the previous generation, Qwen 2.5 VL is significantly elevated in visual positioning, long video understanding and cross-modular reasoning, supporting the generation of structured outputs with coordinates (e.g. JSON box selection information), the analysis of tables/scanners, and can also act as a visual proxy execution tool, one of the flagships in the current open-source visual language model.

![](./images/5-3-offline-vision-language-models-and-applications-01.gif)

#### Model size

| Variant | Notes |
| --- | --- |
| Qwen2.5-VL 3B | Smallest practical local multimodal deployment |
| Qwen2.5-VL 7B | Balanced local quality and resource usage |
| Qwen2.5-VL 32B | Higher quality for workstation or server deployment |
| Qwen2.5-VL 72B | Largest flagship variant for server-scale inference |

### Performance

## Use Qwen2.5VL

Run a model using a run-off command, and if not downloaded locally, olama will download the model before running

![](./images/5-3-offline-vision-language-models-and-applications-05.png)

```bash
ollama run qwen2.5vl:3b
```

## Dialogue Test

![](./images/5-3-offline-vision-language-models-and-applications-06.png)

```bash
who are you?
```

![](./images/5-3-offline-vision-language-models-and-applications-07.png)

## Visual

![](./images/5-3-offline-vision-language-models-and-applications-08.png)

```bash
What do you see in this picture? :./test.png
# Use ": + image_path" in the conversation to let the model analyse the image with its vision capability
```

## Closure of the dialogue

Use the Ctrl+d shortcut or XIAITOKEN0 to end the conversation!

## References

> Ollama.

Official: https://ollama.com/

GitHub: https://github.com/ollama/ollama

> Qwen2.5VL

GitHub: https://github.com/QwenLM/Qwen2.5-VL

Ollama Correlation Model: https://ollama.com/library/qwen2.5vl

### 11.03-02 Google: Gemma3 Visual Multimodal Model

## Introduction

Gemma 3 is the latest open-source series of visual multimodular models for Google, belonging to the Gemma Model Family, providing versions of about 1B to 27B of various sizes, supporting the simultaneous understanding of text and image input (multimodular) and processing context information up to 128 K token, covering a wide range of tasks such as questions and answers, summaries, reasoning, etc. Compared to previous generations, it has significantly increased in visual understanding, multilingualism (140+ languages) and context processing, while being designed more efficiently, easily deployable and able to operate on equipment with limited resources, it is one of the larger models of a strong visual-linguistic understanding in the current open-source ecology.

![](./images/5-3-offline-vision-language-models-and-applications-09.gif)

#### Model size

| Variant | Notes |
| --- | --- |
| Gemma 3 1B | Lightweight baseline |
| Gemma 3 4B | Practical local multimodal deployment |
| Gemma 3 12B | Higher quality with more memory required |
| Gemma 3 27B | Largest open model in the Gemma 3 multimodal family |

### Performance

## Use Gemma3

Run a model using a run-off command, and if not downloaded locally, olama will download the model before running

![](./images/5-3-offline-vision-language-models-and-applications-11.png)

```bash
ollama run gemma3:4b
```

## Dialogue Test

![](./images/5-3-offline-vision-language-models-and-applications-12.png)

```bash
who are you?
```

![](./images/5-3-offline-vision-language-models-and-applications-13.png)

## Visual

![](./images/5-3-offline-vision-language-models-and-applications-14.png)

```bash
What do you see in this picture? :./test.png
# Use ": + image_path" in the conversation to let the model analyse the image with its vision capability
```

## Closure of the dialogue

Use the Ctrl+d shortcut or XIAITOKEN0 to end the conversation!

## References

> Ollama.

Official: https://ollama.com/

GitHub: https://github.com/ollama/ollama

> Gemma3

Ollama Correlation Model: https://ollama.com/library/gemma3

### 11.03-03 Llava Visual Multimodal Model

## Introduction

LLAVA, full name Large Language and Vision Asistant, is an open-source visual-language multi-modular model that achieves both image and text understanding by combining a pre-trained visual encoder with a powerful large language model. LLAVA is able to accept image + text input, perform visual questions and answers, image descriptions, scenario understanding, and output high-quality answers in natural languages, and enhance the effect of the model on the understanding and reasoning of the visual scene through " visual improvement " .

![](./images/5-3-offline-vision-language-models-and-applications-15.gif)

#### Model size

| Variant | Notes |
| --- | --- |
| LLaVA 7B | Common local-entry multimodal model |
| LLaVA 13B | Better quality with higher GPU memory demand |
| LLaVA 34B | Large server-oriented deployment |

### Performance

## Use Llava

Run a model using a run-off command, and if not downloaded locally, olama will download the model before running

![](./images/5-3-offline-vision-language-models-and-applications-18.png)

```bash
ollama run llava:7b
```

## Dialogue Test

![](./images/5-3-offline-vision-language-models-and-applications-19.png)

```bash
who are you?
```

![](./images/5-3-offline-vision-language-models-and-applications-20.png)

## Visual

![](./images/5-3-offline-vision-language-models-and-applications-21.png)

```bash
What do you see in this picture? :./test.png
# Use ": + image_path" in the conversation to let the model analyse the image with its vision capability
```

## Closure of the dialogue

Use the Ctrl+d shortcut or XIAITOKEN0 to end the conversation!

## References

> Ollama.

Official: https://ollama.com/

GitHub: https://github.com/ollama/ollama

> Llava.

Ollama Correlation Model: https://ollama.com/library/llava

### 11.03-04 MiniCPM-V Visual Multimodal Model

## Introduction

MiniCPM-V is an efficient visual-linguistic, multi-modular model series developed by teams such as OpenBMB/Csinghua, with approximately 8B level of parameters, but strong performance in tasks such as visual understanding, image and video analysis, even exceeding large models such as GPT-4V, Gemini Pro on multiple public benchmarks. It uses efficient visual coding and compression strategies to handle high-resolution images, strong OCR recognition, multilingual interaction (30 + languages) and dynamic video content, while optimizing low hallucinations and end-to-end deployment efficiency, supporting the operation of peripheral equipment such as mobile phones, is one of the current visual multiple-modular models with performance and utility in open-source communities.

![](./images/5-3-offline-vision-language-models-and-applications-22.gif)

#### Model size

| Variant | Notes |
| --- | --- |
| MiniCPM-V 8B | Main local deployment variant highlighted in this chapter |

### Performance

## Use MiniCPM-V

Run a model using a run-off command, and if not downloaded locally, olama will download the model before running

![](./images/5-3-offline-vision-language-models-and-applications-24.png)

```bash
ollama run minicpm-v:8b
```

## Dialogue Test

![](./images/5-3-offline-vision-language-models-and-applications-25.png)

```bash
who are you?
```

![](./images/5-3-offline-vision-language-models-and-applications-26.png)

## Visual

![](./images/5-3-offline-vision-language-models-and-applications-27.png)

```bash
What do you see in this picture? :./test.png
# Use ": + image_path" in the conversation to let the model analyse the image with its vision capability
```

## Closure of the dialogue

Use the Ctrl+d shortcut or XIAITOKEN0 to end the conversation!

## References

> Ollama.

Official: https://ollama.com/

GitHub: https://github.com/ollama/ollama

> MiniCPM-V

GitHub: https://github.com/OpenBMB/MiniCPM-o

Ollama Correlation Model: https://ollama.com/library/minicpm-v

### 11.03-05 Multimodal Visual Understanding Application

### Concept introduction

### What is visual understanding?

Visual understanding means giving computers the same ability as humans to understand images or video content, not only to identify objects and scenes that appear in the images, but also to further understand the relationship between those objects, their state and the behaviour or events that are taking place. It is concerned with semantics and logic behind visual information, not just simple classification or testing. By combining visual information with language information, models can describe images, answer questions, reason and even support decision-making, so visual understanding has become one of the key technical capabilities in areas such as large multimodular models, autopilot, smart surveillance and human interaction.

### Principle of realization

The achievement of visual understanding depends mainly on the processing of visual and linguistic input into large models, the process of which can be divided into the following steps:

Image Encoding: Conversion of input images into digital vectors through visual encoders, which include characteristics such as colour, shape, texture, etc. of the images, which are easily understood by the computer.

Text encoding: user questions or descriptions (e.g. " What is the current scene? ) is also converted to text vectors to match image information.

Cross-modular integration: In the Attention Layer, the model integrates the image vector with the text vector to enable the model to focus on the desktop area according to the most relevant area of the problem-based "concern" image, such as the reference to the "desk" in questions.

Generate an answer: The integration information is transmitted to the Large Language Model (LLM), based on which descriptive text is generated or questions answered.

#### Code Parsing

### Key Code

### Tool Layer Entry (largemodel/utils/tools_manager.py)

The seewhat function in this document defines the process of executing the tool.

```bash
#From largemodel/utils/tools_manager.py
class ToolsManager:
  # ...

  def seewhat(self):
  """
  Capture camera frame and analyze environment with AI model.
  Capture a camera frame and analyse the environment with an AI model.

  :return: Dictionary with scene description and image path, or None if failed.
  """
  self.node.get_logger().info("Executing seewhat() tool")
  image_path = self.capture_frame()
  if image_path:
  # Use isolated context for image analysis.
  analysis_text = self._get_actual_scene_description(image_path)

  # Return structured data for the tool chain.
  return {
  "description": analysis_text,
  "image_path": image_path
  }
  else:
  # ... (Error handling)
  return None

  def _get_actual_scene_description(self, image_path, message_context=None):
  """
  Get AI-generated scene description for captured image.
  Get an AI-generated scene description for the captured image.

  :param image_path: Path to captured image file.
  :return: Plain text description of scene.
  """
  try:
  # ... (Build the prompt)
  result = self.node.model_client.infer_with_image(image_path, scene_prompt, message=simple_context)
  # ... (Process the result)
  return description
  except Exception as e:
  # ...
```

### Model interface layer (largemodel/utils/large_model_interface.py)

The infer with image function in this file is the unified access point for all images to understand the task, and it is to be performed using specific models according to configuration.

```bash
#From largemodel/utils/large_model_interface.py
class model_interface:
  # ...
  def infer_with_image(self, image_path, text=None, message=None):
  """Unified image inference interface.
  # ... (prepare messages)
  try:
  # choose the concrete implementation based on `self.llm_platform`
  if self.llm_platform == 'ollama':
  response_content = self.ollama_infer(self.messages, image_path=image_path)
  elif self.llm_platform == 'tongyi':
  # ... logic for calling the Tongyi model
  pass
  # ... (logic for other platforms)
  # ...
  return {'response': response_content, 'messages': self.messages.copy()}
```

### Code Parsing

The functionality was achieved using a stratification architecture, consisting mainly of two components: the tool layer and the model interface. Clear and mutually deconstructed responsibilities are the core foundation for the platform ' s interoperability and scalability.

Tool Layer (tools_manager.py):

The tool layer is responsible for carrying business logic, in which seewhat functions are at the core of the whole visual understanding process.

The act of "visual understanding" is completely sealed. Its execution process first captures current image data via Capture frame;

get actual scene description to generate Prompt to guide large-linguistic models for image analysis;

Upon completion of the above preparatory work, Seewhat reasoned the image data along with the analytical instructions to the model by calling on the harmonized method provided by the model interface layer;

It is important to emphasize that Seewhat does not care which model or platform is specifically used at the bottom, but only relies on a stable interface to make the call;

Ultimately, the tool layer collates and encapsulates the results of the text analysis returned by the model into a structured dictionary for direct use by the upper application.

In this way, the tool layer focuses on the business process itself without being disturbed by the details of the model ' s realization.

Model interface layer (large_model_interface.py):

The model interface layer is responsible for model adaptation and movement, and its core function is infer with image.

Infer with image is equivalent to a single entry or dispatch centre, which will be achieved by dynamic selection of the corresponding reasoning based on the current platform configuration item self.llm platform;

Parameter formats, data coding methods and API call logic required for different model platforms (e.g. Ollama, Thongyi infer) are encapsulated in their respective independent reasoning functions (e.g. ollama infer, toongyi infer);

This way of encapsulating the platform-related differences is confined to the model interface layer, which is fully transparent to the upper layer.

As a result, the tool layer code is free to switch between the back end of different large models, without any modifications, thereby significantly increasing the portability and expansion of the system.

The implementation process of the Seewhat tool reflects a typical design model for segregation of duties:

ToolsManager defines "do what" — obtains images and requests analysis;

Model Interface decides how to do it - selects the appropriate model platform according to configuration and completes the actual interaction.

This structure allows the core business logic to be fully aligned in an online or offline mode, with the need to switch the configuration of the model to fit different operating environments and greatly enhances the interoperability and reuse of tutorials and codes.

## Configure Large Offline Model

### Configure LLM platform (seeed.yaml)

This document determines which large model platform to load at the model service node as its main language model.

Open file in terminal:

![](./images/5-3-offline-vision-language-models-and-applications-28.png)

```bash
Code Block
vim /opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/config/seeed.yaml
```

Modify/confirm llm platform:

```bash
model_service:  #model server node parameters
  ros__parameters:
  language: 'zh'  #LLM interface language
  useolinetts: True  #Not used in text-only mode; can be ignored

  # LLM configuration
  llm_platform: 'ollama'  # Key: make sure this is set to 'ollama'
  regional_setting : "China"
```

#### Configure Model Interface (large_model_interface.yaml)

This document defines which visual model is used when the platform is selected as olama.

Open file in terminal

```bash
# vim /opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/config/large_model_interface.yaml
```

Find the configuration of olama

```bash
#.....
#Offline Large Language Models
#Ollama configuration
ollama_host: "http://127.0.0.1:11434"  # Ollama server address
ollama_model: "qwen2.5vl:3b"  # Key: change this to a multimodal model you have already downloaded
#.....
```

> Note: Make sure that the model specified in the configuration parameters (e.g. qwen2.5vl) handles multi-modular input.

#### Start and test functionality (text input mode)

Connect to the USB camera and start the largemodel master (text mode): Open a terminal and then run the following command:

```bash
cd /opt/seeed/development_guide/12_llm_offline/seeed_ws
source install/setup.bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

> If an error is shown that the version of a numpy does not match, you can update the version with the pip install numpy==2.0.0 command.

Send text instructions: open another terminal again and run the following instructions,

```bash
cd /opt/seeed/development_guide/12_llm_offline/seeed_ws
source install/setup.bash
ros2 run text_chat text_chat
```

And then you start entering text: What do you see?

Observation: In the first terminal where the main program is run, you will see log output, show the system receiving text commands and print text descriptions of captured images from the qwen2.5vl model.

## Common problems and solutions

### Question 1: The log shows that "Failed to call ollama vision mode" or connection is denied.

Solutions:

Check the Ollama service: Run ollama list at the terminal, make sure qwen2.5vl (or your configured model) is downloaded and Ollama service is running.

Check the configuration file: Check carefully whether the configurations in seeed.yaml and large_model_interface.yaml are correct, especially the values llm platform and ollama model.

### Question 2: Seewhat tools returned "can't open the camera" or failed to photograph.

Solutions:

Checking device connections: Runs lls XIAOBAITOKEN* to ensure that the system detects your USB camera.

Question of privileges: Attempts to use a test camera such as cheese or guvcview, if sudo can but can not be an ordinary user, may be a question of dev rules or user group privileges.

# 11.03-06 Multimodal Textile Graphic Applications

## Concept introduction

![](./images/5-3-offline-vision-language-models-and-applications-29.png)

### What is Text-to-Image?

![](./images/5-3-offline-vision-language-models-and-applications-30.png)

Text-to-Image is an artificial intelligence generation technique that refers to the automatic generation of images that match the semantics of the text according to the natural language description entered by the user. It understands not only what is in the picture, but also style, scene, emotion and detail requirements, such as the "blue cat in the grass, cartoon style, soft light". It is usually based on large-scale visual-linguistic models and diffusion models, which translate abstract words into concrete, visualized images by learning the correspondence of big graphics, and have been widely applied in areas such as artistic creation, content generation, product design and education.

### Core principles

Text encoding: Converts a text description to a vector, captures semantic information.

Submarine means that images are mapd into low-dimensional spaces and easily generated.

Conditional generation: Image generation based on text vector using proliferation models or PAN.

![](./images/5-3-offline-vision-language-models-and-applications-31.png)

Multi-modular alignment: ensure that the image content and text semantics are consistent (common CLIP).

Sampling and denocation: gradually generating clear images while following the semantics of the text.

![](./images/5-3-offline-vision-language-models-and-applications-32.png)

Post-processing: Increase image quality or adjust style.

> The olama framework does not support the functions of the graphics, and this chapter provides us with other tools to achieve the functions of the local drawings.

### What's FastSDCPU?

FastSD CPU is a lightweight Stable Diffusion reasoning framework running on CPU, designed to generate high-quality images without GPU. It achieves the rapid generation of text to image (Text-to-Image) by optimizing model loading, reasoning processes and multi-line calculations, while supporting extended modules such as LoRA, ControlNet. FastSD CPU is particularly suited to environments with limited hardware resources, such as ordinary PCs or embedded devices, allowing more users to experience AI image generation without high performance graphic cards.

![](./images/5-3-offline-vision-language-models-and-applications-33.png)

### Core characteristics

CPU Optimizing reasoning: Designed exclusively for GPU-free environments, making full use of CPU multi-line and quantitative calculations, and increasing the speed of reasoning.

![](./images/5-3-offline-vision-language-models-and-applications-34.png)

Light Quantification and Quick Start: Models and relying on optimized, fast-starting, low-resource, hardware-limited equipment.

![](./images/5-3-offline-vision-language-models-and-applications-35.png)

Text to image (Text-to-Image) supports: high-quality images can be generated according to natural language descriptions, compatible with Stable Diffusion standard lines.

![](./images/5-3-offline-vision-language-models-and-applications-36.png)

Extension function support: Supports extensions such as the LoRA fine-tuning model, ControlNet condition control, etc., with flexibility to enhance the generation of effects.

![](./images/5-3-offline-vision-language-models-and-applications-37.png)

Multi-wire and batch processing: multiple images can be generated at the same time, increasing overall throughput capacity on CPU.

![](./images/5-3-offline-vision-language-models-and-applications-38.png)

Offline and light model compatibility: Supporting offline model and light quantitative model (e.g. GGF format) without frequent network downloads to improve safety and stability.

Wide scope of application: Fits to a common PC, embedded device, or an ARM platform such as Jetson, and can experience AI image generation without a high performance graphic card.

### Apply scene

Images can be generated quickly for conceptual validation, product design or creative sketches.

Display AI image generation techniques in classroom or laboratory environments to reduce hardware costs.

Support local models that are suitable for scenarios that limit data privacy or the network environment.

Common CPU computer, notebook or embedded device (e. g. Jetson) AI creation tool

## Project deployment

### Deployment environment

> N.B. If we use our off-site mirrors without the need to deploy the environment, we can skip the deployment steps. Take a direct look at the bottom of the list.

Open a terminal and execute the following code:

```bash
# If Git is not installed yet, run this first
sudo apt update
sudo apt install git -y
sudo apt install python3.10-venv -y

# Add environment variables
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Cloning project

```bash
git clone https://github.com/rupeshs/fastsdcpu.git
cd fastsdcpu
```

Create virtual environments and install dependency

```bash
python -m venv venv
source venv/bin/activate
#Installuv
curl -Ls https://astral.sh/uv/install.sh | sh
```

> This step may not be successful if there is no hanging agent in China, and if this step is not exceeded, the following orders will be executed:

```bash
wget https://mirrors.huaweicloud.com/astral/uv/0.8.4/uv-aarch64-unknown-linux-gnu -O ~/.local/bin/uv
chmod +x ~/.local/bin/uv
```

Install Environment

```bash
chmod +x install.sh start-webui.sh
#Install
./install.sh --disable-gui
```

Installed successfully, exit by any key:

#### LAN access

Prior to startup, a document needs to be modified to support LAN access, otherwise webui can only be accessed locally:

```bash
# vim /opt/seeed/development_guide/12_llm_offline/fastsdcpu/src/frontend/webui/ui.py
```

After opening the ui.py file, turn to the last line and find the code webui.launch (share=share) and change it to webui.launch (server name= "0.0.0.0", share=share)

And save it.

Start:

```bash
# ./start-webui.sh
```

Then you can enter your master board IP:7860 on the browser to access this webui.

### Use Vincent function

Use ifconfig command to view the IP of Jetson at the end, for example, mine is 192.168.137.47.

Then we turn on the browser and enter your master panel ip:7860. For example, I will enter 192.168.137.47:7860 and then get into webui.

And then we click on LCM-LoRA, and this model is relatively small on memory, and if you want to use another model, you can study it yourself.

And then you click on Models, you can see the LCM LoRA model set, you can change the model you want, you can choose the default as I do.

Then click on General Settings, and pull up the Inference Steps, which improves the quality of the images produced.

And then you go back to our Text to Image, and you enter what we want to generate in the dialogue box, and then press Generate, and you can start generating pictures.

When first used, the model needs to be downloaded, and it can be seen at the terminal that the model that was selected by default is being downloaded, and once it is downloaded, it will begin to function as a graphic.

Result generated:

> Note: The English hint supports better, and the resulting pictures are more amenable to description. It is suggested that the English description be used to generate pictures.

### Method for start-up after successful deployment

```bash
cd fastsdcpu #enter the `fastsdcpu` directory
source venv/bin/activate #enter the virtual environment
./start-webui.sh #start the WebUI
```

When webui starts successfully, enter your master plate IP:7860 on the browser, so you can start the vernacular function.

# 11.03-07 Multimodal Video Analysis Application

## Concept introduction

### What's video analysis?

Video analysis refers to the automatic understanding and processing of image sequences in the video stream using computer visual and artificial intelligence techniques. By analysing each frame in the video and its time relationship, it achieves the detection, identification, tracking, and understanding of the behaviour and events of the target, thus translating raw video data into interpretable and decision-making information, and is widely used in security surveillance, intelligent transport, industrial testing and intelligent interaction.

### Brief description of the rationale for realization

The core of offline video analysis is the efficient processing of a large number of frames, while maintaining key content and time-series information for the video, the process of achieving which can be divided into the following steps:

Key frame extraction: The system does not process video on a frame-by-frame basis, but extracts the most representative key frame from a scenario change test or from a fixed time interval, thus significantly reducing the amount of data to be processed.

![](./images/5-3-offline-vision-language-models-and-applications-39.png)

Image Encoding: Each frame key frame is sent into the visual encoder and converted to a digital vector containing image characteristics, similar to the treatment in a single frame visual understanding.

Time-series information integration: To understand the time sequence between key frames, models usually use circular neural networks (RNNs) or Transformer to integrate all key frame vectors into a " video memory vector " , representing the dynamic content of the entire video.

Questions and answers and generation: After the user's text is coded, cross-mode integration with video memory vectors. The Big Language Model (LLM) is then based on integrating information to generate summaries of videos or answers to specific questions.

Summary: It can be understood that the video is "condensed" into several key images and their sequences, with models that understand the content of the story as they look at comics and then generate answers or descriptions in language.

#### Code Parsing

#### Key Code

## Tool Layer Entry (largemodel/utils/tools_manager.py)

The analyze video function in this document defines the process of executing the tool.

```bash
#From largemodel/utils/tools_manager.py
class ToolsManager:
  # ...
  def analyze_video(self, args):
  """
  Analyze video file and provide content description.
  Analyse a video file and provide a content description.

  :param args: Arguments containing video path.
  :return: Dictionary with video description and path.
  """
  self.node.get_logger().info(f"Executing analyze_video() tool with args: {args}")
  try:
  video_path = args.get("video_path")
  # ... (smart path fallback mechanism)

  if video_path and os.path.exists(video_path):
  # ... (Build the prompt)

  # Use a fully isolated, one-time context for video analysis to ensure a plain text description.
  simple_context = [{
  "role": "system",
  "content": "You are a video description assistant. ..."
  }]

  result = self.node.model_client.infer_with_video(video_path, prompt, message=simple_context)

  # ... (Process the result)
  return {
  "description": description,
  "video_path": video_path
  }
  # ... (Error handling)
```

### Model interface layer and frame extraction (largemodel/utils/large_model_interface.py)

The function in this file handles video files and transmits them to the bottom model.

```bash
#From largemodel/utils/large_model_interface.py
class model_interface:
  # ...
  def infer_with_video(self, video_path, text=None, message=None):
  """Unified video inference interface. / Unified video inference interface."""
  # ... (prepare messages)
  try:
  # choose the concrete implementation based on `self.llm_platform`
  if self.llm_platform == 'ollama':
  response_content = self.ollama_infer(self.messages, video_path=video_path)
  # ... (logic for other online platforms)
  # ...
  return {'response': response_content, 'messages': self.messages.copy()}

  def _extract_video_frames(self, video_path, max_frames=5):
  """Extract keyframes from a video for analysis.
  try:
  import cv2
  # ... (video reading and frame interval calculation)
  while extracted_count < max_frames:
  # ... (loop through the video frames)
  if frame_count % frame_interval == 0:
  # ... (save frames as temporary images)
  frame_base64 = self.encode_file_to_base64(temp_path)
  frame_images.append(frame_base64)
  # ...
  return frame_images
  # ... (exception handling)
```

### Code Parsing

Compared to a single image analysis, video analysis has achieved a critical step in processing — video frame extraction. The logic is deliberately placed in a model interface to ensure simplicity and interoperability of the upper levels of business codes.

### Tool Layer (tools_manager.py)

The tool layer is the business portal for video analysis with the core function of analize video.

The main functions of analize video are to receive video file paths and construct analytical instructions (Prompt) for requesting video content interpretation in models;

It then launched the full video analysis process by calling self.node.model client.infer with video;

Consistent with the image analysis tool, the tool layer is not concerned with how the bottom model will be achieved, nor does it need to process any video resolution details, but is responsible only for sending down video resources and analytical instructions.

This design keeps the tool layer focused on "expression of business intent" rather than "realization of technical details".

### Model interface layer (large_model_interface.py)

The model interface level is the core processing module for video analysis, with responsibility for mission movement and data pre-processing.

Infer with video as a unified portal will distribute video analysis requests to the corresponding specific realization function based on the currently configured model platform (self.llm platform);

Unlike a single photo reasoning, video data require additional pre-processing steps before being sent to the model.  extract video frames reflects this common realization logic;

This method is based on the cv2 library to read video files and extract key frames from them (default 5 frames) in accordance with the set policy;

Each frame is considered to be a stand-alone image and is usually converted to the Base64 code format for uniform transmission to large model interfaces;

Ultimately, requests consisting of multiple frame image data together with analytical instructions are sent to large language models that synthesize reasoning based on these continuous visual information and generate an overall description of the entire video content. It is worth noting that the frame extraction and coding process of the Video Multigraph is completely enclosed within the model interface and is fully transparent to the tool layer.

Overall, the generic implementation process for video analysis can be summarized as follows:

ToolsManager initiates the analysis request → model interface takes over the request and dismantles the video as a key frame  model interface by  extract video frames  model interface sends frame data and analysis instructions to the corresponding model platform → model returns the comprehensive description of the video content according to configuration → and returns to ToolsManager to the top.

This stratification effectively isolates video processing details from business logic, ensures stability of the upper application interface and provides a good common basis for subsequent expansion of different models or platforms.

### Practice

#### Configure Large Offline Model

### Configure LLM platform (seeed.yaml)

This document determines which large model platform to load at the model service node as its main language model.

Open file in terminal:

```bash
Code Block
vim /opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/config/seeed.yaml
```

Modify/confirm llm platform:

```bash
model_service:  #model server node parameters
  ros__parameters:
  language: 'zh'  #LLM interface language
  useolinetts: True  #Not used in text-only mode; can be ignored

  # LLM configuration
  llm_platform: 'ollama'  # Key: make sure this is set to 'ollama'
  regional_setting : "China"
```

### Configure Model Interface (large_model_interface.yaml)

This document defines which visual model is used when the platform is selected as olama.

Open file in terminal

```bash
# vim /opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/config/large_model_interface.yaml
```

Find the configuration of olama

```bash
#.....
Offline Large Language Models
Ollama configuration
ollama_host: "http://127.0.0.1:11434"  # Ollama server address
ollama_model: "qwen2.5vl:3b"  # Key: change this to a multimodal model you have already downloaded, such as "llava"
#.....
```

> Note: Ensure that the model specified in the configuration parameters (e.g. qwen2.5vl) handles multi-modular input.

#### Start and test functionality (text input mode)

Prepare video files:

Place a video file to test in the following path:

And then the video is named test video.mp4.

Start the main program (text mode):

![](./images/5-3-offline-vision-language-models-and-applications-40.png)

Open a terminal and then run the following command:

```bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

Send text instructions: open another terminal again and run the following instructions,

```bash
ros2 run text_chat text_chat
```

And then you start inputting text: analyze this video.

Observation: In the first terminal where the main program is run, you will see log output, show the system receiving commands, call the analize video tool, extract key frames and eventually print an AI-to-video summary.

### Common problems and solutions

### Question 1: Point "No video file found" or "No key frame extracted from the video."

Solutions:

Check the path: Ensure that you have a video file named test video.mp4 and have access to it.

Video format/coding: Ensure that the video file is not damaged and is in .mp4 format.

### Question 2: Analysis of a longer video is time-consuming.

Solutions:

Use lighter models: Hardware is limited and can be switched to a video interpretation model with fewer parameters.

#### 11.03-08 Multimodal Visual Positioning Application

## Concept introduction

### What's multi-modular visual positioning?

Multimodal visual positioning means the integration of data from different sensors or information mosaics (e.g. RGB images, depth information, laser radar, IMU or semantic information) to achieve an accurate estimate of the location and attitude of the equipment or target in space through uniform modelling and synergetic reasoning. Compared to single visual positioning, multi-modular visual positioning can take full advantage of the complementary advantages of the various modulations, significantly increasing the stability, scalability and accuracy of positioning in complex contexts such as photo-change, texture thinness or dynamic environments, often in the areas of robotic navigation, autopilot and enhancement of reality.

### Brief description of the rationale for realization

Cross-temporal means learning: the system needs to "read" visual information for the large language model. To this end, the visualization of images or videos is usually carried out through visual networks such as CNN and ViT, and visual features are mapped into embedded spaces consistent with the text semantics through projection or adaptation layers, thus achieving visual-linguistic alignment at the expression level so that models can handle different mosaics of information in a unified semantic space.

Joint training mechanism: Introduction of visual and text data in the same learning framework, and design of loss functions for cross-modular alignment or matching to enable models to develop links between image content and language semantics in the course of training. For example, in visual questions and answers or in graphic interpretation missions, models need to integrate image information and text input to learn how to work together on the reasoning between the two modes.

Visually-led language understanding and generation: At this point, visual information is no longer just a supporting input, but rather an important basis for driving language model reasoning, which not only describes visual content but also allows for questions and answers, reasoning and even implementation of directives based on images, thereby achieving a true multi-modular intelligence understanding and decision-making.

## Code Parsing

### Key Code

### Tool Layer Entry (largemodel/utils/tools_manager.py)

The visual propositioning function in this document defines the process of implementing the tool, in particular how it constructs a Prompt that contains the name and format requirements of the target object.

```bash
#From largemodel/utils/tools_manager.py
class ToolsManager:
  # ...
  def visual_positioning(self, args):
  """
  Locate object coordinates in image and save results to MD file.
  Locate object coordinates in the image and save the result to a Markdown file.

  :param args: Arguments containing image path and object name.
  :return: Dictionary with file path and coordinate data.
  """
  self.node.get_logger().info(f"Executing visual_positioning() tool with args: {args}")
  try:
  image_path = args.get("image_path")
  object_name = args.get("object_name")
  # ... (path fallback mechanism and parameter checks)

  # Construct a prompt asking the large model to identify the coordinates of the specified object.
  if self.node.language == 'zh':
  prompt = f"Please analyze this image carefully and locate each `{object_name}` in the image with separate bounding boxes."
  else:
  prompt = f"Please carefully analyze this image and find the position of all {object_name}..."

  # ... (build an independent message context)

  result = self.node.model_client.infer_with_image(image_path, prompt, message=message_to_use)

  # ... (process and parse the returned coordinate text)

  return {
  "file_path": md_file_path,
  "coordinates_content": coordinates_content,
  "explanation_content": explanation_content
  }
  # ... (Error handling)
```

#### Model interface layer (largemodel/utils/large_model_interface.py)

The infer with image function in this file is the unified entry for all image-related tasks.

```bash
#From largemodel/utils/large_model_interface.py
class model_interface:
  # ...
  def infer_with_image(self, image_path, text=None, message=None):
  """Unified image inference interface.
  # ... (prepare messages)
  try:
  # choose the concrete implementation based on `self.llm_platform`
  if self.llm_platform == 'ollama':
  response_content = self.ollama_infer(self.messages, image_path=image_path)
  elif self.llm_platform == 'tongyi':
  # ... logic for calling the Tongyi model
  pass
  # ... (logic for other platforms)
  # ...
  return {'response': response_content, 'messages': self.messages.copy()}
```

### Code Parsing

The core idea of the visual positioning function is to direct the output of the decryptionable structural results of large-language models through precise command design. In the overall architecture, this function follows the stratification design principles for the decoupling of the tool layer with the model interface.

### Tool Layer (tools_manager.py)

In the tool layer, the visual positioning function performs the main business logic of visual positioning tasks.

This function receives two key input parameters: image path (the path of the image to be analysed) and object name (the name of the object to be located in the image);

The core operation is to build a highly customized Prompt. Unlike a normal image description, the Prompt will embed object name dynamically into a pre-designed command template, clearly requiring the model to locate all specified targets in the image;

At the same time, Prompt will implicitly or visibly bind the model to return the result to a specific structure, such as coordinates of arrays or position information in fixed format, thus creating conditions for subsequent analysis;

After completing the Prompt construction, visual propositioning sends image data to the model with customised commands by using the model interface method;

When the model returns the result, the tool layer also has to perform the necessary reprocessing operations, usually extracting precise coordinates from the natural language output by means such as regular expressions;

Ultimately, the structured coordinate data obtained by resolution will be consolidated and returned to the upper application.

In this way, the tool layer is responsible for both the definition of tasks and the structuralization of results.

### Model interface layer (large_model_interface.py)

The infer with image function in the model interface continues to play the role of the Movement Center in the visual positioning scene.

![](./images/5-3-offline-vision-language-models-and-applications-41.png)

This function receives image data and positioning instructions from the visual positioning and delivers requests to the corresponding backend model according to the currently configured model platform type (self.llm platform);

For visual positioning missions, the processing process at the model interface level is largely consistent with visual understanding tasks, with responsibilities focused mainly on data containment and platform compatibility;

All details of realization related to a specific model platform (e.g., request format, coding method, interface call method, etc.) are isolated within the layer;

The model interface level does not perform any operational-level analysis of the text results returned by the model, but returns them to the tool-level.

The generic implementation process for visual positioning can be summarized as follows:

ToolsManager receives the name of the target object and constructs a precise request to return coordinate information Prompt → ToolsManager calls model interface → model interface packs images with Prompt and returns text with location information according to the configuration sent to the corresponding model platform → Model → model interface returns the result to ToolsManager to parsing text, extracts structured coordinates data and returns to upper application.

The process provides ample evidence of how to use Prompt Engineering technology to achieve more specific, manageable and structured visual positioning tasks for generic large visual models.

## Practice

### Configure Large Offline Model

### Configure LLM platform (seeed.yaml)

This document determines which large model platform to load at the model service node as its main language model.

Open file in terminal:

```bash
Code Block
vim /opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/config/seeed.yaml
```

Modify/confirm llm platform:

```bash
model_service:  #model server node parameters
  ros__parameters:
  language: 'zh'  #LLM interface language
  useolinetts: True  #Not used in text-only mode; can be ignored

  # LLM configuration
  llm_platform: 'ollama'  # Key: make sure this is set to 'ollama'
  regional_setting : "China"
```

#### Configure Model Interface (large_model_interface.yaml)

This document defines which visual model is used when the platform is selected as olama.

Open file in terminal

```bash
# vim /opt/seeed/development_guide/12_llm_offline/seeed_ws/src/largemodel/config/large_model_interface.yaml
```

Find the configuration of olama

```bash
#.....
Offline Large Language Models
Ollama configuration
ollama_host: "http://127.0.0.1:11434"  # Ollama server address
ollama_model: "qwen2.5vl:3b"  # Key: change this to a multimodal model you have already downloaded
#.....
```

> Note: Make sure that the model specified in the configuration parameters (e.g. qwen2.5vl) handles multi-modular input.

### 3.2 Activate and test functionality (text mode)

Prepare photo files:

Place a photo file to test with the following path:

And then name the picture "test image.jpg"

Start the main program (text mode):

Open a terminal and then run the following command:

```bash
ros2 launch largemodel largemodel_control.launch.py text_chat_mode:=true
```

Send text instructions: open another terminal again and run the following instructions,

```bash
ros2 run text_chat text_chat
```

Then you start typing text: "analyze the location of xiaomao in the picture."

Observation: In the first terminal where the main program is running, you will see log output, show the system receiving commands, call the visual propositioning tool, prompt the visual prositioning execution complete, and save the coordinates to the document.

We can find this document in the path of the https://download.docker.com/linux/ubuntu/dists/.

### 1. Common problems and solutions

#### Question 1: Point: "No pictures found."

Solutions:

Check the path: Make sure you have a picture file in the specified path, named test image.jpg, and have access to it.

Photo format: Ensure that the photo file is not damaged and is in .jpg format.

#### 11.03-09 Multimodal Table Scan Application

## Concept introduction

#### What's a multi-mode form scan?

Multi-modular table scanning is the technology for automatic identification, understanding and structured reconstruction of table contents in paper-based or electronic documents in combination with multiple information modulations such as images, text, etc. It not only detects table ranges, identifies cell structure and text content from images, but also integrates semantic context to understand header, field meanings and data relationships, thereby converting unstructured or semi-structured table information into editable, calculable structured data, which is widely used in document digitization, financial statement processing and intelligent office settings.

### Principle of realization

Table location and information recognition
First, the system automatically detects the table range in the document by computer visual means and identifies and transliterates the text content in the table in conjunction with OCR technology. The table structure was then analysed using an in-depth learning model, including column division, cell boundaries and merging relationships, thus converting the original table into a digitalized representation with a clear structure.

2. Multi-modular information integration and understanding
Once the visual structure of the table is obtained with the text content, the system further integrates a variety of modular information, such as table layout features, OCR results and related metadata, and constructs a uniform multi-modular input expression. Improves the overall accuracy of table resolution and structural restoration by introducing a multi-modular model (e.g. LayoutLM) that is specific to document interpretation, by joint modelling of information on different mosaics in order to understand more accurately the semantic meaning of table data and their context.

## Code Parsing

### Key Code

### Tool Layer Entry (largemodel/utils/tools_manager.py)

The scan table function in this document defines the process of implementing the tool, in particular how it constructs a Prompt that requires returns to the Markdown format.

```bash
#From largemodel/utils/tools_manager.py
class ToolsManager:
  # ...
  def scan_table(self, args):
  """
  Scan a table from an image and save the content as a Markdown file.
  Scan a table from the image and save the content as a Markdown file.

  :param args: Arguments containing the image path.
  :return: Dictionary with file path and content.
  """
  self.node.get_logger().info(f"Executing scan_table() tool with args: {args}")
  try:
  image_path = args.get("image_path")
  # ... (path checks and fallback)

  # Construct a prompt asking the large model to recognize the table and return it in Markdown format.
  # Build a prompt asking the large model to recognise the table and return it in Markdown format.
  if self.node.language == 'zh':
  prompt = "Please analyze this image carefully, identify the table in it, and return its contents in Markdown format."
  else:
  prompt = "Please carefully analyze this image, identify the table within it, and return its content in Markdown format."

  result = self.node.model_client.infer_with_image(image_path, prompt)

  # ... (extract Markdown text from the result)

  # Save the recognized content to a Markdown file. / Save the recognized content to a Markdown file.
  md_file_path = os.path.join(self.node.pkg_path, "resources_file", "scanned_tables", f"table_{timestamp}.md")
  with open(md_file_path, 'w', encoding='utf-8') as f:
  f.write(table_content)

  return {
  "file_path": md_file_path,
  "table_content": table_content
  }
  # ... (Error handling)
```

### Model interface layer (largemodel/utils/large_model_interface.py)
