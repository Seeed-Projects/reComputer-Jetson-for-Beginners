# Offline Large Model Development

This chapter introduces offline Large Language Model (LLM) deployment on NVIDIA Jetson devices. Unlike cloud-based AI services that require internet connectivity, offline models run entirely on your local hardware—ensuring **data privacy**, low latency, and independence from network conditions. This is especially valuable for edge AI applications in robotics, industrial automation, and intelligent systems.

<img src="./images/5-banner.gif" alt="5-banner" style="zoom:150%;" />

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamentals of Large Language Models (LLMs)
- Deploy and run LLMs using multiple inference frameworks: **Ollama**, **llama.cpp**, and **vLLM**
- Quickly deploy pre-configured examples using the **jetson-examples** repository
- Build a complete voice interaction pipeline combining **ASR (Automatic Speech Recognition) + LLM + TTS (Text-to-Speech)**

## Chapter Overview

| **Module** | **Content** | **Difficulty** |
|:----------:|:------------|:--------------:|
| Module 5.1 | [Introduction to Large Language Models](./5.1-Introduction-to-LLMs/README.md) | Beginner |
| Module 5.2 | [Getting Started with Ollama](./5.2-Getting-Started-with-Ollama/README.md) | Beginner |
| Module 5.3 | [Running LLMs with llama.cpp](./5.3-Running-LLMs-with-llama.cpp/README.md) | Intermediate |
| Module 5.4 | [High-Performance Inference with vLLM](./5.4-High-Performance-Inference-with-vLLM/README.md) | Advanced |
| Module 5.5 | [Jetson Examples Quick Start](./5.5-Jetson-Examples-Quick-Start/README.md) | Beginner |
| Module 5.6 | [Building ASR + LLM + TTS Pipeline](./5.6-ASR-LLM-TTS-Pipeline/README.md) | Intermediate |

## Suggested Learning Paths

### Path 1: Quick Start (Beginner-Friendly)
Perfect if you want to get LLMs running on Jetson with minimal setup:
1. **5.1** → Understand LLM basics
2. **5.2** → Deploy your first model with Ollama
3. **5.5** → Explore pre-built examples from jetson-examples

### Path 2: Framework Deep Dive (Intermediate)
For developers who want to understand different inference engines:
1. **5.1** → LLM fundamentals
2. **5.2** → Ollama for easy deployment
3. **5.3** → llama.cpp for lightweight inference
4. **5.4** → vLLM for high-performance serving

### Path 3: Building Voice Applications (Advanced)
For creating complete voice-enabled AI systems:
1. **5.1** → LLM basics
2. **5.2** or **5.5** → Get LLMs running
3. **5.6** → Build the complete voice pipeline

## Hardware Requirements

| Component | Minimum | Recommended |
|:----------|:--------|:------------|
| **Device** | Jetson Orin Nano 8GB | Jetson Orin NX 16GB Super |
| **Storage** | 64GB free space | 128GB+ SSD recommended |
| **Swap** | 8GB | 16GB+ for larger models |
| **Network** | Required for initial setup | Stable connection for model downloads |

## Prerequisites

Before starting this chapter, ensure you have:
1. Completed [Chapter 2: reComputer Jetson Platform Overview](../2-reComputer-Jetson-Platform-Overview/README.md)
2. Completed [Chapter 3: Basic Tools and Getting Started](../3-Basic-Tools-and-Getting-Started/README.MD)
4. Docker installed and configured (for containerized examples)
5. Basic familiarity with Linux terminal commands

## Model Size Reference

Understanding model sizes helps you choose appropriate models for your hardware:

| Model Size | RAM Required | Jetson Compatibility | Speed |
|:-----------|:-------------|:-------------------|:------|
| 1B - 3B | 4-6GB | Orin Nano 4GB+ | Fast |
| 7B - 8B | 8-12GB | Orin Nano 8GB+ | Moderate |
| 13B - 14B | 16-24GB | Orin NX 16GB+ | Slower |
| 35B+ | 48GB+ | Not recommended for single-device | Very slow |

## Key Terminology

| Term | Definition |
|:-----|:-----------|
| **LLM** | Large Language Model - AI models trained on vast text data to understand and generate human-like text |
| **Inference** | The process of running a trained model to generate outputs |
| **Quantization** | Reducing model precision (e.g., from 16-bit to 4-bit) to save memory and speed up inference |
| **Context Window** | The maximum amount of text (tokens) a model can process at once |
| **Token** | A unit of text (word, subword, or character) that models process |
| **ASR** | Automatic Speech Recognition - converting speech to text |
| **TTS** | Text-to-Speech - converting text to spoken audio |
| **GGUF** | A binary format for storing quantized models efficiently |

## References

- [jetson-examples Repository](https://github.com/Seeed-Projects/jetson-examples) - Ready-to-run AI examples for Jetson

- [Ollama Official Site](https://ollama.com/) - Get up and running with LLMs locally

- [llama.cpp GitHub](https://github.com/ggerganov/llama.cpp) - LLM inference in C/C++

- [vLLM Documentation](https://docs.vllm.ai/) - High-throughput LLM serving

  

## Next Steps

Start with [Module 5.1: Introduction to Large Language Models](./5.1-Introduction-to-LLMs/README.md) to build your foundation, or jump directly to [Module 5.2: Getting Started with Ollama](./5.2-Getting-Started-with-Ollama/README.md) if you're eager to run your first model!
