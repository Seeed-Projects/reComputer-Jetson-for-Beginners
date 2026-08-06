# 4.9 DeepStream and Jetson

## Why This Matters

By this point in the course, the learner understands computer vision tasks, model training, export, and real-time pipelines. The next step is to see how those ideas scale into a practical edge AI system.

This is where `DeepStream` and `Jetson` become important.

`DeepStream` is not just another demo tool. It is a framework for building high-performance streaming analytics systems. On Jetson devices, it helps connect camera input, inference, tracking, metadata, and system services into a more production-like architecture.

## Learning Objectives

By the end of this section, you should be able to:

- explain what `DeepStream` is and why it matters
- understand why Jetson is a strong edge AI platform
- describe the roles of `JPS`, `VST`, `Redis`, `Ingress`, and `Analytics`
- understand how multi-stream systems differ from single-stream demos
- run and inspect a basic DeepStream/JPS-oriented workflow

## Core Concepts / Theory

### Why Jetson for Edge Vision

Jetson devices are designed for edge AI workloads where local inference matters. Common reasons to use Jetson include:

- low-latency local processing
- reduced cloud dependency
- GPU acceleration on embedded hardware
- support for real-time video analytics

### What DeepStream Adds

Compared with a simple script that reads a camera and runs a model, `DeepStream` provides:

- hardware-accelerated decode
- batch-friendly processing
- integrated tracking
- metadata generation
- better support for multi-stream analytics

### What Jetson Platform Services Adds

`Jetson Platform Services` adds a service-oriented layer around the pipeline.

Important components include:

- `VST` for stream onboarding and video handling
- `Redis` for metadata and service communication
- `Ingress` for unified external access
- `Analytics` for rules such as ROI and line crossing

### From Demo to System

A single demo script often answers:

- can I read one camera
- can I run one model

A system-level deployment must answer more:

- can I scale to multiple streams
- can I manage metadata
- can I recover from instability
- can I expose services to operators and other applications

That is the system mindset this section introduces.

## Key Terms

- `DeepStream`: NVIDIA's streaming analytics framework
- `JPS`: Jetson Platform Services
- `VST`: Video Storage Toolkit
- `Redis`: in-memory data store often used as a metadata backbone
- `Ingress`: routing layer for external service access
- `Analytics`: service for rule-based event logic

## Worked Example / Code Example

### Code Examples

This section includes helper scripts under `code/`:

- `sim_rtsp_stream.py` — Simulates RTSP camera streams for testing
- `ai-nvr/scripts/run_baseline.sh` — Runs baseline inference benchmark
- `ai-nvr/scripts/run_nvr.sh` — Starts DeepStream NVR pipeline with services
- `ai-nvr/scripts/setup_jetson.sh` — Sets up Jetson environment

> **Note:** The NVR scripts require additional files (configs, service code) that are part of a complete AI NVR deployment. Please refer to the NVIDIA Jetson Platform Services documentation for full setup instructions.

### Start Core Services

```bash
sudo systemctl start jetson-redis
sudo systemctl start jetson-ingress
sudo systemctl start jetson-vst
```

### Check That JPS Services Exist

```bash
ls /opt/nvidia/jetson/services
systemctl status jetson-redis
systemctl status jetson-vst
```

### Typical DeepStream Sample Run

```bash
deepstream-app -c /opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/source30_1080p_dec_infer-resnet_tiled_display_int8.txt
```

### Inspect a Managed Stream Path

```bash
gst-launch-1.0 rtspsrc location=rtsp://<jetson-ip>:8554/<stream> latency=200 ! \
  rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

## Common Misunderstandings

- "DeepStream is only for benchmarking."
  - It is also a system-building framework for streaming analytics.
- "If one camera works, multi-stream deployment is just more of the same."
  - Multi-stream systems introduce scheduling, metadata, and stability concerns.
- "Jetson deployment is only about converting the model."
  - Deployment also includes input management, services, monitoring, and event logic.

## Exercises / Reflection

1. Explain in your own words how `DeepStream` differs from a simple OpenCV inference script.
2. List three reasons a multi-stream deployment is harder than a single-stream demo.
3. Draw a simple block diagram showing how `camera -> DeepStream -> tracking -> metadata -> analytics` might flow.
4. Reflect on when a learner should choose Jetson local deployment instead of cloud-only processing.

## Summary

`DeepStream` and Jetson turn computer vision into a full edge system. This section helps the learner shift from thinking about one model on one stream to thinking about scalable, service-based visual analytics.

## Suggested Next Step

Continue to [4.10 Frontier Vision Technologies and Outlook](../4.10-Frontier-Vision-Technologies-and-Outlook/README.md).

## References

- [DeepStream Perception on JPS](https://docs.nvidia.com/jetson/jps/deepstream/deepstream.html)
- [Jetson Platform Services Quick Start](https://docs.nvidia.com/jetson/jps/setup/quick-start.html)
- [Jetson Platform Services Overview](https://docs.nvidia.com/moj/moj-overview.html)

