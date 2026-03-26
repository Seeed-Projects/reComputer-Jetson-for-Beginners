# 2.1 What Is reComputer Jetson

<p align="center">
    <img src="./images/Seeed-NVIDIA-Jetson-Family.png" height="360" alt="Seeed NVIDIA Jetson Family">
</p>

`reComputer Jetson` is not a single device. It is Seeed Studio's edge AI product matrix built around NVIDIA Jetson SoMs. It combines the compute module, carrier board, thermal solution, storage, enclosure, power design, and software support into a platform that helps developers move from evaluation to deployment faster.

## In One Sentence

If a `Jetson module` is the AI compute core, then `reComputer` is the practical edge AI device or development platform built around that core for real integration, deployment, and maintenance.

## What a reComputer Usually Includes

| **Layer** | **Purpose** | **Typical Components** |
|:---------:|:------------|:-----------------------|
| Jetson SoM | Provides compute | CPU, GPU, DLA, memory |
| Carrier Board | Provides interfaces and power delivery | USB, CSI, GbE, CAN, M.2, GPIO |
| Storage / Wireless | Provides system storage and connectivity | NVMe SSD, Wi-Fi, Bluetooth, 4G/5G |
| Thermal / Enclosure | Provides stable operation | Heatsink, fan, enclosure, mounting features |
| BSP / SDK | Provides the software foundation | JetPack, Jetson Linux, CUDA, TensorRT |

## Understanding the reComputer Jetson Product Matrix

The most practical way to understand the Seeed Studio reComputer Jetson product matrix is to look at it from two dimensions:

- `device compute tier`: this roughly maps to the underlying Jetson platform generation
- `application scenario`: this determines the system form factor, interfaces, and deployment style

### Device Compute Tiers

In Seeed Studio's naming scheme, `J10 / J20 / J30 / J40 / J50 / J60` can be understood as broad compute tiers. One important detail is that a `series prefix does not correspond to a single fixed performance number`. Different module capacities, power modes, and JetPack generations can lead to different figures within the same tier.

| **Series Prefix** | **Typical Jetson Platform** | **Official Compute Reference** | **How to Think About It** |
|:-----------------:|:----------------------------|:-------------------------------|:--------------------------|
| J10 | Jetson Nano | `0.5 TFLOPS (FP16)` | An early entry-level platform suitable for Linux, JetPack, and basic vision experiments |
| J20 | Jetson Xavier NX | `21 TOPS` | Better suited than J10 for more complex vision and traditional edge AI workloads, and still relevant in many existing deployments |
| J30 | Jetson Orin Nano | `up to 67 TOPS` | The current entry to mid-range mainstream option for learning, basic multi-camera vision, and lightweight generative AI |
| J40 | Jetson Orin NX | `up to 157 TOPS` | A compact high-performance option better suited to robotics, multi-camera systems, and larger model inference |
| J50 | Jetson AGX Orin | `up to 275 TOPS` | Designed for higher-end autonomous systems and multi-sensor fusion workloads |
| J60 | Jetson AGX Thor | `up to 2070 FP4 TFLOPS` | A next-generation platform for physical AI and robotics, with performance metrics that extend beyond traditional INT8 TOPS |

The point of this table is not to memorize numbers. The more useful mental model is:

- `J10 / J20` are earlier-generation or installed-base platforms
- `J30 / J40` form the core of today's reComputer entry-level and mainstream deployment range
- `J50 / J60` move into higher-end robotics and more demanding edge AI workloads

For this tutorial, most readers should focus on `J30` and `J40`. If your goal is to learn JetPack, OpenCV, YOLO, Docker, and basic ROS, `J30` is often enough. If you already know you want to work on multi-sensor robots, more video streams, or larger generative AI models, `J40` is usually the better fit.

### Application-Oriented Product Families

| **Family** | **Positioning** | **Keywords** |
|:----------:|:----------------|:-------------|
| Classic | General entry-level and standard deployment | Easy to start with, balanced interfaces, good for learning and PoCs |
| Mini | More flexible in size and power input | Compact, wide-voltage input, mobile-device friendly |
| Industrial | Built for industrial environments | Fanless, serial, CAN, dual Ethernet, PoE |
| Super | Higher-performance platforms based on Orin Nano / NX | JetPack 6, 157 TOPS, 4x CSI, dual Ethernet |
| Robotics | Built for robot-body integration | 5G, GMSL2, dual Ethernet, multiple USB, CAN |

One important conclusion follows from this: when choosing a reComputer platform, you should not only ask "Which Jetson should I buy?" You should also ask "What form factor and interface combination does my project actually need?"

## Key Concepts for Beginners

- `reComputer` is a platform family, not a single SKU.
- The `Jetson module` determines the compute ceiling, while the `carrier board and system design` determine what you can connect, how the device is powered, and how well it can be deployed.
- The same Jetson Orin NX can deliver a very different real-world experience in the Classic, Industrial, Super, and Robotics families.

## Section Summary

Everything else in Chapter 2 can be understood through this idea:

`reComputer = NVIDIA Jetson module + Seeed carrier board / system design + JetPack software stack`

## References

- [Seeed reComputer-Jetson Guide](https://wiki.seeedstudio.com/reComputer_Intro/)
- [Seeed reComputer J1010 Getting Started](https://wiki.seeedstudio.com/reComputer_J1010_with_Jetson_getting_start/)
- [Seeed reComputer J20 Getting Started](https://wiki.seeedstudio.com/recomputer_j20_with_jetson_getting_start/)
- [Seeed reComputer J30/J40 Getting Started](https://wiki.seeedstudio.com/reComputer_J30_40_with_Jetson_getting_start/)
- [Seeed reComputer Robotics J501 Mini Getting Started](https://wiki.seeedstudio.com/recomputer_j501_mini_getting_started/)
- [NVIDIA Jetson Modules](https://developer.nvidia.com/embedded/jetson-modules)
