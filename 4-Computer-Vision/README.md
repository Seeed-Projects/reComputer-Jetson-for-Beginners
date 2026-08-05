# Module 4: Computer Vision from Foundations to Jetson Practice

![cv_world](images/cv_world.gif)

This folder contains a rebuilt `Module 4` designed as a **systematic computer vision course for beginners**.

## Course Structure

### Part I: Foundations and Theory

This part is suitable even for learners who do not yet have a Jetson device.

| Module | Topic | Type |
| :-- | :-- | :-- |
| `4.1` | [Introduction to Computer Vision](./4.1-Introduction-to-Computer-Vision/README.md) | Theory |
| `4.2` | [How Computers Represent Images](./4.2-How-Computers-Represent-Images/README.md) | Theory + OpenCV examples |
| `4.3` | [Classical Computer Vision](./4.3-Classical-Computer-Vision/README.md) | Theory + OpenCV examples |
| `4.4` | [Neural Networks and CNNs](./4.4-Neural-Networks-and-CNNs/README.md) | Theory + simple code examples |
| `4.5` | [Deep Learning Computer Vision Tasks](./4.5-Deep-Learning-Computer-Vision-Tasks/README.md) | Theory |
| `4.6` | [Train and Deploy Your Own Vision Model](./4.6-Train-and-Deploy-Your-Own-Vision-Model/README.md) | Theory + code |

### Part II: Edge Deployment and Jetson Practice

This part applies the earlier knowledge to edge AI deployment.

| Module | Topic | Type |
| :-- | :-- | :-- |
| `4.7` | [Model Export and Edge Deployment](./4.7-Model-Export-and-Edge-Deployment/README.md) | Theory + code |
| `4.8` | [Real-Time Vision Pipeline Frameworks](./4.8-Real-Time-Vision-Pipeline-Frameworks/README.md) | Theory + code examples |
| `4.9` | [DeepStream and Jetson](./4.9-DeepStream-and-Jetson/README.md) | Theory + code examples |
| `4.10` | [Frontier Vision Technologies and Outlook](./4.10-Frontier-Vision-Technologies-and-Outlook/README.md) | Theory |

## Appendix

The AI NVR on reComputer appendix has been moved. Check the main repository for updates.

## Recommended Baseline for the Jetson Practice Half

The deployment half of this course assumes:

- `JetPack 6.2.x`
- Jetson Linux `R36.4.x`
- CUDA `12.6`
- TensorRT `10.3`
- cuDNN `9.3`
- `DeepStream 7.1`
- `Jetson Platform Services`

## Teaching Principles

This rebuilt Module 4 follows four teaching principles:

1. Explain the "why" before the "how".
2. Use code to illustrate concepts, not to replace explanation.
3. Treat data, metrics, and error analysis as core topics.
4. Keep deployment in the later half so the learner first builds understanding.

## Suggested Learning Path

If you are a beginner, follow the sections in order from `4.1` to `4.10`.

If you already understand computer vision basics and mainly want Jetson deployment, you can skim `4.1` to `4.6` and then focus on `4.7` to `4.10`.

## References

- [OpenCV Documentation](https://docs.opencv.org/4.x/index.html)
- [Ultralytics YOLO Docs](https://docs.ultralytics.com/)
- [JetPack 6.2.1 Release Notes](https://docs.nvidia.com/jetson/jetpack/release-notes/index.html)
- [Jetson Platform Services Quick Start](https://docs.nvidia.com/jetson/jps/setup/quick-start.html)
- [DeepStream Perception on JPS](https://docs.nvidia.com/jetson/jps/deepstream/deepstream.html)

