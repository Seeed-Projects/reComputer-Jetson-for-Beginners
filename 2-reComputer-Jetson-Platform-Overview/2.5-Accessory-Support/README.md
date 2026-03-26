# 2.5 Accessory Support

An edge AI platform often fails in practice not because the module is too weak, but because the accessories, interfaces, and software stack are incomplete.

## Start with Hardware Expansion

The common expansion directions for the Seeed reComputer Jetson platform can be summarized as follows:

| **Expansion Area** | **Common Interface / Form Factor** | **Typical Purpose** |
|:-------------------|:-----------------------------------|:--------------------|
| Storage | M.2 Key M + NVMe SSD | System disk, model storage, and data cache |
| Wireless Connectivity | M.2 Key E, Mini-PCIe, M.2 Key B | Wi-Fi, Bluetooth, 4G/5G, LoRaWAN |
| Cameras | CSI, optional GMSL2 | Vision inference, multi-camera systems, onboard or industrial camera integration |
| Display and Debug | HDMI, DP over Type-C, USB Device | Local display, flashing, and debug access |
| Industrial and Robotics Bus | CAN, RS232 / 422 / 485, DI / DO | Chassis, motor, PLC, and sensor integration |

## References

- [Seeed reComputer J40 Series Datasheet](https://files.seeedstudio.com/wiki/reComputer/reComputer-J40.pdf)
- [Seeed reComputer Mini Hardware and Interfaces Usage](https://wiki.seeedstudio.com/recomputer_jetson_mini_hardware_interfaces_usage/)
- [Seeed reComputer Industrial Getting Started](https://wiki.seeedstudio.com/reComputer_Industrial_Getting_Started/)
- [Seeed reComputer Super Guide](https://wiki.seeedstudio.com/recomputer_jetson_super_getting_started/)
- [Seeed reComputer Robotics J401 Guide](https://wiki.seeedstudio.com/recomputer_robotics_j401_getting_started/)
- [NVIDIA JetPack 6.2 Brings Super Mode to Jetson Orin Nano and Orin NX Modules](https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/)
