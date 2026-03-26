# 2.3 Seeed Jetson Compatible Carrier Board

Jetson modules are not meant to be dropped directly into a project by themselves. The `carrier board` is what organizes practical interfaces such as USB, Ethernet, CSI, M.2, CAN, serial, and power input into a usable platform.

## Start with One Key Concept

The same board code, such as `J401`, can refer to different carrier boards in different Seeed product families:

- `reComputer J401`: the general-purpose carrier board in the Classic family
- `reComputer Super J401`: the higher-performance carrier board in the Super family
- `reComputer Robotics J401`: the robotics-oriented carrier board in the Robotics family

In engineering practice, that means you cannot look at the board number alone. You need the `full product family + full model name`.

## Typical Carrier Board Comparison

| **Board / Platform** | **Supported Modules** | **Main Interface Characteristics** | **Best Fit** |
|:---------------------|:----------------------|:-----------------------------------|:-------------|
| [reComputer J401](https://www.seeedstudio.com/reComputer-J401-Carrier-Board-for-Jetson-Orin-NX-Orin-Nano-without-Power-Adapter-p-5637.html) | Orin Nano / Orin NX | 4x USB 3.2, HDMI 2.1, 2x CSI, 1x GbE, M.2 Key M, M.2 Key E, CAN, RTC | General development, education, and standard edge AI |
| [reComputer Mini J401](https://www.seeedstudio.com/reComputer-Mini-Carrier-Board-p-6420.html) | Orin Nano / Orin NX | 6x USB 3.0/3.2, DP, 1x GbE, M.2 Key M, M.2 Key E, CAN, 12-54V input | Compact edge devices with stackable form factor |
| [reComputer Mini J501](https://www.seeedstudio.com/reComputer-Mini-J501-Carrier-Board-with-GMSL-Bundle-for-Jetson-AGX-Orintm.html) | AGX Orin | 2x USB 3.2, 8x GMSL2, 2x GbE, 2x CAN, M.2 Key M, M.2 Key E, 1x RS-485 | Compact form factor, very high compute, embodied AI |
| [reComputer Super J401](https://www.seeedstudio.com/reComputer-Super-J401-Carrier-Board-p-6642.html) | Orin Nano / Orin NX | Dual RJ45, 4x CSI, 4x USB 3.2, HDMI 2.1, Mini-PCIe, M.2 Key E/M, CAN | Higher-performance, stronger-IO production platform |
| [reComputer Robotics J401](https://www.seeedstudio.com/reComputer-Robotics-J401-Carrier-Board-optional-accessories.html) | Orin Nano / Orin NX | Dual GbE, 6x USB 3.2, M.2 Key B/E/M, 2x CAN, optional GMSL2, I2C, UART | Robot-body integration, multi-sensor fusion, 5G expansion |

## Section Summary

What a carrier board determines is not the theoretical compute ceiling, but rather `what you can connect, how the system is powered, how it expands, and whether it fits deployment requirements`.

As a result, the practical selection order should be:

1. Define the interfaces and environment required by the application.
2. Choose the category of carrier board that matches those needs.
3. Only then decide which Jetson module to use on that board.

## References

- [Seeed NVIDIA Collection](https://www.seeedstudio.com/tag/nvidia.html)
- [Seeed reComputer J401 Carrier Board Datasheet](https://files.seeedstudio.com/wiki/reComputer-J4012/Carrier-Board-J401/J401-datasheet.pdf)
- [Seeed reComputer Super Guide](https://wiki.seeedstudio.com/recomputer_jetson_super_getting_started/)
- [Seeed reComputer Super User Manual](https://files.seeedstudio.com/products/NVIDIA-Jetson/reComputer_super_user_manual.pdf)
- [Seeed reComputer Robotics J401 Guide](https://wiki.seeedstudio.com/recomputer_robotics_j401_getting_started/)
