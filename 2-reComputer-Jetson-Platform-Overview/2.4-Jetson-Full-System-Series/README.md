# 2.4 Jetson Full System Series

If the previous section focused on boards, this section focuses on the complete systems that Seeed ultimately ships to users.

The value of a full system is that what you receive is not only a carrier board and module, but also:

- a preinstalled or quickly flashable JetPack / BSP environment
- a matched thermal solution
- an enclosure and power design ready for deployment
- a more clearly defined application boundary

## Main Seeed reComputer Full-System Families

| **Family** | **Core Characteristics** | **Best For** |
|:----------:|:-------------------------|:-------------|
| Classic J30/J40 | Standard edge AI box, balanced interfaces, low barrier to entry | Learning JetPack, building PoCs, and quick validation |
| Mini | Compact size with wide-voltage input | Space-constrained devices, mobile platforms, and onboard scenarios |
| Industrial | Fanless, dual Ethernet, serial, CAN, PoE, DIN rail | Factories, control cabinets, and onsite deployment |
| Super | JetPack 6.2, 157 TOPS, 4x CSI, dual Ethernet | Higher-performance vision AI, generative AI, and physical AI |
| Robotics | 5G, GMSL2, dual Ethernet, multiple USB, multiple CAN | Robot-body integration and multi-sensor systems |

## Classic J30/J40: The Default Starting Point

The `Classic` family is the easiest type of reComputer to understand:

- a standard system form factor
- a balanced set of USB / HDMI / CSI / GbE / NVMe capabilities
- a smooth path from development to lightweight deployment

For this tutorial, if your goal is simply to get the environment ready and run the later examples, Classic is usually the safest starting point.

## Mini: Not a Reduced Version, but a More Size- and Power-Flexible System

Seeed's official Mini materials emphasize these points:

- a smaller system footprint
- `12V-54V` wide-voltage input
- retained access to common expansion options such as M.2 Key E, SSD, RJ45, and USB

The value of Mini lies in the following:

- it fits devices with tight space constraints
- it works well for mobile robots or battery-powered systems
- it is a better fit for projects sensitive to size, weight, and mounting position

## Industrial: Trading Comfort for Deployment Certainty

The Industrial family is clearly different from Classic:

- `fanless` passive cooling
- `2x RJ45 GbE`
- `RS232 / RS422 / RS485`
- `CAN`
- `DI / DO`
- `PoE PSE`
- `DIN rail / wall mount / VESA`

That tells you the platform is not optimized for desktop convenience. It is optimized to connect to existing industrial systems and field equipment.

## Super: Seeed's New High-Performance Full-System Interpretation for Orin Nano / NX

The Super family is not just about a higher performance number. It packages a higher-performance target together with stronger IO in a standard full-system form:

- up to `157 TOPS`
- built around `JetPack 6.2`
- `4x CSI`, `dual RJ45`, and `Mini-PCIe` that better fit production-grade vision and edge AI

If you want to push the vision AI, generative AI, or robotics workloads from this tutorial further, the Super family is a natural next step.

## Robotics: From "Edge AI Box" to "Robot Brain"

The Robotics family emphasizes sensor and vehicle-body integration:

- dual Ethernet
- 6x USB 3.2
- 5G / Wi-Fi / Bluetooth expansion
- multiple CAN channels
- optional `GMSL2`
- stronger alignment with ROS / Isaac ROS / sensor fusion workflows

These systems are closer to the central compute unit of a robot than to a simple inference box.

## A Practical Decision Framework

If you are deciding between Classic, Mini, Industrial, Super, and Robotics, do not start with the module model. Start by answering these three questions:

1. Will the system be deployed on a desk, in a cabinet, on a mobile platform, or on a robot body?
2. Does it rely more on USB / HDMI / CSI, or on serial / CAN / dual Ethernet / 5G / GMSL2?
3. Is it for learning and validation, or is it intended to move toward more stable field deployment?

Those questions usually determine the right choice earlier than "Which module is the most powerful?"

## References

- [Seeed reComputer J40 Series Datasheet](https://files.seeedstudio.com/wiki/reComputer/reComputer-J40.pdf)
- [Seeed reComputer Mini Hardware and Interfaces Usage](https://wiki.seeedstudio.com/recomputer_jetson_mini_hardware_interfaces_usage/)
- [Seeed reComputer Industrial Getting Started](https://wiki.seeedstudio.com/reComputer_Industrial_Getting_Started/)
- [Seeed reComputer Super Guide](https://wiki.seeedstudio.com/recomputer_jetson_super_getting_started/)
- [Seeed reComputer Robotics J401 Guide](https://wiki.seeedstudio.com/recomputer_robotics_j401_getting_started/)
