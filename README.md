# STEVE: Simulated Task Exploration | Valve Emulation

**A high-fidelity haptic feedback platform for valve simulation and robotics research**

---

## Overview

STEVE (Simulated Task Exploration | Valve Emulation) is an advanced haptic control system that provides realistic, programmable valve simulation through precise force feedback. Designed for robotics engineers and researchers, STEVE delivers real-time haptic rendering with sophisticated physics-based feedback models, enabling research, training, and development applications in valve control and haptic interaction.

The system simulates realistic valve operation with configurable characteristics including damping, friction, stiffness, and virtual boundaries. Users can experience different valve types and behaviors through programmable presets, making STEVE an ideal platform for developing robotic manipulation skills, testing control algorithms, and exploring haptic interfaces.

## Key Features

### Valve Simulation
- **Realistic haptic feedback** simulating various valve types and conditions
- **Programmable valve characteristics** - damping, friction, stiffness, detents
- **Virtual boundaries** with configurable wall stiffness and damping
- **Multiple presets** for different valve scenarios (smooth, sticky, stiff, etc.)
- **Real-time control** at 1 kHz+ update rates
- **Passivity-based control** ensuring stable, energy-dissipative behavior

### Architecture

STEVE is divided into two main components:

#### 1. Bare-Metal C Firmware (Primary Focus)

The core of STEVE is sophisticated bare-metal firmware for the **STM32H753ZI** microcontroller (ARM Cortex-M7 @ 480 MHz) that implements:

- **Valve simulation engine** with physics-based haptic rendering
- **CAN bus communication** for motor controller integration (ODrive S1)
- **Ethernet networking** with lwIP TCP/IP stack
- **REST API** for remote control and configuration
- **HTTP server** with web-based control interface
- **TCP streaming server** for real-time data logging (8888)
- **Command-line interface** over USB serial
- **Real-time performance monitoring** and diagnostics
- **MISRA-C:2012 compliance** for reliability and maintainability

**Firmware Features:**
- Modular architecture with clear separation of concerns
- Hardware abstraction layer for drivers and peripherals
- Non-volatile configuration storage
- Comprehensive error handling and diagnostics
- Zero-copy networking for optimal performance

#### 2. Python Client Library

The **`pysteve`** Python client provides high-level access to the firmware's REST API, enabling:

- **Easy integration** with custom Python applications
- **Robotics framework support** including:
  - **MuJoCo** physics simulation
  - **Gymnasium** reinforcement learning environments
  - **ROS 2** robotic middleware integration
  - **Isaac Sim** (NVIDIA Omniverse) compatibility
- **Data recording and analysis** with streaming support
- **Preset management** and configuration
- **Example code** for common workflows

The Python client is designed for robotics engineers who need to integrate haptic feedback into simulations, training environments, and automated testing systems.

### Hardware Platform

- **STM32H753ZI Nucleo-144** development board
- **ODrive S1** motor controller for brushless motor FOC
- **CAN FD bus** for low-latency motor commands
- **Ethernet** for remote monitoring and control
- **USB serial** via onboard ST-LINK for CLI access
- **Brushless motor** with encoder for haptic output

### User Interfaces

**For Firmware Interaction:**
- **CLI** - Command-line interface over USB serial (115200 baud)
- **REST API** - HTTP/JSON API over Ethernet (port 8080)
- **Web Interface** - Browser-based control panel (port 8080)
- **TCP Streaming** - Real-time data output (port 8888)

**For Application Development:**
- **Python Client** - High-level API via `pysteve` library
- **Direct REST API** - Accessible from any language/framework
- **TCP Streaming** - Raw data feed for custom logging/analysis

## Project Structure

```
steve_can/
├── firmware/           # STM32H7 bare-metal C firmware (primary focus)
│   ├── src/           # Source code organized by subsystem
│   │   ├── valve/     # Valve simulation and haptic control
│   │   ├── network/   # Ethernet, HTTP, REST API, streaming
│   │   ├── drivers/   # UART, CAN, hardware abstraction
│   │   ├── protocols/ # CAN Simple protocol for ODrive
│   │   └── app/       # CLI, performance monitoring
│   ├── inc/           # Header files
│   ├── third_party/   # lwIP, STM32Cube HAL, CMSIS-DSP
│   ├── Makefile       # Build system with validation
│   └── README.md      # Firmware architecture documentation
│
├── client/            # Python client library and examples
│   ├── pysteve/       # Python package for REST API access
│   │   ├── core/      # Client and configuration
│   │   ├── control/   # Valve control interface
│   │   ├── integrations/ # MuJoCo, Gymnasium, ROS2, Isaac Sim
│   │   └── utils/     # Data recording, streaming
│   ├── examples/      # Example scripts and integrations
│   └── docs/          # Python client documentation
│
└── docs/              # Comprehensive documentation
    ├── firmware/      # Firmware installation and development
    ├── getting-started/ # Quick start guide
    ├── cli/           # CLI command reference
    ├── rest/          # REST API documentation
    ├── stream/        # TCP streaming guide
    └── html/          # Web interface guide
```

## Quick Start

### For Firmware Developers

1. **Install prerequisites**: ARM GCC toolchain, OpenOCD, Make
2. **Build firmware**: `cd firmware && make`
3. **Flash to board**: `make flash`
4. **Connect via serial**: `screen /dev/ttyACM0 115200`
5. **Explore CLI**: Type `help` to see available commands

See [Firmware Installation Guide](docs/firmware/firmware-installation.md) for detailed instructions.

### For Robotics Engineers (Python)

1. **Install Python client**: `pip install -e client/`
2. **Connect to STEVE**: Ensure firmware is running and accessible
3. **Import library**: `from pysteve import SteveClient`
4. **Start coding**: See [Python examples](client/examples/)

See [Python Client Documentation](client/docs/) for API reference and integration guides.

### For Quick Testing

1. **Connect board** via USB (ST-LINK port)
2. **Open serial terminal** at 115200 baud
3. **Enable motor**: `odrive_enable`
4. **Start valve simulation**: `valve_start`
5. **Try different presets**: `valve_preset smooth`, `valve_preset stiff`

See [Getting Started Guide](docs/getting-started/getting-started.md) for complete walkthrough.

## Documentation

Comprehensive documentation is available in the [`docs/`](docs/) directory:

- **[Main Documentation Hub](docs/README.md)** - Complete documentation index
- **[Firmware Installation](docs/firmware/firmware-installation.md)** - Setup and installation
- **[Getting Started](docs/getting-started/getting-started.md)** - First steps with STEVE
- **[CLI Reference](docs/cli/cli-reference.md)** - Command-line interface guide
- **[REST API](docs/rest/rest-api.md)** - HTTP API documentation
- **[Streaming Guide](docs/stream/streaming-guide.md)** - Real-time data streaming
- **[Next Steps](docs/next-steps.md)** - Future development and how to contribute

## Use Cases

STEVE is designed for:

- **Robotics Research** - Developing and testing manipulation algorithms
- **Reinforcement Learning** - Training agents with realistic haptic feedback
- **Human-Robot Interaction** - Studying haptic perception and control
- **Simulation Environments** - Adding haptic realism to virtual environments
- **Control Algorithm Development** - Testing force control and impedance strategies
- **Training Systems** - Teaching valve operation and manipulation skills
- **Haptic Interface Research** - Exploring programmable haptic feedback

## Technical Specifications

- **Control Loop Rate**: 1+ kHz haptic update rate
- **Communication Latency**: <1ms CAN bus to motor controller
- **Network Throughput**: 100 Mbps Ethernet
- **Position Resolution**: Limited by motor encoder (typically <0.1°)
- **Force Range**: Dependent on motor/gearbox configuration
- **API Response Time**: <10ms for REST API calls
- **Streaming Rate**: Configurable, up to 1 kHz data output

## Project Information

**Institution**: University of Washington, Paul G. Allen School of Computer Science & Engineering

**Project Director**: Emma Romig

**Implementation**: Marcus Roessler

**License**: See individual component licenses
- Firmware utilizes open-source components (lwIP, STM32Cube HAL, CMSIS-DSP)
- Python client library licensing MIT
- Firmware license MIT

## Contributing and Feedback

We welcome community engagement! See our [Next Steps](docs/next-steps.md) document for:

- How to report bugs and provide feedback
- Future development directions
- Ways to contribute
- Research collaboration opportunities

## Support and Resources

### Hardware Resources
- [STM32 Nucleo-H753ZI](https://www.st.com/en/evaluation-tools/nucleo-h753zi.html) - Development board
- [ODrive Robotics](https://odriverobotics.com/) - Motor controller

### Software Tools
- [GNU ARM Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) - For firmware compilation
- [OpenOCD](http://openocd.org/) - For firmware flashing and debugging
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) - Alternative programming tool

### Documentation
- [Firmware README](firmware/README.md) - Detailed firmware architecture
- [Python Client README](client/README.md) - Python library documentation
- [Complete Documentation](docs/README.md) - Full documentation hub

## Acknowledgments

STEVE represents research in haptic interfaces, control systems, and embedded software engineering. The system's modular architecture, real-time performance, and comprehensive interfaces make it a powerful platform for robotics research and haptic exploration.

We're grateful to the open-source community for the foundational tools and libraries that make STEVE possible, including the lwIP TCP/IP stack, STM32Cube ecosystem, and ARM CMSIS-DSP library.

---

**Ready to explore haptic valve simulation?**

- **Start with firmware**: [Firmware Installation Guide](docs/firmware/firmware-installation.md)
- **Start with Python**: [Python Client Examples](client/examples/)
- **Read the docs**: [Documentation Hub](docs/README.md)
- **See what's next**: [Next Steps & Future Development](docs/next-steps.md)

---

*STEVE: Bringing realistic haptic feedback to robotics research and simulation.*
