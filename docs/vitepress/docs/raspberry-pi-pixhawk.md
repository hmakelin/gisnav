# Raspberry Pi 5 & Pixhawk FMUv4

This page describes how to run HIL simulation on a Pixhawk board using the Raspberry Pi 5 as a companion computer.

### Prerequisites

- [Install GISNav locally](/install-locally)

### Concepts

This page uses the below terminology:

- Simulation host: Computer that hosts the HIL simulation world (Gazebo in this case)
- Development host: Computer that uploads GISNav services onto the companion computer

#### Setup Raspberry Pi 5 board for development

TODO

### Connect Raspberry Pi 5 and Pixhawk

- We connect our development computer to the Raspberry Pi 5 over Ethernet. This is so that we can upload the containers implementing required onboard services.

- We connect the Raspberry Pi 5 as a secondary NMEA GPS device over the GPS 2 serial port.

- We connect the simulation host computer (assumed to be the same as the development computer but strictly speaking these could be separate computers.)


#### Diagram

```mermaid
graph TB
    subgraph "FMUK66-E (FMUv4)"
        subgraph "GPS 2"
            FMU_TELEM1_RX[RX]
            FMU_TELEM1_TX[TX]
            FMU_TELEM1_GND[GND]
        end
        FMU_USB[micro-USB]
    end
    subgraph "Development host"
        Laptop_ETH[Ethernet]
        Laptop_USB[USB]
    end
    subgraph "Raspberry Pi 5"
        subgraph USB["USB ports (interchangeable)"]
            Pi_USB[USB x2]
            Pi_micro_USB["Micro-USB"]
            Pi_USB_C["USB-C"]
        end
        Pi_HDMI[HDMI]
        Pi_ETH[Ethernet]
    end
    subgraph "USB to UART Converter"
        Converter_RX[RX]
        Converter_TX[TX]
        Converter_GND[GND]
        Converter_USB[USB]
    end
    Socket[Power Supply]
    subgraph "Peripherals (optional)"
        Display[Display]
        Mouse[USB Mouse]
        Keyboard[USB Keyboard]
    end
    FMU_TELEM1_TX --- Converter_RX
    FMU_TELEM1_RX --- Converter_TX
    FMU_TELEM1_GND --- Converter_GND
    FMU_USB ---|Upload PX4 firmware| Laptop_USB
    Converter_USB ---|NMEA 0183| Pi_micro_USB
    Pi_USB_C --- Socket
    Pi_HDMI --- Display
    Pi_USB --- Mouse
    Pi_USB --- Keyboard
    Pi_ETH ---|Upload GISNav services| Laptop_ETH
```
