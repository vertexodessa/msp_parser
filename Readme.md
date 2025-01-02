# MSP Parser & Forwarder

This repository contains a MSP (MultiWii Serial Protocol) parser and command executor system. The code demonstrates how to:

1. **Parse** MSP messages received from a flight controller (or other MSP-speaking device).
2. **Dispatch** these messages to one or more **executors** that handle specific commands (e.g., STATUS, ATTITUDE, RC).
3. **Chain** multiple executors for a single command (e.g., for RC messages, print to console *and* forward over UDP).

## Features

- **SOLID**-oriented design:
  - **Single Responsibility Principle**: Each class handles one core concern (e.g., parsing, dispatching, or executing a single command).
  - **Open/Closed Principle**: Easily add new commands or executors without modifying existing classes.
  - **Liskov Substitution Principle**: Different input sources (UDP, file) can be swapped with each other seamlessly.
  - **Interface Segregation Principle**: Small, focused interfaces (e.g., `IInputSource`, `IMspMessageHandler`, `IMspCommandExecutor`).
  - **Dependency Inversion Principle**: High-level logic depends on abstractions rather than concrete implementations.

- **Input Sources**:
  - **UDP**: Listens on a user-specified UDP port.
  - **File**: Reads MSP bytes from a file for testing or replay.

- **Command Executors**:
  - **STATUS** (`MSP 101`): Updates the armed state.
  - **ATTITUDE** (`MSP 108`): Updates roll, pitch, and heading.
  - **FC_VARIANT** (`MSP 102`): Updates the flight controller identifier.
  - **RC** (`MSP 105`): 
    1. Prints channel data to the console.
    2. Forwards channel data over UDP to alink_drone.

