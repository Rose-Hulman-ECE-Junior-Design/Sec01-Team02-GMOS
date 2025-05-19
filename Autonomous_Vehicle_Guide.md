# 🚗 Autonomous Vehicle Learning Platform 🚗

Welcome to the **ECE362 Autonomous Vehicle Project**! This repository supports a hands-on educational experience where high school students learn real electrical and computer engineering (ECE) concepts through an engaging autonomous vehicle competition.

---

## Project Objective

Redesign the autonomous vehicle to *make ECE concepts visible and engaging*. Students won’t just race cars—they’ll explore software-hardware interactions, power efficiency, feedback control, and wireless communication.

---

## Table of Contents

- [Core Electrical Concepts](#core-electrical-concepts)
- [Core Computer Engineering Concepts](#core-computer-engineering-concepts)
- [Component Explanations](#component-explanations)
- [Series vs. Parallel Circuits](#series-vs-parallel-circuits)
- [EDLC Supercapacitors](#edlc-supercapacitors)
- [PID Control](#pid-control)
- [Bluetooth Serial Communication](#bluetooth-serial-communication)

---

## Core Electrical Concepts

| Concept   | Description |
|-----------|-------------|
| **Voltage (V)** | Electrical potential difference |
| **Current (I)** | Flow of electric charge |
| **Resistance (Ω)** | Opposition to current |
| **Power (P)** | Energy per unit time: `P = V × I` |
| **Energy (J)** | Total work: `E = P × t` |

These are observed and logged live during races!

---

## Core Computer Engineering Concepts

| Concept          | Description |
|------------------|-------------|
| **Control Systems** | Uses feedback and software to drive real-time decisions |
| **Data Logging** | Capturing voltage, current, energy |
| **Software-Hardware Interface** | Code controls motors via commandline interface |
| **Wireless Communication** | Real-time Bluetooth-based user interface |

---

## Component Explanations

| Component      | Why It's Used                              | Common Use Cases                          |
|----------------|--------------------------------------------|-------------------------------------------|
| Resistor       | Current limiting, voltage division         | LED circuits, sensor pull-ups             |
| Capacitor      | Filtering, energy storage                  | Power smoothing, timers                   |
| Diode          | Directional current control                | Power protection, rectification           |
| Zener Diode    | Voltage regulation                         | Clamping circuits, simple regulators      |
| MOSFET/BJT     | Signal amplification, electronic switching | Analog circuits, logic circuits           |

---

## Series vs. Parallel Circuits

### In Series

- Components are connected end-to-end.
- **Same current** flows through all components.
- **Voltages split** across components.

\[
R_{\text{total}} = R_1 + R_2 + R_3 + \dots
\]

### In Parallel

- Components are connected across the same voltage source.
- **Same voltage** across each branch.
- **Currents split** through each branch.

\[
\frac{1}{R_{\text{total}}} = \frac{1}{R_1} + \frac{1}{R_2} + \dots
\]

---

## EDLC Supercapacitors

**EDLC (Electric Double-Layer Capacitors)** are used instead of traditional batteries.

### Why Use Them?

- Ultra-fast charging (minutes vs. hours)
- High power density
- Excellent for high-drain, short-duration tasks

### Design Considerations

- Must avoid **reverse polarity**
- Use **voltage balancing** for series configurations
- Charging profiles must prevent **overvoltage**

### Use Case in Project:

- Vehicle charges via **Qi wireless charging** or 12V wired input
- Students observe voltage rise and energy transfer in real-time

---

## PID Control

**PID = Proportional + Integral + Derivative** controller.

Used to maintain stable autonomous driving performance.

### Why PID?

- Smooth, precise line-following behavior
- Reduces oscillations and over-corrections

### Tunable Parameters:

- `Kp` – Proportional (reacts to error)
- `Ki` – Integral (accumulates past error)
- `Kd` – Derivative (predicts future error)

---

## Bluetooth Serial Communication

# 📶 Serial Protocols and Bluetooth Communication 📶

This section explains the basics of serial communication protocols and how Bluetooth, particularly with ESP32, implements a wireless serial interface.

---

### What Is a Serial Protocol?

A **serial protocol** is a method of transmitting data **one bit at a time** over a single communication line. It’s the simplest and most common form of digital communication used in embedded systems.

#### Key Characteristics

- **Sequential data transmission** (bit-by-bit)
- Simple wiring: often just two lines (TX and RX)
- Widely used for communication between microcontrollers and peripheral devices

#### How It Works:

- Bluetooth modules like the **ESP32** simulate a virtual serial port.
- Devices communicate wirelessly as if they were connected by a physical UART.
- You can use tools like **PuTTY**, **Arduino Serial Monitor**, or mobile apps to send/receive data.

#### Bluetooth on ESP32:

- Uses **Bluetooth SPP** to emulate serial ports.
- Configured with a defined baud rate in software.
- Sends sensor readings, logs, control commands wirelessly.
- Allows live debugging and real-time parameter updates.

