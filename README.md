# Smart Farming Data Logger & Control System (AVR / ATmega128)

Designed and implemented a 3-MCU embedded system simulating a smart farming IoT network using ATmega128 microcontrollers. The system, developed entirely in **Embedded C**, performs wireless sensor data acquisition, centralized data logging, and remote actuation control with a focus on reliability, modularity, and low-power operation.

![Language](https://img.shields.io/badge/Language-Embedded_C-A8B9CC.svg?logo=c&logoColor=white)
![Microcontroller](https://img.shields.io/badge/MCU-ATmega128-CC3344.svg)
![Simulation](https://img.shields.io/badge/Simulation-Proteus_VSM-0078D4.svg)
![Wireless](https://img.shields.io/badge/Wireless-XBee_|_Bluetooth-0078D4.svg)

---

## 📡 System Architecture

The architecture consists of three distinct ATmega128 microcontroller nodes, each with a dedicated role, communicating wirelessly to form a complete data logging and control network.

> **[Image: Proteus schematic of the 3-MCU system]**
>
> *(**Developer Note:** Place a wide screenshot of your complete Proteus simulation schematic here. This is your most important visual. Also maybe add a video or some extra images)*

### 1. Remote Sensor Node (`sensor_mcu.c`)
* **Purpose:** Gathers live environmental data from the "field."
* **Hardware:**
    * ATmega128 MCU
    * Temperature & Soil Moisture Sensors (read via **ADC**)
    * 16x2 LCD for local status display
    * [cite_start]DC Water Pump (controlled via **PWM** for variable speed) 
* **Logic:** Continuously reads sensors. If moisture drops below a threshold, it automatically activates the water pump. It transmits all sensor data to the Logger Node via an **XBee S1 RF module**.

### 2. Central Logger Node (`main_mcu.c`)
* **Purpose:** The central "brain" of the system. It logs all data, manages data integrity, and handles commands.
* **Hardware:**
    * ATmega128 MCU
    * External 2KB 6116 SRAM (for expanded storage)
    * XBee S1 RF module (communicates with Sensor Node)
    * HC-05 Bluetooth module (communicates with User Node)
* **Logic:**
    * **Data Logging:** Stores incoming sensor data in both internal SRAM and the **external 6116 SRAM** using the **XMEM interface**.
    * **Data Integrity:** Implements a **CRC-5-ITU** check on received packets to ensure data consistency before logging.
    * **Command Hub:** Relays commands from the User Node (e.g., "manual pump on") to the Sensor Node.

### 3. User Interface Node (`user_mcu.c`)
* **Purpose:** Provides a remote "dashboard" for the user to monitor the system and send commands.
* **Hardware:**
    * ATmega128 MCU
    * 20x4 LCD for displaying system-wide status and sensor readings.
    * 4x3 Matrix Keypad for user input.
    * HC-05 Bluetooth module (communicates with Logger Node).
* **Logic:** Allows the user to check sensor statuses, view logged data, set new sensor thresholds, and manually override the system (e.g., turn the pump on/off).

---

## ⚙️ Key Technical Features & Concepts

* **Firmware:** The entire system is programmed in **Embedded C** with a focus on modularity and performance.
* **Interrupt-Driven Design:** Heavily utilizes **ISRs (Interrupt Service Routines)** for timers, **ADC** conversions, and **USART** (serial) for both XBee and Bluetooth. This allows the MCUs to enter **IDLE sleep mode** while waiting for events, significantly reducing power consumption.
* **Wireless Communication:** Implements two different wireless protocols over USART:
    * **XBee S1:** For a simple, reliable RF link between the sensor and logger.
    * **HC-05 Bluetooth:** For a user-friendly, high-bandwidth link between the logger and the UI.
* **Custom Messaging Protocol:** A custom 1-byte protocol was designed for efficient, low-overhead command and data exchange between the MCUs.
* **External Memory Interfacing (XMEM):** Demonstrates advanced MCU capability by interfacing with an external 6116 SRAM chip to expand data logging capacity, a critical feature for long-term data collection.
* **System Reliability:**
    * **CRC-5-ITU:** A Cyclic Redundancy Check is implemented to validate packet integrity and reject corrupt data.
    * **Watchdog Timer (WDT):** The WDT is configured to automatically reset the microcontroller in case of a software freeze or fault, ensuring long-term system reliability without manual intervention.

---

## 💻 Simulation & Compilation

### Software
* **Compiler:** `Microchip Studio` (or `avr-gcc`) to compile the C code into `.hex` files.
* **Simulation:** `Proteus VSM` is used to build and simulate the entire 3-MCU circuit, including all peripherals and wireless modules.

### How to Run the Simulation
1.  Open the `project.pdsprj` file in Proteus.
2.  Ensure each of the three ATmega128 microcontrollers is loaded with the correct compiled `.hex` file:
    * `sensor_mcu.hex` -> Sensor Node MCU
    * `main_mcu.hex` -> Logger Node MCU
    * `user_mcu.hex` -> User Interface Node MCU
3.  Run the simulation. You can interact with the User Interface keypad to send commands and observe the Sensor Node's pump and LCD react in real-time.