# Introduction

This project is an automatic blood pressure monitoring system built around an **STM32F411 microcontroller**. It measures cuff pressure with an **HX710B** pressure sensor, controls the pump and valve during measurement, and estimates systolic pressure, diastolic pressure, and mean arterial pressure. The results are displayed on an **ST7789** screen and can be sent to an ESP32 for web-based monitoring.

![Introduction to the blood pressure monitoring project](image/introduction.png)

# Project Choices

The **STM32F411** was selected to coordinate sensor readings, pressure control, display updates, and communication. The **HX710B** provides pressure measurements, while **the oscillometric method and a Bayesian estimator** are used to calculate blood pressure values. **A PWM-controlled valve** helps regulate cuff deflation, the **ST7789** provides a local user interface, and UART connects the STM32 to the ESP32.

![Project hardware and design choices](image/project_choices.png)

# Medical Operating Principle

The monitor uses the oscillometric method. The cuff is first inflated to temporarily restrict blood flow, then slowly deflated. As blood begins flowing through the artery, each heartbeat creates small pressure oscillations in the cuff. The device analyzes these oscillations: their maximum amplitude is associated with mean arterial pressure, while systolic and diastolic pressures are estimated from the oscillation pattern using the device’s algorithm. These are device estimates and should not replace measurements or guidance from a healthcare professional.

![Medical operating principle of oscillometric blood pressure measurement](image/operating_principle.png)

# Results

The system measures cuff pressure and displays the estimated systolic pressure, diastolic pressure, and mean arterial pressure. The measured results are shown below.

![Blood pressure monitoring results](practical_test/result.jpg)

# PCB Layout

The PCB layout shows the component placement and routing used to connect the microcontroller, pressure sensor, display, and other circuit components.

![PCB layout](schematic/pcb_layout.jpg)

# Schematic

The schematic illustrates the main circuit connections between the STM32F411, pressure sensor, display, and supporting components.

![System schematic](schematic/schematic.jpg)
# PCB

The PCB provides the physical platform for the system’s electronic components and their connections.

![Blood pressure monitor PCB](schematic/pcb.png)
