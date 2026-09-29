# Introduction

This project is an automatic blood pressure monitoring system built around an **STM32F411 microcontroller**. It measures cuff pressure with an **HX710B** pressure sensor, controls the pump and valve during measurement, and estimates systolic pressure, diastolic pressure, and mean arterial pressure. The results are displayed on an **ST7789** screen and can be sent to an ESP32 for web-based monitoring.

![Introduction to the blood pressure monitoring project](image/introduction.png)

# Project Choices

The **STM32F411** was selected to coordinate sensor readings, pressure control, display updates, and communication. The **HX710B** provides pressure measurements, while **the oscillometric method and a Bayesian estimator** are used to calculate blood pressure values. **A PWM-controlled valve** helps regulate cuff deflation, the **ST7789** provides a local user interface, and UART connects the STM32 to the ESP32.

![Project hardware and design choices](image/project_choices.png)