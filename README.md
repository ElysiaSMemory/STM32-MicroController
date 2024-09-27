# STM32-MicroController Learning Progress

This repository documents the learning progress and projects using the **STM32F103C8 microcontroller**. Each project explores a different functionality of the STM32, utilizing the **Keil uVision 5 IDE**, and all code is written in **C** using the **Standard Peripheral Library**.

<div align="center">
  <img src="https://github.com/user-attachments/assets/0830c6db-5a2f-48c8-b95f-99af49047fc6" alt="image">
</div>

## Microcontroller Specifications

- **Microcontroller**: STM32F103C8
- **IDE**: Keil uVision 5
- **Language**: C
- **Library**: Standard Peripheral Library
- **Code**: Fully commented for clarity and learning
- **Special Thanks**: 江协科技 for support and hardware supply

## Project Overview

Each directory in the repository contains a distinct project, focusing on specific aspects of the STM32 microcontroller. Below is a summary of each project:

1. **01_STM32ProjectTemplate** - Basic project template setup for STM32.
2. **02_LEDFlash** - Flashing LED implementation.
3. **03_LEDPattern** - Controlling LED patterns.
4. **04_Buzzer** - Basic buzzer control.
5. **05_BottomControlLED** - Button-controlled LED operation.
6. **06_AmbientLightSensorControlBuzzer** - Control a buzzer based on ambient light sensor input.
7. **07_OLEDDisplay** - Display control for an OLED screen.
8. **08_PhotoelectricBeamSensorCounter(EXTI)** - Using a photoelectric sensor with external interrupts (EXTI).
9. **09_RotateEncoder(EXTI)** - Rotary encoder implementation using EXTI.
10. **10_TIMTimerInterrupt(TIMI)** - Timer interrupt with TIM.
11. **11_TIMEXTERNTimerInterrupt(EXTTIMI)** - External timer interrupt.
12. **12_LEDBreath(PWM)** - LED breathing effect using PWM.
13. **13_Servo(PWM)** - Servo motor control via PWM.
14. **14_DCMotor(PWM)** - DC motor control using PWM.
15. **15_InputCaptureFrequency(IC)** - Frequency measurement using input capture.
16. **16_InputCaptureFrequency&Duty(IC_PWMI)** - Frequency and duty cycle measurement using input capture.
17. **17_RotateEncoderSpeed(EncoderInterface)** - Rotary encoder speed measurement using encoder interface.
18. **18_ADCOneChannel(ADC)** - Single-channel ADC implementation.
19. **19_ADCMultiChannel(ADC)** - Multi-channel ADC implementation.
20. **20_DMADataTransfer** - DMA data transfer implementation.
21. **21_ADCMultiChannel(ADC&DMA)** - Multi-channel ADC with DMA.
22. **22_SerialCOMSend** - Sending data via serial communication.
23. **23_SerialSend&Recieve** - Sending and receiving data via serial communication.
24. **24_SerialHexDataPacket** - Sending hexadecimal data packets via serial communication.
25. **25_SerialTextDataPacket** - Sending text data packets via serial communication.
26. **26_SoftwareI2C(MPU6050)** - Implementing software I2C with MPU6050.
27. **27_HardwareI2C(MPU6050)** - Hardware I2C control with MPU6050.
28. **28_SoftwareSPI(W25Q64)** - Implementing software SPI with W25Q64.
29. **29_HardwareSPI(W25Q64)** - Hardware SPI control with W25Q64.

## Notes

For detailed insights on hardware and software integration, refer to the [Notes.md](./Notes.md) file.

## License

This project is open-source and freely available. Special thanks to 江协科技 for their valuable support and hardware resources.
