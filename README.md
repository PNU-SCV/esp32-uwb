# esp32-uwb

This repository contains the code and setup for the ESP32-DW3000 based Ultra-Wideband (UWB) system, which enables real-time location tracking and communication with external devices like Raspberry Pi and STM32. The system utilizes UART for communication and ensures reliability with CRC in the data transmission.

## 1. Hardware
The system is built using the **ESP32-DW3000** module from **Markerfabs**, which integrates the **Qorvo DW3000** UWB chip for accurate distance measurements.

## 2. External Connections

The ESP32 communicates with external systems through UART. Data integrity is ensured using an 8-bit CRC for the transmission to the Raspberry Pi.

### Communication with Raspberry Pi

- **Data Sent to Raspberry Pi:**
  - **cmd**: Command type (1 byte)
  - **dest_x**: Destination x-coordinate (4 bytes, float)
  - **dest_z**: Destination z-coordinate (4 bytes, float)
  - **angle**: Movement angle (4 bytes, float)
  - **crc**: CRC for error checking (1 byte)

- **Data Received from Raspberry Pi:**
  - **stat**: Status response (1 byte)
  - **loc_x**: Current x-coordinate (4 bytes, float)
  - **loc_z**: Current z-coordinate (4 bytes, float)
  - **crc**: CRC for error checking (1 byte)

### Communication with STM32

- **Data Sent to STM32:**
  - **status**: Status data (1 byte)

- **Data Received from STM32:**
  - **cmd**: Command input (1 byte)

## 3. Indoor Position Estimation

The system estimates the device's position using **two anchors**. The device communicates with both anchors to calculate its location based on the round-trip time of UWB signals. The position is derived using the following polling method:

![image](https://github.com/user-attachments/assets/c1f6712c-37f6-47fa-b89e-827c3adeeb84)


## 4. RTOS Task Flow

The system runs on a dual-core ESP32 where tasks are distributed across **Core 0** and **Core 1**. Below is the detailed task flow between the UWB system, Raspberry Pi, and STM32, utilizing UART communication and semaphores for synchronization.

### Core 0
- **DW3000_RTLS_TASK**: This task handles UWB positioning, updating the `tag_position` variable (a pair of floats representing x and z coordinates). The task periodically posts updates to the task queue.
  
- **UART_ESP32_to_RaspberryPi**: This task retrieves the `status` and `tag_position` values, and transmits them to the Raspberry Pi. It also manages pending task queue items related to Raspberry Pi communication.
  
- **UART_ESP32_to_STM32**: This task retrieves and sends `status`, `cmd`, `tag_position`, `tag_angle`, and `dest` to the STM32 for further processing. Semaphore exchange ensures synchronization with other tasks.

### Core 1
- **Interrupt Service Routine (ISR)**: Handles UART interrupts for both UART1 (STM32 communication) and UART2 (Raspberry Pi communication). The ISR responds to incoming data and updates the respective shared variables.

- **UART_STM32_to_ESP32**: Receives status updates from STM32 and passes them to the main task flow via the semaphore exchange. Data is handled by the task queue.
  
- **UART_RaspberryPi_to_ESP32**: Receives commands (`cmd`), destination coordinates (`dest`), and angle (`tag_angle`) from the Raspberry Pi, and writes them to shared variables for further use by other tasks.

### Shared Variables:
- **status**: 8-bit unsigned integer used for communication between all three devices.
- **cmd**: Command input received from Raspberry Pi and sent to STM32.
- **tag_position**: A pair of float32 variables representing the device’s position (x, z coordinates).
- **dest**: A pair of float32 values representing the destination coordinates.
- **tag_angle**: Float32 variable representing the movement angle.

![image](https://github.com/user-attachments/assets/d7e016fa-25b4-4682-90e9-a0913c6d29ee)



## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

This project includes code from the DW3000 library developed by Yannick SILVA.

- **Author**: Yannick SILVA (<yannick.silva@gmail.com>)
- **Maintainer**: Yannick SILVA (<yannick.silva@gmail.com>)
- **Source**: [nconcepts.fr](https://nconcepts.fr)

The DW3000 library supports transmission of messages, timestamp handling (for ranging and location sensing applications) and implements the different operation modes the DW1000 offers. The library design is intended to offer an easy-to-use interface to the otherwise complex and configuration intense handling of the DW1000.
