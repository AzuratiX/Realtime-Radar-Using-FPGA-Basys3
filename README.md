# Realtime-Radar-Using-FPGA-Basys3 (Verilog)

This repository contains a simple yet functional radar system implemented in Verilog for the **Basys 3 FPGA board**. The system rotates an **SG90 servo motor** using Pulse Width Modulation (PWM) and utilizes an **ultrasonic sensor** to measure distances in real-time across a 180-degree field. A **VGA display** is used to visually simulate the radar sweep and object detection.

## 📦 Components Used

- **FPGA Board**: Basys 3 (Xilinx Artix-7)
- **Servo Motor**: SG90 (180° rotation)
- **Ultrasonic Sensor**: HC-SR04 or compatible
- **Display**: VGA monitor (connected to Basys 3)

## 🛠 Features

- Smooth servo sweep across 180° using PWM
- Distance measurement using ultrasonic sensor via time-of-flight logic
- Visual radar simulation rendered on VGA display
- Fully implemented in **Verilog HDL**, no microcontroller used
- Runs entirely on Basys 3 board power (no external power required)

## 📂 Files

- `Radar.v` - Main Verilog module (servo control, ultrasonic timing, VGA rendering)
- `README.md` - Project overview and documentation
- `.xdc` - Pin constraint file for Basys 3 (ensure correct pin mapping based on your setup)

## 📌 Notes

- Ensure that the Basys 3 pin mappings match your setup—refer to the provided `.xdc` file.
- The VGA display is used to show a simulated radar sweep, with objects visualized based on ultrasonic sensor input.
- Power for both the SG90 servo and ultrasonic sensor is supplied directly from the Basys 3 board.

## 💡 Future Improvements

- Add UART or Bluetooth for PC/mobile radar visualization
- Better Visualization Method.
- Use Laser Sensor Instead of Ultrasonic Sensor
