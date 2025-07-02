# Spatial-Mapping-LiDAR
A compact indoor LiDAR system using an MSP432E401Y microcontroller, a VL53L1X ToF sensor, and a stepper motor for 360° spatial mapping, with 3D visualization in Python.

## Description

This project builds a spatial mapping system using:

1. **360° LIDAR Scanning**  
   A VL53L1X time-of-flight sensor mounted on a stepper motor captures distance measurements across a vertical plane, producing up to 32 points per full rotation.

2. **Microcontroller Control & Communication**  
   An MSP432E401Y (ARM Cortex-M4F) communicates with the sensor over I2C, processes readings, and transmits data to a PC over UART at 115200 bps.

3. **Manual Displacement & 3D Mapping**  
   After each 360° scan, the device is manually shifted along the x-axis by a configurable increment (default 10 mm), enabling 3D mapping over multiple slices.

4. **Python Visualization**  
   A Python script uses PySerial to read UART data, applies polar-to-Cartesian transformation, and renders a 3D point cloud or line set with Open3D.

## How It Works

- **Hardware Initialization**  
  - Configure system clock (bus speed 22 MHz), GPIO ports for I2C, UART, stepper control, push buttons, and LEDs.  
  - Initialize VL53L1X sensor, UART, and stepper motor driver modules.

- **Scanning Sequence**  
  - User presses a push button to start scanning.  
  - Sensor measurements: poll for dataready, read distance over I2C, flash status LEDs, and send distance values as ASCII over UART.  
  - Stepper motor rotates fixed steps to complete a full 360° sweep (default 32 steps × 11.25°).  
  - After each full rotation, send marker `i` to indicate slice completion and reverse to home position.

- **Data Logging**  
  - The Python script writes each (x, y, z) coordinate to `tof_radar.xyz`, handling `i` (increment z) and `q` (quit) markers.

- **Visualization & Output**  
  - On completion, the script reads `tof_radar.xyz`, reconstructs 3D coordinates, and displays either a point cloud or a connected line mesh in Open3D.

## Skills & Modules Used

- **C/C++** for firmware development on MSP432E401Y  
- **I2C Communication** with VL53L1X ToF sensor  
- **UART Protocol** at 115200 bps for PC data transmission  
- **Stepper Motor Control** using ULN2003 driver and PH0–PH3 GPIOs  
- **GPIO Interrupts & Polling** for push-button control and LED feedback  
- **Python** scripting with PySerial and Open3D for data capture and 3D rendering  

## Testing

1. **ToF Sensor Accuracy**  
   Verified distance measurements against known reference points; observed errors within 1 mm (≈0.5% at 2000 mm).

2. **Serial Communication**  
   Verified stable UART transmission at 115200 bps with no data loss after prolonged scanning.

3. **Visualization Pipeline**  
   Successfully reconstructed a hallway scan in Open3D.

