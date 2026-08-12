# ESP32-Mini-Drone-Test
ESP32-based Mini Drone Test Dashboard using MPU6050 and Wi-Fi Web Server for real-time monitoring of acceleration, gyroscope, temperature, roll, pitch, and sensor status. Designed as a foundation for PID-based drone stabilization.

## 📌 Project Description

The **ESP32 Mini Drone Test Dashboard** is an embedded IoT project developed to test and monitor the motion and orientation parameters required for a future ESP32-based mini drone.

The system uses an **ESP32 DOIT DevKit V1** as the main microcontroller and an **MPU6050 IMU sensor** to measure acceleration and angular velocity along three axes.

The ESP32 creates a Wi-Fi network/web server and provides a real-time dashboard that can be accessed through a smartphone or computer browser using the IP address displayed in the Arduino IDE Serial Monitor.

The web dashboard displays the current system status and sensor measurements, including:

- ESP32 online/offline status
- Wi-Fi connection status
- MPU6050 connection status
- X, Y, and Z-axis accelerometer values
- X, Y, and Z-axis gyroscope values
- MPU6050 temperature
- Roll angle
- Pitch angle
- Real-time sensor graphs

The project is mainly designed as a **sensor testing and visualization platform for a future mini drone flight-control system**.

In the next stage, the sensor data can be processed using a **PID controller** to stabilize the drone. The PID output can then be used to control ESCs and brushless DC motors for maintaining the required roll and pitch angles.

### Current System

MPU6050
      ↓
ESP32
      ↓
Wi-Fi Web Server
      ↓
Real-Time Web Dashboard

### Future Drone System

MPU6050
      ↓
ESP32
      ↓
Angle Calculation
      ↓
PID Controller
      ↓
Motor Mixing
      ↓
ESCs
      ↓
BLDC Motors
